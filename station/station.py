#!/usr/bin/env python3

import datetime
import json
import socket
import time
import uuid
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "config.json"

HSV_RANGES = {
    "red": [
        ((0, 90, 110), (10, 255, 255)),
        ((170, 90, 110), (179, 255, 255)),
    ],
    "green": [((35, 90, 110), (90, 255, 255))],
    "blue": [((90, 90, 110), (135, 255, 255))],
    "yellow": [((18, 90, 110), (35, 255, 255))],
}


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def open_camera(config):
    index = config["camera_index"]
    camera = cv2.VideoCapture(index)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(config["camera_width"]))
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config["camera_height"]))
    camera.set(cv2.CAP_PROP_FPS, int(config["camera_fps"]))
    if not camera.isOpened():
        raise RuntimeError("камера {} не открылась".format(index))

    for _ in range(15):
        ok, _ = camera.read()
        if not ok:
            camera.release()
            raise RuntimeError("камера открылась, но не отдаёт кадры")
    return camera


def crop_roi(frame, roi):
    height, width = frame.shape[:2]
    x1 = max(0, min(width - 1, int(float(roi["x"]) * width)))
    y1 = max(0, min(height - 1, int(float(roi["y"]) * height)))
    x2 = max(x1 + 1, min(
        width, int((float(roi["x"]) + float(roi["width"])) * width)
    ))
    y2 = max(y1 + 1, min(
        height, int((float(roi["y"]) + float(roi["height"])) * height)
    ))
    return frame[y1:y2, x1:x2]


def color_score(frame, config):
    color = config["target_color"]
    if color not in HSV_RANGES:
        raise RuntimeError("неподдерживаемый target_color: {}".format(color))

    image = crop_roi(frame, config["roi"])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    saturation = int(config["min_saturation"])
    value = int(config["min_value"])
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for lower, upper in HSV_RANGES[color]:
        actual_lower = (lower[0], saturation, value)
        mask = cv2.bitwise_or(
            mask,
            cv2.inRange(
                hsv,
                np.array(actual_lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            ),
        )

    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8)
    )
    return float(cv2.countNonZero(mask)) / float(mask.size)


class StatusLed:
    def __init__(self, config):
        pins = config["status_led"]
        values = (pins["red_gpio"], pins["green_gpio"], pins["blue_gpio"])
        self.led = None
        if any(value is None for value in values):
            return
        try:
            from gpiozero import RGBLED

            self.led = RGBLED(
                red=int(values[0]),
                green=int(values[1]),
                blue=int(values[2]),
                active_high=bool(pins["active_high"]),
            )
        except Exception as error:
            print("status LED отключён: {}".format(error), flush=True)

    def set(self, color):
        if self.led is None:
            return
        colors = {
            "free": (0, 1, 0),
            "pending": (1, 1, 0),
            "reserved": (0, 0, 1),
            "occupied": (1, 0, 0),
        }
        self.led.color = colors[color]

    def close(self):
        if self.led is not None:
            self.led.close()


class Station:
    def __init__(self, config, calibration):
        self.config = config
        self.calibration = calibration
        self.station_id = int(config["station_id"])
        self.threshold = float(calibration["threshold"])
        self.pending = None
        self.state = "free"
        self.reservation_deadline = 0.0
        self.high_since = None
        self.last_score_log = 0.0
        self.led = StatusLed(config)
        self.camera = open_camera(config)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((
            config["listen_ip"],
            int(config["listen_port"]),
        ))
        self.socket.setblocking(False)
        self.detections = ROOT / config["detections_directory"]
        self.detections.mkdir(parents=True, exist_ok=True)
        self.log_path = ROOT / "station-{}.jsonl".format(self.station_id)
        self.led.set("free")

    def log(self, event, **data):
        record = {
            "time": datetime.datetime.now().isoformat(),
            "event": event,
            "station": self.station_id,
            "state": self.state,
        }
        record.update(data)
        line = json.dumps(record, ensure_ascii=False)
        print(line, flush=True)
        with self.log_path.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")

    def send(self, event, request, **extra):
        payload = {
            "team": self.config["team"],
            "event": event,
            "station": self.station_id,
            "uav": request["uav"],
            "request_id": request["request_id"],
        }
        payload.update(extra)
        encoded = json.dumps(payload).encode("utf-8")
        reply_port = (
            request.get("reply_port") or self.config["drone_event_port"]
        )
        destination = (request["ip"], int(reply_port))
        for _ in range(5):
            self.socket.sendto(encoded, destination)
            time.sleep(0.05)
        self.log("udp_sent", message=event, destination=destination[0])

    def receive_messages(self):
        while True:
            try:
                payload, address = self.socket.recvfrom(4096)
            except BlockingIOError:
                return

            try:
                message = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(message, dict):
                continue
            if message.get("team") != self.config["team"]:
                continue

            event = message.get("event")
            station = message.get("station")
            if station not in (self.station_id, "any", None):
                continue

            if event == "REQUEST_LAND":
                self.handle_request(message, address[0])
            elif event == "LANDED":
                self.handle_landed(message)
            elif event == "STATION_RELEASED":
                self.handle_released(message)

    def handle_request(self, message, ip):
        incoming_id = str(message.get("request_id") or "")
        if (
            self.pending is not None
            and incoming_id
            and incoming_id == self.pending["request_id"]
            and str(message.get("uav", "unknown")) == self.pending["uav"]
        ):
            return

        if self.state != "free":
            request = {
                "uav": str(message.get("uav", "unknown")),
                "request_id": incoming_id,
                "ip": ip,
                "reply_port": message.get("reply_port"),
            }
            self.send("STATION_BUSY", request)
            return

        request_id = incoming_id or str(uuid.uuid4())
        self.pending = {
            "uav": str(message.get("uav", "unknown")),
            "request_id": request_id,
            "ip": ip,
            "reply_port": message.get("reply_port"),
            "deadline": time.monotonic()
            + float(self.config["request_timeout"]),
        }
        self.state = "pending"
        self.high_since = None
        self.led.set("pending")
        self.log(
            "landing_requested",
            uav=self.pending["uav"],
            source=ip,
            reported_led=message.get("led"),
        )

    def handle_landed(self, message):
        if not self.matches_reservation(message):
            return
        self.state = "occupied"
        self.reservation_deadline = 0.0
        self.led.set("occupied")
        self.log("uav_landed", uav=message.get("uav"))

    def handle_released(self, message):
        if not self.matches_reservation(message):
            return
        self.log("station_released", uav=message.get("uav"))
        self.reset()

    def matches_reservation(self, message):
        return (
            self.pending is not None
            and str(message.get("uav")) == self.pending["uav"]
            and str(message.get("request_id"))
            == self.pending["request_id"]
        )

    def reset(self):
        self.pending = None
        self.state = "free"
        self.high_since = None
        self.reservation_deadline = 0.0
        self.led.set("free")

    def detect(self, frame):
        now = time.monotonic()
        score = color_score(frame, self.config)

        if now - self.last_score_log >= 1.0:
            self.last_score_log = now
            self.log("color_score", score=round(score, 6))

        if score < self.threshold:
            self.high_since = None
            return
        if self.high_since is None:
            self.high_since = now
            return
        if now - self.high_since < float(self.config["stable_seconds"]):
            return

        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        image_path = self.detections / "{}-{}-{}.jpg".format(
            stamp, self.pending["uav"], self.config["target_color"]
        )
        cv2.imwrite(str(image_path), frame)
        self.log(
            "uav_led_detected",
            uav=self.pending["uav"],
            color=self.config["target_color"],
            score=round(score, 6),
            evidence=str(image_path),
        )
        self.send(
            "LAND_GRANTED",
            self.pending,
            detected_color=self.config["target_color"],
            score=round(score, 6),
        )
        self.state = "reserved"
        self.reservation_deadline = (
            now + float(self.config["reservation_timeout"])
        )
        self.high_since = None
        self.led.set("reserved")

    def run(self):
        self.log(
            "station_started",
            listen_port=self.config["listen_port"],
            target_color=self.config["target_color"],
            threshold=self.threshold,
        )
        while True:
            self.receive_messages()
            now = time.monotonic()

            if (
                self.state == "pending"
                and now >= self.pending["deadline"]
            ):
                self.log("request_timeout", uav=self.pending["uav"])
                self.reset()
            elif (
                self.state == "reserved"
                and now >= self.reservation_deadline
            ):
                self.log("reservation_timeout", uav=self.pending["uav"])
                self.reset()

            ok, frame = self.camera.read()
            if not ok:
                raise RuntimeError("потерян видеопоток камеры")
            if self.state == "pending":
                self.detect(frame)
            time.sleep(0.02)

    def close(self):
        self.camera.release()
        self.socket.close()
        self.led.close()


def validate(config, calibration):
    if int(config["station_id"]) not in (5, 37):
        raise RuntimeError("station_id должен быть 5 или 37")
    if calibration["target_color"] != config["target_color"]:
        raise RuntimeError(
            "target_color изменён после калибровки; запусти calibrate.py снова"
        )
    if calibration["roi"] != config["roi"]:
        raise RuntimeError(
            "roi изменён после калибровки; запусти calibrate.py снова"
        )


def main():
    config = load_json(CONFIG_PATH)
    calibration_path = ROOT / config["calibration_file"]
    if not calibration_path.exists():
        raise RuntimeError("сначала запусти python3 calibrate.py")
    calibration = load_json(calibration_path)
    validate(config, calibration)

    controller = Station(config, calibration)
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.log("station_stopped")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
