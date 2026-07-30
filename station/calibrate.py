#!/usr/bin/env python3

import json
import time
from pathlib import Path

import cv2

import station


ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "config.json"


def capture(camera, path, prompt, config):
    input(prompt)
    for _ in range(8):
        ok, frame = camera.read()
        if not ok:
            raise RuntimeError("камера не отдала кадр")
        time.sleep(0.03)

    if not cv2.imwrite(str(path), frame):
        raise RuntimeError("не удалось сохранить {}".format(path))

    score = station.color_score(frame, config)
    print("{}: score={:.6f}".format(path.name, score))
    return score


def main():
    config = station.load_json(CONFIG_PATH)
    references = ROOT / config["references_directory"]
    references.mkdir(parents=True, exist_ok=True)

    camera = station.open_camera(config)
    try:
        print("Камера должна быть окончательно закреплена и больше не двигаться.")
        print("Дрон поставь в точку ожидания, в которой станция будет его видеть.")

        off_scores = [
            capture(
                camera,
                references / "off_1.jpg",
                "Выключи ленту дрона и нажми Enter для фото 1/4: ",
                config,
            ),
            capture(
                camera,
                references / "off_2.jpg",
                "Лента всё ещё выключена. Нажми Enter для фото 2/4: ",
                config,
            ),
        ]
        on_scores = [
            capture(
                camera,
                references / "on_1.jpg",
                "Включи {} цвет и нажми Enter для фото 3/4: ".format(
                    config["target_color"]
                ),
                config,
            ),
            capture(
                camera,
                references / "on_2.jpg",
                "Цвет включён. Нажми Enter для фото 4/4: ",
                config,
            ),
        ]
    finally:
        camera.release()

    off_max = max(off_scores)
    on_min = min(on_scores)
    gap = on_min - off_max
    min_gap = float(config["min_score_gap"])
    if gap < min_gap:
        raise RuntimeError(
            "цвет отделяется от фона слишком слабо: gap={:.6f}, нужно >= {:.6f}. "
            "Сузь roi в config.json или приблизь камеру.".format(gap, min_gap)
        )

    calibration = {
        "target_color": config["target_color"],
        "threshold": (off_max + on_min) / 2.0,
        "off_scores": off_scores,
        "on_scores": on_scores,
        "roi": config["roi"],
        "min_saturation": config["min_saturation"],
        "min_value": config["min_value"],
    }
    path = ROOT / config["calibration_file"]
    path.write_text(
        json.dumps(calibration, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("Калибровка сохранена: {}".format(path))
    print("Порог: {:.6f}; запас: {:.6f}".format(
        calibration["threshold"], gap
    ))


if __name__ == "__main__":
    main()
