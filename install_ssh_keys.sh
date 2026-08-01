#!/usr/bin/env bash

set -uo pipefail

KEY_FILE="${ENERGY_RACE_KEY:-$HOME/.ssh/energy_race_ed25519}"
TARGETS=(
    energy-uav1
    energy-uav2
    energy-station5
    energy-station37
)

for command in ssh ssh-copy-id; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "ERROR: не найдена команда: $command" >&2
        exit 1
    fi
done

if [ ! -f "$KEY_FILE" ] || [ ! -f "$KEY_FILE.pub" ]; then
    echo "ERROR: не найден ключ $KEY_FILE или $KEY_FILE.pub" >&2
    exit 1
fi

echo "Будет установлен только публичный ключ: $KEY_FILE.pub"
echo "Текущий пароль потребуется один раз для каждого устройства."

failures=0
for target in "${TARGETS[@]}"; do
    echo
    echo "[$target] установка публичного ключа"
    if ssh-copy-id -i "$KEY_FILE.pub" "$target"; then
        echo "[$target] ключ установлен"
    else
        echo "ERROR: [$target] ключ не установлен" >&2
        failures=$((failures + 1))
    fi
done

echo
echo "Проверка входа без пароля"
for target in "${TARGETS[@]}"; do
    if ssh -o BatchMode=yes "$target" 'printf "key-login-ok\n"'; then
        echo "[$target] OK"
    else
        echo "ERROR: [$target] вход по ключу не работает" >&2
        failures=$((failures + 1))
    fi
done

if [ "$failures" -ne 0 ]; then
    echo
    echo "ERROR: обнаружено проблем: $failures" >&2
    exit 1
fi

echo
echo "Готово: все четыре устройства доступны по SSH-ключу."
