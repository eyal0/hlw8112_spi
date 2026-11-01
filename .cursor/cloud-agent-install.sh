#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if ! command -v esphome >/dev/null; then
  pip install --user 'esphome>=2026.10,<2027'
  sudo ln -sf "${HOME}/.local/bin/esphome" /usr/local/bin/esphome
  for bin in esptool espefuse esp_rfc2217_server platformio pio; do
    if [ -x "${HOME}/.local/bin/${bin}" ]; then
      sudo ln -sf "${HOME}/.local/bin/${bin}" "/usr/local/bin/${bin}"
    fi
  done
fi

BK_DIR=/opt/bk72xx_spi
if [ ! -d "${BK_DIR}/.git" ]; then
  sudo mkdir -p /opt
  sudo git clone --depth 1 https://github.com/eyal0/bk72xx_spi.git "${BK_DIR}"
fi
if [ ! -e /bk72xx_spi ]; then
  sudo ln -sf "${BK_DIR}" /bk72xx_spi
fi

if [ ! -f secrets.yaml ]; then
  cat > secrets.yaml <<'EOF'
wifi_ssid: "dev-network"
wifi_password: "dev-password"
EOF
fi

if ! command -v arm-none-eabi-gcc >/dev/null; then
  sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq gcc-arm-none-eabi
fi

if ! command -v scons >/dev/null; then
  pip install --user 'scons>=4.4,<5'
  sudo ln -sf "${HOME}/.local/bin/scons" /usr/local/bin/scons
fi

bash .cursor/bootstrap-platformio-tools.sh
