#!/usr/bin/env bash
# Register PlatformIO core tools without the PlatformIO registry when egress blocks it.
# CI and fresh installs still benefit from api.platformio.org allowlisting; this is a fallback.
set -euo pipefail

PIO_PACKAGES="${HOME}/.platformio/packages"
mkdir -p "${PIO_PACKAGES}"

setup_toolchain() {
  local dir="${PIO_PACKAGES}/toolchain-gccarmnoneeabi"
  mkdir -p "${dir}/bin"
  cat > "${dir}/package.json" <<'EOF'
{"name":"toolchain-gccarmnoneeabi","version":"1.100301.220327","description":"GCC for ARM Cortex-M"}
EOF
  cat > "${dir}/.piopm" <<'EOF'
{"type":"toolchain","name":"toolchain-gccarmnoneeabi","version":"1.100301.220327","spec":{"owner":"platformio","name":"toolchain-gccarmnoneeabi","requirements":"~1.100301.0"}}
EOF
  touch "${dir}/.piokeep"
  for t in gcc g++ ld ar objcopy nm ranlib strip size addr2line gdb; do
    if command -v "arm-none-eabi-${t}" >/dev/null; then
      ln -sf "$(command -v "arm-none-eabi-${t}")" "${dir}/bin/arm-none-eabi-${t}"
    fi
  done
}

setup_scons() {
  local dir="${PIO_PACKAGES}/tool-scons"
  mkdir -p "${dir}"
  cat > "${dir}/package.json" <<'EOF'
{"name":"tool-scons","version":"4.41101.0","description":"SCons tool"}
EOF
  cat > "${dir}/.piopm" <<'EOF'
{"type":"tool","name":"tool-scons","version":"4.41101.0","spec":{"owner":"platformio","name":"tool-scons","requirements":"~4.41101.0"}}
EOF
  touch "${dir}/.piokeep"
  ln -sf /usr/local/bin/scons "${dir}/scons"
  ln -sf /usr/local/bin/scons "${dir}/scons.py"
}

setup_toolchain
setup_scons
