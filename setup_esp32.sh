#!/usr/bin/env bash

set -e

echo "======================================"
echo " ESP32 Linux Setup Script"
echo "======================================"

# -------------------------------------------------
# 1. Install basic dependencies
# -------------------------------------------------

echo
echo "[1/8] Updating package list..."
sudo apt update

echo
echo "[2/8] Installing required packages..."
sudo apt install -y curl ca-certificates

# -------------------------------------------------
# 2. Install Arduino CLI
# -------------------------------------------------

echo
echo "[3/8] Installing Arduino CLI..."

mkdir -p "$HOME/.local/bin"

if command -v arduino-cli >/dev/null 2>&1; then
    echo "arduino-cli is already installed:"
    arduino-cli version
else
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
        | BINDIR="$HOME/.local/bin" sh
fi

# -------------------------------------------------
# 3. Add ~/.local/bin to PATH
# -------------------------------------------------

if ! grep -q 'HOME/.local/bin' "$HOME/.bashrc"; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
fi

export PATH="$HOME/.local/bin:$PATH"

echo
echo "Arduino CLI version:"
arduino-cli version

# -------------------------------------------------
# 4. Initialize Arduino CLI configuration
# -------------------------------------------------

echo
echo "[4/8] Configuring Arduino CLI..."

if [ ! -f "$HOME/.arduino15/arduino-cli.yaml" ] && \
   [ ! -f "$HOME/.arduinoIDE/arduino-cli.yaml" ] && \
   [ ! -f "$HOME/.config/arduino-cli/arduino-cli.yaml" ]; then
    arduino-cli config init
else
    echo "Arduino CLI configuration already exists."
fi

# -------------------------------------------------
# 5. Add Espressif ESP32 board package URL
# -------------------------------------------------

echo
echo "[5/8] Adding ESP32 board package..."

ESP32_URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"

# Avoid adding the same URL repeatedly
if ! arduino-cli config dump | grep -qF "$ESP32_URL"; then
    arduino-cli config add board_manager.additional_urls "$ESP32_URL"
else
    echo "ESP32 package URL is already configured."
fi

echo
echo "Updating board indexes..."
arduino-cli core update-index

# -------------------------------------------------
# 6. Install ESP32 Arduino core/toolchain
# -------------------------------------------------

echo
echo "[6/8] Installing ESP32 core..."

if arduino-cli core list | grep -q '^esp32:esp32'; then
    echo "ESP32 core is already installed."
else
    arduino-cli core install esp32:esp32
fi

echo
echo "Installed cores:"
arduino-cli core list

# -------------------------------------------------
# 7. Create ESP32 test project
# -------------------------------------------------

echo
echo "[7/8] Creating ESP32 test project..."

PROJECT_DIR="$HOME/esp32_test"
SKETCH_FILE="$PROJECT_DIR/esp32_test.ino"

mkdir -p "$PROJECT_DIR"

cat > "$SKETCH_FILE" <<'EOF'
void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println("ESP32 STARTED SUCCESSFULLY");
}

void loop()
{
    Serial.println("ESP32 is running");

    delay(1000);
}
EOF

echo "Created:"
echo "  $SKETCH_FILE"

# -------------------------------------------------
# 8. Test compile
# -------------------------------------------------

echo
echo "[8/8] Compiling starter ESP32 program..."

arduino-cli compile \
    --fqbn esp32:esp32:esp32 \
    "$PROJECT_DIR"

echo
echo "======================================"
echo " SETUP COMPLETED SUCCESSFULLY"
echo "======================================"

echo
echo "Project directory:"
echo "  $PROJECT_DIR"
echo
echo "Source file:"
echo "  $SKETCH_FILE"

echo
echo "Now connect your ESP32 using USB."
echo
echo "Then detect the serial port with:"
echo
echo "  arduino-cli board list"
echo
echo "or:"
echo
echo '  ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null'

echo
echo "If your ESP32 is /dev/ttyUSB0, upload with:"
echo
echo "  arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 $PROJECT_DIR"

echo
echo "Then monitor serial output with:"
echo
echo "  arduino-cli monitor -p /dev/ttyUSB0 --config baudrate=115200"

echo
echo "If Linux reports permission denied for the serial port:"
echo
echo "  sudo usermod -aG dialout \$USER"
echo
echo "Then log out and log back in."
echo
