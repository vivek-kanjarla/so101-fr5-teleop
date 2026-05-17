#!/usr/bin/env bash
# linux_setup.sh — one-time Linux environment setup for SO-101 → FR5 teleop.
#
# Run once per machine after cloning the repo:
#   chmod +x linux_setup.sh
#   ./linux_setup.sh
#
# What this script does:
#   1. Add user to dialout group  (SO-101 serial port access)
#   2. Install Intel RealSense SDK + udev rules  (D405 camera)
#   3. Install Python dependencies
#   4. Verify network interface for FR5 (192.168.58.x)
#   5. Print a checklist of what still needs manual steps

set -euo pipefail

BOLD="\033[1m"
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
RESET="\033[0m"

info()  { echo -e "${BOLD}[INFO]${RESET}  $*"; }
ok()    { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${RESET}   $*"; }
fail()  { echo -e "${RED}[FAIL]${RESET}   $*"; }

echo ""
echo -e "${BOLD}=== SO-101 → FR5 Teleop — Linux Setup ===${RESET}"
echo ""

# ── 1. Serial port group ──────────────────────────────────────────────────────

info "Checking serial port group membership (dialout)..."
if groups "$USER" | grep -qw dialout; then
    ok "Already in dialout group."
else
    warn "Not in dialout group — adding..."
    sudo usermod -aG dialout "$USER"
    warn "You must LOG OUT and back in (or reboot) for this to take effect."
    warn "Until then, SO-101 will fail with 'Permission denied' on /dev/ttyACM0."
fi

# ── 2. Intel RealSense SDK ────────────────────────────────────────────────────

info "Checking for Intel RealSense SDK (librealsense2)..."
if dpkg -l librealsense2 &>/dev/null; then
    ok "librealsense2 already installed."
else
    warn "librealsense2 not found — installing from Intel's apt repo..."

    # Detect Ubuntu codename
    CODENAME=$(lsb_release -cs 2>/dev/null || echo "")
    if [ -z "$CODENAME" ]; then
        fail "Could not detect Ubuntu codename. Install librealsense2 manually:"
        fail "  https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md"
    else
        sudo mkdir -p /etc/apt/keyrings
        curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
            | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
        echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
https://librealsense.intel.com/Debian/apt-repo ${CODENAME} main" \
            | sudo tee /etc/apt/sources.list.d/librealsense.list > /dev/null
        sudo apt-get update -qq
        sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
        ok "librealsense2 installed."
    fi
fi

# Install udev rules so D405 is accessible without root
RULES_FILE="/etc/udev/rules.d/99-realsense-libusb.rules"
if [ -f "$RULES_FILE" ]; then
    ok "RealSense udev rules already in place."
else
    info "Installing RealSense udev rules..."
    RULES_SRC=$(dpkg -L librealsense2 2>/dev/null | grep -m1 99-realsense || true)
    if [ -n "$RULES_SRC" ] && [ -f "$RULES_SRC" ]; then
        sudo cp "$RULES_SRC" "$RULES_FILE"
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        ok "RealSense udev rules installed."
    else
        warn "Could not find packaged udev rules. Download manually:"
        warn "  https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules"
        warn "  sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/"
        warn "  sudo udevadm control --reload-rules && sudo udevadm trigger"
    fi
fi

# ── 3. Python dependencies ────────────────────────────────────────────────────

info "Installing Python dependencies..."
pip install --upgrade pip -q
pip install -r requirements.txt --quiet \
    && ok "Python packages installed." \
    || warn "Some packages failed — check output above. fairino may need manual install."

echo ""
warn "fairino SDK: install from Fairino's distribution .whl file, NOT PyPI."
warn "  pip install /path/to/fairino-*.whl"
echo ""

# ── 4. Network interface check ────────────────────────────────────────────────

FR5_SUBNET="192.168.58"
info "Checking for a network interface on the ${FR5_SUBNET}.x subnet..."

IFACE=$(ip -4 addr show | awk '/inet '"${FR5_SUBNET//./\\.}"'/{print $NF}' | head -1)
if [ -n "$IFACE" ]; then
    IP_ADDR=$(ip -4 addr show dev "$IFACE" | awk '/inet /{print $2}')
    ok "Interface $IFACE has IP $IP_ADDR — FR5 subnet looks configured."
else
    warn "No interface found on ${FR5_SUBNET}.x subnet."
    warn "Configure the Ethernet port connected to FR5. Example (replace eth0 and .10):"
    warn "  sudo ip addr add 192.168.58.10/24 dev eth0"
    warn "  sudo ip link set eth0 up"
    warn ""
    warn "For persistent config on Ubuntu, edit /etc/netplan/ or use nm-connection-editor."
fi

# ── 5. USB latency (ttyUSBx adapters only) ───────────────────────────────────
#
# The SO-101 typically appears as /dev/ttyACM0 (CDC ACM — latency is fine).
# If yours appears as /dev/ttyUSBx (FTDI/CP210x), the default 16ms USB latency
# will blow the 8ms ServoJ budget. Set it to 1ms:

if ls /sys/bus/usb-serial/devices/ttyUSB* &>/dev/null 2>&1; then
    warn "ttyUSBx device detected. Setting USB latency to 1ms..."
    for DEV in /sys/bus/usb-serial/devices/ttyUSB*/latency_timer; do
        echo 1 | sudo tee "$DEV" > /dev/null
        ok "Set latency_timer=1 for $DEV"
    done
    warn "Note: this resets on reboot. Add to /etc/rc.local or a udev rule for persistence."
fi

# ── 6. pynput / keyboard access ──────────────────────────────────────────────

info "Checking pynput display access..."
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    warn "No DISPLAY or WAYLAND_DISPLAY detected."
    warn "pynput keyboard listener requires a display session."
    warn "If running over SSH, add '-X' flag or set DISPLAY manually:"
    warn "  export DISPLAY=:0  # if a desktop session is running on the machine"
    warn "Alternatively, run teleop.py directly at the physical console."
else
    ok "Display environment variable is set (${DISPLAY:-${WAYLAND_DISPLAY}})."
fi

# ── Done ──────────────────────────────────────────────────────────────────────

echo ""
echo -e "${BOLD}=== Setup complete. Manual steps remaining: ===${RESET}"
echo ""
echo "  1. If you were added to dialout: LOG OUT and back in."
echo "  2. Install fairino .whl from Fairino's distribution:  pip install fairino-*.whl"
echo "  3. If Ethernet not yet configured: see IP address instructions above."
echo "  4. Plug in SO-101 and run:  python check_hardware.py"
echo "     Plug in FR5 Ethernet and run: python check_network.py"
echo ""
