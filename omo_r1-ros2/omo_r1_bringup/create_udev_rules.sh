#!/bin/bash
# create_omo_rules.sh
# OMO R1 LiDAR & Motor udev rule setup

echo ""
echo "=== OMO R1 udev rules setup ==="
echo ""

# --- LiDAR rule ---
echo "Setting YDLIDAR rule..."
sudo bash -c 'cat > /etc/udev/rules.d/97-omo-r1-lidar.rules <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyLiDAR"
EOF'
echo "✅ Created: 97-omo-r1-lidar.rules"

# --- MOTOR(MCU) rule ---
echo ""
echo "Setting Motor (MCU) rule..."
sudo bash -c 'cat > /etc/udev/rules.d/98-omo-r1-motor.rules <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyMotor"
EOF'
echo "✅ Created: 98-omo-r1-motor.rules"

# --- Apply ---
echo ""
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "=== Done! ==="
echo "Check with: ls -l /dev | grep tty"
