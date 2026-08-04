#!/bin/bash
# Script for wireless ADB connection to the device

# Avoid 5555 — conflicts with native ZMQ IN (tcp://127.0.0.1:5555)
ADB_PORT="${ADB_PORT:-5557}"

# Get device IP address
get_device_ip() {
    # WiFi interface IP (wlan0 or wlan2)
    local ip=$(adb shell "ip addr | grep 'inet ' | grep -v '127.0.0.1' | grep 'wlan'" | head -1 | awk '{print $2}' | cut -d'/' -f1)

    if [ -z "$ip" ]; then
        # If wlan not found, try any non-loopback interface
        ip=$(adb shell "ip addr | grep 'inet ' | grep -v '127.0.0.1' | grep -v 'rmnet'" | head -1 | awk '{print $2}' | cut -d'/' -f1)
    fi

    echo "$ip"
}

echo "🔌 ADB Wireless Connection Script"
echo "=================================="
echo ""

# Resolve IP (override with DEVICE_IP env var)
if [ -z "$DEVICE_IP" ]; then
    echo "🔍 Auto-detecting IP address..."
    DEVICE_IP=$(get_device_ip)

    if [ -z "$DEVICE_IP" ]; then
        echo "❌ Failed to detect device IP address"
        echo "💡 Make sure the device is connected via USB and WiFi is enabled"
        exit 1
    fi
    echo "✅ Detected IP: $DEVICE_IP"
else
    echo "📌 Using configured IP: $DEVICE_IP"
fi

echo ""

# Check if already connected
if adb devices | grep -q "$DEVICE_IP:$ADB_PORT"; then
    echo "✅ Device already connected over Wi-Fi: $DEVICE_IP:$ADB_PORT"
    adb devices
    exit 0
fi

# If USB is connected, switch to TCP/IP mode
USB_DEVICE=$(adb devices | grep -v "List of devices" | grep -v ":" | grep "device$")
if [ ! -z "$USB_DEVICE" ]; then
    echo "📱 USB connection found: $USB_DEVICE"
    echo "🔄 Switching to TCP/IP mode..."
    adb tcpip $ADB_PORT
    sleep 3
    echo "⏳ Waiting for ADB daemon restart..."
fi

# Connect over Wi-Fi
echo "📡 Connecting to $DEVICE_IP:$ADB_PORT..."
adb connect $DEVICE_IP:$ADB_PORT

sleep 1

# Check result
echo ""
echo "📋 Device list:"
adb devices

if adb devices | grep -q "$DEVICE_IP:$ADB_PORT"; then
    echo ""
    echo "✅ Connected successfully!"
    echo "💡 You can now disconnect the USB cable and use it for Panda"
else
    echo ""
    echo "❌ Connection failed"
    echo "Make sure:"
    echo "  1. Phone and computer are on the same Wi-Fi network"
    echo "  2. IP address is correct (current: $DEVICE_IP)"
    echo "  3. USB debugging is enabled"
fi
