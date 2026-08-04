#!/usr/bin/env python3
"""
Panda Flashing Utility
Based on pandaflash.cpp logic - Python port for flashing Panda devices

Requirements:
    pip install pyusb

Usage:
    python panda_flash.py --firmware panda.bin.signed
    python panda_flash.py --firmware panda_h7.bin.signed --mcu h7
    python panda_flash.py --check-only  # Just check firmware version
"""

import usb.core
import usb.util
import struct
import time
import argparse
import sys
from pathlib import Path
from typing import Optional, List, Tuple
from dataclasses import dataclass


@dataclass
class McuConfig:
    """MCU Configuration"""

    mcu: str
    mcu_idcode: int
    sector_sizes: List[int]
    sector_count: int
    uid_address: int
    block_size: int
    serial_number_address: int
    app_address: int
    app_fn: str
    bootstub_address: int
    bootstub_fn: str

    def sector_address(self, i: int) -> int:
        """Get address of sector i (assume bootstub is in sector 0)"""
        address = self.bootstub_address
        for j in range(i):
            address += self.sector_sizes[j]
        return address


# STM32F4 Configuration (Black Panda)
F4_CONFIG = McuConfig(
    mcu="STM32F4",
    mcu_idcode=0x463,
    sector_sizes=[
        0x4000,
        0x4000,
        0x4000,
        0x4000,
        0x10000,
        0x20000,
        0x20000,
        0x20000,
        0x20000,
        0x20000,
        0x20000,
        0x20000,
        0x20000,
        0x20000,
        0x20000,
        0x20000,
    ],
    sector_count=16,
    uid_address=0x1FFF7A10,
    block_size=0x800,
    serial_number_address=0x1FFF79C0,
    app_address=0x8004000,
    app_fn="panda.bin.signed",
    bootstub_address=0x8000000,
    bootstub_fn="bootstub.panda.bin",
)

# STM32H7 Configuration (Red Panda)
H7_CONFIG = McuConfig(
    mcu="STM32H7",
    mcu_idcode=0x483,
    sector_sizes=[0x20000, 0x20000, 0x20000, 0x20000, 0x20000, 0x20000, 0x20000],
    sector_count=8,
    uid_address=0x1FF1E800,
    block_size=0x400,
    serial_number_address=0x080FFFC0,
    app_address=0x8020000,
    app_fn="panda_h7.bin.signed",
    bootstub_address=0x8000000,
    bootstub_fn="bootstub.panda_h7.bin",
)


class PandaFlasher:
    """Panda Firmware Flasher"""

    # USB VID/PID for Panda
    PANDA_VID = [0xBBAA, 0x3801]
    PANDA_PID = [0xDDCC, 0xDDEE]

    # USB Control Transfer Parameters
    TIMEOUT = 15000  # 15 seconds

    def __init__(self, verbose: bool = True):
        self.verbose = verbose
        self.dev: Optional[usb.core.Device] = None

    def log(self, msg: str):
        """Print log message"""
        if self.verbose:
            print(f"[INFO] {msg}")

    def error(self, msg: str):
        """Print error message"""
        print(f"[ERROR] {msg}", file=sys.stderr)

    def find_panda(self, timeout: int = 10) -> Optional[usb.core.Device]:
        """Find and open Panda device"""
        self.log("Searching for Panda device...")

        start_time = time.time()
        while time.time() - start_time < timeout:
            for vid in self.PANDA_VID:
                for pid in self.PANDA_PID:
                    dev = usb.core.find(idVendor=vid, idProduct=pid)
                    if dev is not None:
                        self.log(f"Found Panda: VID=0x{vid:04x}, PID=0x{pid:04x}")
                        try:
                            # Try to claim the device
                            if dev.is_kernel_driver_active(0):
                                try:
                                    dev.detach_kernel_driver(0)
                                    self.log("Detached kernel driver")
                                except:
                                    pass
                            dev.set_configuration()
                            self.dev = dev
                            return dev
                        except Exception as e:
                            self.error(f"Failed to open device: {e}")
                            continue

            time.sleep(0.5)

        self.error("Panda device not found!")
        return None

    def get_mcu_type(self) -> Optional[str]:
        """Get MCU type (F4 or H7) from hardware type"""
        if not self.dev:
            return None

        try:
            # USB Control Transfer to get hardware type
            # bRequest=0xc1 - Get Hardware Type
            hw_type = self.dev.ctrl_transfer(
                bmRequestType=usb.util.CTRL_IN
                | usb.util.CTRL_TYPE_VENDOR
                | usb.util.CTRL_RECIPIENT_DEVICE,
                bRequest=0xC1,
                wValue=0,
                wIndex=0,
                data_or_wLength=0x40,
                timeout=self.TIMEOUT,
            )

            if len(hw_type) > 0:
                hw_type_id = hw_type[0]
                self.log(f"Hardware Type: {hw_type_id}")

                # Map hardware type to MCU
                # 0x01=WHITE, 0x02=GREY, 0x03=BLACK -> F4
                # 0x07=RED, 0x08=RED_V2 -> H7
                if hw_type_id in [0x01, 0x02, 0x03]:
                    self.log("Detected STM32F4 (Black Panda)")
                    return "F4"
                elif hw_type_id in [0x07, 0x08]:
                    self.log("Detected STM32H7 (Red Panda)")
                    return "H7"
                else:
                    self.error(f"Unknown hardware type: {hw_type_id}")
                    return None
        except Exception as e:
            self.error(f"Failed to get MCU type: {e}")
            return None

    def get_mcu_config(self, mcu_type: str) -> McuConfig:
        """Get MCU configuration"""
        if mcu_type == "H7":
            return H7_CONFIG
        return F4_CONFIG

    def reset(self, bootstub: bool = False):
        """Reset Panda device"""
        if not self.dev:
            return

        try:
            self.log(f"Resetting to {'bootstub' if bootstub else 'normal mode'}...")
            self.dev.ctrl_transfer(
                bmRequestType=usb.util.CTRL_IN
                | usb.util.CTRL_TYPE_VENDOR
                | usb.util.CTRL_RECIPIENT_DEVICE,
                bRequest=0xD1,
                wValue=1 if bootstub else 0,
                wIndex=0,
                data_or_wLength=0,
                timeout=self.TIMEOUT,
            )
            time.sleep(2)  # Wait for reset
        except Exception as e:
            self.log(f"Reset command sent (device disconnected as expected)")

    def flasher_present(self) -> bool:
        """Check if bootloader/flasher is present"""
        if not self.dev:
            return False

        try:
            # bRequest=0xb0 - Check flasher
            fr = self.dev.ctrl_transfer(
                bmRequestType=usb.util.CTRL_IN
                | usb.util.CTRL_TYPE_VENDOR
                | usb.util.CTRL_RECIPIENT_DEVICE,
                bRequest=0xB0,
                wValue=0,
                wIndex=0,
                data_or_wLength=0x0C,
                timeout=self.TIMEOUT,
            )

            if len(fr) == 0x0C:
                # Check for signature: 0xdeadd00d at bytes 4-7
                signature = (
                    fr[4] == 0xDE and fr[5] == 0xAD and fr[6] == 0xD0 and fr[7] == 0x0D
                )
                if signature:
                    self.log("Flasher/bootloader detected")
                    return True
                else:
                    self.log("Flasher signature mismatch")
                    return False
        except Exception as e:
            self.log(f"Flasher not present: {e}")
            return False

    def get_firmware_signature(self) -> Optional[bytes]:
        """Get firmware signature from device"""
        if not self.dev:
            return None

        try:
            # Read signature in two parts (0x40 bytes each)
            # bRequest=0xd3 - Get signature part 1
            part1 = self.dev.ctrl_transfer(
                bmRequestType=usb.util.CTRL_IN
                | usb.util.CTRL_TYPE_VENDOR
                | usb.util.CTRL_RECIPIENT_DEVICE,
                bRequest=0xD3,
                wValue=0,
                wIndex=0,
                data_or_wLength=0x40,
                timeout=self.TIMEOUT,
            )

            # bRequest=0xd4 - Get signature part 2
            part2 = self.dev.ctrl_transfer(
                bmRequestType=usb.util.CTRL_IN
                | usb.util.CTRL_TYPE_VENDOR
                | usb.util.CTRL_RECIPIENT_DEVICE,
                bRequest=0xD4,
                wValue=0,
                wIndex=0,
                data_or_wLength=0x40,
                timeout=self.TIMEOUT,
            )

            if len(part1) == 0x40 and len(part2) == 0x40:
                signature = bytes(part1) + bytes(part2)
                self.log(f"Device signature (first 16 bytes): {signature[:16].hex()}")
                return signature
            else:
                self.error("Failed to read complete signature")
                return None
        except Exception as e:
            self.error(f"Failed to get firmware signature: {e}")
            return None

    def extract_firmware_signature(self, firmware_data: bytes) -> bytes:
        """Extract signature from firmware file (last 0x80 bytes)"""
        return firmware_data[-0x80:]

    def get_packet_versions(self) -> Optional[Tuple[int, int, int]]:
        """Get packet versions from Panda (health, can, can_health)"""
        if not self.dev:
            return None

        try:
            # USB Control Transfer to get packet versions
            # bRequest=0xdd - Get packet versions (correct command!)
            versions = self.dev.ctrl_transfer(
                bmRequestType=usb.util.CTRL_IN
                | usb.util.CTRL_TYPE_VENDOR
                | usb.util.CTRL_RECIPIENT_DEVICE,
                bRequest=0xDD,
                wValue=0,
                wIndex=0,
                data_or_wLength=3,  # 3 bytes: health_ver, can_ver, can_health_ver
                timeout=self.TIMEOUT,
            )

            if len(versions) >= 3:
                health_ver = versions[0]
                can_ver = versions[1]
                can_health_ver = versions[2]
                self.log(
                    f"Packet Versions: health={health_ver}, can={can_ver}, can_health={can_health_ver}"
                )
                return (health_ver, can_ver, can_health_ver)
            else:
                self.log("Failed to read packet versions (too short)")
                return None
        except Exception as e:
            self.log(f"Could not get packet versions (older firmware?): {e}")
            return None

    def flash_firmware(self, firmware_data: bytes, mcu_config: McuConfig) -> bool:
        """Flash firmware to device"""
        if not self.dev:
            self.error("No device connected")
            return False

        self.log("Starting firmware flash process...")

        # Determine sectors to erase
        apps_sectors_cumsum = []
        cumsum = 0
        for size in mcu_config.sector_sizes[1:]:
            cumsum += size
            apps_sectors_cumsum.append(cumsum)

        last_sector = -1
        for i, cumsum in enumerate(apps_sectors_cumsum):
            if cumsum > len(firmware_data):
                last_sector = i + 1
                break

        if last_sector < 1:
            self.error("Binary too small? No sector to erase.")
            return False

        if last_sector >= 7:
            self.error("Binary too large! Risk of overwriting provisioning chunk.")
            return False

        try:
            # Unlock flash
            self.log("Unlocking flash...")
            self.dev.ctrl_transfer(
                bmRequestType=usb.util.CTRL_OUT
                | usb.util.CTRL_TYPE_VENDOR
                | usb.util.CTRL_RECIPIENT_DEVICE,
                bRequest=0xB1,
                wValue=0,
                wIndex=0,
                data_or_wLength=None,
                timeout=self.TIMEOUT,
            )

            # Erase sectors
            self.log(f"Erasing sectors 1 to {last_sector}...")
            for i in range(1, last_sector + 1):
                self.log(f"  Erasing sector {i}/{last_sector}...")
                self.dev.ctrl_transfer(
                    bmRequestType=usb.util.CTRL_OUT
                    | usb.util.CTRL_TYPE_VENDOR
                    | usb.util.CTRL_RECIPIENT_DEVICE,
                    bRequest=0xB2,
                    wValue=i,
                    wIndex=0,
                    data_or_wLength=None,
                    timeout=self.TIMEOUT,
                )

            # Flash data over endpoint 2
            STEP = 0x10
            total_bytes = len(firmware_data)
            self.log(f"Flashing {total_bytes} bytes...")

            for i in range(0, total_bytes, STEP):
                chunk = firmware_data[i : i + STEP]
                try:
                    self.dev.write(0x02, chunk, timeout=self.TIMEOUT)
                except Exception as e:
                    self.error(f"Failed to write at offset {i}: {e}")
                    return False

                # Progress indicator
                if i % (STEP * 100) == 0:
                    progress = (i / total_bytes) * 100
                    self.log(f"  Progress: {progress:.1f}%")

            self.log("Flash write complete!")

            # Reset device
            self.log("Resetting device...")
            try:
                self.dev.ctrl_transfer(
                    bmRequestType=usb.util.CTRL_IN
                    | usb.util.CTRL_TYPE_VENDOR
                    | usb.util.CTRL_RECIPIENT_DEVICE,
                    bRequest=0xD8,
                    wValue=0,
                    wIndex=0,
                    data_or_wLength=0,
                    timeout=self.TIMEOUT,
                )
            except:
                self.log("Reset sent (device disconnected)")

            return True

        except Exception as e:
            self.error(f"Flash failed: {e}")
            return False

    def flash_if_needed(self, firmware_path: Path, force: bool = False) -> bool:
        """Flash firmware if signature doesn't match"""

        # Read firmware file
        if not firmware_path.exists():
            self.error(f"Firmware file not found: {firmware_path}")
            return False

        firmware_data = firmware_path.read_bytes()
        self.log(f"Loaded firmware: {len(firmware_data)} bytes")

        # Find Panda
        if not self.find_panda():
            return False

        # Get MCU type
        mcu_type = self.get_mcu_type()
        if not mcu_type:
            self.error("Could not determine MCU type")
            return False

        mcu_config = self.get_mcu_config(mcu_type)

        # Get signatures
        device_sig = self.get_firmware_signature()
        firmware_sig = self.extract_firmware_signature(firmware_data)

        self.log(f"Firmware signature (first 16 bytes): {firmware_sig[:16].hex()}")

        # Compare signatures
        if device_sig == firmware_sig and not force:
            self.log("✅ Panda firmware is up to date!")
            return True

        if force:
            self.log("Force flash requested...")
        else:
            self.log("⚠️  Firmware mismatch detected!")

        # Reset to bootloader
        self.log("Entering bootloader mode...")
        self.reset(bootstub=True)

        # Close device
        usb.util.dispose_resources(self.dev)
        self.dev = None

        # Wait and reconnect
        time.sleep(3)

        if not self.find_panda(timeout=30):
            self.error("Failed to reconnect to Panda in bootloader mode")
            return False

        # Check if flasher is present
        if not self.flasher_present():
            self.error("Bootloader not detected!")
            return False

        # Flash firmware
        success = self.flash_firmware(firmware_data, mcu_config)

        if success:
            self.log("✅ Firmware flashed successfully!")
            self.log("Waiting for Panda to reboot...")
            time.sleep(3)

            # Cleanup
            if self.dev:
                usb.util.dispose_resources(self.dev)
                self.dev = None

            # Verify
            if self.find_panda(timeout=10):
                new_sig = self.get_firmware_signature()
                if new_sig == firmware_sig:
                    self.log("✅ Firmware verification passed!")
                    return True
                else:
                    self.error("❌ Firmware verification failed!")
                    return False

        return success


def main():
    parser = argparse.ArgumentParser(
        description="Panda Firmware Flasher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Flash firmware (auto-detect MCU type)
  python panda_flash.py --firmware /path/to/panda.bin.signed

  # Force flash even if signatures match
  python panda_flash.py --firmware panda.bin.signed --force

  # Check firmware version only
  python panda_flash.py --check-only

  # Specify MCU type explicitly
  python panda_flash.py --firmware panda_h7.bin.signed --mcu h7
        """,
    )

    parser.add_argument(
        "--firmware", "-f", type=Path, help="Path to firmware file (*.bin.signed)"
    )

    parser.add_argument(
        "--mcu",
        choices=["f4", "h7", "auto"],
        default="auto",
        help="MCU type (default: auto-detect)",
    )

    parser.add_argument(
        "--force", action="store_true", help="Force flash even if firmware matches"
    )

    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Only check firmware version, don't flash",
    )

    parser.add_argument(
        "--quiet", "-q", action="store_true", help="Suppress informational messages"
    )

    args = parser.parse_args()

    # Validation
    if not args.check_only and not args.firmware:
        parser.error("--firmware is required unless --check-only is specified")

    # Create flasher
    flasher = PandaFlasher(verbose=not args.quiet)

    if args.check_only:
        # Just check version
        if flasher.find_panda():
            print("=" * 60)
            print("🔍 Panda Firmware Information")
            print("=" * 60)

            # Get MCU type
            mcu_type = flasher.get_mcu_type()
            if mcu_type:
                mcu_names = {"F4": "STM32F4 (Black Panda)", "H7": "STM32H7 (Red Panda)"}
                print(f"\n📱 MCU Type: {mcu_names.get(mcu_type, mcu_type)}")

            # Get firmware signature
            sig = flasher.get_firmware_signature()
            if sig:
                print(f"\n🔐 Firmware Signature:")
                print(f"   Full (hex): {sig.hex()}")
                print(f"   First 16 bytes: {sig[:16].hex()}")
            else:
                print("\n⚠️  Failed to read firmware signature")

            # Get packet versions (NEW!)
            versions = flasher.get_packet_versions()
            if versions:
                health_ver, can_ver, can_health_ver = versions
                print(f"\n📦 Packet Versions:")
                print(f"   Health packet version:     {health_ver}")
                print(f"   CAN packet version:        {can_ver}")
                print(f"   CAN Health packet version: {can_health_ver}")

                # Check if health version matches expected
                EXPECTED_HEALTH_VERSION = 17
                if health_ver == EXPECTED_HEALTH_VERSION:
                    print(
                        f"\n✅ Health packet version {health_ver} matches expected (v{EXPECTED_HEALTH_VERSION})"
                    )
                    print("   ✓ Firmware is up to date!")
                else:
                    print(f"\n⚠️  Health packet version MISMATCH!")
                    print(f"   Current:  v{health_ver}")
                    print(f"   Expected: v{EXPECTED_HEALTH_VERSION}")
                    print(f"   ⚠️  You may need to update Panda firmware!")
            else:
                print("\n⚠️  Could not read packet versions (older firmware?)")

            print("\n" + "=" * 60)
            return 0
        else:
            print("Panda not found")
            return 1
    else:
        # Flash firmware
        success = flasher.flash_if_needed(args.firmware, force=args.force)
        return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
