# Linux for Motorola Moto G10

Mainline-based Linux kernel for the **Motorola Moto G10** (`capri`, Qualcomm
Snapdragon 460 / SM4250). The `capri-7.2` branch is based on Linux **7.2.9** and
includes the device tree and drivers used to run Nura/postmarketOS with Phosh.

Supported hardware has been tested on the **XT2127-2 with a DJN NT36525B panel**.
Other Moto G10 variants and display panels have not been tested.

## Hardware support

| Component | Status |
| --- | --- |
| Display and backlight | Working, 720 × 1600 DJN NT36525B panel |
| Touchscreen | Working; requires the matching Novatek firmware |
| GPU | Working with Mesa Freedreno, Adreno 610 |
| Wi-Fi | Working |
| Bluetooth | Discovery and power cycling work; pairing and audio/HID profiles untested |
| USB | Device networking, host mode and USB game controllers work |
| Speaker | Working; gain limited to −18 dB |
| Buttons and eMMC | Working |
| Sensors | Automatic rotation works; light, proximity and gyro data available through SSC |
| Battery | Voltage and estimated charge level available; charging control not implemented |
| RTC | Counter available; calendar time requires userspace calibration |
| Suspend | Short RTC wake tests pass; long suspend and power-button wake untested |
| Cellular, cameras and microphone | Not brought up |

The display briefly blanks during rotation. Panel bias power remains managed
by the firmware. Battery percentage is estimated from voltage and has not been
calibrated over a full charge cycle.

## Building

Use a Linux host with the [kernel build dependencies](Documentation/process/changes.rst)
and Python 3. Cross-compiling also requires an AArch64 GCC/binutils toolchain.

```sh
git clone --branch capri-7.2 --single-branch \
    https://github.com/michaelmcallister/linux-motorola-capri.git
cd linux-motorola-capri

export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-

make O=out motorola_capri_defconfig
make O=out -j"$(nproc)" qcom/sm4250-motorola-capri.dtb
python3 scripts/capri/embed-dtb.py \
    out/arch/arm64/boot/dts/qcom/sm4250-motorola-capri.dtb \
    out/include/generated/capri-dtb.h
make O=out -j"$(nproc)" Image modules
```

When building natively on an AArch64 Linux host, leave `CROSS_COMPILE` unset.
Regenerate the embedded header after changing the device tree.

Install modules into the target root filesystem:

```sh
make O=out INSTALL_MOD_PATH=/path/to/rootfs modules_install
```

The device tree is at
[`arch/arm64/boot/dts/qcom/sm4250-motorola-capri.dts`](arch/arm64/boot/dts/qcom/sm4250-motorola-capri.dts).
The supplied configuration enables the board drivers and stock-bootloader
compatibility code.

## Boot images

The Moto G10 uses an Android header-v3 boot image and obtains its firmware
device tree from `vendor_boot`. This kernel embeds the Linux device tree and
preserves the bootloader's RAM map, initramfs location and factory Wi-Fi address.

Wrap the kernel for the stock bootloader's load alignment, then package it with
a Nura initramfs configured for this device:

```sh
python3 scripts/capri/wrap-image.py out/arch/arm64/boot/Image out/Image.capri
mkbootimg --header_version 3 \
    --kernel out/Image.capri \
    --ramdisk /path/to/initramfs \
    --output out/boot.img
```

An unlocked bootloader and a prepared Linux root filesystem are required.
Keep the stock `vendor_boot` and `dtbo` partitions. Kernel modules must match
the kernel being booted. Root filesystem installation and distribution image
packaging are outside this repository.

## Firmware and userspace

The touchscreen, GPU, DSPs, Wi-Fi, Bluetooth and speaker amplifier require
firmware. Device-specific firmware must match the
phone's stock software; firmware binaries are not included here.

The main firmware paths are:

| Hardware | Firmware path under `/lib/firmware` |
| --- | --- |
| Touchscreen | `novatek/capri-djn-nt36525b.bin` |
| GPU | `qcom/motorola/capri/a610_zap.mdt` and its companion segments |
| Modem / ADSP | `modem.mdt`, `adsp2.mdt` and their companion segments |
| Bluetooth | `qca/cmbtfw12.tlv`, `qca/cmnv12.bin` |
| Speaker amplifier | `aw882xx_spk_reg.bin` |
| Wi-Fi | `ath10k/WCN3990/hw1.0/` firmware and board calibration data |

Wi-Fi and audio require the Qualcomm firmware services and modem/ADSP startup
ordering. Speaker playback requires the board's ALSA UCM profile and feedback
stream. Sensors require the SSC/FastRPC userspace service and sensor-proxy
integration. These components belong in the distribution's device packages.

## Reporting issues

Open an [issue](https://github.com/michaelmcallister/linux-motorola-capri/issues)
with the device model, panel variant if known, kernel commit, distribution,
steps to reproduce and relevant kernel messages.

## License

See [COPYING](COPYING) and [LICENSES](LICENSES) for the Linux kernel's licensing
terms. Individual files carry their own SPDX identifiers.
