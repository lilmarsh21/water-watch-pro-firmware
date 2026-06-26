# Water Watch Pro Host GitHub Builder

This repo builds the Water Watch Pro 1.8 Host firmware into a downloadable `.bin` file using GitHub Actions.

## What to upload to Water Watch Pro OTA

Use this file:

`WaterWatchPro_HOST_firmware.bin`

Do **not** upload `partitions.bin` through the Water Watch Pro update page.

## Phone workflow

1. Open this GitHub repo.
2. Go to **Actions**.
3. Open **Build Water Watch Pro Host BIN**.
4. Tap **Run workflow**.
5. Wait for the build to finish.
6. Open the finished workflow run.
7. Download the artifact named **WaterWatchPro-Host-Firmware**.
8. Extract/download `WaterWatchPro_HOST_firmware.bin` to your phone.
9. On the Water Watch Pro host, swipe to **UPDATE MODE**.
10. Tap **START UPDATE**.
11. Connect Android WiFi to `WaterWatchPro-Host`, password `12345678`.
12. Open `192.168.4.1`.
13. Upload `WaterWatchPro_HOST_firmware.bin`.

## Important

This package is for the HOST display only.
Do not upload this firmware to the remote display.
