#pragma once

// Waveshare ESP32-S3 Touch AMOLED 1.8 pin configuration
// Used by Water Watch Pro 1.8 Host/Remote builds.

#define XPOWERS_CHIP_AXP2101

#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_SCLK 11
#define LCD_CS 12
#define LCD_WIDTH 368
#define LCD_HEIGHT 448

#define IIC_SDA 15
#define IIC_SCL 14
#define TP_INT 21

#define I2S_MCK_IO 16
#define I2S_BCK_IO 9
#define I2S_DI_IO 10
#define I2S_WS_IO 45
#define I2S_DO_IO 8

#define PA 46

// FT3168/FT3x68 touch controller address used by Waveshare examples.
#ifndef FT3168_DEVICE_ADDRESS
#define FT3168_DEVICE_ADDRESS 0x38
#endif
