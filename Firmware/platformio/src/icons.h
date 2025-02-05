#include "config.h"

#define DISPLAY_ICON_FAN_00_WIDTH 13
#define DISPLAY_ICON_FAN_00_HEIGHT 13
static const unsigned char DISPLAY_ICON_FAN_00_BITS[]  = {
   0xfc, 0x07, 0x7a, 0x08, 0x71, 0x10, 0x61, 0x18, 0x61, 0x1c, 0x41, 0x1f,
   0xff, 0x1f, 0x5f, 0x10, 0xc7, 0x10, 0xc3, 0x10, 0xc1, 0x11, 0xc2, 0x0b,
   0xfc, 0x07 };
#define DISPLAY_ICON_FAN_01_WIDTH 13
#define DISPLAY_ICON_FAN_01_HEIGHT 13
static const unsigned char DISPLAY_ICON_FAN_01_BITS[]  = {
   0xfc, 0x07, 0xc2, 0x0f, 0x01, 0x13, 0x03, 0x13, 0x0f, 0x11, 0xbf, 0x10,
   0x43, 0x18, 0xa3, 0x1c, 0x11, 0x1f, 0x31, 0x1c, 0x39, 0x18, 0x7e, 0x08,
   0xfc, 0x07 };
#define DISPLAY_ICON_FAN_02_WIDTH 13
#define DISPLAY_ICON_FAN_02_HEIGHT 13
static const unsigned char DISPLAY_ICON_FAN_02_BITS[]  = {
   0xfc, 0x07, 0x1a, 0x0e, 0x1f, 0x16, 0x1f, 0x1f, 0x39, 0x1f, 0xa1, 0x11,
   0x41, 0x10, 0xb1, 0x10, 0x9f, 0x13, 0x1f, 0x1f, 0x0d, 0x1f, 0x0e, 0x0b,
   0xfc, 0x07 };
#define DISPLAY_ICON_HEATER_ON_WIDTH 16
#define DISPLAY_ICON_HEATER_ON_HEIGHT 16
static const unsigned char DISPLAY_ICON_HEATER_ON_BITS[]  = {
   0x80, 0x01, 0x80, 0x01, 0x06, 0x30, 0x06, 0x30, 0xc0, 0x03, 0xe0, 0x07,
   0xf0, 0x0f, 0xf3, 0xcf, 0xf3, 0xcf, 0xf0, 0x0f, 0xe0, 0x07, 0xc0, 0x03,
   0x0c, 0x30, 0x0c, 0x30, 0x80, 0x01, 0x80, 0x01 };
#define DISPLAY_ICON_HEATER_SENSOR_NOK_WIDTH 7
#define DISPLAY_ICON_HEATER_SENSOR_NOK_HEIGHT 10
static const unsigned char DISPLAY_ICON_HEATER_SENSOR_NOK_BITS[]  = {
   0x00, 0x00, 0x22, 0x14, 0x08, 0x14, 0x22, 0x00, 0x00, 0x3e };
#define DISPLAY_ICON_HEATER_SENSOR_OK_WIDTH 7
#define DISPLAY_ICON_HEATER_SENSOR_OK_HEIGHT 10
static const unsigned char DISPLAY_ICON_HEATER_SENSOR_OK_BITS[]  = {
   0x04, 0x0c, 0x0e, 0x2e, 0x67, 0x77, 0x73, 0x7a, 0x00, 0x3e };
#define DISPLAY_ICON_HEATER_TEMP_MAX_WIDTH 7
#define DISPLAY_ICON_HEATER_TEMP_MAX_HEIGHT 8
static const unsigned char DISPLAY_ICON_HEATER_TEMP_MAX_BITS[]  = {
   0x7f, 0x00, 0x08, 0x1c, 0x08, 0x08, 0x08, 0x08 };
#define DISPLAY_ICON_HUMIDTEMP_SENSOR_NOK_WIDTH 7
#define DISPLAY_ICON_HUMIDTEMP_SENSOR_NOK_HEIGHT 7
static const unsigned char DISPLAY_ICON_HUMIDTEMP_SENSOR_NOK_BITS[]  = {
   0x7c, 0x51, 0x4a, 0x44, 0x4a, 0x51, 0x7c };
#define DISPLAY_ICON_INSIDE_SENSOR_OK_WIDTH 7
#define DISPLAY_ICON_INSIDE_SENSOR_OK_HEIGHT 7
static const unsigned char DISPLAY_ICON_INSIDE_SENSOR_OK_BITS[]  = {
   0x7c, 0x40, 0x48, 0x5f, 0x48, 0x40, 0x7c };
#define DISPLAY_ICON_INSIDE_TEMP_TARGET_WIDTH 8
#define DISPLAY_ICON_INSIDE_TEMP_TARGET_HEIGHT 8
static const unsigned char DISPLAY_ICON_INSIDE_TEMP_TARGET_BITS[]  = {
   0x3c, 0x42, 0x99, 0xa5, 0xa5, 0x99, 0x42, 0x3c };
#define DISPLAY_ICON_OUTSIDE_SENSOR_OK_WIDTH 7
#define DISPLAY_ICON_OUTSIDE_SENSOR_OK_HEIGHT 7
static const unsigned char DISPLAY_ICON_OUTSIDE_SENSOR_OK_BITS[]  = {
   0x7c, 0x40, 0x42, 0x5f, 0x42, 0x40, 0x7c };
#define DISPLAY_ICON_SPIFFS_NOK_WIDTH 14
#define DISPLAY_ICON_SPIFFS_NOK_HEIGHT 9
static const unsigned char DISPLAY_ICON_SPIFFS_NOK_BITS[]  = {
   0x48, 0x06, 0x58, 0x09, 0x68, 0x09, 0x48, 0x06, 0x00, 0x00, 0xa7, 0x2b,
   0xa9, 0x19, 0x29, 0x1a, 0xa7, 0x2b };
#define DISPLAY_ICON_WIFI_NOK_WIDTH 12
#define DISPLAY_ICON_WIFI_NOK_HEIGHT 12
static const unsigned char DISPLAY_ICON_WIFI_NOK_BITS[]  = {
   0x00, 0x00, 0x82, 0x00, 0x44, 0x00, 0x28, 0x00, 0x10, 0x00, 0x28, 0x00,
   0x44, 0x00, 0x82, 0x00, 0x00, 0x06, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x06 };
#define DISPLAY_ICON_WIFI_OK_WIDTH 12
#define DISPLAY_ICON_WIFI_OK_HEIGHT 12
static const unsigned char DISPLAY_ICON_WIFI_OK_BITS[]  = {
   0xc0, 0x07, 0xe0, 0x03, 0x78, 0x00, 0x1c, 0x00, 0x0e, 0x07, 0xc6, 0x03,
   0xe7, 0x00, 0x63, 0x00, 0x33, 0x06, 0x33, 0x0f, 0x11, 0x0f, 0x00, 0x06 };
