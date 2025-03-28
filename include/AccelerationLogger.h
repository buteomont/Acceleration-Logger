#define BRAKE_LED_PORT 1
#define BRAKE_ON HIGH
#define BRAKE_OFF LOW

int const INTERRUPT_PIN = 3;  // The interruption #0 pin on the MPU6050

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define SDA_PIN 0
#define SCL_PIN 2

#define SSID_SIZE 20
#define PASSWORD_SIZE 20
