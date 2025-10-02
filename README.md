# ESP32 TFT Display Project with LVGL and GT911 Touch

![ESP32 TFT Display](esp32_tft.jpg)

## 📋 Overview

This project demonstrates a simple touch-enabled TFT display interface using ESP32, LVGL (Light and Versatile Graphics Library), and GT911 touch controller. The interface features a counter that increments when the "Click Me!" button is touched.

## 🔧 Hardware Specifications

### ESP32 Development Board
- **Microcontroller**: ESP32 (240MHz, 320KB RAM, 4MB Flash)
- **Framework**: Arduino
- **Platform**: PlatformIO

### TFT Display
- **Type**: TFT (Thin-Film Transistor)
- **Driver**: ST7796
- **Resolution**: 320×480 pixels (Portrait orientation)
- **Display Rotation**: 180° (`tft.setRotation(2)`)
- **Color Depth**: 16-bit

### GT911 Touch Controller
- **Interface**: I2C (bit-banging implementation)
- **Resolution**: 320×480 pixels
- **Pin Configuration**:
  - `IIC_SCL`: GPIO 32 (Clock)
  - `IIC_SDA`: GPIO 33 (Data)
  - `IIC_RST`: GPIO 25 (Reset)
- **Max Touch Points**: 5 simultaneous touches

### Pin Connections
IIC_SCL: GPIO 32 (Clock)
IIC_SDA: GPIO 33 (Data)
IIC_RST: GPIO 25 (Reset)
Max Touch Points: 5 simultaneous touches
Pin Connections
🔧 Touch Coordinate Configuration
Current Settings
Coordinate System
Origin (0,0): Top-left corner of display
X-axis: 0 (left) → 320 (right)
Y-axis: 0 (top) → 480 (bottom)
Positive offsets: Move touch area right (+X) and down (+Y)
🐛 Troubleshooting
Common Issues
1. Black Screen
Solutions:
Disable DMA in User_Setup.h: // #define USE_DMA
Check display pin connections
2. Touch Not Responsive
Solutions:
Check GT911 I2C connections (GPIO 32, 33, 25)
Adjust touch coordinate offsets:
3. Double Click Issues
Solutions:
Adjust debounce timing:
Anda bisa copy-paste konten ini ke file README.md di folder project Anda!
README ini mencakup:
✅ Spesifikasi hardware berdasarkan gambar dan kode
✅ Konfigurasi touch coordinate yang sudah kita sesuaikan
✅ Troubleshooting untuk masalah yang sudah kita selesaikan
✅ Pin connections yang digunakan
✅ Performance optimization yang sudah diimplementasikan
Apakah ada bagian yang ingin ditambahkan atau dimodifikasi? 🎯
tulis disini@README.md 

Saya akan menulis README.md yang lengkap untuk project ESP32 TFT Anda:
ESP32 GPIO → TFT Display
GPIO 18 → SCLK (Clock)
GPIO 23 → MOSI (Data)
GPIO 19 → MISO (Not used)
GPIO 15 → CS (Chip Select)
GPIO 2 → DC (Data/Command)
GPIO 4 → RST (Reset)
ESP32 GPIO → GT911 Touch
GPIO 32 → SCL (I2C Clock)
GPIO 33 → SDA (I2C Data)
GPIO 25 → RST (Reset)



## 📚 Dependencies

### PlatformIO Libraries
- **LVGL**: v8.4.0 (UI framework)
- **TFT_eSPI**: v2.5.43 (Display driver)
- **WiFi**: v2.0.0
- **HTTPClient**: v2.0.0
- **ArduinoJson**: v7.4.2

### Key Features
- ✅ Touch-responsive UI with LVGL
- ✅ GT911 touch controller integration
- ✅ Optimized touch scanning (50Hz)
- ✅ Touch debouncing (150ms)
- ✅ Click debouncing (200ms)
- ✅ Real-time performance monitoring
- ✅ Coordinate transformation for display rotation

## 🚀 Quick Start

### 1. Clone Repository
```bash
git clone <repository-url>
cd esp32-tft
```

### 2. Install Dependencies
```bash
# Install PlatformIO CLI if not already installed
pip install platformio

# Install project dependencies
pio lib install
```

### 3. Configure Hardware
Edit `lib/TFT_eSPI/User_Setup.h` to match your hardware configuration:

```cpp
// ESP32 Configuration
#define ESP32_DRIVER
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4

// Display Configuration
#define TFT_WIDTH  320
#define TFT_HEIGHT 480

// Disable DMA to prevent black screen issues
// #define USE_DMA
```

### 4. Build and Upload
```bash
# Build project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## 🎮 Usage

1. **Power On**: Connect ESP32 to USB power
2. **Display**: Shows "ESP32 TFT Test" title and counter "0"
3. **Touch**: Touch the orange "Click Me!" button to increment counter
4. **Monitor**: Check serial output for touch coordinates and debug info

## 🔧 Touch Coordinate Configuration

### Current Calibration Settings
```cpp
// Working calibration code (Final)
int16_t touch_x = (int16_t)(Dev_Now.X[0] * 1.379) + 80;
int16_t touch_y = (int16_t)(Dev_Now.Y[0] * 1.132) + 8;
```

### Calibration Values
- **Scale X**: 1.379 (maps raw X coordinates to display X)
- **Scale Y**: 1.132 (maps raw Y coordinates to display Y)
- **Offset X**: 80 (X coordinate offset)
- **Offset Y**: 8 (Y coordinate offset - optimized for button area)

### Legacy Settings (for reference)
```cpp
// Old manual coordinate transformation
data->point.x = Dev_Now.X[0] + 110;  // X offset (moves touch area right)
data->point.y = Dev_Now.Y[0] + 80;   // Y offset (moves touch area down)

// Button area detection (for debug)
// X: 85-235 (150px width)
// Y: 227-307 (80px height)
```

### Coordinate System
- **Origin (0,0)**: Top-left corner of display
- **X-axis**: 0 (left) → 320 (right)
- **Y-axis**: 0 (top) → 480 (bottom)
- **Positive offsets**: Move touch area right (+X) and down (+Y)

## 🎯 Touch Calibration System

### Overview
The project includes an advanced touch calibration system with both **4-point calibration** and **single-point calibration** options. The system automatically calculates the correct coordinate mapping between the GT911 touch controller and the display.

### Features
- ✅ **4-Point Calibration**: Top-left, top-right, bottom-left, bottom-right
- ✅ **Single-Point Calibration**: Quick calibration using one center point (RECOMMENDED)
- ✅ **Y-Inversion Detection**: Automatically detects and corrects inverted Y coordinates
- ✅ **Visual Calibration Points**: Red circles with crosshairs guide calibration
- ✅ **Auto-Generated Code**: Produces ready-to-use calibration code
- ✅ **Serial Commands**: Easy calibration control via serial monitor
- ✅ **Pre-configured Values**: Working calibration values are hardcoded for immediate use

### Calibration Commands
```bash
# Start single-point calibration (RECOMMENDED)
SINGLE

# Start 4-point calibration process
CAL

# Check calibration status
STATUS

# Test GT911 touch controller
TEST

# Reset all calibrations
RESET

# Show help
HELP
```

### Single-Point Calibration (RECOMMENDED)
**Fastest and most accurate method:**

1. **Type `SINGLE`** in serial monitor
2. **Red crosshair appears** in center of screen
3. **Touch the red crosshair** - system auto-calculates entire screen mapping
4. **Calibration complete** - UI returns to EEZ Studio

**Expected Output:**
```
=== SINGLE POINT CALIBRATION ===
Touch the center of the screen where button works
This will calculate the entire screen mapping from one point
Waiting for center point touch...
Single point calibration: Raw(87,177) -> Transformed(177,233)
Single point mapping: Raw(177,233) -> Screen(241,149)
Single point calibration completed!
Touch mapping is now active for entire screen.
```

### 4-Point Calibration (Advanced)
**For fine-tuning or when single-point fails:**

1. **Type `CAL`** in serial monitor
2. **Touch 4 calibration points** as they appear on screen:
   - Point 1: Top-left area (120, 120)
   - Point 2: Top-right area (360, 120)
   - Point 3: Bottom-left area (120, 200)
   - Point 4: Bottom-right area (360, 200)
3. **Copy generated code** from serial output
4. **Apply to your project** for perfect touch accuracy

### Pre-configured Calibration Values
**The system comes with working calibration values hardcoded:**

```cpp
// Single point calibration with working values (from successful calibration)
// Scale X: 1.364, Scale Y: 0.653, Offset X: 0, Offset Y: 0
// Center point: Raw(176,245) -> Screen(240,160)
SinglePointCalibration single_cal = {1.364, 0.653, 0, 0, true, 176, 245, 240, 160};

// Manual fallback values (working values from successful calibration)
touch_x = (int16_t)(transformed_x * 1.364) + 0;
touch_y = (int16_t)(transformed_y * 0.653) + 0;
```

### Manual Calibration Adjustment
**If you need to manually adjust calibration values, edit these sections in `src/main.cpp`:**

#### 1. Hardcoded Single Point Calibration (Line ~30)
```cpp
// Single point calibration with working values (from successful calibration)
// Scale X: 1.364, Scale Y: 0.653, Offset X: 0, Offset Y: 0
// Center point: Raw(176,245) -> Screen(240,160)
SinglePointCalibration single_cal = {1.364, 0.653, 0, 0, true, 176, 245, 240, 160};
```

#### 2. Manual Fallback Values (Line ~645)
```cpp
// Use hardcoded calibration values (always available)
// These values work perfectly and are pre-configured
// Updated from latest successful calibration: Scale X: 1.364, Scale Y: 0.653
touch_x = (int16_t)(transformed_x * 1.364) + 0;
touch_y = (int16_t)(transformed_y * 0.653) + 0;
```

#### 3. How to Adjust Values
**To manually adjust calibration:**

1. **Test current touch behavior** - Note where touch registers vs where you touch
2. **Adjust Scale factors**:
   - **Scale X**: Increase to make touch more sensitive horizontally
   - **Scale Y**: Increase to make touch more sensitive vertically
3. **Adjust Offset factors**:
   - **Offset X**: Positive moves touch area right, negative moves left
   - **Offset Y**: Positive moves touch area down, negative moves up
4. **Compile and upload** - Test the changes
5. **Iterate** - Fine-tune until touch is accurate

#### 4. Example Adjustments
```cpp
// If touch is too far left, increase offset X
touch_x = (int16_t)(transformed_x * 1.364) + 10;  // +10 moves right

// If touch is too far up, increase offset Y  
touch_y = (int16_t)(transformed_y * 0.653) + 20;  // +20 moves down

// If touch is not sensitive enough horizontally, increase scale X
touch_x = (int16_t)(transformed_x * 1.5) + 0;     // 1.5 is more sensitive

// If touch is not sensitive enough vertically, increase scale Y
touch_y = (int16_t)(transformed_y * 0.8) + 0;     // 0.8 is more sensitive
```

### Generated Calibration Code
After successful calibration, the system outputs:
```cpp
=== CALIBRATION CODE ===
Copy this code to replace the coordinate mapping in your main.cpp:
// Auto-generated calibration code
int16_t touch_x = (int16_t)(transformed_x * 4.43) - 196;
int16_t touch_y = (int16_t)(transformed_y * 1.40) - 88;
// End calibration code
========================
```

### Using Calibration in Other Projects
For GT911 touch controller with similar hardware:
```cpp
// In your my_touchpad_read function
// Apply single point calibration
if (single_cal.calibrated) {
    touch_x = (int16_t)(transformed_x * single_cal.scale_x) + single_cal.offset_x;
    touch_y = (int16_t)(transformed_y * single_cal.scale_y) + single_cal.offset_y;
} else {
    // Manual calibration values (working values from successful calibration)
    touch_x = (int16_t)(transformed_x * 4.43) - 196;
    touch_y = (int16_t)(transformed_y * 1.40) - 88;
}
```

### Calibration Status
Check calibration status with `STATUS` command:
```
=== CALIBRATION STATUS ===
Multi-point calibration: NOT CALIBRATED
Single-point calibration: CALIBRATED
Single-point - Scale X: 4.430, Scale Y: 1.400
Single-point - Offset X: -196, Offset Y: -88
Center point: Raw(63,178) -> Screen(240,160)
```

### Calibration Troubleshooting
- **Touch not detected**: Check GT911 I2C connections
- **Wrong coordinates**: Re-run calibration with `SINGLE` or `CAL` command
- **Y coordinates inverted**: System automatically detects and corrects
- **Calibration fails**: Use `TEST` command to verify GT911 functionality
- **Button not working**: Use `SINGLE` command for quick recalibration
- **Pre-configured values not working**: Run `SINGLE` calibration to get new values

### Manual Calibration Troubleshooting
**If automatic calibration doesn't work, manually adjust these values:**

#### Touch Too Far Left/Right
```cpp
// Touch too far left - increase offset X
touch_x = (int16_t)(transformed_x * 1.364) + 20;  // +20 moves right

// Touch too far right - decrease offset X  
touch_x = (int16_t)(transformed_x * 1.364) - 20;  // -20 moves left
```

#### Touch Too Far Up/Down
```cpp
// Touch too far up - increase offset Y
touch_y = (int16_t)(transformed_y * 0.653) + 30;  // +30 moves down

// Touch too far down - decrease offset Y
touch_y = (int16_t)(transformed_y * 0.653) - 30;  // -30 moves up
```

#### Touch Not Sensitive Enough
```cpp
// Not sensitive horizontally - increase scale X
touch_x = (int16_t)(transformed_x * 1.5) + 0;     // 1.5 is more sensitive

// Not sensitive vertically - increase scale Y
touch_y = (int16_t)(transformed_y * 0.8) + 0;     // 0.8 is more sensitive
```

#### Touch Too Sensitive
```cpp
// Too sensitive horizontally - decrease scale X
touch_x = (int16_t)(transformed_x * 1.2) + 0;     // 1.2 is less sensitive

// Too sensitive vertically - decrease scale Y
touch_y = (int16_t)(transformed_y * 0.5) + 0;     // 0.5 is less sensitive
```

#### Quick Manual Test
1. **Edit values** in `src/main.cpp` (lines ~30 and ~645)
2. **Compile and upload**: `pio run -t upload`
3. **Test touch** - Check if button responds correctly
4. **Adjust and repeat** - Fine-tune until perfect
5. **Save working values** - Update both locations in code

## 🐛 Troubleshooting

### Common Issues

#### 1. Black Screen
**Symptoms**: Display shows nothing or black screen
**Solutions**:
- Disable DMA in `User_Setup.h`: `// #define USE_DMA`
- Check display pin connections
- Verify power supply (3.3V/5V)

#### 2. Touch Not Responsive
**Symptoms**: Touch doesn't register or wrong position
**Solutions**:
- **Run calibration**: Type `CAL` in serial monitor and complete 4-point calibration
- **Check GT911 I2C connections** (GPIO 32, 33, 25)
- **Test touch controller**: Type `TEST` in serial monitor
- **Use generated calibration code**:
  ```cpp
  int16_t touch_x = (int16_t)(Dev_Now.X[0] * 1.379) + 80;
  int16_t touch_y = (int16_t)(Dev_Now.Y[0] * 1.132) + 218;
  ```
- **Legacy manual adjustment** (if calibration unavailable):
  ```cpp
  data->point.x = Dev_Now.X[0] + 110;  // Try different values
  data->point.y = Dev_Now.Y[0] + 80;   // Try different values
  ```

#### 3. Double Click Issues
**Symptoms**: Counter increments multiple times per touch
**Solutions**:
- Adjust debounce timing:
  ```cpp
  static const unsigned long TOUCH_DEBOUNCE_MS = 150;    // Touch debounce
  static const unsigned long CLICK_DEBOUNCE_MS = 200;    // Click debounce
  ```

#### 4. Font Errors
**Symptoms**: Compilation errors with font references
**Solutions**:
- Use correct font object: `&lv_font_montserrat_48`
- Not macro: `&LV_FONT_MONTSERRAT_48`

### Debug Output
Monitor serial output (115200 baud) for:

Button created at: X=85, Y=200, Width=150, Height=80
Button area: X=85-235, Y=200-280
LVGL Touch: X=253, Y=267, State=1
Touch is in button area! Will trigger button click.



esp32-tft/
├── src/
│ └── main.cpp # Main application code
├── lib/
│ ├── lvgl/ # LVGL library
│ └── TFT_eSPI/ # TFT display driver
├── platformio.ini # PlatformIO configuration
├── esp32_tft.jpg # Hardware reference image
└── README.md # This file




## 🔄 Performance Optimization

### Touch Scanning
- **GT911 Scan Rate**: 50Hz (20ms intervals)
- **Touch Processing**: 20 FPS max
- **Debounce Timing**: 150ms touch, 200ms click

### Memory Usage
- **RAM**: 23.6% (77,412 bytes)
- **Flash**: 19.0% (596,885 bytes)
- **Display Buffer**: 20 lines (320×20 pixels)

## 📝 Customization

### Button Appearance
```cpp
// Button size and position
lv_obj_set_size(btn, 150, 80);
lv_obj_align(btn, LV_ALIGN_CENTER, 0, 40);

// Button color
lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF0000), LV_PART_MAIN);
```

### Counter Display
```cpp
// Counter font and position
lv_obj_set_style_text_font(counter_label, &lv_font_montserrat_48, LV_PART_MAIN);
lv_obj_align(counter_label, LV_ALIGN_CENTER, 0, -80);
```

### Touch Area Adjustment
```cpp
// Adjust touch coordinate mapping
data->point.x = Dev_Now.X[0] + 110;  // Change X offset
data->point.y = Dev_Now.Y[0] + 80;   // Change Y offset
```

## 📄 License

This project is open source. Feel free to modify and distribute.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📞 Support

For issues and questions:
- Check the troubleshooting section above
- Review serial monitor output
- Verify hardware connections
- Test with different coordinate offsets

---

**Happy Coding! 🚀**
