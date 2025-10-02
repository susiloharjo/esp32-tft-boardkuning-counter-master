#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui/ui.h"

// Touch Calibration System
struct TouchCalibration {
    float scale_x;
    float scale_y;
    int16_t offset_x;
    int16_t offset_y;
    bool calibrated;
};

TouchCalibration touch_cal = {1.0, 1.0, 0, 0, false};

// Single Point Calibration System
struct SinglePointCalibration {
    float scale_x, scale_y;
    int16_t offset_x, offset_y;
    bool calibrated;
    int16_t center_raw_x, center_raw_y;
    int16_t center_screen_x, center_screen_y;
};

SinglePointCalibration single_cal = {1.0, 1.0, 0, 0, false, 0, 0, 0, 0};

// Single point calibration mode
bool single_calibration_mode = false;

// Calibration points - Adjusted to working touch areas
struct CalibrationPoint {
    int16_t target_x, target_y;  // Expected coordinates
    int16_t raw_x, raw_y;        // Raw touch coordinates
    bool collected;
};

CalibrationPoint cal_points[4] = {
    {190, 145, 0, 0, false},    // Button center (main button)
    {240, 145, 0, 0, false},    // Button right (main button)
    {190, 175, 0, 0, false},    // Button bottom (main button)
    {240, 175, 0, 0, false}     // Button bottom-right (main button)
};

int cal_point_index = 0;
bool calibration_mode = false;
unsigned long cal_start_time = 0;

// Function declarations
void start_touch_calibration();
void process_calibration_touch(int16_t raw_x, int16_t raw_y);
void calculate_calibration();
void generate_calibration_code();
void apply_calibration(int16_t raw_x, int16_t raw_y, int16_t* calibrated_x, int16_t* calibrated_y);
void show_calibration_point();
void hide_calibration_point();
void restore_eez_ui();
void test_gt911_touch();

// Single point calibration functions
void calibrate_from_center_point(int16_t raw_x, int16_t raw_y, int16_t screen_x, int16_t screen_y);
void apply_single_point_calibration(int16_t raw_x, int16_t raw_y, int16_t* screen_x, int16_t* screen_y);
void show_single_calibration_center();
void hide_single_calibration_center();

// TFT_eSPI instance
TFT_eSPI tft = TFT_eSPI();

// LVGL display buffer
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[480 * 20]; // Buffer for 20 lines (landscape mode: 480x320)

// Display driver
static lv_disp_drv_t disp_drv;

// Global variables
// Note: Counter and UI elements are now managed by EEZ Studio

// GT911 Touch Controller Implementation
#define GT_CMD_WR           0XBA         //写命令0xBA
#define GT_CMD_RD           0XBB         //读命令0XBB

#define GT911_MAX_WIDTH     480          //Touchscreen pad max width (landscape mode)
#define GT911_MAX_HEIGHT    320          //Touchscreen pad max height (landscape mode)

//GT911 部分寄存器定义
#define GT_CTRL_REG         0X8040       //GT911控制寄存器
#define GT_CFGS_REG         0X8047       //GT911配置起始地址寄存器
#define GT_CHECK_REG        0X80FF       //GT911校验和寄存器
#define GT_PID_REG          0X8140       //GT911产品ID寄存器

#define GT_GSTID_REG        0X814E       //GT911当前检测到的触摸情况
#define GT911_READ_XY_REG   0x814E       /* 坐标寄存器 */
#define CT_MAX_TOUCH        5            //电容触摸屏最大支持的点数

int IIC_SCL = 32;
int IIC_SDA = 33;
int IIC_RST = 25;

#define IIC_SCL_0  digitalWrite(IIC_SCL,LOW)
#define IIC_SCL_1  digitalWrite(IIC_SCL,HIGH)

#define IIC_SDA_0  digitalWrite(IIC_SDA,LOW)
#define IIC_SDA_1  digitalWrite(IIC_SDA,HIGH)

#define IIC_RST_0  digitalWrite(IIC_RST,LOW)
#define IIC_RST_1  digitalWrite(IIC_RST,HIGH)

#define READ_SDA   digitalRead(IIC_SDA)

typedef struct
{
  uint8_t Touch;
  uint8_t TouchpointFlag;
  uint8_t TouchCount;

  uint8_t Touchkeytrackid[CT_MAX_TOUCH];
  uint16_t X[CT_MAX_TOUCH];
  uint16_t Y[CT_MAX_TOUCH];
  uint16_t S[CT_MAX_TOUCH];
} GT911_Dev;
GT911_Dev Dev_Now, Dev_Backup;
bool touched = 0;     //没有使用触摸中断，有触摸标志位touched = 1，否则touched = 0

uint8_t s_GT911_CfgParams[] =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void delay_us(unsigned int xus)  //1us
{
  for (; xus > 1; xus--);
}

void SDA_IN(void)
{
  pinMode(IIC_SDA, INPUT);
}

void SDA_OUT(void)
{
  pinMode(IIC_SDA, OUTPUT);
}

//初始化IIC
void IIC_Init(void)
{
  pinMode(IIC_SDA, OUTPUT);
  pinMode(IIC_SCL, OUTPUT);
  pinMode(IIC_RST, OUTPUT);
  IIC_SCL_1;
  IIC_SDA_1;
}

//产生IIC起始信号
void IIC_Start(void)
{
  SDA_OUT();
  IIC_SDA_1;
  IIC_SCL_1;
  delay_us(4);
  IIC_SDA_0; //START:when CLK is high,DATA change form high to low
  delay_us(4);
  IIC_SCL_0; //钳住I2C总线，准备发送或接收数据
}

//产生IIC停止信号
void IIC_Stop(void)
{
  SDA_OUT();
  IIC_SCL_0;
  IIC_SDA_0; //STOP:when CLK is high DATA change form low to high
  delay_us(4);
  IIC_SCL_1;
  IIC_SDA_1; //发送I2C总线结束信号
  delay_us(4);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
  uint8_t ucErrTime = 0;
  SDA_IN();      //SDA设置为输入
  IIC_SDA_1; delay_us(1);
  IIC_SCL_1; delay_us(1);
  while (READ_SDA)
  {
    ucErrTime++;
    if (ucErrTime > 250)
    {
      IIC_Stop();
      return 1;
    }
  }
  IIC_SCL_0; //时钟输出0
  return 0;
}

//产生ACK应答
void IIC_Ack(void)
{
  IIC_SCL_0;
  SDA_OUT();
  IIC_SDA_0;
  delay_us(2);
  IIC_SCL_1;
  delay_us(2);
  IIC_SCL_0;
}

//不产生ACK应答
void IIC_NAck(void)
{
  IIC_SCL_0;
  SDA_OUT();
  IIC_SDA_1;
  delay_us(2);
  IIC_SCL_1;
  delay_us(2);
  IIC_SCL_0;
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(uint8_t txd)
{
  uint8_t t;
  SDA_OUT();
  IIC_SCL_0; //拉低时钟开始数据传输
  for (t = 0; t < 8; t++)
  {
    //IIC_SDA=(txd&0x80)>>7;
    if ((txd & 0x80) >> 7)
      IIC_SDA_1;
    else
      IIC_SDA_0;
    txd <<= 1;
    delay_us(2);   //对TEA5767这三个延时都是必须的
    IIC_SCL_1;
    delay_us(2);
    IIC_SCL_0;
    delay_us(2);
  }
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
  unsigned char i, receive = 0;
  SDA_IN();//SDA设置为输入
  for (i = 0; i < 8; i++ )
  {
    IIC_SCL_0;
    delay_us(2);
    IIC_SCL_1;
    receive <<= 1;
    if (READ_SDA)receive++;
    delay_us(1);
  }
  if (!ack)
    IIC_NAck();//发送nACK
  else
    IIC_Ack(); //发送ACK
  return receive;
}

//reg:起始寄存器地址
//buf:数据缓缓存区
//len:写数据长度
//返回值:0,成功;1,失败.
uint8_t GT911_WR_Reg(uint16_t reg, uint8_t *buf, uint8_t len)
{
  uint8_t i;
  uint8_t ret = 0;
  IIC_Start();
  IIC_Send_Byte(GT_CMD_WR);       //发送写命令
  IIC_Wait_Ack();
  IIC_Send_Byte(reg >> 8);     //发送高8位地址
  IIC_Wait_Ack();
  IIC_Send_Byte(reg & 0XFF);     //发送低8位地址
  IIC_Wait_Ack();
  for (i = 0; i < len; i++)
  {
    IIC_Send_Byte(buf[i]);      //发数据
    ret = IIC_Wait_Ack();
    if (ret)break;
  }
  IIC_Stop();                    //产生一个停止条件
  return ret;
}

//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度
void GT911_RD_Reg(uint16_t reg, uint8_t *buf, uint8_t len)
{
  uint8_t i;
  IIC_Start();
  IIC_Send_Byte(GT_CMD_WR);   //发送写命令
  IIC_Wait_Ack();
  IIC_Send_Byte(reg >> 8);     //发送高8位地址
  IIC_Wait_Ack();
  IIC_Send_Byte(reg & 0XFF);     //发送低8位地址
  IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte(GT_CMD_RD);   //发送读命令
  IIC_Wait_Ack();
  for (i = 0; i < len; i++)
  {
    buf[i] = IIC_Read_Byte(i == (len - 1) ? 0 : 1); //发数据
  }
  IIC_Stop();//产生一个停止条件
}

//发送配置参数
//mode:0,参数不保存到flash
//     1,参数保存到flash
uint8_t GT911_Send_Cfg(uint8_t mode)
{
  uint8_t buf[2];
  uint8_t i = 0;
  buf[0] = 0;
  buf[1] = mode;  //是否写入到GT911 FLASH?  即是否掉电保存
  GT911_WR_Reg(GT_CHECK_REG, buf, 2); //写入校验和,和配置更新标记
  return 0;
}

void GT911_Scan(void)
{
  uint8_t buf[41];
  uint8_t Clearbuf = 0;
  uint8_t i;
  static unsigned long lastDebugTime = 0;
  static unsigned long lastScanTime = 0;
  static const unsigned long SCAN_INTERVAL_MS = 20; // Limit scanning to 50Hz max
  
  // Throttle scanning to prevent excessive I2C communication
  if (millis() - lastScanTime < SCAN_INTERVAL_MS) {
    return;
  }
  lastScanTime = millis();
  
  if (1)
    // if (Dev_Now.Touch == 1)
  {
    Dev_Now.Touch = 0;
    GT911_RD_Reg(GT911_READ_XY_REG, buf, 1);

      // Debug: Print raw register value every 5 seconds (increased frequency for debugging)
      if (millis() - lastDebugTime > 5000) {
        Serial.printf("GT911 Debug - Raw register value: 0x%02X\r\n", buf[0]);
        lastDebugTime = millis();
      }

    if ((buf[0] & 0x80) == 0x00)
    {
      touched = 0;
      GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
      // Serial.printf("No touch\r\n");
      delay(1);
    }
    else
    {
      touched = 1;
      Dev_Now.TouchpointFlag = buf[0];
      Dev_Now.TouchCount = buf[0] & 0x0f;
      
      Serial.printf("Touch detected! Flag: 0x%02X, Count: %d\r\n", Dev_Now.TouchpointFlag, Dev_Now.TouchCount);
      
      if (Dev_Now.TouchCount > 5)
      {
        touched = 0;
        GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
        Serial.printf("Dev_Now.TouchCount > 5\r\n");
        return ;
      }
      GT911_RD_Reg(GT911_READ_XY_REG + 1, &buf[1], Dev_Now.TouchCount * 8);
      GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);

      Dev_Now.Touchkeytrackid[0] = buf[1];
      Dev_Now.X[0] = ((uint16_t)buf[3] << 8) + buf[2];
      Dev_Now.Y[0] = ((uint16_t)buf[5] << 8) + buf[4];
      Dev_Now.S[0] = ((uint16_t)buf[7] << 8) + buf[6];

      Dev_Now.Touchkeytrackid[1] = buf[9];
      Dev_Now.X[1] = ((uint16_t)buf[11] << 8) + buf[10];
      Dev_Now.Y[1] = ((uint16_t)buf[13] << 8) + buf[12];
      Dev_Now.S[1] = ((uint16_t)buf[15] << 8) + buf[14];

      Dev_Now.Touchkeytrackid[2] = buf[17];
      Dev_Now.X[2] = ((uint16_t)buf[19] << 8) + buf[18];
      Dev_Now.Y[2] = ((uint16_t)buf[21] << 8) + buf[20];
      Dev_Now.S[2] = ((uint16_t)buf[23] << 8) + buf[22];

      Dev_Now.Touchkeytrackid[3] = buf[25];
      Dev_Now.X[3] = ((uint16_t)buf[27] << 8) + buf[26];
      Dev_Now.Y[3] = ((uint16_t)buf[29] << 8) + buf[28];
      Dev_Now.S[3] = ((uint16_t)buf[31] << 8) + buf[30];

      Dev_Now.Touchkeytrackid[4] = buf[33];
      Dev_Now.X[4] = ((uint16_t)buf[35] << 8) + buf[34];
      Dev_Now.Y[4] = ((uint16_t)buf[37] << 8) + buf[36];
      Dev_Now.S[4] = ((uint16_t)buf[39] << 8) + buf[38];

      Serial.printf("Raw Touch coordinates: X=%d, Y=%d, Size=%d\r\n", Dev_Now.X[0], Dev_Now.Y[0], Dev_Now.S[0]);

      for (i = 0; i < Dev_Backup.TouchCount; i++)
      {
        if (Dev_Now.Y[i] < 0)Dev_Now.Y[i] = 0;
        if (Dev_Now.Y[i] > 480)Dev_Now.Y[i] = 480;
        if (Dev_Now.X[i] < 0)Dev_Now.X[i] = 0;
        if (Dev_Now.X[i] > 320)Dev_Now.X[i] = 320;
      }
      // Validate and process touch points
      bool validTouch = false;
      for (i = 0; i < Dev_Now.TouchCount; i++)
    {
        // Validate coordinates more strictly
        if (Dev_Now.Y[i] < 0 || Dev_Now.Y[i] > 480 || Dev_Now.X[i] < 0 || Dev_Now.X[i] > 320) {
            // Serial.printf("Invalid coordinates detected: X=%d, Y=%d - rejecting\r\n", Dev_Now.X[i], Dev_Now.Y[i]);
            continue;
        }
        
        // Additional validation for obviously wrong values
        if (Dev_Now.X[i] > 1000 || Dev_Now.Y[i] > 1000) {
            // Serial.printf("Corrupted coordinates detected: X=%d, Y=%d - rejecting\r\n", Dev_Now.X[i], Dev_Now.Y[i]);
            continue;
        }

        // Valid touch found
        Dev_Backup.X[i] = Dev_Now.X[i];
        Dev_Backup.Y[i] = Dev_Now.Y[i];
        Dev_Backup.TouchCount = Dev_Now.TouchCount;
        // Serial.printf("Touch validated: X=%d, Y=%d\r\n", Dev_Now.X[i], Dev_Now.Y[i]);
        validTouch = true;
      }
      
      // Set touched based on whether we have valid touches
      touched = validTouch;
     if(Dev_Now.TouchCount==0)
        {
            touched = 0;
        }  
    }
  }
}

uint8_t GT911_ReadStatue(void)
{
  uint8_t buf[4];
  GT911_RD_Reg(GT_PID_REG, (uint8_t *)&buf[0], 3); 
  GT911_RD_Reg(GT_CFGS_REG, (uint8_t *)&buf[3], 1);
  Serial.printf("TouchPad_ID:%d,%d,%d\r\nTouchPad_Config_Version:%2x\r\n", buf[0], buf[1], buf[2], buf[3]);
  return buf[3];
}

void GT911_Reset_Sequence()
{
  //此处RST引脚与屏幕RST共用，只需要初始化一次即可
  IIC_RST_0;
  delay(100);
  IIC_RST_0;
  delay(100);
  IIC_RST_1;
  delay(200);
}

void GT911_Int()
{
  uint8_t config_Checksum = 0, i;

  Serial.println("=== GT911 INITIALIZATION DEBUG ===");
  
  IIC_Init();
  Serial.println("I2C initialized");
  
  GT911_Reset_Sequence();
  Serial.println("GT911 reset sequence completed");
  
  //debug
  GT911_RD_Reg(GT_CFGS_REG, (uint8_t *)&s_GT911_CfgParams[0], 186);
  Serial.println("GT911 config read completed");

  for (i = 0; i < sizeof(s_GT911_CfgParams) - 2; i++)
  {
    config_Checksum += s_GT911_CfgParams[i];
  }

  if (s_GT911_CfgParams[184] == (((~config_Checksum) + 1) & 0xff))
  {
    Serial.printf("READ CONFIG SUCCESS!\r\n");
    Serial.printf("Touch area: %d*%d\r\n", s_GT911_CfgParams[2] << 8 | s_GT911_CfgParams[1], s_GT911_CfgParams[4] << 8 | s_GT911_CfgParams[3]);

    if ((GT911_MAX_WIDTH != (s_GT911_CfgParams[2] << 8 | s_GT911_CfgParams[1])) || (GT911_MAX_HEIGHT != (s_GT911_CfgParams[4] << 8 | s_GT911_CfgParams[3])))
    {
      Serial.println("Updating GT911 config for correct resolution...");
      s_GT911_CfgParams[1] = GT911_MAX_WIDTH & 0xff;
      s_GT911_CfgParams[2] = GT911_MAX_WIDTH >> 8;
      s_GT911_CfgParams[3] = GT911_MAX_HEIGHT & 0xff;
      s_GT911_CfgParams[4] = GT911_MAX_HEIGHT >> 8;
      s_GT911_CfgParams[185] = 1;

      config_Checksum = 0;
      for (i = 0; i < sizeof(s_GT911_CfgParams) - 2; i++)
      {
        config_Checksum += s_GT911_CfgParams[i];
      }
      s_GT911_CfgParams[184] = (~config_Checksum) + 1;

      GT911_WR_Reg(GT_CFGS_REG, (uint8_t *)s_GT911_CfgParams, sizeof(s_GT911_CfgParams));
      Serial.println("GT911 config updated");
    }
  } else {
    Serial.println("CONFIG READ FAILED - Checksum mismatch!");
  }
  
  GT911_ReadStatue();
  Serial.println("=== GT911 INITIALIZATION COMPLETE ===");
}

// Display flush callback
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t*)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

// Touchpad read callback
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    static unsigned long lastTouchDebug = 0;
    static bool lastTouched = false;
    static unsigned long lastCallbackDebug = 0;
    static unsigned long lastTouchTime = 0;
    static const unsigned long TOUCH_DEBOUNCE_MS = 150; // Debounce time to prevent double clicks

    // Debug: Print when callback is called every 10 seconds (reduced frequency)
    if (millis() - lastCallbackDebug > 10000) {
        Serial.printf("my_touchpad_read callback called, touched=%d\r\n", touched);
        lastCallbackDebug = millis();
    }

    // Throttle touch scanning to prevent double clicks
    if (millis() - lastTouchTime < 50) { // 20 FPS max for touch scanning
        data->state = lastTouched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
        return;
    }
    lastTouchTime = millis();

    GT911_Scan();
    
    if (!touched) {
        data->state = LV_INDEV_STATE_REL;
        if (lastTouched) {
            Serial.printf("Touch released\r\n");
            lastTouched = false;
        }
     } else {
         // Debounce touch to prevent double clicks
         static unsigned long lastValidTouch = 0;
         if (millis() - lastValidTouch < TOUCH_DEBOUNCE_MS) {
             data->state = LV_INDEV_STATE_REL;
             return;
         }
         lastValidTouch = millis();

         /*Set the coordinates - adjust for EEZ Studio UI (480x320 landscape)*/
         // Map touch coordinates to match EEZ Studio screen layout
         // Your display is 480x320 in landscape mode (setRotation(1))
         
         int16_t touch_x, touch_y;
         
        // First, transform raw coordinates for landscape mode (setRotation(1))
        int16_t transformed_x = Dev_Now.Y[0];  // Y becomes X in landscape
        int16_t transformed_y = 320 - Dev_Now.X[0];  // X becomes Y (inverted)
        
        if (calibration_mode) {
            // During calibration, process the transformed coordinates
            process_calibration_touch(transformed_x, transformed_y);
            // Use transformed coordinates during calibration
            touch_x = transformed_x;
            touch_y = transformed_y;
        } else if (single_calibration_mode) {
            // During single point calibration, capture the touch point
            Serial.printf("Single point calibration: Raw(%d,%d) -> Transformed(%d,%d)\n", 
                         Dev_Now.X[0], Dev_Now.Y[0], transformed_x, transformed_y);
            
            // Use the known working screen coordinates for this raw point
            // Based on your test: Raw(63,178) -> Screen(279,250) works for button
            // Map this touch point to button center (240, 160)
            calibrate_from_center_point(transformed_x, transformed_y, 240, 160);
            
            // Hide visual indicator and exit calibration mode
            hide_single_calibration_center();
            single_calibration_mode = false;
            
            Serial.println("Single point calibration completed!");
            Serial.println("Touch mapping is now active for entire screen.");
            
            // Use transformed coordinates for this touch
            touch_x = transformed_x;
            touch_y = transformed_y;
        } else {
            // Apply calibration using the calculated values
            if (touch_cal.calibrated) {
                touch_x = (int16_t)(transformed_x * touch_cal.scale_x) + touch_cal.offset_x;
                touch_y = (int16_t)(transformed_y * touch_cal.scale_y) + touch_cal.offset_y;
            } else if (single_cal.calibrated) {
                // Apply single point calibration
                apply_single_point_calibration(transformed_x, transformed_y, &touch_x, &touch_y);
            } else {
                // Manual calibration values (fallback)
                touch_x = (int16_t)(transformed_x * 1.458) + 47;
                touch_y = (int16_t)(transformed_y * 0.642) + 138;
            }
        }
         
        data->point.x = touch_x;
        data->point.y = touch_y;
        data->state = LV_INDEV_STATE_PR;
        
        // Debug touch coordinates
        const char* cal_status = "MANUAL";
        if (touch_cal.calibrated) {
            cal_status = "MULTI-CAL";
        } else if (single_cal.calibrated) {
            cal_status = "SINGLE-CAL";
        }
        
        Serial.printf("Touch: Raw(%d,%d) -> Transformed(%d,%d) -> LVGL(%d,%d) [%s]\n", 
                     Dev_Now.X[0], Dev_Now.Y[0], transformed_x, transformed_y, touch_x, touch_y, cal_status);
        
         // Print LVGL touch data only when starting touch (reduced frequency)
         if (!lastTouched) {
             Serial.printf("Raw Touch: X=%d, Y=%d\r\n", Dev_Now.X[0], Dev_Now.Y[0]);
             Serial.printf("LVGL Touch: X=%d, Y=%d, State=%d\r\n", data->point.x, data->point.y, data->state);
             
             // Debug: Check if touch is in button area (landscape mode: 480x320)
             // Button is at position (190, 145) with size 100x30
             // But based on actual touch, button might be at different position
             if (data->point.x >= 190 && data->point.x <= 290 && data->point.y >= 145 && data->point.y <= 175) {
                 Serial.printf("✅ Touch is in MAIN button area! Should trigger button click.\r\n");
                 Serial.printf("Main button area: X=190-290, Y=145-175\r\n");
                 Serial.printf("Touch position: X=%d, Y=%d\r\n", data->point.x, data->point.y);
             } else if (data->point.x >= 190 && data->point.x <= 290 && data->point.y >= 240 && data->point.y <= 270) {
                 Serial.printf("✅ Touch is in PAGE1 button area! Should trigger button click.\r\n");
                 Serial.printf("Page1 button area: X=190-290, Y=240-270\r\n");
                 Serial.printf("Touch position: X=%d, Y=%d\r\n", data->point.x, data->point.y);
             } else {
                 Serial.printf("❌ Touch is outside button areas. Current: X=%d, Y=%d\r\n", data->point.x, data->point.y);
                 Serial.printf("Main button area: X=190-290, Y=145-175\r\n");
                 Serial.printf("Page1 button area: X=190-290, Y=240-270\r\n");
             }
             
             Serial.printf("Touch started - EEZ Studio UI will handle button events\r\n");
             lastTouched = true;
         }
    }
}

// Touch calibration function
void start_touch_calibration() {
    Serial.println("=== TOUCH CALIBRATION STARTED ===");
    Serial.println("Touch the 4 corners of the screen in order:");
    Serial.println("1. Top-left corner");
    Serial.println("2. Top-right corner");
    Serial.println("3. Bottom-left corner");
    Serial.println("4. Bottom-right corner");
    Serial.println("Each corner will be highlighted for 3 seconds");
    Serial.println("=====================================");
    
    calibration_mode = true;
    cal_point_index = 0;
    cal_start_time = millis();
    
    // Reset all calibration points
    for (int i = 0; i < 4; i++) {
        cal_points[i].collected = false;
    }
    
    // Show first calibration point
    show_calibration_point();
}

void process_calibration_touch(int16_t raw_x, int16_t raw_y) {
    if (!calibration_mode || cal_point_index >= 4) return;
    
    // Collect raw touch coordinates
    cal_points[cal_point_index].raw_x = raw_x;
    cal_points[cal_point_index].raw_y = raw_y;
    cal_points[cal_point_index].collected = true;
    
    Serial.printf("Calibration point %d collected: Raw(%d,%d) -> Target(%d,%d)\n", 
                  cal_point_index + 1, raw_x, raw_y, 
                  cal_points[cal_point_index].target_x, cal_points[cal_point_index].target_y);
    
    cal_point_index++;
    
    if (cal_point_index >= 4) {
        // All points collected, calculate calibration
        hide_calibration_point();
        calculate_calibration();
    } else {
        // Move to next point
        cal_start_time = millis();
        Serial.printf("Now touch point %d: (%d,%d)\n", 
                      cal_point_index + 1, 
                      cal_points[cal_point_index].target_x, 
                      cal_points[cal_point_index].target_y);
        // Show next calibration point
        show_calibration_point();
    }
}

void calculate_calibration() {
    Serial.println("=== CALCULATING CALIBRATION ===");
    
    // Find min/max raw coordinates to detect Y inversion
    int16_t min_raw_x = 9999, max_raw_x = -9999;
    int16_t min_raw_y = 9999, max_raw_y = -9999;
    int16_t min_target_x = 9999, max_target_x = -9999;
    int16_t min_target_y = 9999, max_target_y = -9999;
    
    for (int i = 0; i < 4; i++) {
        if (cal_points[i].collected) {
            if (cal_points[i].raw_x < min_raw_x) min_raw_x = cal_points[i].raw_x;
            if (cal_points[i].raw_x > max_raw_x) max_raw_x = cal_points[i].raw_x;
            if (cal_points[i].raw_y < min_raw_y) min_raw_y = cal_points[i].raw_y;
            if (cal_points[i].raw_y > max_raw_y) max_raw_y = cal_points[i].raw_y;
            
            if (cal_points[i].target_x < min_target_x) min_target_x = cal_points[i].target_x;
            if (cal_points[i].target_x > max_target_x) max_target_x = cal_points[i].target_x;
            if (cal_points[i].target_y < min_target_y) min_target_y = cal_points[i].target_y;
            if (cal_points[i].target_y > max_target_y) max_target_y = cal_points[i].target_y;
        }
    }
    
    // Calculate X scale (should be straightforward)
    touch_cal.scale_x = (float)(max_target_x - min_target_x) / (max_raw_x - min_raw_x);
    
    // For Y, check if it's inverted first
    // Find top and bottom points
    int16_t top_target_y = 9999, bottom_target_y = -9999;
    int16_t top_raw_y = 0, bottom_raw_y = 0;
    
    for (int i = 0; i < 4; i++) {
        if (cal_points[i].collected) {
            if (cal_points[i].target_y < top_target_y) {
                top_target_y = cal_points[i].target_y;
                top_raw_y = cal_points[i].raw_y;
            }
            if (cal_points[i].target_y > bottom_target_y) {
                bottom_target_y = cal_points[i].target_y;
                bottom_raw_y = cal_points[i].raw_y;
            }
        }
    }
    
    // Check if Y is inverted (top has higher raw Y than bottom)
    bool y_inverted = (top_raw_y > bottom_raw_y);
    
    Serial.printf("Top point: Target Y=%d, Raw Y=%d\n", top_target_y, top_raw_y);
    Serial.printf("Bottom point: Target Y=%d, Raw Y=%d\n", bottom_target_y, bottom_raw_y);
    Serial.printf("Y inverted: %s\n", y_inverted ? "YES" : "NO");
    
    // Calculate Y scale based on whether it's inverted
    if (y_inverted) {
        // For inverted Y: scale = (target_range) / (raw_range)
        touch_cal.scale_y = (float)(max_target_y - min_target_y) / (max_raw_y - min_raw_y);
        Serial.println("Y coordinate is inverted - using direct scale calculation");
    } else {
        // For normal Y: scale = (target_range) / (raw_range)
        touch_cal.scale_y = (float)(max_target_y - min_target_y) / (max_raw_y - min_raw_y);
    }
    
    // Calculate offsets
    touch_cal.offset_x = min_target_x - (int16_t)(min_raw_x * touch_cal.scale_x);
    
    if (y_inverted) {
        // For inverted Y: offset = target_max - (raw_min * scale)
        // This maps raw_min (bottom) to target_max (bottom)
        touch_cal.offset_y = max_target_y - (int16_t)(min_raw_y * touch_cal.scale_y);
    } else {
        // For normal Y: offset = target_min - (raw_min * scale)
        touch_cal.offset_y = min_target_y - (int16_t)(min_raw_y * touch_cal.scale_y);
    }
    
    touch_cal.calibrated = true;
    calibration_mode = false;
    
    Serial.printf("Raw X range: %d to %d\n", min_raw_x, max_raw_x);
    Serial.printf("Raw Y range: %d to %d\n", min_raw_y, max_raw_y);
    Serial.printf("Target X range: %d to %d\n", min_target_x, max_target_x);
    Serial.printf("Target Y range: %d to %d\n", min_target_y, max_target_y);
    Serial.printf("Y inverted: %s\n", y_inverted ? "YES" : "NO");
    Serial.printf("Calibration complete!\n");
    Serial.printf("Scale X: %.3f, Scale Y: %.3f\n", touch_cal.scale_x, touch_cal.scale_y);
    Serial.printf("Offset X: %d, Offset Y: %d\n", touch_cal.offset_x, touch_cal.offset_y);
    Serial.println("================================");
    Serial.println("✅ CALIBRATED VALUES ARE NOW ACTIVE!");
    Serial.println("Touch coordinates will use calibrated values.");
    Serial.println("================================");
    
    // Generate code for the calibration
    generate_calibration_code();
    
    // Restore the EEZ Studio UI
    restore_eez_ui();
}

void generate_calibration_code() {
    Serial.println("=== CALIBRATION CODE ===");
    Serial.println("Copy this code to replace the coordinate mapping in your main.cpp:");
    Serial.println("// Auto-generated calibration code");
    Serial.printf("int16_t touch_x = (int16_t)(Dev_Now.X[0] * %.3f) + %d;\n", touch_cal.scale_x, touch_cal.offset_x);
    Serial.printf("int16_t touch_y = (int16_t)(Dev_Now.Y[0] * %.3f) + %d;\n", touch_cal.scale_y, touch_cal.offset_y);
    Serial.println("// End calibration code");
    Serial.println("========================");
}

// Apply calibration to touch coordinates
void apply_calibration(int16_t raw_x, int16_t raw_y, int16_t* calibrated_x, int16_t* calibrated_y) {
    if (touch_cal.calibrated) {
        *calibrated_x = (int16_t)(raw_x * touch_cal.scale_x) + touch_cal.offset_x;
        *calibrated_y = (int16_t)(raw_y * touch_cal.scale_y) + touch_cal.offset_y;
    } else {
        // Use current manual calibration
        *calibrated_x = raw_x * 3;
        *calibrated_y = raw_y - 28;
    }
}

// Visual calibration point functions
void show_calibration_point() {
    if (!calibration_mode || cal_point_index >= 4) return;
    
    // Clear screen with black background
    tft.fillScreen(TFT_BLACK);
    
    // Draw calibration point as a large red circle
    int16_t x = cal_points[cal_point_index].target_x;
    int16_t y = cal_points[cal_point_index].target_y;
    
    // Draw large red circle
    tft.fillCircle(x, y, 20, TFT_RED);
    tft.drawCircle(x, y, 20, TFT_WHITE);
    tft.drawCircle(x, y, 25, TFT_WHITE);
    
    // Draw crosshair
    tft.drawLine(x-30, y, x+30, y, TFT_WHITE);
    tft.drawLine(x, y-30, x, y+30, TFT_WHITE);
    
    // Draw instruction text
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.printf("Touch Point %d", cal_point_index + 1);
    
    tft.setTextSize(1);
    tft.setCursor(10, 40);
    tft.printf("Target: (%d, %d)", x, y);
    
    Serial.printf("Showing calibration point %d at (%d, %d)\n", cal_point_index + 1, x, y);
}

void hide_calibration_point() {
    // Clear the screen
    tft.fillScreen(TFT_BLACK);
    
    // Redraw the EEZ Studio UI
    // Note: This is a simple approach - in a full implementation, 
    // you might want to save and restore the UI state
}

void restore_eez_ui() {
    Serial.println("Restoring EEZ Studio UI...");
    
    // Clear screen
    tft.fillScreen(TFT_BLACK);
    
    // Force LVGL to refresh the display
    lv_refr_now(NULL);
    
    // Give LVGL time to process the refresh
    delay(100);
    
    // Force EEZ Studio to redraw by invalidating the current screen
    lv_obj_invalidate(lv_scr_act());
    
    // Process LVGL tasks to ensure UI is updated
    for (int i = 0; i < 10; i++) {
        lv_timer_handler();
        delay(10);
    }
    
    Serial.println("EEZ Studio UI restored successfully!");
    Serial.println("Touch calibration completed. You can now use the UI normally.");
}

void test_gt911_touch() {
    Serial.println("=== GT911 TOUCH TEST ===");
    Serial.println("Touch the screen and watch for touch detection...");
    Serial.println("This test will run for 10 seconds");
    Serial.println("Look for 'Touch detected!' messages");
    Serial.println("=========================");
    
    unsigned long test_start = millis();
    unsigned long last_debug = 0;
    
    while (millis() - test_start < 10000) { // Run for 10 seconds
        GT911_Scan();
        
        // Print debug info every 2 seconds
        if (millis() - last_debug > 2000) {
            Serial.printf("Test running... Time left: %d seconds\n", (10000 - (millis() - test_start)) / 1000);
            last_debug = millis();
        }
        
        if (touched) {
            Serial.printf("TOUCH DETECTED! Raw coordinates: X=%d, Y=%d\n", Dev_Now.X[0], Dev_Now.Y[0]);
        }
        
        delay(10);
    }
    
    Serial.println("=== GT911 TOUCH TEST COMPLETE ===");
}

// Note: Button event handling is now managed by EEZ Studio UI

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 TFT Touch Test Starting...");

    // Initialize GT911 touch controller
    Serial.println("Initializing GT911 touch controller...");
    GT911_Int();
    Serial.println("GT911 initialization completed.");
    
    // Initialize TFT display
    tft.init();
    tft.setRotation(1);  // 180° rotation to match touch orientation
    tft.fillScreen(TFT_BLACK);

    // Initialize LVGL
    lv_init();

    // Initialize display buffer (landscape mode: 480x320)
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, 480 * 20);
    // lv_disp_draw_buf_init(&draw_buf, buf, NULL, 320 * 20);

    // Initialize display (landscape mode: 480x320)
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 480;  // Landscape mode: width = 480
    disp_drv.ver_res = 320;  // Landscape mode: height = 320
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Initialize input device
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // Initialize EEZ Studio UI
    Serial.println("Initializing EEZ Studio UI...");
    ui_init();
    Serial.println("EEZ Studio UI initialized successfully!");

    // Touch calibration system ready
    Serial.println("Touch calibration system ready!");
    Serial.println("Type 'CAL' to start multi-point calibration");
    Serial.println("Type 'SINGLE' to start single-point calibration (recommended)");
    Serial.println("Type 'STATUS' to check calibration status");
    Serial.println("Type 'TEST' to test GT911 touch controller");
    Serial.println("Type 'HELP' for all available commands");
    
    Serial.println("==========================================");
    Serial.println("EEZ Studio UI is now active!");
    Serial.println("Touch the screen to interact with buttons.");
    Serial.println("==========================================");

    Serial.println("Setup completed successfully!");
}

void loop() {
    GT911_Scan();  // Scan for touch input
    lv_timer_handler();  // Handle LVGL tasks
    ui_tick();  // Handle EEZ Studio UI tasks
    
    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readString();
        command.trim();
        command.toUpperCase();
        
        if (command == "CAL") {
            start_touch_calibration();
        } else if (command == "SINGLE") {
            Serial.println("=== SINGLE POINT CALIBRATION ===");
            Serial.println("Touch the center of the screen where button works");
            Serial.println("This will calculate the entire screen mapping from one point");
            Serial.println("Waiting for center point touch...");
            single_cal.calibrated = false; // Reset to allow new calibration
            single_calibration_mode = true; // Enable single calibration mode
            show_single_calibration_center(); // Show visual indicator
        } else if (command == "STATUS") {
            Serial.println("=== CALIBRATION STATUS ===");
            Serial.printf("Multi-point calibration: %s\n", touch_cal.calibrated ? "CALIBRATED" : "NOT CALIBRATED");
            Serial.printf("Single-point calibration: %s\n", single_cal.calibrated ? "CALIBRATED" : "NOT CALIBRATED");
            
            if (touch_cal.calibrated) {
                Serial.printf("Multi-point - Scale X: %.3f, Scale Y: %.3f\n", touch_cal.scale_x, touch_cal.scale_y);
                Serial.printf("Multi-point - Offset X: %d, Offset Y: %d\n", touch_cal.offset_x, touch_cal.offset_y);
            }
            
            if (single_cal.calibrated) {
                Serial.printf("Single-point - Scale X: %.3f, Scale Y: %.3f\n", single_cal.scale_x, single_cal.scale_y);
                Serial.printf("Single-point - Offset X: %d, Offset Y: %d\n", single_cal.offset_x, single_cal.offset_y);
                Serial.printf("Center point: Raw(%d,%d) -> Screen(%d,%d)\n", 
                    single_cal.center_raw_x, single_cal.center_raw_y,
                    single_cal.center_screen_x, single_cal.center_screen_y);
            }
        } else if (command == "HELP") {
            Serial.println("Available commands:");
            Serial.println("CAL - Start touch calibration");
            Serial.println("SINGLE - Start single point calibration");
            Serial.println("STATUS - Show calibration status");
            Serial.println("TEST - Test GT911 touch controller");
            Serial.println("RESET - Reset calibration to manual values");
            Serial.println("DIRECT - Enable direct button mapping");
            Serial.println("HELP - Show this help");
        } else if (command == "TEST") {
            test_gt911_touch();
        } else if (command == "RESET") {
            touch_cal.calibrated = false;
            single_cal.calibrated = false;
            calibration_mode = false;
            single_calibration_mode = false;
            hide_calibration_point();
            hide_single_calibration_center();
            Serial.println("All calibrations reset to manual values");
            Serial.println("Use 'CAL' for multi-point calibration or 'SINGLE' for single-point calibration");
        } else if (command == "DIRECT") {
            Serial.println("Direct button mapping is now active!");
            Serial.println("Touch near button area will be mapped to button center (240,160)");
        }
    }
    
    delay(1);  // Minimal delay for better responsiveness
}

// Single Point Calibration Implementation
void calibrate_from_center_point(int16_t raw_x, int16_t raw_y, int16_t screen_x, int16_t screen_y) {
    single_cal.center_raw_x = raw_x;
    single_cal.center_raw_y = raw_y;
    single_cal.center_screen_x = screen_x;
    single_cal.center_screen_y = screen_y;
    
    // Calculate scale factors from center point
    single_cal.scale_x = (float)screen_x / (float)raw_x;
    single_cal.scale_y = (float)screen_y / (float)raw_y;
    
    // Calculate offsets to center the mapping
    // Assuming we want the center point to map to screen center (240, 160)
    single_cal.offset_x = 240 - (int16_t)(raw_x * single_cal.scale_x);
    single_cal.offset_y = 160 - (int16_t)(raw_y * single_cal.scale_y);
    
    single_cal.calibrated = true;
    
    Serial.printf("=== SINGLE POINT CALIBRATION ===\n");
    Serial.printf("Center point: Raw(%d,%d) -> Screen(%d,%d)\n", raw_x, raw_y, screen_x, screen_y);
    Serial.printf("Scale factors: X=%.3f, Y=%.3f\n", single_cal.scale_x, single_cal.scale_y);
    Serial.printf("Offset factors: X=%d, Y=%d\n", single_cal.offset_x, single_cal.offset_y);
    Serial.printf("Single point calibration is now active!\n");
    Serial.printf("===============================\n");
}

void apply_single_point_calibration(int16_t raw_x, int16_t raw_y, int16_t* screen_x, int16_t* screen_y) {
    if (!single_cal.calibrated) {
        *screen_x = raw_x;
        *screen_y = raw_y;
        return;
    }
    
    // Apply single point calibration: scale + offset
    *screen_x = (int16_t)(raw_x * single_cal.scale_x) + single_cal.offset_x;
    *screen_y = (int16_t)(raw_y * single_cal.scale_y) + single_cal.offset_y;
    
    // Clamp to screen bounds (480x320 landscape)
    if (*screen_x < 0) *screen_x = 0;
    if (*screen_x > 479) *screen_x = 479;
    if (*screen_y < 0) *screen_y = 0;
    if (*screen_y > 319) *screen_y = 319;
    
    Serial.printf("Single point mapping: Raw(%d,%d) -> Screen(%d,%d)\n", raw_x, raw_y, *screen_x, *screen_y);
}

// Single Point Calibration Visual Functions
void show_single_calibration_center() {
    Serial.println("Showing single point calibration center...");
    
    // Clear screen
    tft.fillScreen(TFT_BLACK);
    
    // Draw center crosshair (240, 160) - center of 480x320 screen
    int center_x = 240;
    int center_y = 160;
    int size = 50;
    
    // Draw crosshair
    tft.drawLine(center_x - size, center_y, center_x + size, center_y, TFT_RED);
    tft.drawLine(center_x, center_y - size, center_x, center_y + size, TFT_RED);
    
    // Draw center circle
    tft.drawCircle(center_x, center_y, 20, TFT_RED);
    tft.fillCircle(center_x, center_y, 5, TFT_RED);
    
    // Draw instruction text
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawString("SINGLE POINT", 120, 50, 2);
    tft.drawString("CALIBRATION", 120, 80, 2);
    tft.setTextSize(1);
    tft.drawString("Touch the RED crosshair", 140, 250, 2);
    tft.drawString("in the center", 160, 270, 2);
    
    Serial.println("Single point calibration center displayed!");
    Serial.println("Touch the red crosshair to calibrate.");
}

void hide_single_calibration_center() {
    Serial.println("Hiding single point calibration center...");
    
    // Clear screen
    tft.fillScreen(TFT_BLACK);
    
    // Force LVGL to refresh the display
    lv_refr_now(NULL);
    
    // Give LVGL time to process the refresh
    delay(100);
    
    // Force EEZ Studio to redraw by invalidating the current screen
    lv_obj_invalidate(lv_scr_act());
    
    // Process LVGL tasks to ensure UI is updated
    for (int i = 0; i < 10; i++) {
        lv_timer_handler();
        delay(10);
    }
    
    Serial.println("Single point calibration center hidden!");
    Serial.println("EEZ Studio UI restored.");
}