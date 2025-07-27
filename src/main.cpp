// FM Receiver with QN8035

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// #define USE_HSI_CLOCK 1
//  I2C address of the QN8035 tuner.
#define QN8035_ADDRESS 0x10

// Chip ID related to QN8035 tuner.
#define QN8035_ID 0x84

#define REG_SYSTEM1 0x00   // Device modes.
#define REG_CCA 0x01       // CCA parameters.
#define REG_SNR 0x02       // Estimate RF input CNR value.
#define REG_RSSISIG 0x03   // In-band signal RSSI value.
#define REG_STATUS1 0x04   // System status.
#define REG_CID1 0x05      // Device ID numbers.
#define REG_CID2 0x06      // Device ID numbers.
#define REG_CH 0x07        // Lower 8 bits of 10-bit channel index.
#define REG_CH_START 0x08  // Lower 8 bits of 10-bit channel scan start channel index.
#define REG_CH_STOP 0x09   // Lower 8 bits of 10-bit channel scan stop channel index.
#define REG_CH_STEP 0x0A   // Channel scan frequency step. Highest 2 bits of channel indexes.
#define REG_RDSD0 0x0B     // RDS data byte 0.
#define REG_RDSD1 0x0C     // RDS data byte 1.
#define REG_RDSD2 0x0D     // RDS data byte 2.
#define REG_RDSD3 0x0E     // RDS data byte 3.
#define REG_RDSD4 0x0F     // RDS data byte 4.
#define REG_RDSD5 0x10     // RDS data byte 5.
#define REG_RDSD6 0x11     // RDS data byte 6.
#define REG_RDSD7 0x12     // RDS data byte 7.
#define REG_STATUS2 0x13   // RDS status indicators.
#define REG_VOL_CTL 0x14   // Audio controls.
#define REG_XTAL_DIV0 0x15 // Frequency select of reference clock source.
#define REG_XTAL_DIV1 0x16 // Frequency select of reference clock source.
#define REG_XTAL_DIV2 0x17 // Frequency select of reference clock source.
#define REG_INT_CTRL 0x18  // RDS control.

// Undocumented registers (based on https://github.com/ukrtrip/QN8035-qn8035-FM-chip-library/).
#define REG_CCA_SNR_TH_1 0x39
#define REG_CCA_SNR_TH_2 0x3A
#define REG_NCCFIR3 0x40

// Scanning steps for REG_CH_STEP.
#define REG_CH_STEP_50KHZ 0x00
#define REG_CH_STEP_100KHZ 0x40
#define REG_CH_STEP_200KHZ 0x80

// Bit definitions of REG_SYSTEM1.
#define REG_SYSTEM1_CCA_CH_DIS 0x01 // CH (channel index) selection method. 0 - CH is determined by internal CCA; 1 - CH is determined by the content in CH[9:0].
#define REG_SYSTEM1_CHSC 0x02       // Channel Scan mode enable. 0 - Normal operation; 1 - Channel Scan mode operation.
#define REG_SYSTEM1_FORCE_MO 0x04   // Force receiver in MONO mode, 0 - Auto, 1 - Forced in MONO mode.
#define REG_SYSTEM1_RDSEN 0x08      // RDS enable.
#define REG_SYSTEM1_RXREQ 0x10      // Receiving request. 0 - Non RX mode, 1 - Enter receive mode.
#define REG_SYSTEM1_STNBY 0x20      // Request immediately to enter Standby mode.
#define REG_SYSTEM1_RECAL 0x40      // Reset the state to initial states and recalibrate all blocks.
#define REG_SYSTEM1_SWRST 0x80      // Reset all registers to default values.

// Bit definitions of REG_STATUS1.
#define REG_STATUS1_ST_MO_RX 0x01   // Stereo receiving status.
#define REG_STATUS1_RXAGC 0x02      // AGC error status.
#define REG_STATUS1_RXAGCSET 0x04   // AGC settling status.
#define REG_STATUS1_RXCCA_FAIL 0x08 // RXCCA Status Flag.
#define REG_STATUS1_FSM 0x70        // FSM state indicator.

// RDS group definitions.
#define RDS_GROUP 0xF800
#define RDS_GROUP_A0 0x0000
#define RDS_GROUP_B0 0x0080

// Volume control settings
#define REG_VOL_CTL_MAX_ANALOG_GAIN 0x07
#define REG_VOL_CTL_MIN_ANALOG_GAIN 0x00

// RDS buffering mode.
#define RDS_DOUBLE_BUFFER_ENABLE 0xFF
#define RDS_DOUBLE_BUFFER_DISABLE 0x00

#define FREQ_TO_WORD(f) ((USHORT)((f - 60) / 0.05))

#define UCHAR unsigned char
#define USHORT unsigned short
#define CHAR char
#define INT int
#define DOUBLE double
#define VOID void

#define GET_REG ReadReg

// Size of the RDS buffer.
#define RDS_INFO_MAX_SIZE 16
CHAR rdsInfo[RDS_INFO_MAX_SIZE]; // = {'-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-'};
CHAR tempRDSBuffer[RDS_INFO_MAX_SIZE];
long i; // general purpose counter

int Mode;

long Freq = 108100; // current frequency

// OLED SSD1306
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // OLED configuration
// Create a U8g2log object

HardwareTimer *MyTim1 = new HardwareTimer(TIM1); //
// HardwareTimer *MyTim3 = new HardwareTimer(TIM3); //

TIM_HandleTypeDef htim3;

int SignalNoise;
int RSSI, RSSIcut;
DOUBLE freq = 102.9; // MHz
USHORT freqWord;
CHAR stereoStatus;
int RDSstate = 1;

void WriteReg(int Addr, int Reg);
int ReadReg(int Addr);
void SetFreq(USHORT Freq);
void powerOff(void);
VOID decodeRDSInfo(unsigned char useDoubleBuffer);
VOID resetRDSInfo();
VOID shutdownTuner();
void SetVol(int vol);

int EncOld = 0, EncNew = 0;
int Stereo;
int triggerUP;
int triggerENC;
int Freq_Volume;
int volume = 7;

int temp1;

static void MX_TIM3_Init(void);
// ========================================================================================================= SETUP =====================================
void setup(void)
{
  delay(20); // delay
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Initialize buttons
  pinMode(PB0, INPUT_PULLUP); // POWER button input with pullup
  pinMode(PB1, INPUT_PULLUP); // DOWN button input with pullup
  pinMode(PB2, INPUT_PULLUP); // UP button input with pullup
  pinMode(PA5, INPUT);        // Encoder button

  // Initialize LED outputs
  pinMode(PA15, OUTPUT); // LED indicator
  pinMode(PB10, OUTPUT); // LED indicator

  // Initialize OLED display
  u8g2.begin();

  u8g2.setFont(u8g2_font_7x14B_mf);
  u8g2.drawStr(10, 40, "QN8035FMRX v1.0");
  u8g2.sendBuffer(); // transfer internal memory to the display

  MyTim1->setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE, PA8);
  MyTim1->setPrescaleFactor(1);                 // Due to setOverflow with MICROSEC_FORMAT, prescaler will be computed automatically based on timer input clock
  MyTim1->setOverflow(32768 * 2, HERTZ_FORMAT); // 100000 microseconds = 100 milliseconds
  // MyTim1->attachInterrupt(Update_IT_callback);
  // MyTim1->setInterruptPriority(0, 0);
  MyTim1->resume();
  // for (;;)
  // {
  // }

  // Initialize Timer 3 for encoder input

  MX_TIM3_Init();

  // Initialize QN8035 tuner

  Wire.begin(); // join i2c bus

  WriteReg(0x00, 0x80); // программный сброс

  freqWord = FREQ_TO_WORD(freq);
  SetFreq(freqWord);
  // WriteReg(0x0A, 0x03);       // настройка на канал 320 = 100 МГц
  // WriteReg(0x07, 0x20);       // настройка на канал 320 = 100 МГц
  WriteReg(0x00, 0b00010001); // включение приема, RDS ON, MONO(1)/STEREO(0), сканирование выкл, ручной выбор канала,
  WriteReg(0x14, 0b00000111); // Громкость 111 - макс

  freq = 100.000;
  freqWord = FREQ_TO_WORD(freq);
  SetFreq(freqWord);

  // u8g2.drawStr(8 * 10, 48, "[OK]"); // write something to the internal memory

  delay(2000); // пауза

  u8g2.setContrast(0); // 0-255
}

//====================================================================================================== LOOP ========================================
void loop(void)
{
  // Read tuner status
  SignalNoise = ReadReg(REG_SNR);
  RSSI = ReadReg(REG_RSSISIG) - 43;
  stereoStatus = (ReadReg(REG_STATUS1) & REG_STATUS1_ST_MO_RX) ? 'M' : 'S';

  // Process RDS data if enabled
  if (RDSstate == 1)
    decodeRDSInfo(RDS_DOUBLE_BUFFER_ENABLE);

  EncNew = ((int)(TIM3->CNT) - 32768) / 4;

  if (Freq_Volume == 1) // управление громкостью
  {
    if (EncNew != EncOld)
    {
      volume += EncOld - EncNew;
      if (volume < 0)
        volume = 0;
      if (volume > 7)
        volume = 7;

      SetVol(volume);
    }
    EncOld = EncNew;
  }
  else
  {
    if (EncNew != EncOld)
    {
      freq += (EncOld - EncNew) * 0.05;
      freqWord = FREQ_TO_WORD(freq);
      SetFreq(freqWord);
      resetRDSInfo();
    }
    EncOld = EncNew;
  }
  // ------------- UP button ----------------
  if (digitalRead(PB2) == LOW) // RESET button pressed
  {

    if (digitalRead(PB1) == LOW) // RESET button pressed
    {
      digitalWrite(PB10, HIGH); // LED on
      delay(100);
      if (digitalRead(PB1) == LOW) // RESET button pressed
        powerOff();
    }

    if (triggerUP == 1)
    {
      triggerUP = 0;
      if (Stereo == 1)
      {
        Stereo = 0;
        WriteReg(0x00, 0b00011001); // включение приема, RDS ON, MONO(1)/STEREO(0), сканирование выкл, ручной выбор канала,
        digitalWrite(PA15, HIGH);   // светодиод вкл
      }
      else
      {
        Stereo = 1;
        WriteReg(0x00, 0b00011101); // включение приема, RDS ON, MONO(1)/STEREO(0), сканирование выкл, ручной выбор канала,
        digitalWrite(PA15, LOW);    // светодиод вкл
      }
    }
  }
  else
  {
    triggerUP = 1;
  }

  // ------------- кнопка DOWN ----------------
  if (digitalRead(PB1) == LOW) // нажата кнопка RESET
  {
    digitalWrite(PA15, HIGH); // светодиод вкл

    if (digitalRead(PB2) == LOW) // нажата кнопка RESET
    {
      digitalWrite(PB10, HIGH); // светодиод вкл
      delay(100);
      if (digitalRead(PB2) == LOW) // нажата кнопка RESET
        powerOff();
    }

    freq -= 0.050;
    freqWord = FREQ_TO_WORD(freq);
    SetFreq(freqWord);

    resetRDSInfo();

    delay(50);

    digitalWrite(PA15, LOW); // светодиод выкл
  }

  if (digitalRead(PA5) == LOW) // нажата кнопка энкодера'
  {
    if (triggerENC == 1)
    {
      triggerENC = 0;
      if (Freq_Volume == 1)
      {
        Freq_Volume = 0;
        digitalWrite(PB10, LOW); // светодиод вкл
      }
      else
      {
        Freq_Volume = 1;
        digitalWrite(PB10, HIGH); // светодиод вкл
      }

      delay(50);
    }
  }
  else
    triggerENC = 1;

  //-------------------------------------------- Display on screen ----------------------------
  int temp = 0;
  u8g2.print(temp, 16); // Workaround to fix a strange bug

  u8g2.clearBuffer(); // clear the internal memory

  // Set font for frequency display

  u8g2.setFont(u8g2_font_logisoso20_tr);
  u8g2.setCursor(0, 21);
  if (freq < 99.98)
    u8g2.setCursor(13, 21);
  u8g2.print(freq, 2);
  u8g2.setFont(u8g2_font_7x14B_mf);
  u8g2.setCursor(78, 21);
  u8g2.print("MHz V:");
  u8g2.print(volume);

  if (Freq_Volume == 1)
    u8g2.drawRFrame(103, 9, 25, 13, 2);

  u8g2.setFont(u8g2_font_6x13B_mf);
  u8g2.setCursor(100, 10);
  if (stereoStatus == 'S')
  {
    // u8g2.print("ST");
    u8g2.drawCircle(119 - 20, 4, 4);
    u8g2.drawCircle(123 - 20, 4, 4);
  }
  else
    u8g2.drawDisc(121 - 20, 4, 3);

  int VBat = 3800;
  int X_bat = 112;
  int Y_bat = 0;
  u8g2.drawFrame(0 + X_bat, 0 + Y_bat, 14, 7);
  u8g2.drawBox(15 + X_bat, 2 + Y_bat, 1, 3);
  if (VBat >= 3500)
    u8g2.drawBox(2 + X_bat, 2 + Y_bat, 1, 3);
  if (VBat >= 3690)
    u8g2.drawBox(X_bat + 2 + 3, Y_bat + 2, 1, 3);
  if (VBat >= 3780)
    u8g2.drawBox(X_bat + 2 + 3 + 3, Y_bat + 2, 1, 3);
  if (VBat >= 3850)
    u8g2.drawBox(X_bat + 2 + 3 + 3 + 3, Y_bat + 2, 1, 3);

  u8g2.setFont(u8g2_font_7x14B_mf);
  u8g2.setCursor(0, 34);
  u8g2.print("S/N");
  u8g2.setCursor(92 + 6, 34);
  if (SignalNoise < 10)
    u8g2.setCursor(100 + 6, 34);
  u8g2.print(SignalNoise);
  u8g2.setCursor(111 + 4, 34);
  u8g2.print("dB");

  if (SignalNoise > 61)
    SignalNoise = 61;
  u8g2.drawFrame(30, 26, 95 - 30, 6); // уровень SignalNoise
  u8g2.drawBox(32, 28, SignalNoise, 2);

  u8g2.setCursor(0, 35 + 12);
  u8g2.print("RSSI");
  u8g2.setCursor(98, 35 + 12);
  if (RSSI < 100)
    u8g2.setCursor(98, 35 + 12);
  if (RSSI < 10)
    u8g2.setCursor(106, 35 + 12);
  u8g2.print(RSSI);
  u8g2.setCursor(115, 35 + 12);
  u8g2.print("dB");
  RSSIcut = RSSI;
  if (RSSIcut > 61)
    RSSIcut = 61;
  u8g2.drawFrame(30, 27 + 12, 95 - 30, 6); // уровень RSSI
  u8g2.drawBox(32, 29 + 12, RSSIcut, 2);

  if (RDSstate == 1)
  {

    u8g2.setCursor(0, 63);
    u8g2.print("RDS: ");
    u8g2.setFont(u8g2_font_9x18B_mf);

    // u8g2.print(rdsInfo);
    for (i = 0; i < RDS_INFO_MAX_SIZE; i++)
    {
      u8g2.print(rdsInfo[i]);
    }
  }

  u8g2.setCursor(30, 63);
  u8g2.sendBuffer(); // transfer internal memory to the display
}

VOID decodeRDSInfo(unsigned char useDoubleBuffer)
{
  UCHAR offset;
  CHAR char1, char2;
  CHAR *buffer;
  USHORT groupB;

  // Construct RDS A,B,C,D packets.
  USHORT rdsA = GET_REG(REG_RDSD1) | GET_REG(REG_RDSD0) << 8;
  USHORT rdsB = GET_REG(REG_RDSD3) | GET_REG(REG_RDSD2) << 8;
  USHORT rdsC = GET_REG(REG_RDSD5) | GET_REG(REG_RDSD4) << 8;
  USHORT rdsD = GET_REG(REG_RDSD7) | GET_REG(REG_RDSD6) << 8;

  // Check for valid group A or B RDS packet(s).
  groupB = rdsB & RDS_GROUP;
  if ((groupB == RDS_GROUP_A0) || (groupB == RDS_GROUP_B0))
  {
    offset = (rdsB & 0x03) << 1;
    char1 = (CHAR)(rdsD >> 8);
    char2 = (CHAR)(rdsD & 0xFF);

    // Fill extracted characters and buffer offsets into primary and secondary arrays.
    if (offset < RDS_INFO_MAX_SIZE)
    {
      if (tempRDSBuffer[offset] == char1)
      {
        // 1st character verification is successful.
        rdsInfo[offset] = char1;
      }
      else if (isprint(char1))
      {
        buffer = useDoubleBuffer ? tempRDSBuffer : rdsInfo;
        buffer[offset] = char1;
      }

      if (tempRDSBuffer[offset + 1] == char2)
      {
        // 2nd character verification is successful.
        rdsInfo[offset + 1] = char2;
      }
      else if (isprint(char2))
      {
        buffer = useDoubleBuffer ? tempRDSBuffer : rdsInfo;
        buffer[offset + 1] = char2;
      }
    }
  }
}

VOID resetRDSInfo()
{
  // Fill primary RDS buffer with whitespaces.
  memset(rdsInfo, ' ', (RDS_INFO_MAX_SIZE - 1));
  rdsInfo[RDS_INFO_MAX_SIZE - 1] = 0x00;

  memset(tempRDSBuffer, ' ', (RDS_INFO_MAX_SIZE - 1));
  tempRDSBuffer[RDS_INFO_MAX_SIZE - 1] = 0x00;
}

void powerOff(void)
{
  u8g2.clearBuffer(); // clear the internal memory
  u8g2.setFont(u8g2_font_mercutio_basic_nbp_tf);
  u8g2.drawStr(0, 12, "STNDBY RX .."); // write something to the internal memory
  u8g2.sendBuffer();                   // transfer internal memory to the display
  shutdownTuner();
  delay(200); // delay
  u8g2.setFont(u8g2_font_mercutio_basic_nbp_tf);
  u8g2.drawStr(0, 24, "HI-Z GPIO ");  // write something to the internal memory
  u8g2.sendBuffer();                  // transfer internal memory to the display
  delay(100);                         // delay
  u8g2.drawStr(8 * 10, 24, "[OK]");   // write something to the internal memory
  delay(200);                         // пауза
  u8g2.sendBuffer();                  // transfer internal memory to the display
  u8g2.drawStr(0, 36, "Stop Timer "); // write something to the internal memory
  u8g2.sendBuffer();                  // transfer internal memory to the display
  delay(50);                          // пауза
  u8g2.drawStr(8 * 10, 36, "[OK]");   // write something to the internal memory
  delay(200);                         // пауза
  u8g2.sendBuffer();                  // transfer internal memory to the display
  u8g2.drawStr(0, 48, "POWER OFF ");  // write something to the internal memory
  u8g2.sendBuffer();                  // transfer internal memory to the display

  delay(400);                       // пауза для загрузки приемника
  u8g2.drawStr(8 * 10, 48, "[OK]"); // write something to the internal memory
  u8g2.sendBuffer();                // transfer internal memory to the display

  delay(400);          // пауза
  pinMode(PB0, INPUT); // вход кнопки POWER
}

void SetFreq(USHORT Freq)
{

  WriteReg(REG_CH, (Freq & 0xFF));             // Lo
  WriteReg(REG_CH_STEP, ((Freq >> 8) & 0x03)); // Hi
  // usleep(100);
  delayMicroseconds(100);

  temp1 = 1 << 4 | 1 << 3 | Stereo << 2 | 1 << 0;

  // temp1 = 0b00010101;
  WriteReg(REG_SYSTEM1, temp1);

  // Update global (default) frequency value.
  // currentFreq = Freq;
}

void WriteReg(int Addr, int Reg)
{
  Wire.beginTransmission(QN8035_ADDRESS); // transmit to device
  Wire.write(Addr);
  Wire.write(Reg);
  Wire.endTransmission(); // stop transmitting
}

int ReadReg(int Addr)
{
  int read;
  Wire.beginTransmission(QN8035_ADDRESS); // transmit to device
  Wire.write(Addr);
  Wire.endTransmission(); // stop transmitting

  Wire.beginTransmission(QN8035_ADDRESS); // transmit to device
  Wire.requestFrom(QN8035_ADDRESS, 1);
  while (Wire.available())
  {
    read = Wire.read();
  }
  Wire.endTransmission(); // stop transmitting
  return (read);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{
  RCC->APB1ENR = RCC_APB1ENR_TIM3EN;

  TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

  TIM3->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P;

  TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

  TIM3->ARR = 65535;

  TIM3->CNT = 32768;

  TIM3->CR1 = TIM_CR1_CEN; // Enable timer
}

VOID shutdownTuner()
{
  // Reset and recalibrate the receiver.
  WriteReg(REG_SYSTEM1, REG_SYSTEM1_RECAL | REG_SYSTEM1_SWRST);
  delayMicroseconds(100);

  // Enter tuner into the standby mode.
  WriteReg(REG_SYSTEM1, REG_SYSTEM1_STNBY);
}

void SetVol(int vol)
{
  UCHAR volReg;
  // Update volume control with new value.
  volReg = (GET_REG(REG_VOL_CTL) & 0xF8) | vol;
  WriteReg(REG_VOL_CTL, volReg);
}
