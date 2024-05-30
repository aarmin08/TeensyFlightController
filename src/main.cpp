#include <Arduino.h>
#include <Wire.h>
#include <RF24.h>


// Create git repository for this btw

#include "SDFat.h"

#define alpha 0.5

// MPU declarations
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ, prevrolla, prevpitcha, gyroroll, gyropitch; 
float fusedrolla=0, fusedpitcha=0, uncertainroll=2, uncertainpitch=2; 
float prevAccRoll, prevAccPitch;
float RollAngle, PitchAngle, YawAngle; 
float RateCalbrationNumber; 
float RateCalRoll, RateCalPitch, RateCalYaw; 

float k1dout[] = {0,0}; // Prediction, uncertainty

float curtime, prevtime, elapsedtime; 
bool launched = false; 

#define CE 40
#define CSN 41
RF24 rf(CE, CSN); 

uint8_t address[6] = { "00001"};

const int chipSelect = 254; 
SdFat sd;
SdFile file;
SdFile file2; 

//BMP declarations
uint16_t digT1, digP1; 
int16_t dig_T2, dig_T3, dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9; 

float AltitudeBarometer, initAlt; 
int AltCal; 

//NRF 
struct DataTransmit {
  float ax,ay,az,rolla,pitcha,alt, maxAlt;
  int status; 
};
bool testing = false; 
DataTransmit data; 

int st= 1; 

//Some more stuff
float maxalt, prevAlt;
#define LEDPIN 33

void kal1d(float kalmanstate, float kalmanuncertainty, float input, float measurement) {
  kalmanstate = kalmanstate + 0.03 * input; 
  kalmanuncertainty = kalmanuncertainty + 0.03 * 0.03 * 4 * 4; 
  float gain = kalmanuncertainty * 1 / (1 * kalmanuncertainty + 3 * 3); 
  kalmanstate = kalmanstate + gain * (measurement-kalmanstate); 
  kalmanuncertainty = (1-gain) * kalmanuncertainty; 

  k1dout[0] = kalmanstate;
  k1dout[1] = kalmanuncertainty;
}

void LogEvent(String eventMessage) {
  file.close(); 
    if (file2.open("events.txt", O_RDWR | O_CREAT | O_AT_END)) {
      file2.println(eventMessage + ", AT: " + String(curtime/1000));
      file2.close();
    }
}

bool succesfulSDCardInitialization= true; 
void InitializeSDCard() {
  if (!sd.begin(SdioConfig(FIFO_SDIO))) {
    Serial.println("SD card initialization failed!");
    succesfulSDCardInitialization = false; 
    return;
  }

  succesfulSDCardInitialization = true; 

  if (file.exists("data.csv")) {
    file.remove("data.csv"); 
  } 

  if (file2.exists("events.txt")) {
    file2.remove("events.txt"); 
  }

  if (file.open("data.csv", O_RDWR | O_CREAT | O_AT_END)) {
    file.println("AX,AY,AZ,ROLLA,PITCHA,ALT,MAXALT");
    file.close();
  } else {
    Serial.println("Error opening file.");
  }
  if (file2.open("events.txt", O_RDWR | O_CREAT | O_AT_END)) {
    file2.println("Commence");
    file2.close();
  } else {
    Serial.println("Error opening file.");
  }
}


void SensorConfig() {
  Wire.beginTransmission(0x68); 
  Wire.write(0x1A); 
  Wire.write(0x05); 
  Wire.endTransmission(); 

  Wire.beginTransmission(0x68); 
  Wire.write(0x1C); 
  Wire.write(0x10);
  Wire.endTransmission();  
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); 
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  int16_t LSBX = Wire.read() << 8 | Wire.read(); 
  int16_t LSBY = Wire.read() << 8 | Wire.read(); 
  int16_t LSBZ = Wire.read() << 8 | Wire.read(); 

  Wire.beginTransmission(0x68);
  Wire.write(0x43); 
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6); 

  int16_t GyroX = Wire.read() << 8 | Wire.read(); 
  int16_t GyroY = Wire.read() << 8 | Wire.read(); 
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX/65.5 ; 
  RatePitch = (float)GyroY/65.5 ; 
  RateYaw = (float)GyroZ/65.5 ; 

  AccX = (float)LSBX/4096-0.05; 
  AccY = (float)LSBY/4096+0.01; 
  AccZ = (float)LSBZ/4096-0.11; 

  // Replace with  complementary filter
  RollAngle = atan(AccY/sqrt(AccX * AccX + AccZ*AccZ))*1/(3.142/180) ; // Convert Radians into degrees
  PitchAngle = -atan(AccX/sqrt(AccY*AccY + AccZ * AccZ)) * 1/(3.142/180);
}

void CalcOffsets() {
  for (RateCalbrationNumber = 0; RateCalbrationNumber < 2000; RateCalbrationNumber++) { 
    SensorConfig(); 
    RateCalRoll += RateRoll; 
    RateCalPitch += RatePitch; 
    RateCalYaw += RateYaw; 
  }

  RateCalRoll /= 2000; 
  RateCalPitch /= 2000; 
  RateCalYaw /= 2000; 
}

// TODO: Fix the damn SD card thign
void initmpu() {
  Wire.setClock(400000); 
  Wire.begin();
  delay(250); 
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B); 
  Wire.write(0x00); 
  Wire.endTransmission();
  CalcOffsets();
}


void PerformKalmanEquations() {
  kal1d(fusedrolla, uncertainroll, RateRoll, RollAngle); 
  fusedrolla = k1dout[0]; 
  uncertainroll = k1dout[1]; 
  kal1d(fusedpitcha, uncertainpitch, RatePitch, PitchAngle); 
  fusedpitcha = k1dout[0]; 
  uncertainpitch = k1dout[1]; 
}

float timeSinceLaunch; 

void checkforlaunch() {
  static float prevAcc = 0.0; // Initialize prevAcc and make it static to retain its value between function calls
  float curAcc = AccZ; 
  if (curAcc > 2 && prevAcc > 2) {
    st = 2;
    launched = true; 
    timeSinceLaunch = curtime; 
    LogEvent("LAUNCHED DETECT ACCELERATION: " + String(AccZ)); 
  }

  prevAcc = curAcc; 
}


void CheckForApogee() {
  if (AltitudeBarometer > prevAlt && launched) { 
    maxalt = AltitudeBarometer; 
  }
  if (AltitudeBarometer < maxalt) {
    Serial.println("EVENT: altitude reached");
    LogEvent("MAX ALT REACHED: " + String(maxalt) + " TIME FROM LAUNCH TO APOGEE: " + String(curtime - timeSinceLaunch));
    st = 3; 
  }
}

// BMP STUFF - I havent even the slightest idea what this shit does but it works and i copied a tutorial lol
void BmpConfig() {
  Wire.beginTransmission(0x76); 
  Wire.write(0xF7); 
  Wire.endTransmission(); 
  Wire.requestFrom(0x76, 7); 
  uint32_t press_msb = Wire.read(); 
  uint32_t press_lsb = Wire.read(); 
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read(); 
  uint32_t temp_lsb = Wire.read(); 
  uint32_t temp_xlsb = Wire.read(); 

  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4); 
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4); 

  signed long int var1, var2; 
  var1 = ((((adc_T >> 3) - ((signed long int)digT1 << 1))) * ((signed long int)dig_T2)) >> 11; 
  var2 = ((((adc_T >> 4)-((signed long int)digT1))) >> 12 * ((signed long int)dig_T3)) >> 14;

  signed long int t_fine = var1 + var2; 

  unsigned long int p; 
  var1 = (((signed long int)t_fine) >> 1)-(signed long int)64000; 
  var2 = (((var1 >> 2) * (var1>>2)) >> 11) * ((signed long int)dig_P6); 
  var2 = var2 + ((var1*((signed long int)dig_P5)) << 1); 
  var2 = (var2>>2) + (((signed long int)dig_P4)<<16); 

  var1 = (((dig_P3 * (((var1 >> 2) * (var1>>2))>>13))>>3)+((((signed long int)dig_P2)*var1)>>1))>>18; 
  var1 = ((((32768+var1))*((signed long int)digP1))>>15); 
  if (var1==0) {p=0;};
  p = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125; 
  if (p<0x80000000) {p = (p<<1) / ((unsigned long int) var1); } 
  else {p = (p/(unsigned long int)var1) * 2;}; 
  var1 = (((signed long int)dig_P9)*((signed long int)(((p>>3)*(p>>3))>>13)))>>12; 
  var2 = (((signed long int)(p>>2))*((signed long int)dig_P8))>>13; 
  p = (unsigned long int)((signed long int)p+((var1+var2+dig_P7)>>4)); 

  double pressure = (double)p/100; 
  AltitudeBarometer = 44330*(1-pow(pressure/1013.25, 1/5.255)); 
 }

void initBarometer() {
  Wire.beginTransmission(0x76); 
  Wire.write(0xF4);
  Wire.write(0xF7);
  Wire.endTransmission(); 

  Wire.beginTransmission(0x76); 
  Wire.write(0xF5);
  Wire.write(0x14);
  Wire.endTransmission(); 

  uint8_t data[24], i=0; 

  Wire.beginTransmission(0x76); 
  Wire.write(0x88); 
  Wire.endTransmission(); 
  Wire.requestFrom(0x76,24); 
  while (Wire.available()) { 
    data[i] = Wire.read(); 
    i++; 
  }

  digT1 = (data[1] << 8) | data[0]; 
  dig_T2 = (data[3] << 8) | data[2]; 
  dig_T3 = (data[5] << 8) | data[4]; 
  digP1 = (data[7] << 8) | data[6]; 
  dig_P2 = (data[9] << 8) | data[8]; 
  dig_P3 = (data[11] << 8) | data[10]; 
  dig_P4 = (data[13] << 8) | data[12]; 
  dig_P5 = (data[15] << 8) | data[14]; 
  dig_P6 = (data[17] << 8) | data[16]; 
  dig_P7 = (data[19] << 8) | data[18]; 
  dig_P8 = (data[21] << 8) | data[20]; 
  dig_P9 = (data[23] << 8) | data[22];
  delay(250); 

  for (AltCal = 0; AltCal < 2000; AltCal++) {
    BmpConfig();
    initAlt += AltitudeBarometer; 
    delay(1); 
  };  

  initAlt /= 2000; 
}

bool somethingwentwrong = false; 
void setup() {
  Serial.begin(57600);
  initmpu();
  initBarometer();
  pinMode(LEDPIN, OUTPUT); 

  if(!rf.begin()) {
    somethingwentwrong = true; 
    Serial.println("no wokr");
  }
  InitializeSDCard();

  if (succesfulSDCardInitialization == false) {
    somethingwentwrong = true; 
  }

  rf.openWritingPipe(address);
  rf.setPALevel(RF24_PA_MIN);
  rf.setDataRate(RF24_250KBPS);
  rf.stopListening();
  st=1;

  if (somethingwentwrong) {
    Serial.println("ERR");
    LogEvent("Error with either SD card or with NRF"); 
  }

  digitalWrite(LEDPIN, somethingwentwrong ? LOW: HIGH); 
  LogEvent("SETUP COMPLETE, ENTERING LOOP");
}

void UpdateSDCSVFile() {
  // open file, write to files 
  if (file.open("data.csv", O_RDWR | O_CREAT | O_AT_END)) {
      file.println(String(AccX) + "," + String(AccY) + "," + String(AccZ) + "," + String(fusedrolla) + "," +  String(fusedpitcha) + "," + String(AltitudeBarometer) + "," + String(maxalt));
      file.close();
  } else {
      LogEvent("Failed to load to DATA.CSV"); 
  }
} 

void loop() {
  SensorConfig();
  BmpConfig();
  PerformKalmanEquations(); 
  curtime=millis(); 
  
  RateRoll -= RateCalRoll; 
  RatePitch -= RateCalPitch; 
  RateYaw -= RateCalYaw; 

  UpdateSDCSVFile();

  switch (st) {
    case 1: 
      checkforlaunch(); 
      break; 
    case 2: 
    //NOTE: the max alt is only generated here after launch, so expect the max alt to be 0 until launch is detected
      CheckForApogee();
      break; 
    case 3: 
      digitalWrite(LEDPIN, HIGH); 
      delay(800); 
      digitalWrite(LEDPIN, LOW); 
      delay(800); 
      break; 
  }

  data.ax = AccX;
  data.ay= AccY;
  data.az = AccZ;
  data.rolla = fusedrolla; 
  data.pitcha = fusedpitcha;
  data.alt = AltitudeBarometer; 
  data.maxAlt = maxalt; 
  data.status = st; 
  
  rf.write(&data, sizeof(data));
  prevAlt = AltitudeBarometer; 
}


