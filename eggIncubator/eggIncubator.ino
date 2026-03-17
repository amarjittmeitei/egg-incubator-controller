#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SdFat.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SHT31.h>
#include <time.h>
#include <PID_v1.h>

#define DEVICE_ID "AT010I"
#define SDCARD_LIFESPAN 3

//HEATING PID SYS
#define WINDOW_SIZE_MS   1000    // burst-fire window (1s = 50 cycles @ 50Hz)
#define RELAY_STEP       (WINDOW_SIZE_MS * 0.80)  // 80% power during tune
#define NOISE_BAND       0.3     // °C — ignore oscillations smaller than this
#define TUNE_CYCLES      4       // oscillation cycles to average
#define TEMP_READ_MS     1000    // temperature read interval (ms)
#define LCD_UPDATE_MS    500     // LCD refresh interval (ms)

double setTemperature = 37.5;    // ← Egg incubator set-point (°C)

double currentTemperature = 0.0;
double pidOutput          = 0.0;
double Kp = 1.0, Ki = 0.1, Kd = 0.0;   // overwritten by autoTuner()
bool   heaterState        = false;

enum HeatPhase { PHASE_WARMUP, PHASE_AUTOTUNING, PHASE_RUNNING };
static HeatPhase heatPhase = PHASE_WARMUP;

// Auto-tune internals
static bool          relayOn      = false;
static double        lastPeakHigh = -999.0;
static double        lastPeakLow  =  999.0;
static int           peakCount    = 0;
static int           cycleCount   = 0;
static double        periodSum    = 0.0;
static double        amplitudeSum = 0.0;
static unsigned long lastPeakTime = 0;
static bool          lookForHigh  = true;
static double        tempHist[5]  = {0};
static int           histIdx      = 0;

// Timing
static unsigned long windowStart  = 0;
static unsigned long lastTempRead = 0;
static unsigned long lastLcdUpd   = 0;

PID heaterPID(&currentTemperature, &pidOutput, &setTemperature,Kp, Ki, Kd, DIRECT);

struct Settings {
  double p;
  double i;
  double d;
  float sTemp1;
  uint8_t sHum1;
  float sTemp2;
  uint8_t sHum2;
  float csTemp1;
  uint8_t csHum1;
  int csTemp2;
  uint8_t csHum2;
  uint8_t settingPhase;
  uint8_t hatchingPhase;
  uint8_t csettingPhase;
  uint8_t chatchingPhase;
  uint8_t proc;
  char logFile[20];
  bool setup;
  //bool motorCal;
  uint8_t motorPwm;
  uint8_t motorCycTime;
  uint8_t moveCount;
  bool humSys;
  bool temSys;
  uint16_t time;
  uint32_t date;
  //uint8_t sec;
  uint32_t sdate;
  uint8_t sSetAge;
};
//myConfig = {1.5,0.3,0.05,0,0,0,0,0,0,0,0,0,"",0,0,0,0,0,0,0};
enum ErrorType { NONE, ERROR_SLOW, ERROR_RAPID,ERROR_SINGLE, ERROR_TRIPLE };
enum OperationType {NO_OPT, FRESH_START, CUSTM_START, DEMO_START, AUTO_TUNE, COMP_TEST};

Settings myConfig;

RTC_DS1307 rtc;

const uint8_t chipSelect = 5;
const uint8_t ONE_WIRE_BUS = 4;

const uint8_t fanRelay = 13;
const uint8_t humidifierRelay = 14;
const uint8_t heater = 15;
const uint8_t waterLevel = 33;
const uint8_t menuBott = 32;
const uint8_t selectBott = 35;
const uint8_t accessibilityBott = 34;
const uint8_t limitSwitch1 = 36;
const uint8_t limitSwitch2 = 39;
const uint8_t buzz = 2;
const uint8_t motorIN1 = 25;
const uint8_t motorIN2 = 26;
const uint8_t motorEN = 27;

SdExFat sd;
DateTime now;
LiquidCrystal_I2C lcd(0x27,16,2);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
ErrorType currentError = NONE;
OperationType currentOperation = NO_OPT;

const char* CONFIG_FILE = "settings.json";
const char* LOG_DIR = "/IncLogs";
const char* SET_LOG_DIR = "/SetLog";
const char* BRAND_NAME = "YAWOLART";

unsigned long BuzzPrevTime = 0;
bool buzzerState = false;
int beepCount = 0;
unsigned long lastTime = 0;
uint8_t scount = 0; //to swtich between tem and hum on main display
uint8_t mainDisplayWait = 0; //wait time to return to the main display after going throught the main menu
bool allowOperation = false; //allow a Operation to run

//PID control variable
double Input, Output;

//heater time window
int WindowSize = 2000;             // 2 second window
unsigned long windowStartTime;


uint8_t menuPointer = 0;
uint8_t optionPointer = 0;
uint8_t selectPointer = 0;

// float temSetV1 = 30;
// float humSetV1 = 50;
// float temSetV2 = 27;
// float humSetV2 = 40;

void buzzerHandler();
void setError();
void setOperation();
void mainMenu();
void operationHandler();
float getTem();
float getHum();
bool setDate(bool);
bool setTime();
float getIncDay();
bool getWaterLevel();
bool sdCardWarning(bool mode = 0); // mode 1 - print data ; mode 0 - only return
bool accessibilityHandler();
double daysBetween(int y1,int m1,int d1,int h1,int min1,int y2,int m2,int d2,int h2,int min2);

static void runRelayTune();
static void finishTune();
static void updateLCD();

static void finishTune();
static void updateLCD();
static void runRelayTune();
void heatingSys();
void autoTuner();
void abortAutoTune();


// 1. CREATE or OVERRIDE (Modify)
bool saveJson(const char* path, const Settings& data) {
  ExFile file;
  // O_TRUNC wipes the file if it exists, allowing a clean override
  if (!file.open(path, O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.println(F("Error: Could not open for writing"));
    return 0;
  }

  StaticJsonDocument<512> doc;
 // doc["magic"] = data.magic;
  doc["id1"] = 0x4A;
  doc["p"]     = data.p;
  doc["i"]     = data.i;
  doc["d"]     = data.d;
  doc["sTemp1"] = data.sTemp1;
  doc["sHum1"]  = data.sHum1;
  doc["sTemp2"] = data.sTemp2;
  doc["sHum2"]  = data.sHum2;
  doc["csTemp1"] = data.csTemp1;
  doc["csHum1"]  = data.csHum1;
  doc["csTemp2"] = data.csTemp2;
  doc["csHum2"]  = data.csHum2;
  doc["settingPhase"] = data.settingPhase;
  doc["hatchingPhase"] = data.hatchingPhase;
  doc["csettingPhase"] = data.csettingPhase;
  doc["chatchingPhase"] = data.chatchingPhase;
  doc["proc"]  = data.proc;
  doc["logFile"] = data.logFile;
  //doc["am"]    = data.am;
  doc["setup"] = data.setup;
  doc["motorPwm"] = data.motorPwm;
  doc["motorCycTime"] = data.motorCycTime;
  doc["moveCount"] = data.moveCount;
  doc["humSys"] = data.humSys;
  doc["temSys"] = data.temSys;
  doc["time"]    = data.time;
  doc["date"]   = data.date;
  //doc["sec"]   = data.sec;
  doc["sdate"] = data.sdate;
  doc["sSetAge"] = data.sSetAge;
  doc["id2"] = 0xA4;

  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Error: JSON write failed"));
  } else {
    Serial.println(F("Settings saved/overridden."));
  }
  file.close();
  return 1;
}

// 2. READ JSON into Struct
bool loadJson(const char* path, Settings& data) {
  ExFile file;
  if (!file.open(path, O_RDONLY)) {
    Serial.println(F("Error: File not found"));
    return false;
  }

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println(error.c_str());
    return false;
  }

  // Map values back to struct
  //data.magic = doc["magic"];
  if(doc["id1"] == 0x4A && doc["id2"] == 0xA4)
  {
    data.p = doc["p"];
    data.i = doc["i"];
    data.d = doc["d"];
    data.sTemp1 = doc["sTemp1"];
    data.sHum1 = doc["sHum1"];
    data.sTemp2 = doc["sTemp2"];
    data.sHum2 = doc["sHum2"];
    data.csTemp1 = doc["csTemp1"];
    data.csHum1 = doc["csHum1"];
    data.csTemp2 = doc["csTemp2"];
    data.csHum2 = doc["csHum2"];\
    data.settingPhase = doc["settingPhase"];
    data.hatchingPhase = doc["hatchingPhase"];
    data.csettingPhase = doc["csettingPhase"];
    data.chatchingPhase = doc["chatchingPhase"];
    data.proc = doc["proc"];
    strncpy(data.logFile,doc["logFile"],sizeof(doc["logFile"]));
    //data.logFile = doc["logFile"];
    // data.am    = doc["am"];
    data.setup = doc["setup"];
    data.motorPwm = doc["motorPwm"];
    data.motorCycTime = doc["motorCycTime"];
    data.moveCount = doc["moveCount"];
    data.humSys = doc["humSys"];
    data.temSys = doc["temSys"];
    data.time  = doc["time"];
    data.date = doc["date"];
    //data.sec = doc["sec"];
    data.sdate = doc["sdate"];
    data.sSetAge = doc["sSetAge"];
  }
  else
  {
    lcdPrint("Data corrupted!!","Run AutoFix-(OK)");
    Serial.println("Sd card data corrupted!");
    //auto fix logic: delete the corrupted file and write the default setting
    while(true)
    {
      if(!digitalRead(selectBott))
      {
        deleteFile(CONFIG_FILE);
        lcdPrint("Auto fix done","Sys restarting..");
        ESP.restart();
        break;
      }
      delay(1000);
    } 
  }
  return true;
}

// 3. DELETE File
void deleteFile(const char* path) {
  if (sd.exists(path)) {
    if (sd.remove(path)) Serial.println(F("Settings file deleted."));
  }
}

bool checkFile(const char* filename) {
  if (sd.exists(filename)) {
    return true;
  }
  return false;
}

void printConfig(Settings s) {
  Serial.println("----Current configuration----");
  Serial.print("P: "); Serial.println(s.p);
  Serial.print("I: "); Serial.println(s.i);
  Serial.print("D: "); Serial.println(s.d);
  Serial.print("sTemp1: "); Serial.println(s.sTemp1);
  Serial.print("sHum1: "); Serial.println(s.sHum1);
  Serial.print("sTemp2: "); Serial.println(s.sTemp2);
  Serial.print("sHum2: "); Serial.println(s.sHum2);
  Serial.print("csTemp1: "); Serial.println(s.csTemp1);
  Serial.print("csHum1: "); Serial.println(s.csHum1);
  Serial.print("csTemp2: "); Serial.println(s.csTemp2);
  Serial.print("csHum2: "); Serial.println(s.csHum2);
  Serial.print("settingPhase: "); Serial.println(s.settingPhase);
  Serial.print("hatchingPhase: "); Serial.println(s.hatchingPhase);
  Serial.print("csettingPhase: "); Serial.println(s.csettingPhase);
  Serial.print("chatchingPhase: "); Serial.println(s.chatchingPhase);
  Serial.print("proc: "); Serial.println(s.proc);
  Serial.print("logFile: "); Serial.println(s.logFile);
  // Serial.print("am: "); Serial.println(s.am);
  Serial.print("setup: "); Serial.println(s.setup);
  Serial.print("motorPwm: "); Serial.println(s.motorPwm);
  Serial.print("moveCount: "); Serial.println(s.moveCount);
  Serial.print("humSys: "); Serial.println(s.humSys);
  Serial.print("temSys: "); Serial.println(s.temSys);
  Serial.print("Time: "); Serial.println(s.time);
  Serial.print("Date: "); Serial.println(s.date);
  Serial.print("SD card Date: "); Serial.println(s.sdate);
  Serial.print("SD card Set Age: "); Serial.println(s.sSetAge);
  Serial.println("-----------------------------");
}

// Time gettime()
// {
//   Time crtime;
//   crtime.proc = 0;
//   crtime.hr = 15;
//   crtime.min = 56;
//   crtime.sec = 34;
//   crtime.dd = 04;
//   crtime.mm = 03;
//   crtime.yy = 26;
//   return crtime;
// }

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);
  rtc.begin();
  sensors.begin();
  sensors.setResolution(12);

  Serial.println("-YAWOLART-");
  Serial.println("Booting up...");
  Serial.println("Device type: Incubator");
  Serial.print("Device ID: "); Serial.println(DEVICE_ID);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcdPrint(BRAND_NAME,"Eggs Incubator");
  //delay(1000);

  pinMode(fanRelay,OUTPUT);
  pinMode(humidifierRelay,OUTPUT);
  pinMode(heater,OUTPUT);
  pinMode(buzz,OUTPUT);
  pinMode(motorIN1,OUTPUT);
  pinMode(motorIN2,OUTPUT);
  pinMode(motorEN,OUTPUT);
  pinMode(menuBott,INPUT_PULLUP);
  pinMode(waterLevel,INPUT_PULLUP); // this pin dont have internal pullup
  pinMode(selectBott,INPUT); //this pin dont have internal pullup
  pinMode(accessibilityBott,INPUT); //this pin dont have internal pullup
  pinMode(limitSwitch1,INPUT); //this pin dont have interal pullup
  pinMode(limitSwitch2,INPUT); //this pin dont have internal pullup


  // //relay test start
  // Serial.println("relay test in progress...");
  // lcdPrint("relay test","in progress");
  // digitalWrite(fanRelay,HIGH);
  // digitalWrite(humidifierRelay,LOW);
  // delay(500);
  // digitalWrite(fanRelay,LOW);
  // delay(500);
  // digitalWrite(humidifierRelay,HIGH);
  // delay(500);
  // digitalWrite(humidifierRelay,LOW);
  // Serial.println("relay test ended.");
  // lcdPrint("Relay test","ended");
  // delay(1000);
  // //relay test end

  // //input pin test 

  digitalWrite(heater,LOW);
  digitalWrite(fanRelay,LOW);
  digitalWrite(humidifierRelay,LOW);
  digitalWrite(motorIN1,LOW);
  digitalWrite(motorIN2,LOW);
  digitalWrite(motorEN,LOW);

  digitalWrite(buzz,HIGH);
  delay(1000);
  digitalWrite(buzz,LOW);

  // lcdPrint("input test","is running");
  // while(1)
  // {
  //   Serial.println();
  //   Serial.print("menuBott: "); Serial.println(digitalRead(menuBott));
  //   Serial.print("optionBoot: "); Serial.println(digitalRead(selectBott));
  //   Serial.print("accessibilityBott: "); Serial.println(digitalRead(accessibilityBott));
  //   Serial.print("waterLevel: "); Serial.println(digitalRead(waterLevel));
  //   Serial.print("limitSwitch1: "); Serial.println(digitalRead(limitSwitch1));
  //   Serial.print("limitSwitch2: "); Serial.println(digitalRead(limitSwitch2));
  //   delay(1000);
  // }

  if (!sht31.begin(0x44)) 
  {
    Serial.println("SHT3x not found");
    lcdPrint(BRAND_NAME,"SHT3x not found");
  }

  sensors.requestTemperatures();
  delay(800);
  double t = sensors.getTempCByIndex(0);
  currentTemperature = (t != DEVICE_DISCONNECTED_C) ? t : 0.0;
  if (t == DEVICE_DISCONNECTED_C)
    Serial.println(F("WARNING: DS18B20 not found!"));
  

  if (!rtc.begin()) {
    Serial.println("RTC not found");
    lcdPrint(BRAND_NAME,"RTC---NotWorking");
    //while (1);
  }
  else
  {
    lcdPrint(BRAND_NAME,"RTC------Working");
    delay(1000);
    now = rtc.now();
    if (!rtc.isrunning()) {
      Serial.println("RTC not running, setting time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    if(now.year()<2026)
    {
      lcdPrint(BRAND_NAME,"Incorrect time!");
      delay(1000);
    }
  }
  
  
  if (!sd.begin(chipSelect, SD_SCK_MHZ(16)))
  {
    Serial.println("Sd card cannot be read");
    lcdPrint(BRAND_NAME,"SD Card-----Fail");
     //return;
  }
  else
  {
    lcdPrint(BRAND_NAME,"SD Card----Found");
    delay(1000);
    //deleteFile(CONFIG_FILE); //     <----------------------------DETELE FILE
    Serial.println("SD Card Tree: ");
    sd.ls(&Serial, LS_SIZE);
    if(sd.exists(CONFIG_FILE))
    {
      lcdPrint(BRAND_NAME,"Data-----Fetched");
      loadJson(CONFIG_FILE,myConfig);
    }
    else
    {
      Serial.println("f----settings file created");
      lcdPrint(BRAND_NAME,"Data-----Written");
      myConfig = {1.0,0.1,0,37.5,55,37.3,70,0,0,0,0,17,2,0,0,0,"",1,0,0,0,0,0,0,0,0,3};
      saveJson(CONFIG_FILE, myConfig);
    }

    if(myConfig.sdate == 0)
      setDate(0); // mode - 0 : set date for sd card, mode - 1 : set date for incubation start

    sdCardWarning(1); // sd card replacement warning check

    Serial.print("Incubation Status: ");
    if(myConfig.proc < 0)
    {
      Serial.println("READY");
      lcdPrint("Incubation","Starting...");
      delay(1000);
    }
    else if(myConfig.proc == 0)
      Serial.println("NO_INCUBATION");
    else
    {
      Serial.println("ACTIVE");
    }
    

    if(!sd.exists(LOG_DIR))
    {
      sd.mkdir(LOG_DIR);
      Serial.println("f----New log directory created");
    }

    if(!sd.exists(SET_LOG_DIR))
    {
      sd.mkdir(SET_LOG_DIR);
      Serial.println("f----New Setting log directory created");
    }
  }
  delay(1000);
  //deleteFile("settings.json");
  // sd.ls(&Serial, LS_SIZE);
  // // Initialize your struct
  // myConfig = {1.5, 0.3, 0.05, 22.5, 45,0,0,0,0,0,13,45,53,13,4,2016};
  // saveJson(CONFIG_FILE, myConfig);
  
  // if(setIncStatus())
  //   Serial.println("inc set successful");
  // else
  //   Serial.println("inc set failed");
  //saveJson(CONFIG_FILE, myConfig);
  // Settings loadedConfig;
  // loadJson(CONFIG_FILE, loadedConfig);
  printConfig(myConfig);
  
  //pid sys setup
  Kp = myConfig.p;
  Ki = myConfig.i;
  Kd = myConfig.d;


  heaterPID.SetOutputLimits(0, WINDOW_SIZE_MS);
  heaterPID.SetSampleTime(TEMP_READ_MS);
  heaterPID.SetMode(MANUAL);
  pidOutput = 0;

  windowStart  = millis();
  lastTempRead = millis() - TEMP_READ_MS;
  lastLcdUpd   = millis();
  heatPhase    = PHASE_WARMUP;
  
  
  // if (loadJson(CONFIG_FILE, loadedConfig)) {
  //   printConfig(loadedConfig);
  // // --- Test 1: Create ---
  // // saveJson(CONFIG_FILE, myConfig);

  // // --- Test 2: Modify & Override ---
  // // myConfig.sTemp = 25.0; // Change a value
  // // myConfig.proc = false;
  // // saveJson(CONFIG_FILE, myConfig);

  // // --- Test 3: Read ---
  
  // }

  // --- Test 4: List all files ---
  
  //currentError = ERROR_TRIPLE;


}

void loop() {

  buzzerHandler();
  accessibilityHandler();
  if(allowOperation && currentOperation == AUTO_TUNE)
  {
    autoTuner();
  }
  //operationHandler();

  unsigned long currentTime = millis()/1000;
  now = rtc.now();


  if(myConfig.proc == 0) 
  {
    //hadling menu button press and its behavior 
    // !allowOperation -> dont allow to enter the main menu while a Operation is running
    if(!digitalRead(menuBott) && !allowOperation)
    {
      while(!digitalRead(menuBott)); // waiting for the botton to reslease;
      menuPointer = (menuPointer+1)%5;
      mainMenu();
      mainDisplayWait = 5;
    }

    //hadling select button press and its behavior 
    if(!digitalRead(selectBott))
    {
      while(!digitalRead(selectBott));
      allowOperation = true;
      operationHandler();
    }

    if(currentTime-lastTime >= 1)
    {
      lastTime = currentTime;

      if(allowOperation && currentOperation != NO_OPT)
        operationHandler();

      // MAIN DISPLAY
      if(mainDisplayWait == 0 && !allowOperation)
      {
        char line1[17];
        char line2[17];
        if(myConfig.setup == 0)
          snprintf(line1,17,"%d-%d-%4d  Setup",now.day(),now.month(),now.year());
        else
        {
          snprintf(line1,17,"%d-%d-%4d  %c%c%c%c%c",now.day(),now.month(),now.year(),myConfig.motorPwm?' ':'M',myConfig.humSys?' ':'H',myConfig.temSys?' ':'T',sdCardWarning()?' ':'S',getWaterLevel()?' ':'W');
        }
        if(scount <= 2) // show the tem on line2 for 3 second 
          snprintf(line2,17,"%d:%d:%d T%.2fC",now.hour(),now.minute(),now.second(),getTem());
        else // show the hum on line2 for 3 second after tem
          snprintf(line2,17,"%d:%d:%d H%.2f%%",now.hour(),now.minute(),now.second(),getHum());
        lcdPrint(line1,line2);
        scount = (scount+1)%6; //rotating the value of scount from 0 to 5; 0-2 for tem , 3-5 for hum
        menuPointer = 0; //reset the menu pointer to zero to always start the menu from the beginning
        setError(NONE);
        setOperation(NO_OPT); //it the code reached here, there should be no active optiton running
        allowOperation = false;  
      }
      //calculating wait time to return to the main display mainDisplayWait should not > 10
      if(mainDisplayWait > 0 && mainDisplayWait <= 15)
        mainDisplayWait--;
      else
        mainDisplayWait == 0;
        // if(mainDisplayWait < 0)
        //   mainDisplayWait = 0;
      // else if(mainDisplayWait > 15)
      //   mainDisplayWait = 0;

      
    }
    
  }
}

float getHum()
{
  float hum = sht31.readHumidity();
  if (!isnan(hum))
      return hum;
  return -1;
}

float getTem()
{
  sensors.requestTemperatures();      
  float temp = sensors.getTempCByIndex(0);
  return temp;
}

bool getWaterLevel()
{
  //water level sensor should be atteched unside down
  if(digitalRead(waterLevel))
    return true;
  return false;
}

bool setDate(bool mode)
{
  now = rtc.now();
  uint32_t date = 0;
  if(mode)
  {
    date += now.year();
    date = (date*100) + now.month();
    date = (date*100) + now.day();
    myConfig.date = date;
    if(saveJson(CONFIG_FILE,myConfig))
      return 1;
    else 0;
  }

  else
  {
    date += now.year();
    date = (date*100) + now.month();
    date = (date*100) + now.day();
    myConfig.sdate = date;
    if(saveJson(CONFIG_FILE,myConfig))
      return 1;
    return 0;
  }
}

bool setTime()
{
  now = rtc.now();
  uint16_t time = 0;
  time += now.hour();
  time = (time*100) + now.minute();
  myConfig.time = time;
  if(saveJson(CONFIG_FILE,myConfig))
    return 1;
  return 0;
}

bool sdCardWarning(bool mode)
{
  now = rtc.now();
  bool warn = false;
  double age;
  uint32_t sdate = myConfig.sdate;
  uint16_t yr = 0;
  uint8_t mn = 0;
  uint8_t dy = 0;

  dy = sdate%100;
  sdate /=100;
  mn = sdate%100;
  sdate /= 100;
  yr = sdate;
  //Serial.print("calculated sdate: "); Serial.print(dy); Serial.print(":"); Serial.print(mn); Serial.print(":"); Serial.println(yr);
  age = daysBetween(yr,mn,dy,0,0,now.year(),now.month(),now.day(),0,0);
  
  if(mode){Serial.print("SD card age: "); Serial.print(age); Serial.println("days");}
  if(mode) Serial.print("SD card replacement warning: ");
  if(age > (365*myConfig.sSetAge))
  {
    if(mode) Serial.println("TRUE");
    return 0;
  }
  if(mode) Serial.println("FALSE");
  return 1;
}

float getIncDay()
{

  uint32_t date = myConfig.date;
  uint16_t time = myConfig.time;

  if(!date)
    return 0;

  now = rtc.now();
  float days;

  uint16_t yr = 0;
  uint8_t mn = 0;
  uint8_t dy = 0;

  uint8_t hr = 0;
  uint8_t min = 0;

  dy = date%100;
  date /=100;
  mn = date%100;
  date /= 100;
  yr = date;

  min = time%100;
  time /= 100;
  hr = time;

  days = daysBetween(yr,mn,dy,hr,min,now.year(),now.month(),now.day(),now.hour(),now.minute());
  return days;
}

void setError(ErrorType type) 
{
  if (currentError != type) 
  {
    currentError = type;
    beepCount = 0;
    BuzzPrevTime = millis();
  }
}

void setOperation(OperationType type)
{
  if(currentOperation != type)
    currentOperation = type;
}

bool setIncStatus(bool mode)
{
  //myConfig.proc = !mode?1:2;
//   myConfig.hr = 14;
//   myConfig.min = 53;
//  // myConfig.sec = 25;
//   myConfig.dd = 1;
//   myConfig.mm = 6;
//   myConfig.yy = 2023;

  now = rtc.now();
  unsigned long date = 0;
  unsigned int time = 0;

  date += now.year();
  date = (date*100) + now.month();
  date = (date*100) + now.day();
  Serial.print("inc set date: "); Serial.println(date);

  time += now.hour();
  time = (time*100) + now.minute();
  Serial.print("inc set time: "); Serial.println(time);

  myConfig.time = time;
  myConfig.date = date;

  if(saveJson(CONFIG_FILE,myConfig))
    return 1;
  else
    return 0;
}

bool accessibilityHandler()
{
  //unsigned long accessCurrentTime = millis()/1000;
  int i;
  char tempLine[17];
   if(myConfig.proc == 0)
   {
      if(allowOperation && !digitalRead(accessibilityBott))
      {
        digitalWrite(buzz,LOW);
        for(i = 3; i>0; i--)
        {
          Serial.print("Operation Abort in "); Serial.println(i);
          snprintf(tempLine,17,"in %d",i);
          lcdPrint("Operation abort",tempLine);
          delay(1000);
          if(digitalRead(accessibilityBott))
            break;
        } 
        if(i == 0)
        {
          allowOperation = false;
          setOperation(NO_OPT);
          Serial.println("Operation aborted by user");
          lcdPrint("Operation","aborted by user");
          while(!digitalRead(accessibilityBott));
          if(currentOperation == AUTO_TUNE)
            abortAutoTune();
          return 1;
        }
      }

      else if(!allowOperation && !digitalRead(accessibilityBott))
      {
        digitalWrite(buzz,LOW);
        for(i = 3; i>0; i--)
        {
          Serial.print("Incuabtion Start in "); Serial.println(i);
          snprintf(tempLine,17,"in %d",i);
          lcdPrint("Incubation start",tempLine);
          delay(1000);
          if(digitalRead(accessibilityBott))
            break;
        } 
        if(i == 0)
        {
          // write logic to start incubation;
          Serial.println("Incubation started");
          lcdPrint("Incubation","started");
          while(!digitalRead(accessibilityBott));
        }
      }
   }
  return 0;
}

void lcdPrint(String a, String b)
{
  lcd.clear();
  lcd.home();
  lcd.print(a);
  lcd.setCursor(0,1);
  lcd.print(b);
}


void mainMenu()
{
  switch(menuPointer)
  {
    case 1:
      lcdPrint("MENU: Incubation","-> Fresh Start");
      setOperation(FRESH_START);
      break;

    case 2:
      lcdPrint("MENU: Incubation","-> Custom Start");
      setOperation(CUSTM_START);
      break;

    case 3:
      lcdPrint("MENU: Incubation","-> Demo Start");
      setOperation(DEMO_START);
      break;

    case 4:
      lcdPrint("MENU: TuneHeater","-> Auto Tune");
      setOperation(AUTO_TUNE);
      break;

    case 5:
      lcdPrint("MENU: Run Test","-> ComponentTest");
      setOperation(COMP_TEST);
      break;

    default:
    {
      char tempLine2[17];
      snprintf(tempLine2,17,"DeviceID: %s", DEVICE_ID);
      lcdPrint(BRAND_NAME,tempLine2);
      setOperation(NO_OPT);
    }
      
  }
}

void operationHandler()
{
  switch(currentOperation)
  {
    case NO_OPT:
      lcdPrint("SSID:Inc_ART010I","IP:192.168.1.1");
      //mainDisplayWait = 10;
      // dont allow Operation to run as NO_OPT mean not be run any Operation.
      // this line of code for safety even if somewhere in the code enable the allowOperation by mistake.
      allowOperation = false; 
      break;
    
    case FRESH_START:
      lcdPrint("Inct running","Fresh start");
      setError(ERROR_SLOW);
      digitalWrite(heater,HIGH);
      //mainDisplayWait = 10;
      //while(1);
      break;

    case CUSTM_START:
      lcdPrint("Inct running","Custom start");
      setError(ERROR_RAPID);
      digitalWrite(heater,LOW);
      //mainDisplayWait = 10;
      //while(1);
      break;

    case DEMO_START:
      lcdPrint("Inct running","Demo start");
      setError(ERROR_TRIPLE);
      //mainDisplayWait = 10;
      //while(1);
      break;

    case AUTO_TUNE:
      // this operation need separate timing system
      // it have a separate handler on the main loop
      break;

    case COMP_TEST:
    {
      
    }
    break;

    default:
      lcdPrint("Internal Err!","-> Invalid opt");
      mainDisplayWait = 3;
  }
}

double daysBetween(int y1,int m1,int d1,int h1,int min1,int y2,int m2,int d2,int h2,int min2)
{
    struct tm t1 = {0};
    struct tm t2 = {0};

    t1.tm_year = y1 - 1900;
    t1.tm_mon  = m1 - 1;
    t1.tm_mday = d1;
    t1.tm_hour = h1;
    t1.tm_min  = min1;

    t2.tm_year = y2 - 1900;
    t2.tm_mon  = m2 - 1;
    t2.tm_mday = d2;
    t2.tm_hour = h2;
    t2.tm_min  = min2;

    time_t time1 = mktime(&t1);
    time_t time2 = mktime(&t2);

    double minutes = difftime(time2, time1) / 60.0;
    return minutes / 1440.0;   // minutes to days
}

void buzzerHandler() {
  unsigned long BuzzCurrentTime = millis();

  switch (currentError) {
    case ERROR_SLOW: // Steady slow beep
      if (BuzzCurrentTime - BuzzPrevTime >= 1000) {
        BuzzPrevTime = BuzzCurrentTime;
        buzzerState = !buzzerState;
        digitalWrite(buzz, buzzerState);
      }
      break;

    case ERROR_RAPID: // Fast alert beep
      if (BuzzCurrentTime - BuzzPrevTime >= 150) {
        BuzzPrevTime = BuzzCurrentTime;
        buzzerState = !buzzerState;
        digitalWrite(buzz, buzzerState);
      }
      break;
    
    case ERROR_SINGLE: // One 100ms beep followed by 3 seconds of silence
      if (buzzerState) {
        if (BuzzCurrentTime - BuzzPrevTime >= 100) { // ON duration
          digitalWrite(buzz, LOW);
          buzzerState = false;
          BuzzPrevTime = BuzzCurrentTime;
        }
      } else {
        if (BuzzCurrentTime - BuzzPrevTime >= 3000) { // OFF duration (3s)
          digitalWrite(buzz, HIGH);
          buzzerState = true;
          BuzzPrevTime = BuzzCurrentTime;
        }
      }
      break;

    case ERROR_TRIPLE: // Three quick beeps then a pause
      if (buzzerState) {
        if (BuzzCurrentTime - BuzzPrevTime >= 100) { // Beep ON duration
          digitalWrite(buzz, LOW);
          buzzerState = false;
          BuzzPrevTime = BuzzCurrentTime;
          beepCount++;
        }
      } else {
        int pause = (beepCount >= 3) ? 1000 : 100; // Long pause after 3 beeps
        if (BuzzCurrentTime - BuzzPrevTime >= pause) {
          if (beepCount >= 3) beepCount = 0;
          digitalWrite(buzz, HIGH);
          buzzerState = true;
          BuzzPrevTime = BuzzCurrentTime;
        }
      }
      break;

    case NONE:
    default:
      digitalWrite(buzz, LOW);
      buzzerState = false;
      break;
  }
}

//---------------------PID------------------
void autoTuner() {
  Serial.println(F("\n=== autoTuner() started ==="));

  // ── Phase 1: WARMUP — full power until within 5°C of target ──
  Serial.println(F("Phase: WARMUP (full power)"));
  while (heatPhase == PHASE_WARMUP) {
    if(accessibilityHandler()) return; // to abort the operation safely in between
    unsigned long now = millis();

    if (now - lastTempRead >= TEMP_READ_MS) {
      lastTempRead = now;
      sensors.requestTemperatures();
      double t = sensors.getTempCByIndex(0);
      if (t != DEVICE_DISCONNECTED_C) currentTemperature = t;

      pidOutput = WINDOW_SIZE_MS;   // full power

      Serial.printf("[WARMUP] T=%.2f / SP=%.1f\n",
                    currentTemperature, setTemperature);

      if (currentTemperature >= setTemperature - 5.0) {
        heatPhase    = PHASE_AUTOTUNING;
        relayOn      = false;
        pidOutput    = 0;
        lookForHigh  = true;
        peakCount    = 0;
        cycleCount   = 0;
        periodSum    = 0.0;
        amplitudeSum = 0.0;
        lastPeakTime = millis();
        histIdx      = 0;
        for (int i = 0; i < 5; i++) tempHist[i] = currentTemperature;
        lcd.clear();
        Serial.println(F("\nPhase: AUTOTUNING (relay oscillation)"));
      }
    }

    // SSR burst-fire output
    if (now - windowStart >= WINDOW_SIZE_MS) windowStart += WINDOW_SIZE_MS;
    bool newState = (pidOutput > (double)(now - windowStart));
    if (newState != heaterState) {
      heaterState = newState;
      digitalWrite(heater, heaterState ? HIGH : LOW);
    }

    // LCD
    if (now - lastLcdUpd >= LCD_UPDATE_MS) {
      lastLcdUpd = now;
      updateLCD();
    }

    
  }

  // ── Phase 2: AUTOTUNING — relay oscillation ──
  while (heatPhase == PHASE_AUTOTUNING) {
    if(accessibilityHandler()) return; // to abort the operation safely in between
    unsigned long now = millis();

    if (now - lastTempRead >= TEMP_READ_MS) {
      lastTempRead = now;
      sensors.requestTemperatures();
      double t = sensors.getTempCByIndex(0);
      if (t != DEVICE_DISCONNECTED_C) currentTemperature = t;

      runRelayTune();   // may call finishTune() which sets PHASE_RUNNING
    }

    // SSR output
    if (now - windowStart >= WINDOW_SIZE_MS) windowStart += WINDOW_SIZE_MS;
    bool newState = (pidOutput > (double)(now - windowStart));
    if (newState != heaterState) {
      heaterState = newState;
      digitalWrite(heater, heaterState ? HIGH : LOW);
    }

    // LCD
    if (now - lastLcdUpd >= LCD_UPDATE_MS) {
      lastLcdUpd = now;
      updateLCD();
    }
  }

  Serial.println(F("=== autoTuner() complete"));
  //add code to save to pid constants to the sd card;
}

void heatingSys() {
  unsigned long now = millis();
  
  // 0. Safe self-init guard
  // Runs once if autoTuner() was never called.
  // Switches PID to AUTOMATIC with a bumpless transfer:
  //   - reads current temperature first
  //   - seeds pidOutput to a safe small value (not 0, not full)
  //     so the integral starts calm with no overshoot spike
  if (heaterPID.GetMode() == MANUAL) {
    sensors.requestTemperatures();
    double t = sensors.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C) currentTemperature = t;

    // Seed output at 5% of window — just enough to start warming
    // gently, avoids full-power blast on first compute
    pidOutput = WINDOW_SIZE_MS * 0.05;

    heaterPID.SetMode(AUTOMATIC);   // bumpless transfer seeds integral here
    heatPhase    = PHASE_RUNNING;
    windowStart  = millis();
    lastTempRead = millis() - TEMP_READ_MS;
    lastLcdUpd   = millis();

    Serial.println(F("[heatingSys] "
                     "running with existing Kp/Ki/Kd gains."));
    Serial.printf("  Kp=%.4f  Ki=%.4f  Kd=%.4f\n", Kp, Ki, Kd);
  }

  // 1. Read temp & compute PID 
  if (now - lastTempRead >= TEMP_READ_MS) {
    lastTempRead = now;

    sensors.requestTemperatures();
    double t = sensors.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C) currentTemperature = t;

    heaterPID.Compute();

    Serial.printf("[RUN] T=%.2f SP=%.1f Out=%.0fms/%dms Heater=%s\n",
                  currentTemperature, setTemperature,
                  pidOutput, WINDOW_SIZE_MS, heaterState ? "ON" : "OFF");
  }

  // 2. Burst-fire SSR output 
  if (now - windowStart >= WINDOW_SIZE_MS) windowStart += WINDOW_SIZE_MS;
  bool newState = (pidOutput > (double)(now - windowStart));
  if (newState != heaterState) {
    heaterState = newState;
    digitalWrite(heater, heaterState ? HIGH : LOW);
  }

  // 3. LCD 
  if (now - lastLcdUpd >= LCD_UPDATE_MS) {
    lastLcdUpd = now;
    updateLCD();
  }
}

static void runRelayTune() {
  unsigned long now = millis();

  // Relay switch
  if (currentTemperature < setTemperature - NOISE_BAND) {
    if (!relayOn) { relayOn = true;  pidOutput = RELAY_STEP; }
  } else if (currentTemperature > setTemperature + NOISE_BAND) {
    if (relayOn)  { relayOn = false; pidOutput = 0; }
  }

  // Rolling 5-point history for peak detection
  tempHist[histIdx % 5] = currentTemperature;
  histIdx++;
  if (histIdx < 5) return;

  double prev2 = tempHist[(histIdx - 5 + 5) % 5];
  double prev1 = tempHist[(histIdx - 4 + 5) % 5];
  double curr  = tempHist[(histIdx - 3 + 5) % 5];
  double next1 = tempHist[(histIdx - 2 + 5) % 5];
  double next2 = tempHist[(histIdx - 1 + 5) % 5];

  bool isPeak   = (curr > prev1) && (curr > prev2) &&
                  (curr > next1) && (curr > next2) &&
                  (curr > setTemperature + NOISE_BAND);

  bool isValley = (curr < prev1) && (curr < prev2) &&
                  (curr < next1) && (curr < next2) &&
                  (curr < setTemperature - NOISE_BAND);

  if (lookForHigh && isPeak) {
    if (peakCount > 0) {
      double halfPeriod = (double)(now - lastPeakTime) / 1000.0;
      double amplitude  = (curr - lastPeakLow) / 2.0;

      if (halfPeriod > 0.5 && amplitude > NOISE_BAND) {
        periodSum    += halfPeriod * 2.0;
        amplitudeSum += amplitude;
        cycleCount++;

        Serial.printf("[TUNE] Cycle %d: Pu=%.2fs  a=%.3f degC\n",
                      cycleCount, halfPeriod * 2.0, amplitude);

        if (cycleCount >= TUNE_CYCLES) { finishTune(); return; }
      }
    }
    lastPeakHigh = curr;
    lastPeakTime = now;
    peakCount++;
    lookForHigh  = false;

  } else if (!lookForHigh && isValley) {
    lastPeakLow  = curr;
    lastPeakTime = now;
    peakCount++;
    lookForHigh  = true;
  }
}

void abortAutoTune() {
  // 1. Heater OFF immediately
  digitalWrite(heater, LOW);
  heaterState = false;
  pidOutput   = 0;

  // 2. PID gains — use partial data if available, else defaults
  if (cycleCount >= 1) {
    double avgPu  = periodSum    / cycleCount;
    double avgAmp = amplitudeSum / cycleCount;
    double Ku     = (4.0 * (RELAY_STEP / 2.0)) / (PI * avgAmp);
    Kp = 0.6 * Ku;
    double Ti = 0.5 * avgPu;
    double Td = 0.125 * avgPu;
    Ki = (Ti > 0) ? (Kp / Ti) : 0.0;
    Kd = Kp * Td;
    Serial.println(F("[ABORT] Using partial tune data."));
  } else {
    Kp = 1.0; Ki = 0.1; Kd = 0.0;
    Serial.println(F("[ABORT] No tune data — using default gains."));
  }

  // 3. Bumpless transfer into AUTOMATIC
  pidOutput = WINDOW_SIZE_MS * 0.05;
  heaterPID.SetTunings(Kp, Ki, Kd);
  heaterPID.SetMode(AUTOMATIC);

  // 4. State and timing
  heatPhase    = PHASE_RUNNING;
  windowStart  = millis();
  lastTempRead = millis() - TEMP_READ_MS;
  lastLcdUpd   = millis();

  Serial.printf("[ABORT] Kp=%.4f Ki=%.4f Kd=%.4f\n", Kp, Ki, Kd);
  Serial.println(F("[ABORT] autoTuner aborted — heatingSys() safe to run."));

  lcd.clear();
}

//  Internal: Compute gains, apply to PID
static void finishTune() {
  double avgPu  = periodSum    / cycleCount;
  double avgAmp = amplitudeSum / cycleCount;
  double Ku     = (4.0 * (RELAY_STEP / 2.0)) / (PI * avgAmp);

  Kp = 0.6  * Ku;
  double Ti = 0.5   * avgPu;
  double Td = 0.125 * avgPu;
  Ki = (Ti > 0) ? (Kp / Ti) : 0.0;
  Kd = Kp * Td;

  Serial.println(F("\n====== TUNE COMPLETE ======"));
  Serial.printf("  Pu=%.2fs  a=%.3f degC  Ku=%.4f\n", avgPu, avgAmp, Ku);
  Serial.printf("  Kp=%.4f  Ki=%.4f  Kd=%.4f\n", Kp, Ki, Kd);
  Serial.println(F("===========================\n"));

  //saving data to sd card
  myConfig.p = Kp;
  myConfig.i = Ki;
  myConfig.d = Kd;
  saveJson(CONFIG_FILE,myConfig);

  heaterPID.SetTunings(Kp, Ki, Kd);
  heaterPID.SetMode(AUTOMATIC);
  heatPhase = PHASE_RUNNING;

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("Tune Done! PID  "));
  char buf[17];
  snprintf(buf, sizeof(buf), "Kp=%.2f Ki=%.3f ", Kp, Ki);
  lcd.setCursor(0, 1); lcd.print(buf);
  delay(3000);
  lcd.clear();
}


//  Internal: LCD display
static void updateLCD() {
  char row0[17], row1[17];

  if (heatPhase == PHASE_WARMUP) {
    snprintf(row0, sizeof(row0), "WARMUP  SP:%5.1f", setTemperature);
    snprintf(row1, sizeof(row1), "CT:%5.1f        ", currentTemperature);

  } else if (heatPhase == PHASE_AUTOTUNING) {
    snprintf(row0, sizeof(row0), "TUNING  SP:%5.1f", setTemperature);
    snprintf(row1, sizeof(row1), "CT:%5.1f CYC:%d/%d",
             currentTemperature, cycleCount, TUNE_CYCLES);

  } else {
    snprintf(row0, sizeof(row0), "SP:%5.1f CT:%5.1f",
             setTemperature, currentTemperature);
    snprintf(row1, sizeof(row1), "OUT:%4.0fms %s  ",
             pidOutput, heaterState ? "[ON ]" : "[OFF]");
  }

  row0[16] = '\0'; row1[16] = '\0';
  lcd.setCursor(0, 0); lcd.print(row0);
  lcd.setCursor(0, 1); lcd.print(row1);
}

