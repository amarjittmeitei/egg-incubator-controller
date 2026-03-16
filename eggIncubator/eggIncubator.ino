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

#define DEVICE_ID "AT010I"
#define SDCARD_LIFESPAN 3

struct Settings {
  float p;
  float i;
  float d;
  uint8_t sTemp1;
  uint8_t sHum1;
  uint8_t sTemp2;
  uint8_t sHum2;
  uint8_t csTemp1;
  uint8_t csHum1;
  uint8_t csTemp2;
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
enum FunctionType {NO_FUNC, FRESH_START, CUSTM_START, DEMO_START, AUTO_SETUP, COMP_TEST};

Settings myConfig;

RTC_DS1307 rtc;

const uint8_t chipSelect = 5;
const uint8_t ONE_WIRE_BUS = 4;

const uint8_t fanRelay = 13;
const uint8_t humidifierRelay = 14;
const uint8_t waterLevel = 33;
const uint8_t menuBott = 32;
const uint8_t selectBott = 35;
const uint8_t quickStartInc = 34;
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
FunctionType currentFunction = NO_FUNC;

const char* CONFIG_FILE = "settings.json";
const char* LOG_DIR = "/IncLogs";
const char* SET_LOG_DIR = "/SetLog";
const char* BRAND_NAME = "YAWOLART";

unsigned long BuzzPrevTime = 0;
bool buzzerState = false;
int beepCount = 0;
//int beepCycle = -1;
unsigned long lastTime = 0;
uint8_t scount = 0; //to swtich between tem and hum on main display
uint8_t mainDisplayWait = 0; //wait time to return to the main display after going throught the main menu
bool allowFunction = false; //allow a function to run

uint8_t menuPointer = 0;
uint8_t optionPointer = 0;
uint8_t selectPointer = 0;

// float temSetV1 = 30;
// float humSetV1 = 50;
// float temSetV2 = 27;
// float humSetV2 = 40;

void buzzerHandler();
void setError();
void setFunction();
void mainMenu();
void functionHandler();
float getTem();
float getHum();
bool setDate(bool);
bool setTime();
float getIncDay();
bool getWaterLevel();
bool sdCardWarning(bool mode = 0); // mode 1 - print data ; mode 0 - only return
double daysBetween(int y1,int m1,int d1,int h1,int min1,int y2,int m2,int d2,int h2,int min2);



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
  pinMode(buzz,OUTPUT);
  pinMode(motorIN1,OUTPUT);
  pinMode(motorIN2,OUTPUT);
  pinMode(motorEN,OUTPUT);
  pinMode(menuBott,INPUT_PULLUP);
  pinMode(waterLevel,INPUT_PULLUP); // this pin dont have internal pullup
  pinMode(selectBott,INPUT); //this pin dont have internal pullup
  pinMode(quickStartInc,INPUT); //this pin dont have internal pullup
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
  digitalWrite(buzz,HIGH);
  delay(1000);
  digitalWrite(buzz,LOW);
  // lcdPrint("input test","is running");
  // while(1)
  // {
  //   Serial.println();
  //   Serial.print("menuBott: "); Serial.println(digitalRead(menuBott));
  //   Serial.print("optionBoot: "); Serial.println(digitalRead(selectBott));
  //   Serial.print("quickStartInc: "); Serial.println(digitalRead(quickStartInc));
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
      myConfig = {1.5,0.3,0.05,37,55,37,70,0,0,0,0,17,2,0,0,0,"",1,0,0,0,0,0,0,0,0,3};
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
  //functionHandler();

  unsigned long currentTime = millis()/1000;
  now = rtc.now();


  if(myConfig.proc == 0) 
  {
    //hadling menu button press and its behavior 
    // !allowFunction -> dont allow to enter the main menu while a function is running
    if(!digitalRead(menuBott) && !allowFunction)
    {
      while(!digitalRead(menuBott)); // waiting for the botton to reslease;
      menuPointer = (menuPointer+1)%6;
      mainMenu();
      mainDisplayWait = 5;
    }

    //hadling select button press and its behavior 
    if(!digitalRead(selectBott))
    {
      while(!digitalRead(selectBott));
      allowFunction = true;
      functionHandler();
    }

    if(currentTime-lastTime >= 1)
    {
      lastTime = currentTime;

      if(allowFunction && currentFunction != NO_FUNC)
        functionHandler();

      // MAIN DISPLAY
      if(mainDisplayWait == 0 && !allowFunction)
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
        setFunction(NO_FUNC); //it the code reached here, there should be no active funciton running
        allowFunction = false;  
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

void setFunction(FunctionType type)
{
  if(currentFunction != type)
    currentFunction = type;
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
      setFunction(FRESH_START);
      break;

    case 2:
      lcdPrint("MENU: Incubation","-> Custom Start");
      setFunction(CUSTM_START);
      break;

    case 3:
      lcdPrint("MENU: Incubation","-> Demo Start");
      setFunction(DEMO_START);
      break;

    case 4:
      lcdPrint("MENU: Setup","-> Auto Setup");
      setFunction(AUTO_SETUP);
      break;

    case 5:
      lcdPrint("MENU: Run Test","-> ComponentTest");
      setFunction(COMP_TEST);
      break;

    default:
    {
      char tempLine2[17];
      snprintf(tempLine2,17,"DeviceID: %s", DEVICE_ID);
      lcdPrint(BRAND_NAME,tempLine2);
      setFunction(NO_FUNC);
    }
      
  }
}

void functionHandler()
{
  switch(currentFunction)
  {
    case NO_FUNC:
      lcdPrint("SSID:Inc_ART010I","IP:192.168.1.1");
      mainDisplayWait = 10;
      // dont allow function to run as NO_FUNC mean not be run any function.
      // this line of code for safety even if somewhere in the code enable the allowFunction by mistake.
      allowFunction = false; 
      break;
    
    case FRESH_START:
      lcdPrint("Inct running","Fresh start");
      setError(ERROR_SLOW);
      mainDisplayWait = 10;
      //while(1);
      break;

    case CUSTM_START:
      lcdPrint("Inct running","Custom start");
      setError(ERROR_RAPID);
      mainDisplayWait = 10;
      //while(1);
      break;

    case DEMO_START:
      lcdPrint("Inct running","Demo start");
      setError(ERROR_SINGLE);
      mainDisplayWait = 10;
      //while(1);
      break;

    case AUTO_SETUP:
      lcdPrint("Setup running","Auto setup");
      setError(ERROR_TRIPLE);
      mainDisplayWait = 10;
      //while(1);
      break;

    case COMP_TEST:
    {
      char tempLine2[17];
      float tempHum = getHum();
      snprintf(tempLine2,17,"-> Hum: %.2f",tempHum);
      lcdPrint("Test running",tempLine2);
      setError(ERROR_SINGLE);
      //mainDisplayWait = -1;
      if(tempHum<62)
        digitalWrite(humidifierRelay,HIGH);
      else if(tempHum>62)
        digitalWrite(humidifierRelay,LOW);
    }
      break;

    default:
      lcdPrint("Internal Err!","-> Invalid func");
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
