#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SdFat.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <RTClib.h>

struct Settings {
  float p;
  float i;
  float d;
  float sTemp;
  uint8_t sHum;
  uint8_t proc;
  char logFile[20];
  bool setup;
  bool motorCal;
  bool humSys;
  bool temSys;
  uint8_t hr;
  uint8_t min;
  //uint8_t sec;
  uint8_t dd;
  uint8_t mm;
  unsigned int yy;
  uint8_t sdd;
  uint8_t smm;
  unsigned int syy;
};

Settings myConfig;

RTC_DS1307 rtc;
DateTime now;
LiquidCrystal_I2C lcd(0x27,16,2);
const uint8_t chipSelect = 5;
SdExFat sd;

uint8_t scount = 0; //to swtich between tem and hum on main display

const char* CONFIG_FILE = "settings.json";
const char* LOG_DIR = "/logs";

unsigned long lastTime = 0;

uint8_t mpointer = 0;
uint8_t spointer = 0;
uint8_t mspointer = 0;

float temSetV1 = 30;
float humSetV1 = 50;
float temSetV2 = 27;
float humSetV2 = 40;

float getTem();
float getHum();
bool waterLevel();


// 1. CREATE or OVERRIDE (Modify)
bool saveJson(const char* path, const Settings& data) {
  ExFile file;
  // O_TRUNC wipes the file if it exists, allowing a clean override
  if (!file.open(path, O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.println(F("Error: Could not open for writing"));
    return 0;
  }

  StaticJsonDocument<256> doc;
 // doc["magic"] = data.magic;
  doc["id1"] = 0x4A;
  doc["p"]     = data.p;
  doc["i"]     = data.i;
  doc["d"]     = data.d;
  doc["sTemp"] = data.sTemp;
  doc["sHum"]  = data.sHum;
  doc["proc"]  = data.proc;
  doc["logFile"] = data.logFile;
  //doc["am"]    = data.am;
  doc["setup"] = data.setup;
  doc["motorCal"] = data.motorCal;
  doc["humSys"] = data.humSys;
  doc["temSys"] = data.temSys;
  doc["hr"]    = data.hr;
  doc["min"]   = data.min;
  //doc["sec"]   = data.sec;
  doc["dd"] = data.dd;
  doc["mm"] = data.mm;
  doc["yy"] = data.yy;
  doc["sdd"] = data.sdd;
  doc["smm"] = data.smm;
  doc["syy"] = data.syy;
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

  StaticJsonDocument<256> doc;
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
    data.sTemp = doc["sTemp"];
    data.sHum = doc["sHum"];
    data.proc = doc["proc"];
    strncpy(data.logFile,doc["logFile"],sizeof(doc["logFile"]));
    //data.logFile = doc["logFile"];
    // data.am    = doc["am"];
    data.setup = doc["setup"];
    data.motorCal = doc["motorCal"];
    data.humSys = doc["humSys"];
    data.temSys = doc["temSys"];
    data.hr  = doc["hr"];
    data.min = doc["min"];
    //data.sec = doc["sec"];
    data.dd = doc["dd"];
    data.mm = doc["mm"];
    data.yy = doc["yy"];
    data.sdd = doc["sdd"];
    data.smm = doc["smm"];
    data.syy = doc["syy"];
  }
  else
  {
    lcdPrint("Data corrupted!!","Restart device");
    while(true){} //for now fix the sd card manully restart the system
    // in future write auto fix algorithm here to override the corrupted data...
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
  Serial.print("P: "); Serial.println(s.p);
  Serial.print("I: "); Serial.println(s.i);
  Serial.print("D: "); Serial.println(s.d);
  Serial.print("sTemp: "); Serial.println(s.sTemp);
  Serial.print("sHum: "); Serial.println(s.sHum);
  Serial.print("proc: "); Serial.println(s.proc);
  Serial.print("logFile: "); Serial.println(s.logFile);
  // Serial.print("am: "); Serial.println(s.am);
  Serial.print("setup: "); Serial.println(s.setup);
  Serial.print("motorCal: "); Serial.println(s.motorCal);
  Serial.print("humSys: "); Serial.println(s.humSys);
  Serial.print("temSys: "); Serial.println(s.temSys);
  Serial.print("Time: ");
  Serial.print(s.hr); Serial.print(":");
  Serial.println(s.min);
  Serial.print("Date: ");
  Serial.print(s.dd); Serial.print("-");
  Serial.print(s.mm); Serial.print("-");
  Serial.println(s.yy);
  Serial.print("SD card Date: ");
  Serial.print(s.sdd); Serial.print("-");
  Serial.print(s.smm); Serial.print("-");
  Serial.println(s.syy);
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

bool setIncStatus(bool mode)
{
  myConfig.proc = !mode?1:2;
  myConfig.sTemp = !mode?temSetV1:temSetV2;
  myConfig.sHum = !mode?humSetV1:humSetV2;
  myConfig.hr = 14;
  myConfig.min = 53;
 // myConfig.sec = 25;
  myConfig.dd = 1;
  myConfig.mm = 6;
  myConfig.yy = 2023;
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

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);
  rtc.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcdPrint("YAWOLART","Eggs Incubator");

  if (!rtc.begin()) {
    Serial.println("RTC not found");
    lcdPrint("YAWOLART","RTC---NotWorking");
    //while (1);
  }
  else
  {
    lcdPrint("YAWOLART","RTC------Working");
  }
  delay(1000);
  if (!rtc.isrunning()) {
    Serial.println("RTC not running, setting time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  if (!sd.begin(chipSelect, SD_SCK_MHZ(16)))
  {
    Serial.println("Sd card cannot be read");
    lcdPrint("YAWOLART","SD Card-----Fail");
     //return;
  }
  else
  {
    lcdPrint("YAWOLART","SD Card-------OK");
    //deleteFile(CONFIG_FILE);
    delay(1000);
    if(sd.exists(CONFIG_FILE))
    {
      lcdPrint("YAWOLART","Data-----Fetched");
      loadJson(CONFIG_FILE,myConfig);
    }
    else
    {
      Serial.println("f----settings file created");
      lcdPrint("YAWOLART","Data-----Written");
      sd.ls(&Serial, LS_SIZE);
      // Initialize your struct
      myConfig = {1.5,0.3,0.05,37,60,0,"",1,0,0,0,15,58,11,3,2025,11,6,2024};
      saveJson(CONFIG_FILE, myConfig);

      // if(setIncStatus(0))
      //   Serial.println("inc set successful");
      // else
      //   Serial.println("inc set failed");
    }

    if(!sd.exists(LOG_DIR))
    {
      sd.mkdir(LOG_DIR);
      Serial.println("f----log directory created");
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
  



}

void loop() {

  unsigned long currentTime = millis()/1000;
  now = rtc.now();

  if(myConfig.proc == 0)
  {
    if(currentTime-lastTime >= 1)
    {
      
      char line1[16];
      char line2[16]; // = "DATE";
      if(myConfig.setup == 0)
      {
        snprintf(line1,16,"%d-%d-%4d  Setup",now.day(),now.month(),now.year());
      }
      else
      {
        snprintf(line1,16,"%d-%d-%4d  %c%c%c%c",now.day(),now.month(),now.year(),myConfig.motorCal?' ':'M',myConfig.humSys?' ':'H',myConfig.temSys?' ':'T','S');
      }
      if(scount <= 2) // show the tem on line2 for 3 second 
        snprintf(line2,16,"%d:%d:%d T%.2fC",now.hour(),now.minute(),now.second(),getTem());
      else // show the hum on line2 for 3 second after tem
        snprintf(line2,16,"%d:%d:%d H%.2f%%",now.hour(),now.minute(),now.second(),getHum());
      lcdPrint(line1,line2);
      scount = (scount+1)%6; //rotate the value of scount from 0 to 5; 0-2 for tem , 3-5 for hum
    }
    lastTime = currentTime;
  }
}

float getTem()
{
  //hardware for this fuction is not installed yet
  return 0.0;
}

float getHum()
{
  //hardware for this fuction is not installed yet
  return 0.0;
}

bool waterLevel()
{
  //hardware for this fuction is not installed yet
  return true;
}
