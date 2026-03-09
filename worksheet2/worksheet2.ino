#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SdFat.h>
#include <ArduinoJson.h>

LiquidCrystal_I2C lcd(0x27,16,2);
const uint8_t chipSelect = 5;
SdExFat sd;

struct Settings {
  float p;
  float i;
  float d;
  float sTemp;
  uint8_t sHum;
  bool proc;
  uint8_t hr;
  uint8_t min;
  uint8_t sec;
  uint8_t dd;
  uint8_t mm;
  unsigned int yy;
};

Settings myConfig;

const char* CONFIG_FILE = "settings.json";
//const char* INC_STATUS = "inc_status.json";

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
  doc["p"]     = data.p;
  doc["i"]     = data.i;
  doc["d"]     = data.d;
  doc["sTemp"] = data.sTemp;
  doc["sHum"]  = data.sHum;
  doc["proc"]  = data.proc;
  //doc["am"]    = data.am;
  doc["hr"]    = data.hr;
  doc["min"]   = data.min;
  doc["sec"]   = data.sec;
  doc["dd"] = data.dd;
  doc["mm"] = data.mm;
  doc["yy"] = data.yy;

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
  data.p = doc["p"];
  data.i = doc["i"];
  data.d = doc["d"];
  data.sTemp = doc["sTemp"];
  data.sHum = doc["sHum"];
  data.proc = doc["proc"];
  // data.am    = doc["am"];
  data.hr  = doc["hr"];
  data.min = doc["min"];
  data.sec = doc["sec"];
  data.dd = doc["dd"];
  data.mm = doc["mm"];
  data.yy = doc["yy"];

  return true;
}

// 3. DELETE File
void deleteFile(const char* path) {
  if (sd.exists(path)) {
    if (sd.remove(path)) Serial.println(F("Settings file deleted."));
  }
}

void printConfig(Settings s) {
  Serial.print("P: "); Serial.println(s.p);
  Serial.print("I: "); Serial.println(s.i);
  Serial.print("D: "); Serial.println(s.d);
  Serial.print("sTemp: "); Serial.println(s.sTemp);
  Serial.print("sHum: "); Serial.println(s.sHum);
  Serial.print("proc: "); Serial.println(s.proc);
  // Serial.print("am: "); Serial.println(s.am);
  Serial.print("Time: ");
  Serial.print(s.hr); Serial.print(":");
  Serial.print(s.min); Serial.print(":");
  Serial.println(s.sec);
  Serial.print("Date: ");
  Serial.print(s.dd); Serial.print("-");
  Serial.print(s.mm); Serial.print("-");
  Serial.println(s.yy);
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

bool setIncStatus()
{
  myConfig.proc = 1;
  myConfig.hr = 14;
  myConfig.min = 53;
  myConfig.sec = 25;
  myConfig.dd = 1;
  myConfig.mm = 1;
  myConfig.yy = 1;
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
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  if (!sd.begin(chipSelect, SD_SCK_MHZ(16)))
  {
    Serial.println("Sd card cannot be read");
     //return;
  }
  //deleteFile("settings.json");
  sd.ls(&Serial, LS_SIZE);
  // Initialize your struct
  myConfig = {1.5, 0.3, 0.05, 22.5, 45,0,13,45,53,13,4,2016};
  saveJson(CONFIG_FILE, myConfig);
  //Settings loadedConfig;
  //loadJson(CONFIG_FILE, loadedConfig);
  if(setIncStatus())
    Serial.println("inc set successful");
  else
    Serial.println("inc set failed");
  //saveJson(CONFIG_FILE, myConfig);
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

void loop() {}
