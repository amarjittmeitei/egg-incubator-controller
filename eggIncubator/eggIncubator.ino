#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SdFat.h>
#include <ArduinoJson.h>
//#include <EEPROM.h>

#define EEPROM_ADDR 0
#define MAGIC_NUMBER 0x55AA

LiquidCrystal_I2C lcd(0x27,16,2);

// CS pin for Arduino Uno is typically 10
const uint8_t chipSelect = 10;
const char* CONFIG_FILE = "settings.json";

// Use SdExFat for dedicated exFAT support to save memory on Uno
SdExFat sd;


unsigned long displayTime = 0;

uint8_t but1 = 6; //use to know the button is press - menu driven
uint8_t but2 = 7; //use to know the button is press - menu driven
uint8_t pulse = 0; //pwm for heater

byte but1Flag = 0;
byte but2Flag = 0;

struct Settings {
  uint16_t magic;   // validation marker
  float p;
  float i;
  float d;
  float sTemp;
  uint8_t sHum;
  bool proc;
  bool am;
  uint8_t hr;
  uint8_t min;
  uint8_t sec;
};

Settings config;

float readHum();
uint8_t readTemp();


// uint8_t writeConfigIfEmpty() {
//   Settings temp;
//   EEPROM.get(EEPROM_ADDR, temp);

//   if (temp.magic != MAGIC_NUMBER) {
//     Serial.println("First time setup. Writing defaults...");
//     config.magic = MAGIC_NUMBER;
//     config.p = 2.0;
//     config.i = 0.5;
//     config.d = 1.0;
//     config.sTemp = 35;
//     config.sHum = 65;
//     config.proc = false;
//     config.am = false;
//     config.hr = 12;
//     config.min = 0;
//     config.sec = 0;

//     EEPROM.put(EEPROM_ADDR, config);
//     return 0;
//   }
//   else {
//     Serial.println("Config already exists.");

//     return 1;
//   }
// }


// Settings readConfig() {
//   Settings temp;
//   EEPROM.get(EEPROM_ADDR, temp);

//   if (temp.magic == MAGIC_NUMBER) {
//     return temp;
//   } else {
//     Serial.println("Invalid config!");
//     return temp;   // caller must verify
//   }
// }


// void updateConfig(Settings newConfig) {
//   newConfig.magic = MAGIC_NUMBER;
//   EEPROM.put(EEPROM_ADDR, newConfig);
//   Serial.println("Config updated.");
// }

//for debug only
void printConfig(Settings s) {
  Serial.print("P: "); Serial.println(s.p);
  Serial.print("I: "); Serial.println(s.i);
  Serial.print("D: "); Serial.println(s.d);
  Serial.print("sTemp: "); Serial.println(s.sTemp);
  Serial.print("sHum: "); Serial.println(s.sHum);
  Serial.print("proc: "); Serial.println(s.proc);
  Serial.print("am: "); Serial.println(s.am);
  Serial.print("Time: ");
  Serial.print(s.hr); Serial.print(":");
  Serial.print(s.min); Serial.print(":");
  Serial.println(s.sec);
}

// // 1. CREATE or OVERRIDE (Modify)
void saveSettings(const char* path, const Settings& data) {
  ExFile file;
  // O_TRUNC wipes the file if it exists, allowing a clean override
  if (!file.open(path, O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.println(F("Error: Could not open for writing"));
    return;
  }

  StaticJsonDocument<256> doc;
  doc["magic"] = data.magic;
  doc["p"]     = data.p;
  doc["i"]     = data.i;
  doc["d"]     = data.d;
  doc["sTemp"] = data.sTemp;
  doc["sHum"]  = data.sHum;
  doc["proc"]  = data.proc;
  doc["am"]    = data.am;
  doc["hr"]    = data.hr;
  doc["min"]   = data.min;
  doc["sec"]   = data.sec;

  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Error: JSON write failed"));
  } else {
    Serial.println(F("Settings saved/overridden."));
  }
  file.close();
}

// 2. READ JSON into Struct
bool loadSettings(const char* path, Settings& data) {
  ExFile file;
  if (!file.open(path, O_RDONLY)) {
    Serial.println(F("Error: File not found"));
    return false;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println(F("Error: JSON Parse failed"));
    return false;
  }

  // Map values back to struct
  data.magic = doc["magic"];
  data.p     = doc["p"];
  data.i     = doc["i"];
  data.d     = doc["d"];
  data.sTemp = doc["sTemp"];
  data.sHum  = doc["sHum"];
  data.proc  = doc["proc"];
  data.am    = doc["am"];
  data.hr    = doc["hr"];
  data.min   = doc["min"];
  data.sec   = doc["sec"];

  return true;
}

// 3. DELETE File
void deleteSettings(const char* path) {
  if (sd.exists(path)) {
    if (sd.remove(path)) Serial.println(F("Settings file deleted."));
  }
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
  // put your setup code here, to run once:
  lcd.init();
  lcd.backlight();
  lcd.clear();

  Serial.begin(9600);
  pinMode(but1, INPUT_PULLUP);
  pinMode(but2, INPUT_PULLUP);

  //while (!Serial) {} // Wait for serial port to connect

  lcdPrint("YAWOLART", "Egg Incubator");
  // Initialize the SD card
  if (!sd.begin(chipSelect, SD_SCK_MHZ(16))) {
    Serial.println("Initialization failed! Check your wiring and CS pin.");
    lcdPrint("YAWOLART","SD card-----Fail");
    delay(1000);
  }
  else
  {
    lcdPrint("YAWOLART","SD Card-------OK");
    // Serial.println("Listing files and directories:");
    // Serial.println("------------------------------");

  // ls() parameters:
  // &Serial: Output stream
  // LS_R: Recursive (shows subdirectories)
  // LS_SIZE: Shows file sizes in bytes
    Settings myConfig = {0xBEEF, 1.5, 0.2, 0.05, 22.5, 45, true, false, 12, 30, 0};
    saveSettings(CONFIG_FILE, myConfig);
    sd.ls(&Serial, LS_R | LS_SIZE);
    Serial.println("------------------------------");
    Serial.println("Done.");
    //saveSettings("settings.json",config);
    Settings temps;
    loadSettings(CONFIG_FILE,temps);
    printConfig(temps);
    delay(1000);
  }


  // if(writeConfigIfEmpty())
  //   lcdPrint("YAWOLART","EEPROM-----Found");
  // else
  //   lcdPrint("YAWOLART","EEPROM--NotFound");
  // delay(1000);

  // config = readConfig();  // read existing config
  // printConfig(config);

  // // Example modification
  // config.p = 3.3;
  // config.proc = 0;
  // config.am = true;
  // config.hr = 0;
  // config.min = 0;
  // updateConfig(config);

  // Serial.println("After modification:");
  // config = readConfig();
  // printConfig(config);

  
  lcdPrint("YAWOLART","Press to start");
  while(digitalRead(but1) && digitalRead(but2)){} //waiting for any button to press
 
}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long currentTime = millis();

  if(config.proc)
  {

  }
  else if(!config.proc)
  {
    if(!digitalRead(but1))
    {
      but1Flag = (but1Flag+1) % 2;
      while(!digitalRead(but1)){}
    }

    if(digitalRead(but2))
    {
      but2Flag = (but2Flag+1) % 2;
      while(!digitalRead(but2)){}
    }


    if (currentTime/1000 - displayTime >= 1) 
    {

      lcdPrint("Yawolart","display check");

      displayTime = currentTime/1000;
    }
  }
  

}

float readHum()
{
  //sensor reading
  return 0.0;
}

uint8_t readTemp()
{
  //sensor reading
  return 0.0;
}

