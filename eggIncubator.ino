#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

#define EEPROM_ADDR 0
#define MAGIC_NUMBER 0x55AA

LiquidCrystal_I2C lcd(0x27,16,2);

#define SD_CS 8   
File file;

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
float readTemp();


uint8_t writeConfigIfEmpty() {
  Settings temp;
  EEPROM.get(EEPROM_ADDR, temp);

  if (temp.magic != MAGIC_NUMBER) {

    Serial.println("First time setup. Writing defaults...");

    config.magic = MAGIC_NUMBER;
    config.p = 2.0;
    config.i = 0.5;
    config.d = 1.0;
    config.sTemp = 35;
    config.sHum = 65;
    config.proc = true;
    config.am = false;
    config.hr = 12;
    config.min = 0;
    config.sec = 0;

    EEPROM.put(EEPROM_ADDR, config);
    return 0;
  }
  else {
    Serial.println("Config already exists.");
    return 1;
  }
}


Settings readConfig() {
  Settings temp;
  EEPROM.get(EEPROM_ADDR, temp);

  if (temp.magic == MAGIC_NUMBER) {
    return temp;
  } else {
    Serial.println("Invalid config!");
    return temp;   // caller must verify
  }
}


void updateConfig(Settings newConfig) {
  newConfig.magic = MAGIC_NUMBER;
  EEPROM.put(EEPROM_ADDR, newConfig);
  Serial.println("Config updated.");
}

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

  lcdPrint("YAWOLART", "Egg Incubator");
  if (!SD.begin(8)) {
    Serial.println("SD Card initialization failed!");
    lcdPrint("YAWOLART","SD card-----Fail");
    delay(1000);
  }
  else
  {
    Serial.println("SD Card initialized.");
    lcdPrint("YAWOLART","SD card-------OK");
    delay(1000);
  }


  if(writeConfigIfEmpty())
    lcdPrint("YAWOLART","EEPROM-----Found");
  else
    lcdPrint("YAWOLART","EEPROM--NotFound");
  delay(1000);

  // config = readConfig();  // read existing config
  // printConfig(config);

  // // Example modification
  // config.p = 3.3;
  // config.hr = 18;
  // updateConfig(config);

  // Serial.println("After modification:");
  // config = readConfig();
  // printConfig(config);

  
  lcdPrint("YAWOLART","Press to start");
 
}

void loop() {
  // put your main code here, to run repeatedly:
  Settings s;
  if(!but1Flag && !but2Flag && (!digitalRead(but1) || !digitalRead(but2)))
  {
    lcdPrint("A - Monitor Data", "B - Incubation");
    but1Flag = true;
    while(!digitalRead(but1) || !digitalRead(but2)){}
  }

  if((but1Flag == 1) && !digitalRead(but1))
  {
    while(but1Flag == 1)
    {
      float tTemp = readTemp();
      float tHum = readHum();
      //print to lcd
      if(!digitalRead(but1))
        but1Flag++;
    }

  }
}

float readHum()
{
  //sensor reading
  return 0.0;
}

float readTemp()
{
  //sensor reading
  return 0.0;
}

