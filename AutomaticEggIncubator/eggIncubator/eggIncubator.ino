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
#include <esp_task_wdt.h>
#define DEVICE_ID "AT010I"

//HEATING PID SYS
#define WINDOW_SIZE_MS   1000    // burst-fire window (1s = 50 cycles @ 50Hz)
#define RELAY_STEP       (WINDOW_SIZE_MS * 0.80)  // 80% power during tune
#define NOISE_BAND       0.3     // °C — ignore oscillations smaller than this
#define TUNE_CYCLES      4       // oscillation cycles to average
#define TEMP_READ_MS     1000    // temperature read interval (ms)
#define LCD_UPDATE_MS    500     // LCD refresh interval (ms)

double setTemperature = 37.5;    // value is to be use by auto tuner

double currentTemperature = 0.0;
double pidOutput          = 0.0;
double Kp = 1.0, Ki = 0.1, Kd = 0.0;   // value is to be use by auto tunner
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

int incLcdCount = 0;

#define HYST_LOW          2.0    // %RH below set-point → turn ON
#define HYST_HIGH         3.0    // %RH above set-point → turn OFF
#define MIN_ON_TIME_MS    10000  // relay stays ON  minimum 10s
#define MIN_OFF_TIME_MS   15000  // relay stays OFF minimum 15s
#define HUMID_READ_MS     2000   // read interval (ms)

double setHumidity    = 60.0;

double currentHumidity = 0.0;   // updated every HUMID_READ_MS
bool   humidifierState = false;  // true = relay energised = humidifier ON

static unsigned long humidLastRead     = 0;
static unsigned long relayLastSwitched = 0;

static void applyRelay(bool turnOn);

PID heaterPID(&currentTemperature, &pidOutput, &setTemperature,Kp, Ki, Kd, DIRECT);

struct Settings {
  double p;
  double i;
  double d;
  bool safety;
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
  uint8_t humWarn;
  uint8_t temWarn;
  int proc;
  char logFile[36];
  bool wifi;
  //bool motorCal;
  uint8_t motorPwm;
  uint8_t moveInterval;
  uint16_t lastMoveTime;
  uint32_t lastMoveDate;
  uint16_t motorTimeOut;
  bool trayStopDone;// = false;
  bool trayCentred;//  = false;
  double lastPidOutput;// = 0.0;  // saved every 30s, restored on power-failure bumpless restart
  double lastTem;// = 0;
  float restartErr;
  uint16_t time;
  uint32_t date;
  //uint8_t sec;
  uint32_t sdate;
  uint8_t sSetAge;
};
//myConfig = {1.0,0.1,0,1,33.5,70,37.3,70,0,0,0,0,17,2,0,0,10,15,0,"",0,0,0,0,30,0,0,0,3};
//myConfig = {1.5,0.3,0.05,0,0,0,0,0,0,0,0,0,"",0,0,0,0,0,0,0};
enum ErrorType { NONE, ERROR_SLOW, ERROR_RAPID,ERROR_SINGLE, ERROR_TRIPLE };
enum OperationType {NO_OPT, FRESH_START, CUSTM_START, SAFETY, AUTO_TUNE};

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
const char* LOG_DIR = "/Logs";
const char* SET_LOG_DIR = "/SetLog";
const char* BRAND_NAME = "YAWOLART";

// ── LEDC (motor PWM) ────────────────────────────────────────
#define MOTOR_LEDC_FREQ      1000   // 1 kHz PWM
#define MOTOR_LEDC_RES       8      // 8-bit resolution (0-255)
// Motor timeout: user-configurable via myConfig.motorTimeOut (seconds)
// Falls back to 30 s if not set
#define MOTOR_TIMEOUT_MS  ((unsigned long)(myConfig.motorTimeOut ? myConfig.motorTimeOut : 30) * 1000UL)

// ── Motor state ──────────────────────────────────────────────
enum MotorState { MOTOR_IDLE, MOTOR_MOVING, MOTOR_CENTRING };
static MotorState        motorState     = MOTOR_IDLE;
static bool              trayAtFront    = true;
static unsigned long     motorWarnClearMs = 0;  // motor timeout warning auto-clear
// try to save on sd card
// static bool              trayCentred    = false;
// static bool              trayStopDone   = false;
static unsigned long     motorMoveStart = 0;
static unsigned long     fullTravelMs   = 0;

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
bool bumpLessRestart = false;


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
int getIncDay();
bool getWaterLevel();
bool sdCardWarning(bool mode = 0); // mode 1 - print data ; mode 0 - only return
bool accessibilityHandler();
double daysBetween(int y1,int m1,int d1,int h1,int min1,int y2,int m2,int d2,int h2,int min2, bool mode = 0);

static void runRelayTune();
static void finishTune();
static void updateLCD();

static void finishTune();
static void updateLCD();
static void runRelayTune();
void heatingSys();
void autoTuner();
void abortAutoTune();

void humiditySys();
static void applyRelay(bool turnOn);

bool initLogFile(const char*, const char*);
bool appendCSV(const char* , const char* );
void motorHandler();
void centreTrayStart();
bool centreTrayTick();

void fanSys(bool);



// ============================================================
//  WEB SERVER
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>   // already included in main .ino

// ── AP credentials ──────────────────────────────────────────
static const char* AP_SSID     = "Inc_ART010I";
static const char* AP_PASSWORD = "";            // open network
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_SUBNET(255, 255, 255, 0);

// ── Web server on port 80 ───────────────────────────────────
WebServer webServer(80);
bool wifiEnabled = false;

// ── SD path for the web UI ──────────────────────────────────
static const char* WEB_ROOT = "/www/index.html";

// ── Forward declarations ────────────────────────────────────
static void handleRoot();
static void handleGetSettings();
static void handlePostSettings();
static void handleGetLive();
static void handleSyncTime();
static void handleStartInc();
static void handleStopInc();
static void handleNotFound();
static void handleGetLogs();
static void handleDownloadLog();
static void addCORSHeaders();
static void sendSettingsJson();
static bool  applySettingsJson(const String& body);

// ============================================================
//  webServerSetup()  —  call once from setup() in main .ino
// ============================================================
void webServerSetup() {
  // ── 1. Start Access Point ─────────────────────────────────
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_SUBNET);

  if (strlen(AP_PASSWORD) > 0)
    WiFi.softAP(AP_SSID, AP_PASSWORD);
  else
    WiFi.softAP(AP_SSID);

  Serial.print(F("[WEB] AP started  SSID: "));
  Serial.println(AP_SSID);
  Serial.print(F("[WEB] AP IP: "));
  Serial.println(WiFi.softAPIP());

  // ── 2. Register routes ────────────────────────────────────
  webServer.on("/",               HTTP_GET,  handleRoot);
  webServer.on("/api/settings",   HTTP_GET,  handleGetSettings);
  webServer.on("/api/settings",   HTTP_POST, handlePostSettings);
  webServer.on("/api/live",       HTTP_GET,  handleGetLive);
  webServer.on("/api/synctime",   HTTP_POST, handleSyncTime);
  webServer.on("/api/startinc",   HTTP_POST, handleStartInc);
  webServer.on("/api/stopinc",    HTTP_POST, handleStopInc);
  webServer.on("/api/logs",              HTTP_GET,  handleGetLogs);
  webServer.on("/api/logs/download",     HTTP_GET,  handleDownloadLog);
  webServer.onNotFound(handleNotFound);

  // Handle OPTIONS preflight for CORS (browser sometimes sends these)
  webServer.on("/api/settings",   HTTP_OPTIONS, [](){
    addCORSHeaders();
    webServer.send(204);
  });
  webServer.on("/api/synctime",   HTTP_OPTIONS, [](){
    addCORSHeaders();
    webServer.send(204);
  });
  webServer.on("/api/startinc",   HTTP_OPTIONS, [](){
    addCORSHeaders();
    webServer.send(204);
  });
  webServer.on("/api/stopinc",    HTTP_OPTIONS, [](){
    addCORSHeaders();
    webServer.send(204);
  });

  webServer.begin();
  wifiEnabled = true;
  Serial.println(F("[WEB] HTTP server started on port 80"));
}

// ============================================================
//  webServerLoop()  —  call every loop() iteration in main .ino
// ============================================================
void webServerLoop() {
  if (wifiEnabled) webServer.handleClient();
}

void wifiOn() {
  if (wifiEnabled) return;
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_SUBNET);
  if (strlen(AP_PASSWORD) > 0)
    WiFi.softAP(AP_SSID, AP_PASSWORD);
  else
    WiFi.softAP(AP_SSID);
  webServer.begin();
  wifiEnabled = true;
  Serial.println(F("[WEB] Wi-Fi ON"));
}

void wifiOff() {
  if (!wifiEnabled) return;
  webServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiEnabled = false;
  Serial.println(F("[WEB] Wi-Fi OFF"));
}

// ============================================================
//  HANDLERS
// ============================================================

// ── GET /  ──────────────────────────────────────────────────
// Serves /www/index.html from SD card.
// Falls back to a minimal inline redirect page if SD is absent.
static void handleRoot() {
  addCORSHeaders();

  if (!sd.exists(WEB_ROOT)) {
    // SD not available or UI file missing
    webServer.send(200, "text/html",
      "<!DOCTYPE html><html><body style='font-family:monospace;background:#0d0f14;color:#e84040;"
      "display:flex;align-items:center;justify-content:center;height:100vh;margin:0'>"
      "<div style='text-align:center'><h2>SD CARD NOT FOUND</h2>"
      "<p>Insert SD card and restart device</p></div></body></html>");
    return;
  }

  // Stream the HTML file from SD
  ExFile htmlFile;
  if (!htmlFile.open(WEB_ROOT, O_RDONLY)) {
    webServer.send(500, "text/plain", "Failed to open UI file");
    return;
  }

  // Use chunked streaming so we never load the whole file into RAM
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "text/html", "");

  const size_t BUF = 512;
  uint8_t buf[BUF];
  int n;
  while ((n = htmlFile.read(buf, BUF)) > 0) {
    //esp_task_wdt_reset();
    webServer.sendContent_P((const char*)buf, n);
  }
  htmlFile.close();
  webServer.sendContent("");   // signal end of chunked body
}

// ── GET /api/settings  ──────────────────────────────────────
static void handleGetSettings() {
  addCORSHeaders();
  sendSettingsJson();
}

// ── POST /api/settings  ─────────────────────────────────────
// Accepts a JSON body with any subset of the Settings fields.
// Only the fields present in the JSON are updated; the rest
// keep their current myConfig values.  Saves to SD after.
// Blocked while incubation is active to protect running state.
static void handlePostSettings() {
  addCORSHeaders();

  if (webServer.method() == HTTP_OPTIONS) {
    webServer.send(204);
    return;
  }

  if (!webServer.hasArg("plain")) {
    webServer.send(400, "application/json", "{\"error\":\"No body\"}");
    return;
  }

  const String& body = webServer.arg("plain");

  if (!applySettingsJson(body)) {
    webServer.send(400, "application/json", "{\"error\":\"Parse failed\"}");
    return;
  }

  if (!saveJson(CONFIG_FILE, myConfig)) {
    webServer.send(500, "application/json", "{\"error\":\"SD write failed\"}");
    return;
  }

  // Apply PID tunings live if they were changed
  Kp = myConfig.p;
  Ki = myConfig.i;
  Kd = myConfig.d;
  heaterPID.SetTunings(Kp, Ki, Kd);

  webServer.send(200, "application/json", "{\"ok\":true}");
  Serial.println(F("[WEB] Settings updated via web UI"));
}

// ── GET /api/live  ──────────────────────────────────────────
static void handleGetLive() {
  addCORSHeaders();

  // Lightweight SD check — avoid full sd.begin() on every live poll
  bool sdOk = sd.exists(CONFIG_FILE);

  // Read RTC for current time/date strings
  now = rtc.now();
  char timeBuf[6];   // HH:MM
  char dateBuf[12];  // YYYY-MM-DD
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", now.hour(), now.minute());
  snprintf(dateBuf, sizeof(dateBuf), "%04d-%02d-%02d",
           now.year(), now.month(), now.day());

  // Today's date as YYYYMMDD integer — used by the web UI for day counting
  uint32_t todayInt = (uint32_t)now.year()  * 10000UL
                    + (uint32_t)now.month() * 100UL
                    + now.day();

  // Build JSON manually to avoid heap fragmentation on small payloads
  char json[256];
  snprintf(json, sizeof(json),
    "{\"temperature\":%.2f,\"humidity\":%.2f,"
    "\"sdOk\":%s,\"waterLevel\":%s,"
    "\"heaterOn\":%s,\"humidifierOn\":%s,"
    "\"time\":\"%s\",\"date\":\"%s\","
    "\"rtcTime\":%u,\"rtcDate\":%lu}",
    currentTemperature, currentHumidity,
    sdOk            ? "true" : "false",
    getWaterLevel() ? "false" : "true",
    heaterState       ? "true" : "false",
    humidifierState   ? "true" : "false",
    timeBuf, dateBuf,
    myConfig.time, (unsigned long)todayInt
  );

  webServer.send(200, "application/json", json);
}

// ── POST /api/synctime  ─────────────────────────────────────
// Body: { "time": HHMM, "date": YYYYMMDD }
// Decodes integers → sets RTC → updates myConfig → saves SD.
static void handleSyncTime() {
  addCORSHeaders();

  if (!webServer.hasArg("plain")) {
    webServer.send(400, "application/json", "{\"error\":\"No body\"}");
    return;
  }

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, webServer.arg("plain"));
  if (err) {
    webServer.send(400, "application/json", "{\"error\":\"Bad JSON\"}");
    return;
  }

  uint16_t newTime = doc["time"].as<uint16_t>(); // HHMM
  uint32_t newDate = doc["date"].as<uint32_t>(); // YYYYMMDD

  if (newTime == 0 && newDate == 0) {
    webServer.send(400, "application/json", "{\"error\":\"Invalid time/date\"}");
    return;
  }

  // Decode HHMM → hours, minutes
  uint8_t hr  = newTime / 100;
  uint8_t min = newTime % 100;

  // Decode YYYYMMDD → year, month, day
  uint32_t d  = newDate;
  uint8_t  dy = d % 100; d /= 100;
  uint8_t  mo = d % 100; d /= 100;
  uint16_t yr = (uint16_t)d;

  // Basic sanity
  if (yr < 2026 || mo < 1 || mo > 12 || dy < 1 || dy > 31 ||
      hr > 23 || min > 59) {
    webServer.send(400, "application/json", "{\"error\":\"Out-of-range values\"}");
    return;
  }

  // Adjust the RTC
  rtc.adjust(DateTime(yr, mo, dy, hr, min, 0));

  // Update myConfig and persist
  myConfig.time = newTime;
  myConfig.date = newDate;
  if (!saveJson(CONFIG_FILE, myConfig)) {
    webServer.send(500, "application/json", "{\"error\":\"SD write failed\"}");
    return;
  }

  char msg[128];
  snprintf(msg, sizeof(msg),
    "{\"ok\":true,\"synced\":\"%04d-%02d-%02d %02d:%02d\"}",
    yr, mo, dy, hr, min);
  webServer.send(200, "application/json", msg);

  Serial.printf("[WEB] RTC synced to %04d-%02d-%02d %02d:%02d\n",
                yr, mo, dy, hr, min);
}

// ── POST /api/startinc  ─────────────────────────────────────
// Body: { "mode": 1 }   1=fresh, 2=custom
// Sets proc = -(mode) so the main setup() safety check runs
// on next boot, then restarts.
// Blocked if incubation is already running.
static void handleStartInc() {
  addCORSHeaders();

  if (myConfig.proc > 0) {
    webServer.send(409, "application/json",
      "{\"error\":\"Incubation already active\"}");
    return;
  }

  if (!webServer.hasArg("plain")) {
    webServer.send(400, "application/json", "{\"error\":\"No body\"}");
    return;
  }

  StaticJsonDocument<64> doc;
  if (deserializeJson(doc, webServer.arg("plain"))) {
    webServer.send(400, "application/json", "{\"error\":\"Bad JSON\"}");
    return;
  }

  int mode = doc["mode"].as<int>();
  if (mode != 1 && mode != 2) {
    webServer.send(400, "application/json",
      "{\"error\":\"mode must be 1 (fresh) or 2 (custom)\"}");
    return;
  }

  // Custom mode requires csettingPhase > 0
  // if (mode == 2 && myConfig.csettingPhase == 0) {
  //   webServer.send(400, "application/json",
  //     "{\"error\":\"Custom setting phase days must be > 0\"}");
  //   return;
  // }

  webServer.send(200, "application/json",
    "{\"ok\":true,\"message\":\"Restarting to start incubation\"}");

  delay(200);   // let response flush

  // Negative proc signals the startup handler to begin incubation
  myConfig.proc = -mode;
  saveJson(CONFIG_FILE, myConfig);

  Serial.printf("[WEB] Incubation start requested  mode=%d — restarting\n",
                mode);
  delay(300);
  ESP.restart();
}

// ── POST /api/stopinc  ──────────────────────────────────────
// Aborts the running incubation: writes [WEB ABORT] to log,
// sets proc=0, saves, restarts.
static void handleStopInc() {
  addCORSHeaders();

  if (myConfig.proc == 0) {
    webServer.send(409, "application/json",
      "{\"error\":\"No incubation is running\"}");
    return;
  }

  webServer.send(200, "application/json",
    "{\"ok\":true,\"message\":\"Incubation stopped. Restarting.\"}");

  delay(200);

  myConfig.proc = 0;
  saveJson(CONFIG_FILE, myConfig);
  appendCSV(myConfig.logFile, "[INCUBATION STOPPED VIA WEB UI]");

  Serial.println(F("[WEB] Incubation stopped via web UI — restarting"));
  delay(300);
  ESP.restart();
}


// ── GET /api/logs  ──────────────────────────────────────────
// Returns JSON array of { name, size } for every file in /Logs
// Max log files we will list — keeps this stack-only, no heap
// Show the newest MAX_LOG_FILES log files regardless of total count.
// Filenames are "YYYYMMDD-HHMM.csv" — lexicographic order = chronological.
// Single-pass "running top-N" algorithm:
//   Maintain a sorted array of the N largest filenames seen so far.
//   Each new file either replaces the oldest entry or is discarded.
//   No ceiling on total files scanned — only MAX_LOG_FILES held in RAM.
#define MAX_LOG_FILES 32

static void handleGetLogs() {
  addCORSHeaders();

  ExFile dir;
  if (!dir.open(LOG_DIR, O_RDONLY)) {
    webServer.send(200, "application/json", "[]");
    return;
  }

  if(myConfig.proc > 0 )
  {
    webServer.send(200, "application/json", "[]");
    return;
  }

  // Fixed static buffers — MAX_LOG_FILES * (64 + 4) bytes in BSS
  static char     names[MAX_LOG_FILES][64];
  static uint32_t sizes[MAX_LOG_FILES];
  uint8_t count = 0;   // how many slots are filled (0..MAX_LOG_FILES)

  char     curName[64];
  uint32_t curSize;
  ExFile   entry;

  while (entry.openNext(&dir, O_RDONLY)) {
    esp_task_wdt_reset();
    if (entry.isDir()) { entry.close(); continue; }

    entry.getName(curName, sizeof(curName));
    curSize = entry.fileSize();
    entry.close();

    if (count < MAX_LOG_FILES) {
      // Array not full yet — insert in sorted position (ascending)
      int8_t j = (int8_t)count - 1;
      while (j >= 0 && strcmp(names[j], curName) > 0) {
        esp_task_wdt_reset();
        strncpy(names[j + 1], names[j], 64);
        sizes[j + 1] = sizes[j];
        j--;
      }
      strncpy(names[j + 1], curName, 64);
      sizes[j + 1] = curSize;
      count++;
    } else {
      // Array full — only keep if newer than the oldest entry (names[0])
      if (strcmp(curName, names[0]) > 0) {
        // Shift oldest out, insert new file in sorted position
        int8_t j = 1;
        while (j < MAX_LOG_FILES && strcmp(names[j], curName) < 0) {
          esp_task_wdt_reset();
          strncpy(names[j - 1], names[j], 64);
          sizes[j - 1] = sizes[j];
          j++;
        }
        strncpy(names[j - 1], curName, 64);
        sizes[j - 1] = curSize;
      }
      // else: file is older than everything we have — discard
    }
  }
  dir.close();

  if (count == 0) {
    webServer.send(200, "application/json", "[]");
    return;
  }

  // Stream newest first (descending — names[count-1] is newest)
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "application/json", "");
  webServer.sendContent("[");

  char entryBuf[128];
  bool first = true;
  for (int8_t i = (int8_t)count - 1; i >= 0; i--) {
    esp_task_wdt_reset();
    snprintf(entryBuf, sizeof(entryBuf),
             "%s{\"name\":\"%s\",\"size\":%lu}",
             first ? "" : ",", names[i], (unsigned long)sizes[i]);
    webServer.sendContent(entryBuf);
    first = false;
  }

  webServer.sendContent("]");
  webServer.sendContent("");
}

// ── GET /api/logs/download?file=<filename>  ─────────────────
// Streams the requested file from /Logs as a download.
static void handleDownloadLog() {
  addCORSHeaders();

  if (!webServer.hasArg("file")) {
    webServer.send(400, "application/json", "{\"error\":\"Missing file param\"}");
    return;
  }

  // Build full path — reject any path traversal attempts
  String reqName = webServer.arg("file");
  if (reqName.indexOf('/') >= 0 || reqName.indexOf('\\') >= 0 ||
      reqName.indexOf("..") >= 0) {
    webServer.send(400, "application/json", "{\"error\":\"Invalid filename\"}");
    return;
  }

  char fullPath[128];
  snprintf(fullPath, sizeof(fullPath), "%s/%s", LOG_DIR, reqName.c_str());

  if (!sd.exists(fullPath)) {
    webServer.send(404, "application/json", "{\"error\":\"File not found\"}");
    return;
  }

  ExFile logFile;
  if (!logFile.open(fullPath, O_RDONLY)) {
    webServer.send(500, "application/json", "{\"error\":\"Cannot open file\"}");
    return;
  }

  // Set Content-Disposition so the browser saves it with the correct filename
  String disp = String("attachment; filename=\"") + reqName + "\"";
  webServer.sendHeader("Content-Disposition", disp);
  webServer.sendHeader("Content-Type",        "text/csv");
  webServer.setContentLength(logFile.fileSize());
  webServer.send(200, "text/csv", "");

  const size_t BUF = 512;
  uint8_t buf[BUF];
  int n;
  while ((n = logFile.read(buf, BUF)) > 0) {
    esp_task_wdt_reset();
    webServer.sendContent_P((const char*)buf, n);
  }
  logFile.close();
  webServer.sendContent("");
}

// ── 404 handler  ────────────────────────────────────────────
static void handleNotFound() {
  addCORSHeaders();
  webServer.send(404, "application/json",
    "{\"error\":\"Not found\"}");
}

// ============================================================
//  HELPERS
// ============================================================

// Adds headers that allow the browser page (served from the
// same origin 192.168.1.1) to call the API without CORS issues.
static void addCORSHeaders() {
  webServer.sendHeader("Access-Control-Allow-Origin",  "*");
  webServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  webServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  webServer.sendHeader("Cache-Control",                "no-cache");
}

// Serialise the global myConfig into a JSON string.
// Matches exactly what saveJson() writes so the web UI always
// sees the same field names as settings.json.
static void sendSettingsJson() {
  // Use a stack-only char buffer — no heap String allocation
  // StaticJsonDocument stays on stack, serialized into a fixed buf
  StaticJsonDocument<640> doc;
  doc["p"]             = myConfig.p;
  doc["i"]             = myConfig.i;
  doc["d"]             = myConfig.d;
  doc["safety"]        = myConfig.safety;
  doc["sTemp1"]        = myConfig.sTemp1;
  doc["sHum1"]         = myConfig.sHum1;
  doc["sTemp2"]        = myConfig.sTemp2;
  doc["sHum2"]         = myConfig.sHum2;
  doc["csTemp1"]       = myConfig.csTemp1;
  doc["csHum1"]        = myConfig.csHum1;
  doc["csTemp2"]       = myConfig.csTemp2;
  doc["csHum2"]        = myConfig.csHum2;
  doc["settingPhase"]  = myConfig.settingPhase;
  doc["hatchingPhase"] = myConfig.hatchingPhase;
  doc["csettingPhase"] = myConfig.csettingPhase;
  doc["chatchingPhase"]= myConfig.chatchingPhase;
  doc["humWarn"]       = myConfig.humWarn;
  doc["temWarn"]       = myConfig.temWarn;
  doc["proc"]          = myConfig.proc;
  doc["logFile"]       = myConfig.logFile;
  doc["wifi"]          = myConfig.wifi;
  doc["motorPwm"]      = myConfig.motorPwm;
  doc["motorTimeOut"]  = myConfig.motorTimeOut;
  doc["moveInterval"]  = myConfig.moveInterval;
  doc["lastMoveTime"]  = myConfig.lastMoveTime;
	doc["lastMoveDate"]  = myConfig.lastMoveDate;
  doc["trayStopDone"]  = myConfig.trayStopDone;
  doc["trayCentred"]   = myConfig.trayCentred;
  doc["lastPidOutput"] = myConfig.lastPidOutput;
  doc["lastTem"]       = myConfig.lastTem;
  doc["restartErr"]    = myConfig.restartErr;
  doc["time"]          = myConfig.time;
  doc["date"]          = myConfig.date;
  doc["sdate"]         = myConfig.sdate;
  doc["sSetAge"]       = myConfig.sSetAge;

  // Serialize directly into a fixed stack buffer — no heap involvement
  char buf[700];
  serializeJson(doc, buf, sizeof(buf));
  webServer.send(200, "application/json", buf);
}

// Deserialise a JSON string from the web UI and apply only the
// fields that are present.  Unknown fields are silently ignored.
// Returns false if the JSON cannot be parsed at all.
static bool applySettingsJson(const String& body) {
  StaticJsonDocument<640> doc;
  if (deserializeJson(doc, body)) return false;

  // ── PID ───────────────────────────────────────────────────
  if (doc.containsKey("p"))          myConfig.p             = doc["p"];
  if (doc.containsKey("i"))          myConfig.i             = doc["i"];
  if (doc.containsKey("d"))          myConfig.d             = doc["d"];

  // ── Safety ────────────────────────────────────────────────
  if (doc.containsKey("safety"))     myConfig.safety        = (bool)doc["safety"];

  // ── Fresh incubation defaults ─────────────────────────────
  if (doc.containsKey("sTemp1"))     myConfig.sTemp1        = doc["sTemp1"];
  if (doc.containsKey("sHum1"))      myConfig.sHum1         = doc["sHum1"];
  if (doc.containsKey("sTemp2"))     myConfig.sTemp2        = doc["sTemp2"];
  if (doc.containsKey("sHum2"))      myConfig.sHum2         = doc["sHum2"];
  if (doc.containsKey("settingPhase"))  myConfig.settingPhase  = doc["settingPhase"];
  if (doc.containsKey("hatchingPhase")) myConfig.hatchingPhase = doc["hatchingPhase"];

  // ── Custom incubation ─────────────────────────────────────
  if (doc.containsKey("csTemp1"))    myConfig.csTemp1       = doc["csTemp1"];
  if (doc.containsKey("csHum1"))     myConfig.csHum1        = doc["csHum1"];
  if (doc.containsKey("csTemp2"))    myConfig.csTemp2       = doc["csTemp2"];
  if (doc.containsKey("csHum2"))     myConfig.csHum2        = doc["csHum2"];
  if (doc.containsKey("csettingPhase")) myConfig.csettingPhase = doc["csettingPhase"];
  // {
  //   uint8_t v = doc["csettingPhase"];
  //   if (v > 0) myConfig.csettingPhase = v;   // enforce >0 on server side too
  // }
  if (doc.containsKey("chatchingPhase")) myConfig.chatchingPhase = doc["chatchingPhase"];

  // ── Alerts ────────────────────────────────────────────────
  if (doc.containsKey("humWarn"))    myConfig.humWarn       = doc["humWarn"];
  if (doc.containsKey("temWarn"))    myConfig.temWarn       = doc["temWarn"];

  // ── Motor ─────────────────────────────────────────────────
  if (doc.containsKey("motorPwm"))   myConfig.motorPwm      = doc["motorPwm"];
  if (doc.containsKey("motorTimeOut")) myConfig.motorTimeOut  = doc["motorTimeOut"];
  if (doc.containsKey("moveInterval")) myConfig.moveInterval = doc["moveInterval"];

  // ── SD card age tracking ──────────────────────────────────
  if (doc.containsKey("sSetAge"))    myConfig.sSetAge       = doc["sSetAge"];

  // ── Power loss recovery ───────────────────────────────────
  if (doc.containsKey("restartErr")) myConfig.restartErr    = doc["restartErr"];

  // NOTE: proc, logFile, date, time, sdate are NOT writable via
  // POST /api/settings — they are managed by the firmware only.
  // Use /api/synctime for time/date and /api/startinc|stopinc
  // for the incubation proc field.

  return true;
}


// 1. CREATE or OVERRIDE (Modify)
bool saveJson(const char* path, const Settings& data) {
  ExFile file;
  // O_TRUNC wipes the file if it exists, allowing a clean override
  if (!file.open(path, O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.println(F("Error: Could not open for writing"));
    return 0;
  }

  StaticJsonDocument<640> doc;
 // doc["magic"] = data.magic;
  doc["id1"] = 0x4A;
  doc["p"]     = data.p;
  doc["i"]     = data.i;
  doc["d"]     = data.d;
  doc["safety"] = data.safety;
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
  //doc["humError"] = data.humError;
  doc["humWarn"] = data.humWarn;
  doc["temWarn"] = data.temWarn;
  doc["proc"]  = data.proc;
  doc["logFile"] = data.logFile;
  doc["wifi"] = data.wifi;
  //doc["am"]    = data.am;
  doc["motorPwm"] = data.motorPwm;
  doc["motorTimeOut"] = data.motorTimeOut;
  doc["moveInterval"] = data.moveInterval;
  doc["lastMoveTime"] = data.lastMoveTime;
  doc["lastMoveDate"] = data.lastMoveDate;
  doc["trayStopDone"] = data.trayStopDone;
  doc["trayCentred"] = data.trayCentred;
  doc["lastPidOutput"] = data.lastPidOutput;
  doc["lastTem"] = data.lastTem;
  doc["restartErr"] = data.restartErr;
  //doc["motorCentred"] = data.motorCentred;
  //doc["moveCount"] = data.moveCount;
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

  StaticJsonDocument<640> doc;
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
    data.safety = doc["safety"];
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
    //data.humError = doc["humError"];
    data.humWarn = doc["humWarn"];
    data.temWarn = doc["temWarn"];
    data.proc = doc["proc"];
    strncpy(data.logFile,doc["logFile"],sizeof(data.logFile));
    data.wifi = doc["wifi"];
    //data.logFile = doc["logFile"];
    // data.am    = doc["am"];
    data.motorPwm = doc["motorPwm"];
    data.motorTimeOut = doc["motorTimeOut"];
    data.moveInterval = doc["moveInterval"];
    data.lastMoveTime = doc["lastMoveTime"];
    data.lastMoveDate = doc["lastMoveDate"];
    data.trayStopDone  = doc["trayStopDone"];
    data.trayCentred   = doc["trayCentred"];
    data.lastPidOutput = doc["lastPidOutput"];// | 0.0;
    data.lastTem = doc["lastTem"];
    data.restartErr = doc["restartErr"];
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
      esp_task_wdt_reset();
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
  Serial.print("Safety: "); Serial.println(s.safety);
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
  //Serial.print("humError: "); Serial.println(s.humError);
  Serial.print("humWarn: "); Serial.println(s.humWarn);
  Serial.print("temWwarn: "); Serial.println(s.temWarn);
  Serial.print("proc: "); Serial.println(s.proc);
  Serial.print("logFile: "); Serial.println(s.logFile);
  Serial.print("wifi: "); Serial.println(s.wifi);
  // Serial.print("am: "); Serial.println(s.am);
  Serial.print("motorPwm: "); Serial.println(s.motorPwm);
  Serial.print("motorInterval: "); Serial.println(s.moveInterval);
  Serial.print("motorTimeOut: "); Serial.println(s.motorTimeOut);
  Serial.print("lastMoveTime: "); Serial.println(s.lastMoveTime);
  Serial.print("lastMoveDate: "); Serial.println(s.lastMoveDate);
  Serial.print("trayStopDone: "); Serial.println(s.trayStopDone);
  Serial.print("trayCentred: "); Serial.println(s.trayCentred);
  Serial.print("lastPidOutput: "); Serial.println(s.lastPidOutput);
  Serial.print("lastTem: "); Serial.println(s.lastTem);
  Serial.print("restartErr: "); Serial.println(myConfig.restartErr);
  //Serial.print("motorCentred: "); Serial.println(s.motorCentred);
  //Serial.print("moveCount: "); Serial.println(s.moveCount);
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
  ledcAttach(motorEN, MOTOR_LEDC_FREQ, MOTOR_LEDC_RES); // LEDC for motor PWM
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
  ledcWrite(motorEN, 0); // motor off

  digitalWrite(buzz,HIGH);
  delay(200);
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
    //lcdPrint(BRAND_NAME,"SHT3x not found");
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
    delay(500);
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
    delay(500);
     //return;
  }
  else
  {
    lcdPrint(BRAND_NAME,"SD Card----Found");
    delay(500);
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
      //myConfig = {1.0,0.1,0,1,33.5,70,37.3,70,0,0,0,0,17,2,0,0,10,15,0,"",0,0,0,0,30,0,0,0,3};
      myConfig.p = 1.0;
      myConfig.i = 0.1;
      myConfig.d = 0;
      myConfig.safety = 1;
      myConfig.sTemp1 = 37.5;
      myConfig.sHum1 = 50;
      myConfig.sTemp2 = 37.3;
      myConfig.sHum2 = 70;
      myConfig.csTemp1 = 0;
      myConfig.csHum1 = 0;
      myConfig.csTemp2 = 0;
      myConfig.csHum2 = 0;
      myConfig.settingPhase = 18;
      myConfig.hatchingPhase = 3;
      myConfig.csettingPhase = 0;
      myConfig.chatchingPhase = 0;
      myConfig.humWarn = 10;
      myConfig.temWarn = 5;
      myConfig.proc = 0;
      strncpy(myConfig.logFile," ",sizeof(myConfig.logFile));
      myConfig.wifi = false;
      myConfig.motorPwm = 0;
      myConfig.moveInterval = 0;
      myConfig.lastMoveTime = 0;
      myConfig.lastMoveDate = 0;
      myConfig.motorTimeOut = 30;
      myConfig.lastPidOutput = 0;
      myConfig.lastTem = 0;
      myConfig.restartErr = 0.5;
      //myConfig.motorCentred = false;
      myConfig.time  = 0;
      myConfig.date = 0;
      myConfig.sdate = 0;
      myConfig.sSetAge = 3;
      saveJson(CONFIG_FILE, myConfig);
    }
    

    if(myConfig.sdate < 20260000)
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
    {
      setTemperature = myConfig.csTemp1 <= 0 ? 37.5 : myConfig.csTemp1; // to be use by auto tunner
      Serial.println("NO_INCUBATION");
    }
      
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
  delay(500);
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

  windowStart  = millis();
  lastTempRead = millis() - TEMP_READ_MS;
  lastLcdUpd   = millis();

  // ── Bumpless restart after power failure ─────────────────
  // If incubation is active (proc > 0), the PID was already
  // running before the reset. Instead of starting from zero
  // output and re-running warmup, seed the output with an
  // estimate based on current temperature so the heater
  // resumes close to where it was without a cold restart.
  // if(myConfig.proc > 0 && myConfig.p > 0) {
  //   sensors.requestTemperatures();
  //   double t = sensors.getTempCByIndex(0);
  //   if(t != DEVICE_DISCONNECTED_C) {
  //     currentTemperature = t;
  //     double err = myConfig.lastTem - t;

  //     // ── Bumpless seed logic ───────────────────────────────
  //     // Use the last saved PID output as seed — exact restoration.
  //     // lastPidOutput is saved every 30s so worst case we are
  //     // 30s out of date, but that is far better than starting
  //     // from 5% or estimating from temperature alone.
  //     // Clamp to valid range in case of corrupt saved value.
  //     if(myConfig.lastPidOutput > 0) {
  //       pidOutput = constrain(myConfig.lastPidOutput,
  //                             WINDOW_SIZE_MS * 0.05,
  //                             WINDOW_SIZE_MS * 0.90);
  //     } else {
  //       // No saved value — fall back to proportional estimate
  //       double holdBase = Kp * 0.5;
  //       pidOutput = constrain(Kp * err + holdBase,
  //                             WINDOW_SIZE_MS * 0.10,
  //                             WINDOW_SIZE_MS * 0.80);
  //     }
  //     heatPhase = PHASE_RUNNING;   // skip warmup — already at temperature
  //     heaterPID.SetMode(AUTOMATIC);
  //     Serial.printf("[PID] Bumpless restart: T=%.2f SP=%.2f err=%.2f seed=%.0fms\n",
  //                   t, (double)resumeSP, err, pidOutput);
  //   } else {
  //     pidOutput  = 0;
  //     heatPhase  = PHASE_WARMUP;
  //   }
  // } else {
  //   pidOutput  = 0;
  //   heatPhase  = PHASE_WARMUP;
  // }
  

  humidifierState = false;

  // Seed timing so MIN_OFF_TIME is respected from boot
  relayLastSwitched = millis();
  humidLastRead     = millis() - HUMID_READ_MS;   // read immediately on first call

  // Initial read using your existing getHum()
  double h = (double)getHum();
  if (h >= 0.0 && h <= 100.0) currentHumidity = h;

  Serial.printf("Set Humidity  : %.1f %%RH\n",   setHumidity);
  Serial.printf("Hysteresis    : ON < %.1f%%  OFF > %.1f%%\n",
                setHumidity - HYST_LOW, setHumidity + HYST_HIGH);
  Serial.printf("Min ON time   : %d ms\n",        MIN_ON_TIME_MS);
  Serial.printf("Min OFF time  : %d ms\n",        MIN_OFF_TIME_MS);
  Serial.printf("Current RH    : %.1f %%RH\n",    currentHumidity);
  
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

  // ── Web server ────────────────────────────────────────────
  webServerSetup();
  if(!myConfig.wifi)
    wifiOff();
  
  //currentError = ERROR_TRIPLE;

  //myConfig.proc = -1,-2,-3 is a request to run an incubation demo start, custom start, fresh start
  if(myConfig.proc < 0)
  {
    myConfig.proc = abs(myConfig.proc); //approving the incubation request
    Serial.printf("Current proc: %d\n",myConfig.proc);

    //if safety variable is false -> safety check will bypassed
    if(!myConfig.safety)
    {
      lcdPrint("Safety check","[BYPASSED]");
      Serial.println("Safety check bypassed!");
      //saveJson(CONFIG_FILE,myConfig);
      delay(1000);
      //return;
    }
    else
    {
      // check for sys and component failure
      // if any error found request for inbuation will be denied.
      int error;
      lcdPrint("Safety check","[PROCESSING]");
      Serial.println("Safety check processing...");
      delay(1000);
      if (!sd.begin(chipSelect, SD_SCK_MHZ(16)))
      {
        Serial.println("Sd card cannot be read");
        lcdPrint("Sys error","No SD-Card");
        myConfig.proc = 0;
        digitalWrite(buzz,HIGH);
        delay(1500);
        digitalWrite(buzz,LOW);
        delay(1500);
      }

      now = rtc.now();
      if(now.year() < 2026)
      {
        Serial.println("Invalid time and date setting");
        lcdPrint("Sys error","Invalid DateTime");
        myConfig.proc = 0;
        digitalWrite(buzz,HIGH);
        delay(1500);
        digitalWrite(buzz,LOW);
        delay(1500);
      }

      error = getHum();
      if(error < 0)
      {
        Serial.println("Humidity sensor failed");
        lcdPrint("Sys error","HumSensor-> Fail");
        myConfig.proc = 0;
        digitalWrite(buzz,HIGH);
        delay(1500);
        digitalWrite(buzz,LOW);
        delay(1500);
      }
      error = getTem();
      if(error < -120)
      {
        Serial.println("Temperature sensor failed");
        lcdPrint("Sys error","TemSensor-> Fail");
        myConfig.proc = 0;
        digitalWrite(buzz,HIGH);
        delay(1500);
        digitalWrite(buzz,LOW);
        delay(1500);
      }

      if(!getWaterLevel())  // false = pin LOW = water low
      {
        Serial.println("Low water level");
        lcdPrint("Sys error","Low water Level!");
        myConfig.proc = 0;
        digitalWrite(buzz,HIGH);
        delay(1500);
        digitalWrite(buzz,LOW);
        delay(1500);
      }
      
    }

    // Reset motor state for new incubation
    trayAtFront  = !digitalRead(limitSwitch1); // LOW = pressed = at front
    myConfig.trayCentred  = false;
    myConfig.trayStopDone = false;
    myConfig.lastMoveTime = 0;
    myConfig.lastMoveDate = 0;
    myConfig.lastPidOutput = 0;
    myConfig.lastTem = 0;
    saveJson(CONFIG_FILE,myConfig);
    fullTravelMs = 0;
    motorState   = MOTOR_IDLE;

    // Record incubation start date+time atomically from a single rtc.now() read
    // so getIncDay() always has a consistent start reference
    {
      now = rtc.now();
      uint32_t startDate = (uint32_t)now.year() * 10000UL
                         + (uint32_t)now.month() * 100UL
                         + now.day();
      uint16_t startTime = now.hour() * 100 + now.minute();
      myConfig.date = startDate;
      myConfig.time = startTime;
      saveJson(CONFIG_FILE, myConfig);
      Serial.printf("[INC] Start recorded: date=%lu time=%u\n",
                    (unsigned long)startDate, startTime);
    }

    //creating log csv file
    snprintf(myConfig.logFile,sizeof(myConfig.logFile),"%d-%d.csv",myConfig.date,myConfig.time);
    Serial.printf("Log FileName: %s\n",myConfig.logFile);
    initLogFile(myConfig.logFile,"dd_mm_yyy,hr_min_sec,f/c,safety,sHum,cHum,sTem,cTem,pidOut,heater,humidifier,motorState,day");
    delay(500);
    char intro[256];
    snprintf(intro,sizeof(intro),"PID constants\nKp: %.2f\nKi: %.2f\nKd - %.2f\nSettingDays: %d\nHatchingDays: %d",
        Kp,Ki,Kd,
        myConfig.proc == 1?myConfig.settingPhase:myConfig.csettingPhase,
        myConfig.proc == 1?myConfig.hatchingPhase:myConfig.chatchingPhase);
    
    appendCSV(myConfig.logFile,intro);
    delay(500);
    saveJson(CONFIG_FILE,myConfig);
  }

  else if(myConfig.proc > 0)
  {
    lcdPrint("Incubation","[RESUME]");
    appendCSV(myConfig.logFile,"[SYS INTERRUPTED]");
    Serial.println("[SYS] requested for bumpless restart");
    delay(500);
    bumpLessRestart = true;
  }
  

  //watchcdog timmer init
  const esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 10000,   // 10 seconds in milliseconds
    .idle_core_mask = 0,   // don't watch idle taskss
    .trigger_panic = true  // reset on timeout
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); 

}

void loop() {

  //feeding watchdogtimer
  esp_task_wdt_reset();

  // ── INCUBATION MODE: run only critical systems ─────────────
  // WiFi, LCD, Serial debug, and menu are suspended to give the
  // PID / humidity / motor systems uninterrupted CPU time.
  if(myConfig.proc > 0)
  {
    // Only allow abort button during active incubation
    accessibilityHandler();

    // Critical systems — run every loop iteration
    heatingSys();
    humiditySys();
    fanSys(HIGH);
    motorHandler();
    buzzerHandler();  // runs every loop so beep timing is accurate

    // LCD update — rate limited by LCD_UPDATE_MS (500ms) inside updateLCD()
    // Uses existing lastLcdUpd timing so no extra overhead
    if(millis() - lastLcdUpd >= LCD_UPDATE_MS)
    {
      lastLcdUpd = millis();
      updateLCD();
    }

    // Rate-limited tasks — once per second only
    static unsigned long incLastSec = 0;
    unsigned long incMs = millis();
    if(incMs - incLastSec >= 1000)
    {
      incLastSec = incMs;
      now = rtc.now();

      int day = getIncDay();

      // Update setpoints from phase
      if(myConfig.proc == 1)
      {
        setTemperature = day <= myConfig.settingPhase ? myConfig.sTemp1 : myConfig.sTemp2;
        setHumidity    = day <= myConfig.settingPhase ? myConfig.sHum1  : myConfig.sHum2;

        if(day > (myConfig.settingPhase + myConfig.hatchingPhase))
        {
          myConfig.proc = 0;
          saveJson(CONFIG_FILE,myConfig);
          lcdPrint("INCUBATION DONE","Restarting sys..");
          for(int i=0;i<3;i++)
          {
            esp_task_wdt_reset();
            digitalWrite(buzz,HIGH);
            delay(1000);
            digitalWrite(buzz,LOW);
            delay(1000);
          }
          ESP.restart();
        }
      }
      else if(myConfig.proc == 2)
      {
        setTemperature = day <= myConfig.csettingPhase ? myConfig.csTemp1 : myConfig.csTemp2;
        setHumidity    = day <= myConfig.csettingPhase ? myConfig.csHum1  : myConfig.csHum2;

        if(day > (myConfig.csettingPhase + myConfig.chatchingPhase))
        {
          myConfig.proc = 0;
          saveJson(CONFIG_FILE,myConfig);
          lcdPrint("INCUBATION DONE","Restarting sys..");
          for(int i=0;i<3;i++)
          {
            esp_task_wdt_reset();
            digitalWrite(buzz,HIGH);
            delay(1000);
            digitalWrite(buzz,LOW);
            delay(1000);
          }
          ESP.restart();
        }
      }

      // Update error state once per second
      // Motor timeout warning takes priority — don't clear it until its 10s timer expires
      if(currentTemperature < -120 || currentHumidity < 0)
        setError(ERROR_RAPID);
      else if((abs(currentTemperature-setTemperature) > myConfig.temWarn) || (abs(currentHumidity-setHumidity) > myConfig.humWarn))
        setError(ERROR_TRIPLE);
      else if(!getWaterLevel())  // false = pin LOW = water low
        setError(ERROR_SINGLE);
      else if(motorWarnClearMs == 0)  // only clear if no motor warning active
        setError(NONE);
    }

    // ── Buffered logging ──────────────────────────────────────
    // Row sampled every 6s into RAM buffer (5 slots).
    // Buffer flushed to SD in one write every 30s.
    // SD write happens once per 30s — no SD latency in the critical loop.
    #define LOG_BUF_ROWS  5
    #define LOG_ROW_LEN   256
    static char    logBuf[LOG_BUF_ROWS][LOG_ROW_LEN];
    static uint8_t logBufIdx    = 0;
    static unsigned long incLastSample = 0;
    static unsigned long incLastFlush  = 0;

    // Sample a row every 6 seconds into RAM
    if(incMs - incLastSample >= 6000)
    {
      incLastSample = incMs;
      // Guard: never write past the end of the buffer.
      // If flush was delayed (slow SD), old rows stay until next flush.
      // Oldest unwritten row is simply overwritten — we never lose the
      // most recent data, only potentially duplicate an old row on overflow.
      if(logBufIdx >= LOG_BUF_ROWS)
        logBufIdx = LOG_BUF_ROWS - 1;  // clamp: overwrite last slot

      now = rtc.now();
      int day = getIncDay();
      snprintf(logBuf[logBufIdx], LOG_ROW_LEN,
              "%d_%d_%d,%d_%d_%d,%c,%d,%.2frH,%.2frH,%.2fC,%.2fC,%.2fms,%s,%s,%s,%d",
              now.day(), now.month(), now.year(),
              now.hour(), now.minute(), now.second(),
              myConfig.proc == 1 ? 'f' : 'c',
              myConfig.safety,
              (float)setHumidity, currentHumidity,
              setTemperature, currentTemperature,
              pidOutput,
              heaterState ? "On" : "Off",
              humidifierState ? "On" : "Off",
              motorState == MOTOR_CENTRING ? "CENTERING" : motorState == MOTOR_IDLE ? "IDLE" : "MOVING",
              day);
      logBufIdx++;
    }

    // Flush all buffered rows to SD in one transaction every 30 seconds
    if(incMs - incLastFlush >= 30000)
    {
      incLastFlush = incMs;
      for(uint8_t i = 0; i < logBufIdx; i++)
        appendCSV(myConfig.logFile, logBuf[i]);
      logBufIdx = 0;  // reset — safe, buffer is static so memory stays allocated
      // Save current PID output and tempareture so bumpless restart can restore it exactly
      if(heatPhase == PHASE_RUNNING)
      {
        myConfig.lastPidOutput = pidOutput;
        myConfig.lastTem = currentTemperature;
        saveJson(CONFIG_FILE, myConfig);
      }
    }

    // Web server runs but at a throttled rate — max once every 500ms
    // This keeps the UI responsive without stealing CPU from PID
    static unsigned long webLastMs = 0;
    if(wifiEnabled && (incMs - webLastMs >= 500))
    {
      webLastMs = incMs;
      webServer.handleClient();
    }

    return; // skip all idle/menu code below
  }

  // ── IDLE MODE: all systems run normally ────────────────────
  webServerLoop();
  buzzerHandler();
  accessibilityHandler();

  unsigned long currentTime = millis()/1000;
  now = rtc.now();

  if(myConfig.proc == 0)
  {
    fanSys(LOW);

    if(allowOperation && currentOperation == AUTO_TUNE)
    {
     autoTuner();
    }
    //hadling menu button press and its behavior 
    // !allowOperation -> dont allow to enter the main menu while a Operation is running
    if(!digitalRead(menuBott) && !allowOperation)
    {
      while(!digitalRead(menuBott)){esp_task_wdt_reset();} // waiting for the botton to reslease;
      menuPointer = (menuPointer+1)%5;
      mainMenu();
      mainDisplayWait = 5;
    }

    //hadling select button press and its behavior 
    if(!digitalRead(selectBott))
    {
      while(!digitalRead(selectBott)){esp_task_wdt_reset();}
      allowOperation = true;
      operationHandler();
    }

    if(currentTime-lastTime >= 1)
    {
      //updating this variable as many other function use this to share common current sensor data
      currentTemperature = getTem();
      currentHumidity = getHum();


      lastTime = currentTime;

      if(allowOperation && currentOperation != NO_OPT)
        operationHandler();

      // MAIN DISPLAY
      if(mainDisplayWait == 0 && !allowOperation)
      {
        char line1[17];
        char line2[17];
      
        
        // Use cached values — avoids 3x blocking DS18B20 reads (750ms each) per display tick
        snprintf(line1,17,"%d-%d-%4d  %c%c%c%c%c",now.day(),now.month(),now.year(),myConfig.motorPwm?' ':'M',currentHumidity >= 0?' ':'H',currentTemperature > -120?' ':'T',sdCardWarning()?' ':'S',getWaterLevel()?' ':'W');
        if(scount <= 2) // show the tem on line2 for 3 second
          snprintf(line2,17,"%d:%d:%d T%.2fC",now.hour(),now.minute(),now.second(),(float)currentTemperature);
        else // show the hum on line2 for 3 second after tem
          snprintf(line2,17,"%d:%d:%d H%.2f%%",now.hour(),now.minute(),now.second(),(float)currentHumidity);
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

int getIncDay()
{
  uint32_t date = myConfig.date;
  uint16_t time = myConfig.time;

  // No incubation start recorded yet
  if(!date) return 0;

  // Decode start YYYYMMDD
  uint8_t  dy = date % 100; date /= 100;
  uint8_t  mn = date % 100; date /= 100;
  uint16_t yr = (uint16_t)date;

  // Decode start HHMM
  uint8_t  startMin = time % 100; time /= 100;
  uint8_t  startHr  = (uint8_t)time;

  now = rtc.now();

  // daysBetween returns fractional days — floor it so we never
  // jump ahead due to sub-day fractions, then add 1 (day 1 = start day)
  double days = daysBetween(yr, mn, dy, startHr, startMin,
                            now.year(), now.month(), now.day(),
                            now.hour(), now.minute());

  // Guard against negative result (RTC drift / wrong stored date)
  if(days < 0) days = 0;

  return (int)days + 1;
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
  char tempLine2[17];
    if(myConfig.proc == 0)
   {
      if(allowOperation && !digitalRead(accessibilityBott))
      {
        digitalWrite(buzz,LOW);
        for(i = 3; i>0; i--)
        {
          esp_task_wdt_reset();
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
          digitalWrite(heater,LOW);
          Serial.println("Operation aborted by user");
          lcdPrint("Operation","aborted by user");
          while(!digitalRead(accessibilityBott)){esp_task_wdt_reset();}
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
          esp_task_wdt_reset();
          Serial.printf("Wifi %s in %d\n",myConfig.wifi?"OFF":"ON",i);
          snprintf(tempLine,17,"Wifi turn %s",myConfig.wifi?"OFF":"ON");
          snprintf(tempLine2,17,"in %d",i);
          lcdPrint(tempLine,tempLine2);
          delay(1000);
          if(digitalRead(accessibilityBott))
            break;
        } 
        if(i == 0)
        {
          lcdPrint("Release button","To continue");
          while(!digitalRead(accessibilityBott)){esp_task_wdt_reset();}
          myConfig.wifi = !myConfig.wifi;

          if(myConfig.wifi)
            wifiOn();
          else
            wifiOff();

          saveJson(CONFIG_FILE,myConfig);
          delay(500);
        }
      }
   }

   else
   {
      if(!digitalRead(accessibilityBott))
      {
        digitalWrite(buzz,LOW);
        for(i = 5; i>0; i--)
        {
          esp_task_wdt_reset();
          Serial.print("Incubation Abort in "); Serial.println(i);
          snprintf(tempLine,17,"in %d",i);
          lcdPrint("Incubation abort",tempLine);
          delay(1000);
          if(digitalRead(accessibilityBott))
            break;
        } 
        if(i == 0)
        {
          myConfig.proc = 0;
          saveJson(CONFIG_FILE,myConfig);
          appendCSV(myConfig.logFile,"[INCUBATION ABORTED BY USER]");
          Serial.println("Incubation aborted by user");
          lcdPrint("INCUBATION","ABORTED [USER]");
          delay(1000);
          lcdPrint("Release the but-","ton to restart");
          while(!digitalRead(accessibilityBott)){esp_task_wdt_reset();}
          ESP.restart();
          return 1;
        }
      }
   }
  return 0;
}

void fanSys(bool state)
{
  digitalWrite(fanRelay,state);
}

void humiditySys() {
  unsigned long currentMs = millis();

  if (currentMs - humidLastRead >= HUMID_READ_MS) {
    humidLastRead = currentMs;

    // ── 1. Read humidity via your getHum() ───────
    double h = (double)getHum();

    if (h >= 0.0 && h <= 100.0) {
      currentHumidity = h;   // valid reading — update
    } else {
      // Sensor disconnected — turn humidifier OFF, set sentinel for error check
      currentHumidity = -1.0;
      if (humidifierState) applyRelay(false);
      return;
    }

    // ── 2. Hysteresis control + relay time guard ──
    unsigned long timeInState = currentMs - relayLastSwitched;

    if (!humidifierState) {
      // Currently OFF — turn ON if below lower threshold
      // AND relay has been OFF long enough
      if (currentHumidity < (setHumidity - HYST_LOW) &&
          timeInState >= MIN_OFF_TIME_MS) {
        applyRelay(true);
      }
    } else {
      // Currently ON — turn OFF if above upper threshold
      // AND relay has been ON long enough
      if (currentHumidity > (setHumidity + HYST_HIGH) &&
          timeInState >= MIN_ON_TIME_MS) {
        applyRelay(false);
      }
    }
  }
}

static void applyRelay(bool turnOn) {
  humidifierState   = turnOn;
  relayLastSwitched = millis();

  // Active HIGH relay board: HIGH = relay ON, LOW = relay OFF
  digitalWrite(humidifierRelay, turnOn ? HIGH : LOW);

  Serial.printf("[HUM] Humidifier %s\n", turnOn ? "ON" : "OFF");
}

void lcdPrint(const char* a, const char* b)
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
    {
      char tempLine[17];
      snprintf(tempLine,17,"-> %s",myConfig.safety? "True":"False");
      lcdPrint("MENU:SafetyCheck",tempLine);
      setOperation(SAFETY);
    }
      break;

    case 4:
      lcdPrint("MENU: TuneHeater","-> Auto Tune");
      setOperation(AUTO_TUNE);
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
      lcdPrint("SSID:Inc_ART010I","IP:192.168.4.1");
      mainDisplayWait = 5;
      // dont allow Operation to run as NO_OPT mean not be run any Operation.
      // this line of code for safety even if somewhere in the code enable the allowOperation by mistake.
      allowOperation = false; 
      break;
    
    case FRESH_START:
      lcdPrint("Restarting sys","to FreshStartInc");
      myConfig.proc = -1;
      saveJson(CONFIG_FILE,myConfig);
      delay(1000);
      ESP.restart();
      break;

    case CUSTM_START:
      lcdPrint("Restarting sys","to CustmStartInc");
      myConfig.proc = -2;
      saveJson(CONFIG_FILE,myConfig);
      delay(1000);
      ESP.restart();
      break;

    case SAFETY:
    {
      mainDisplayWait = 5;
      allowOperation = false;
      char tempLine[17];
      myConfig.safety = !myConfig.safety;
      snprintf(tempLine,17,"-> %s",myConfig.safety? "True":"False");
      lcdPrint("MENU:SafetyCheck",tempLine);
      saveJson(CONFIG_FILE,myConfig);
    }
      break;

    case AUTO_TUNE:
      // this operation need separate timing system
      // it have a separate handler on the main loop
      // so no code need here
      break;


    default:
      lcdPrint("Internal Err!","-> Invalid opt");
      mainDisplayWait = 3;
  }
}

double daysBetween(int y1,int m1,int d1,int h1,int min1,int y2,int m2,int d2,int h2,int min2,bool mode)
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
    if(mode) return minutes;
    else return minutes / 1440.0;   // minutes to days
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
  Serial.println(F("\nautoTuner() started"));

  //turn off wifi and web service to avoid interruption
  wifiOff();


  // ── Reset all internal tune state ────────────
  // Safe to re-run after abort — clears any
  // stale peaks, cycles or timing from last run
  heaterPID.SetMode(MANUAL);
  pidOutput    = 0;
  heaterState  = false;
  digitalWrite(heater, LOW);
  relayOn      = false;
  lastPeakHigh = -999.0;
  lastPeakLow  =  999.0;
  peakCount    = 0;
  cycleCount   = 0;
  periodSum    = 0.0;
  amplitudeSum = 0.0;
  lastPeakTime = 0;
  lookForHigh  = true;
  histIdx      = 0;
  for (int i = 0; i < 5; i++) tempHist[i] = 0;
  heatPhase    = PHASE_WARMUP;
  windowStart  = millis();
  lastTempRead = millis() - TEMP_READ_MS;
  lastLcdUpd   = millis();
  lcd.clear();
  Serial.println(F("State reset — starting fresh."));

  // ── Phase 1: WARMUP — full power until within 5°C of target ──
  Serial.println(F("Phase: WARMUP (full power)"));
  while (heatPhase == PHASE_WARMUP) {
    esp_task_wdt_reset();
    if(accessibilityHandler())
    { 
      if(myConfig.wifi) wifiOn();
      return;
    }
    unsigned long currentMs = millis();

    if (currentMs - lastTempRead >= TEMP_READ_MS) {
      lastTempRead = currentMs;
      sensors.requestTemperatures();
      double t = sensors.getTempCByIndex(0);
      if (t != DEVICE_DISCONNECTED_C) currentTemperature = t;

      pidOutput = WINDOW_SIZE_MS;   // full power

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
    if (currentMs - windowStart >= WINDOW_SIZE_MS) windowStart += WINDOW_SIZE_MS;
    bool newState = (pidOutput > (double)(currentMs - windowStart));
    if (newState != heaterState) {
      heaterState = newState;
      digitalWrite(heater, heaterState ? HIGH : LOW);
    }

    // LCD
    if (currentMs - lastLcdUpd >= LCD_UPDATE_MS) {
      lastLcdUpd = currentMs;
      updateLCD();
    }
  }

  // ── Phase 2: AUTOTUNING — relay oscillation ──
  while (heatPhase == PHASE_AUTOTUNING) {
    esp_task_wdt_reset();
    if(accessibilityHandler())
    { 
      if(myConfig.wifi) wifiOn();
      return;
    }
    unsigned long currentMs = millis();

    if (currentMs - lastTempRead >= TEMP_READ_MS) {
      lastTempRead = currentMs;
      sensors.requestTemperatures();
      double t = sensors.getTempCByIndex(0);
      if (t != DEVICE_DISCONNECTED_C) currentTemperature = t;

      runRelayTune();   // may call finishTune() which sets PHASE_RUNNING
    }

    // SSR output
    if (currentMs - windowStart >= WINDOW_SIZE_MS) windowStart += WINDOW_SIZE_MS;
    bool newState = (pidOutput > (double)(currentMs - windowStart));
    if (newState != heaterState) {
      heaterState = newState;
      digitalWrite(heater, heaterState ? HIGH : LOW);
    }

    // LCD
    if (currentMs - lastLcdUpd >= LCD_UPDATE_MS) {
      lastLcdUpd = currentMs;
      updateLCD();
    }
  }

  Serial.println(F("autoTuner() complete\n"));
  lcdPrint("Auto Tune","[COMPLETED]");
  if(myConfig.wifi) wifiOn();
  while(true)
  {
    esp_task_wdt_reset();
    if(accessibilityHandler()) return;
  }
}

void heatingSys() {

  //bumpless restart control logic
  //if the err = myConfig.lastTem - t, where t is the current temperature
  //is not greater than 0.5 then it will resume with the lastPidOutput
  //if the err is greater than 0.5 then the heater will manually heat up in full power
  //untill it get a err <0.5
  if(bumpLessRestart)
  {
    if(myConfig.lastPidOutput <= 0 || myConfig.lastTem <= 0)
    {
      bumpLessRestart = false;
      Serial.println("[heatingSys] no saved data found for bumpless restart");
      return;
    }
    sensors.requestTemperatures();
    double t = sensors.getTempCByIndex(0);
    currentTemperature = t;
    if(t == DEVICE_DISCONNECTED_C) {
      currentTemperature = -127.0;
      pidOutput = 0;
      return; 
    }

    double err = myConfig.lastTem - t;
    if(err <= myConfig.restartErr)
    {
      digitalWrite(heater,LOW);
      pidOutput = constrain(myConfig.lastPidOutput,
                               WINDOW_SIZE_MS * 0.05,
                               WINDOW_SIZE_MS * 0.90);
      Serial.printf("[heatingSys] Current temperature archive near the last save temperature. Err: %.2f seed: %.0fms\n",
                    err,pidOutput);
      heaterPID.SetMode(AUTOMATIC);
      heatPhase    = PHASE_RUNNING;
      windowStart  = millis();
      lastTempRead = millis() - TEMP_READ_MS;
      lastLcdUpd   = millis();

      Serial.println(F("[PID] Heating system resume — "
                      "PID running with Kp/Ki/Kd gains."));
      Serial.printf("  Kp=%.4f  Ki=%.4f  Kd=%.4f\n", Kp, Ki, Kd);
      bumpLessRestart = false;
    }
    else
    {
      heatPhase = PHASE_WARMUP;
      pidOutput = 0;
      heaterPID.SetMode(MANUAL);
      digitalWrite(heater,HIGH);
      return;
    }
  }

  // ── 0. Safe self-init guard ───────────────────
  // Runs once if autoTuner() was never called.
  // Switches PID to AUTOMATIC with a bumpless transfer:
  //   - reads current temperature first
  //   - seeds pidOutput to a safe small value (not 0, not full)
  //     so the integral starts calm with no overshoot spike
  if (heaterPID.GetMode() == MANUAL) 
  {
    sensors.requestTemperatures();
    double t = sensors.getTempCByIndex(0);
    if (t == DEVICE_DISCONNECTED_C) {
      currentTemperature = -127.0;
      pidOutput = 0;
      return;  // stay MANUAL — re-engages automatically when sensor appears
    }
    currentTemperature = t;

    // Seed output at 5% of window — just enough to start warming
    pidOutput = WINDOW_SIZE_MS * 0.05;

    heaterPID.SetMode(AUTOMATIC);   // bumpless transfer seeds integral here
    heatPhase    = PHASE_RUNNING;
    windowStart  = millis();        // reset here first
    lastTempRead = millis() - TEMP_READ_MS;
    lastLcdUpd   = millis();

    Serial.println(F("[heatingSys] heating system started — "
                     "PID run with Kp/Ki/Kd gains."));
    Serial.printf("  Kp=%.4f  Ki=%.4f  Kd=%.4f\n", Kp, Ki, Kd);
  }


  // ── currentMs captured AFTER guard so windowStart is always fresh ──
  unsigned long currentMs = millis();

  // ── 1. Read temp & compute PID ──────────────
  if (currentMs - lastTempRead >= TEMP_READ_MS) 
  {
    lastTempRead = currentMs;
    sensors.requestTemperatures();
    double t = sensors.getTempCByIndex(0);
    if (t == DEVICE_DISCONNECTED_C) 
    {
      // Sensor disconnected: cut heater immediately, pause PID
      currentTemperature = -127.0;
      heaterPID.SetMode(MANUAL);
      pidOutput   = 0;
      heaterState = false;
      digitalWrite(heater, LOW);
    } 
    else 
    {
      currentTemperature = t;
      // Re-engage PID if it was paused after a disconnect
      if (heaterPID.GetMode() == MANUAL) 
      {
        heatPhase   = PHASE_RUNNING;
        windowStart = millis();
        pidOutput   = WINDOW_SIZE_MS * 0.05;
        heaterPID.SetMode(AUTOMATIC);
        Serial.println(F("[heatingSys] Sensor reconnected — PID re-engaged"));
      }
      heaterPID.Compute();
    }
  }

  // ── 2. Burst-fire SSR output ─────────────────
  if (currentMs - windowStart >= WINDOW_SIZE_MS) windowStart += WINDOW_SIZE_MS;
  bool newState = (pidOutput > (double)(currentMs - windowStart));
  if (newState != heaterState) {
    heaterState = newState;
    digitalWrite(heater, heaterState ? HIGH : LOW);
  }

  // LCD update handled by the incubation fast path to avoid double-calling
}

static void runRelayTune() {
  unsigned long currentMs = millis();

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
      double halfPeriod = (double)(currentMs - lastPeakTime) / 1000.0;
      double amplitude  = (curr - lastPeakLow) / 2.0;

      if (halfPeriod > 0.5 && amplitude > NOISE_BAND) {
        periodSum    += halfPeriod * 2.0;
        amplitudeSum += amplitude;
        cycleCount++;



        if (cycleCount >= TUNE_CYCLES) { finishTune(); return; }
      }
    }
    lastPeakHigh = curr;
    lastPeakTime = currentMs;
    peakCount++;
    lookForHigh  = false;

  } else if (!lookForHigh && isValley) {
    lastPeakLow  = curr;
    lastPeakTime = currentMs;
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
    // Tyreus-Luyben tuning rules (consistent with finishTune)
    Kp = 0.4545 * Ku;
    double Ti = 2.2   * avgPu;
    double Td = 0.1587 * avgPu;
    Ki = (Ti > 0) ? (Kp / Ti) : 0.0;
    Kd = Kp * Td;
    Serial.println(F("[ABORT] Using partial tune data."));
    
    //save partial data to sd card
    myConfig.p = Kp;
    myConfig.i = Ki;
    myConfig.d = Kd;
    saveJson(CONFIG_FILE,myConfig);
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
  Serial.println(F("[ABORT] autoTuner aborted "));

  lcd.clear();
}

//  Internal: Compute gains, apply to PID
static void finishTune() {
  double avgPu  = periodSum    / cycleCount;
  double avgAmp = amplitudeSum / cycleCount;
  double Ku     = (4.0 * (RELAY_STEP / 2.0)) / (PI * avgAmp);

  // Tyreus-Luyben tuning rules (better than ZN for slow thermal systems)
  Kp = 0.4545 * Ku;
  double Ti = 2.2   * avgPu;
  double Td = 0.1587 * avgPu;
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
static void updateLCD() 
{
  static unsigned long lastScreenChange = 0;
  unsigned long currentMs = millis();
  if (currentMs - lastScreenChange >= 5000) {
    lastScreenChange = currentMs;
    incLcdCount++;
    if (incLcdCount > 2) incLcdCount = 0;
    lcd.clear();
  }

  char row0[17], row1[17];
  if (heatPhase == PHASE_WARMUP) {
    snprintf(row0, sizeof(row0), "WARMUP  SP:%5.1f", setTemperature);
    snprintf(row1, sizeof(row1), "CT:%5.1f        ", currentTemperature);

  } else if (heatPhase == PHASE_AUTOTUNING) {
    snprintf(row0, sizeof(row0), "TUNING  SP:%5.1f", setTemperature);
    snprintf(row1, sizeof(row1), "CT:%5.1f CYC:%d/%d",
             currentTemperature, cycleCount, TUNE_CYCLES);

  }
  else {
    if (incLcdCount == 0) {
      snprintf(row0, sizeof(row0), "SP:%.1f CT:%.1f",
               setTemperature, currentTemperature);
      snprintf(row1, sizeof(row1), "OUT:%4.0fms %s  ",
               pidOutput, heaterState ? "[ON ]" : "[OFF]");
    }
    else if (incLcdCount == 1) {
      snprintf(row0, sizeof(row0), "SH:%3.0f%% CH:%3.0f%%",   // float format, not %d
               setHumidity, currentHumidity);
      snprintf(row1, sizeof(row1), "Humidifier %s  ",          // removed pidOutput
               humidifierState ? "[ON ]" : "[OFF]");
    }
    else if (incLcdCount == 2) {
      int s = myConfig.proc == 1 ? myConfig.settingPhase  : myConfig.csettingPhase;
      int h = myConfig.proc == 1 ? myConfig.hatchingPhase : myConfig.chatchingPhase;
      snprintf(row0, sizeof(row0), "[DAY %d]          ",       // padded to 16
               (int)getIncDay());
      snprintf(row1, sizeof(row1), "%s %d/%d          ",       // padded to 16
               getIncDay() <= s ? "Setting " : "Hatching",
               getIncDay() <= s ? (int)getIncDay()       : (int)getIncDay()-s,
               getIncDay() <= s ? s                      : h);
    }
  }

  row0[16] = '\0'; row1[16] = '\0';
  lcd.setCursor(0, 0); lcd.print(row0);
  lcd.setCursor(0, 1); lcd.print(row1);
}

bool initLogFile(const char* filename, const char* header) 
{
  if (!sd.exists(LOG_DIR)) {
    if (!sd.mkdir(LOG_DIR)) {
      Serial.println("Dir create failed");
      return false;
    }
  }

  char fullPath[64];
  snprintf(fullPath, sizeof(fullPath), "%s/%s", LOG_DIR, filename);

  // If file exists remove it
  if (sd.exists(fullPath)) {
    if (!sd.remove(fullPath)) {
      Serial.println("Failed to delete existing file");
      return false;
    }
    Serial.println("Old log file deleted");
  }

  ExFile file;

  // Create fresh file
  if (!file.open(fullPath, O_CREAT | O_WRITE)) {
    Serial.println("File create failed");
    return false;
  }

  // Write CSV header
  file.println(header);
  file.flush();   // ensure header is written
  file.close();

  Serial.println("New log file created");

  return true;
}

// ================================================================
//  MOTOR / TRAY TURNING SYSTEM
// ================================================================

static void motorStop() {
  digitalWrite(motorIN1, LOW);
  digitalWrite(motorIN2, LOW);
  ledcWrite(motorEN, 0);
}

static void motorForward() {
  digitalWrite(motorIN1, HIGH);
  digitalWrite(motorIN2, LOW);
  ledcWrite(motorEN, myConfig.motorPwm);// ? myConfig.motorPwm : 128);
}

static void motorBackward() {
  digitalWrite(motorIN1, LOW);
  digitalWrite(motorIN2, HIGH);
  ledcWrite(motorEN, myConfig.motorPwm);// ? myConfig.motorPwm : 128);
}

static long minutesSinceLastMove() {
  if (!myConfig.lastMoveDate) return 9999;

  uint32_t d = myConfig.lastMoveDate;
  uint8_t  dy = d % 100; d /= 100;
  uint8_t  mn = d % 100; d /= 100;
  uint16_t yr = (uint16_t)d;

  uint16_t t  = myConfig.lastMoveTime;
  uint8_t  lm = t % 100; t /= 100;
  uint8_t  lh = (uint8_t)t;

  now = rtc.now();
  double mins = daysBetween(yr, mn, dy, lh, lm,
                            now.year(), now.month(), now.day(),
                            now.hour(), now.minute(), true);
  return (long)mins;
}

static void saveLastMove() {
  now = rtc.now();
  myConfig.lastMoveTime = now.hour() * 100 + now.minute();
  myConfig.lastMoveDate = (uint32_t)now.year()  * 10000UL
                        + (uint32_t)now.month() * 100UL
                        + now.day();
  saveJson(CONFIG_FILE, myConfig);
}

// ── centreTray() ─────────────────────────────────────────────
// centreTray() is now a non-blocking state machine.
// Each sub-step is handled inside motorHandler() every loop iteration
// so heatingSys() is never blocked.
// States: MOTOR_CENTRING reused, sub-step tracked by centreStep
enum CentreStep { CS_HOME, CS_TIME_TRAVEL, CS_SETTLE1, CS_REVERSE, CS_SETTLE2, CS_DONE };
static CentreStep centreStep   = CS_HOME;
static unsigned long centreT0  = 0;

void centreTrayStart() {
  // Called once to kick off the non-blocking centring sequence
  centreStep = CS_HOME;
  centreT0   = 0;
  motorState = MOTOR_CENTRING;
  motorBackward();  // Step 1: drive to front
}

// Called every loop iteration while motorState == MOTOR_CENTRING
// Returns true when centring is complete
bool centreTrayTick() {
  unsigned long now_ms = millis();

  switch(centreStep) {

    case CS_HOME:
      // Waiting for front limit switch (LOW = pressed)
      if(!digitalRead(limitSwitch1)) {
        motorStop();
        trayAtFront = true;
        centreT0    = now_ms;          // start settle timer
        centreStep  = CS_SETTLE1;
      } else if(now_ms - centreT0 > MOTOR_TIMEOUT_MS && centreT0 != 0) {
        motorStop();                   // timeout — give up, mark centred
        motorState = MOTOR_IDLE;
        myConfig.trayCentred  = true;
        myConfig.trayStopDone = true;
        saveJson(CONFIG_FILE,myConfig);
        centreStep = CS_DONE;
        setError(ERROR_SLOW);
        motorWarnClearMs = millis() + 10000UL;
        return true;
      } else if(centreT0 == 0) {
        centreT0 = now_ms;             // arm timeout on first tick
      }
      break;

    case CS_SETTLE1:
      if(now_ms - centreT0 >= 300) {  // 300ms settle — no delay()
        motorForward();                // Step 2: time full travel
        centreT0   = now_ms;
        centreStep = CS_TIME_TRAVEL;
      }
      break;

    case CS_TIME_TRAVEL:
      // Waiting for back limit switch (LOW = pressed)
      if(!digitalRead(limitSwitch2)) {
        fullTravelMs = now_ms - centreT0;
        motorStop();
        trayAtFront = false;
        centreT0    = now_ms;
        centreStep  = CS_SETTLE2;
      } else if(now_ms - centreT0 > MOTOR_TIMEOUT_MS) {
        motorStop();
        motorState = MOTOR_IDLE;
        myConfig.trayCentred  = true;
        myConfig.trayStopDone = true;
        saveJson(CONFIG_FILE,myConfig);
        centreStep = CS_DONE;
        setError(ERROR_SLOW);
        motorWarnClearMs = millis() + 10000UL;
        return true;
      }
      break;

    case CS_SETTLE2:
      if(now_ms - centreT0 >= 300) {  // 300ms settle
        motorBackward();               // Step 3: reverse half travel
        centreT0   = now_ms;
        centreStep = CS_REVERSE;
      }
      break;

    case CS_REVERSE:
      if(now_ms - centreT0 >= fullTravelMs / 2) {
        motorStop();
        myConfig.trayCentred  = true;
        myConfig.trayStopDone = true;
        saveJson(CONFIG_FILE,myConfig);
        motorState = MOTOR_IDLE;
        centreStep = CS_DONE;
        return true;
      }
      break;

    case CS_DONE:
      return true;
  }
  return false;
}

// ── motorHandler() ───────────────────────────────────────────
// Call every loop() iteration while proc > 0.
void motorHandler() {
  // Auto-clear ERROR_SLOW after 10s following a motor timeout
  if(motorWarnClearMs && millis() >= motorWarnClearMs) {
    motorWarnClearMs = 0;
    setError(NONE);
  }

  // getIncDay() is expensive (RTC I2C every call) — cache it once per second
  // using the same incLastSec gate that the main loop already uses
  static int  cachedDay     = 1;
  static unsigned long dayLastMs = 0;
  unsigned long nowMs = millis();
  if(nowMs - dayLastMs >= 1000) {
    dayLastMs  = nowMs;
    cachedDay  = getIncDay();
  }

  uint8_t sPhase = (myConfig.proc == 2)
                 ? myConfig.csettingPhase
                 : myConfig.settingPhase;
  bool inHatching = (cachedDay > sPhase);

  // Hatching: stop turning, centre once then do nothing
  if (inHatching) {
    if (!myConfig.trayStopDone) {
      motorStop();
      motorState            = MOTOR_IDLE;
      myConfig.trayStopDone = true;
      myConfig.trayCentred  = false;
      saveJson(CONFIG_FILE,myConfig);
    }
    if (!myConfig.trayCentred) {
      if(motorState != MOTOR_CENTRING)
        centreTrayStart();   // kick off non-blocking centring
      else
        centreTrayTick();    // advance state machine each loop iteration
    }
    return;
  }

  // Setting phase state machine
  switch (motorState) {

    case MOTOR_IDLE: {
      long elapsed = minutesSinceLastMove();
      if (elapsed < (long)myConfig.moveInterval) return;
      // Interval elapsed — start next move
      if (trayAtFront) {
        motorForward();
        Serial.println(F("[MOTOR] FRONT→BACK"));
      } else {
        motorBackward();
        Serial.println(F("[MOTOR] BACK→FRONT"));
      }
      motorMoveStart = millis();
      motorState     = MOTOR_MOVING;
      break;
    }

    case MOTOR_MOVING: {
      bool sw1     = !digitalRead(limitSwitch1); // LOW=pressed=front
      bool sw2     = !digitalRead(limitSwitch2); // LOW=pressed=back
      bool reached = trayAtFront ? sw2 : sw1;
      bool timeout = (millis() - motorMoveStart) > MOTOR_TIMEOUT_MS;

      if (reached || timeout) {
        motorStop();
        if (timeout && !reached) {
          Serial.println(F("[MOTOR] WARNING: travel timeout"));
          setError(ERROR_SLOW);
          motorWarnClearMs = millis() + 10000UL;
        } else {
          trayAtFront = !trayAtFront;
          Serial.printf("[MOTOR] Reached %s\n", trayAtFront ? "FRONT" : "BACK");
        }
        saveLastMove();
        motorState = MOTOR_IDLE;
      }
      break;
    }

    case MOTOR_CENTRING:
      // centreTray() handles this state directly
      break;
  }
}


bool appendCSV(const char* filename, const char* row) 
{
  char fullPath[64];
  snprintf(fullPath, sizeof(fullPath), "%s/%s", LOG_DIR, filename);

  if (!sd.exists(fullPath)) {
    Serial.println("File not found");
    return false;
  }

  ExFile file;

  if (!file.open(fullPath, O_WRITE | O_APPEND)) {
    Serial.println("Open failed");
    return false;
  }

  file.println(row);

  // for reliability
  //file.flush();

  file.close();

  return true;
}



