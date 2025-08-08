#include <M5Unified.h>
#include <BluetoothSerial.h>

// tipo di M5Stack
//#define M5STACK_CORE2

#define DEBUG

// Def GPIO
#ifdef M5STACK_CORE2
   
  #define USE_TOUCH_BUTTONS
#else
  // M5Stack Core classico
  #define ButtonC GPIO_NUM_37
  #define ButtonB GPIO_NUM_38
  #define ButtonA GPIO_NUM_39
#endif

// Def PID
#define PID_COOLANT_TEMP        "0105"
#define PID_AIR_INTAKE_TEMP     "010F"
#define PID_RPM                 "010C"
#define PID_ENGINE_LOAD         "0104"
#define PID_MAF                 "0110"
#define PID_FUEL_SYSTEM_STATUS  "0103"
#define PID_BAROMETRIC_PRESSURE "0133"
#define PID_BATTERY_VOLTAGE     "0142"
#define PID_DTC_STATUS          "0101"
#define PID_VEHICLE_SPEED       "010D"

#define m5Name           "M5Stack_OBD"

BluetoothSerial ELM_PORT;
bool counter = false;
volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 100;

// Variabili per touch (Core2)
#ifdef USE_TOUCH_BUTTONS
  bool lastTouchState = false;
  int lastTouchX = 0, lastTouchY = 0;
#endif

// Dichiarazione funzioni
bool ELMinit();
bool BTconnect();
bool sendAndReadCommand(const char* cmd, String& response, int delayTime);
void updateDisplay();
void dataRequestOBD();
void displayDebugMessage(const char* message, int x, int y, uint16_t textColour);
void sendOBDCommand(const char* cmd);
void writeToCircularBuffer(char c);
void handleOBDResponse();
void handleInput(); // Nuova funzione per gestire input universale
void drawTouchButtons(); // Per Core2

// funzioni lcd
void mainScreen();
void runningScreen();
void coolantScreen();
void rpmScreen();
void engineLoadScreen();
void mafScreen();
void barometricScreen();
void dtcStatusScreen();
void IRAM_ATTR indexUp();
void IRAM_ATTR indexDown();
String readFromCircularBuffer(int numChars);
String bufferSerialData(int timeout, int numChars);
void parseOBDData(const String& response);

float parseCoolantTemp(const String& response);
float parseIntakeTemp(const String& response);
float parseRPM(const String& response);
float parseEngineLoad(const String& response);
float parseOBDVoltage(const String& response);
float parseMAF(const String& response);
float parseBarometricPressure(const String& response);
String parseDTCStatus(const String& response);

uint8_t BLEAddress0[6] = {0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03};  // Indirizzo Bluetooth del modulo ELM327
uint8_t BLEAddress1[6] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xBA};  // Indirizzo Bluetooth del modulo ELM327 economico

float coolantTemp = 0.0;
float oilTemp = 0.0;
float intakeTemp = 0.0;
float rpm = 0.0;
float obdVoltage = 0.0;
float engineLoad = 0.0;
float MAF = 0.0;
float speed = 0.0;
float lastRpm = 0.0;
float lastSpeed = 0.0;
float lastCoolantTemp = -999.0;
float lastIntakeTemp = -999.0;
float lastOilTemp = -999.0;
float lastEngineLoad = -999.0;
float lastMAFvalue = -999.0;
float barometricPressure = 0.0;
float dtcStatus = 0.0;

bool firstMainScreen = true;
bool firstRunningScreen = true;
bool firstEngineScreen = true;
bool firstRpmScreen = true;
bool firstCoolantScreen = true;
bool firstBarScreen = true;
bool firstMafScreen = true;

const unsigned long voltageQueryInterval = 1000; // Intervallo di X secondi

unsigned long lastVoltageQueryTime = 0;
unsigned long lastOBDQueryTime = 0;
unsigned long OBDQueryInterval = 200; // Intervallo query OBD in millisecondi

const int BUFFER_SIZE = 256;
char circularBuffer[BUFFER_SIZE];
int writeIndex = 0;
int readIndex = 0;
int screenIndex[7] = {0, 1, 2, 3, 4, 5, 6};
int z = 1;

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  
  M5.Display.setTextSize(2);
  M5.Display.setRotation(1);
  M5.Display.setTextColor(WHITE);
  M5.Display.fillScreen(BLACK);
  
  Serial.begin(115200);
  Serial.println("Setup in corso...");

#ifdef M5STACK_CORE2
  Serial.println("M5Stack Core2 detected - Using touch interface");
  drawTouchButtons();
#else
  Serial.println("M5Stack Core detected - Using physical buttons");
  //pinMode(ButtonA, INPUT); //non è possibile collegare un interrupt
  pinMode(ButtonB, INPUT);
  pinMode(ButtonC, INPUT);
  
  //attachInterrupt(digitalPinToInterrupt(ButtonA), indexUp, RISING);
  attachInterrupt(digitalPinToInterrupt(ButtonB), indexUp, RISING);
  attachInterrupt(digitalPinToInterrupt(ButtonC), indexDown, RISING);
#endif

  BTconnect();
  delay(500);
  ELMinit();
  delay(500);
}

void loop() {
  M5.update(); 

  // Gestione input universale (NON USATO)
  handleInput();

  switch (screenIndex[z]) {
    case 0: mainScreen(); break;
    case 1: runningScreen(); break;
    case 2: coolantScreen(); break;
    case 3: rpmScreen(); break;
    case 4: engineLoadScreen(); break;
    case 5: barometricScreen(); break;
    case 6: mafScreen(); break;
  }

  
// PER DEBUG
/*
coolantTemp = coolantTemp + 1;
intakeTemp = intakeTemp + 1;
rpm = rpm + 1000;
obdVoltage = obdVoltage + 1;
engineLoad = engineLoad + 1;
MAF = MAF + 1;
*/
}

void handleInput() {
#ifdef USE_TOUCH_BUTTONS
  // Gestione touch per Core2
  auto t = M5.Touch.getDetail();
  if (t.wasPressed()) {
    if (t.x < 160 && t.y > 200) { // Pulsante sinistro
      indexDown();
    } else if (t.x >= 160 && t.y > 200) { // Pulsante destro
      indexUp();
    }
  }
#else
  // Per Core BASIC, gli interrupt gestiranno i pulsanti
#endif
}

#ifdef USE_TOUCH_BUTTONS
void drawTouchButtons() {
  // Disegna pulsanti virtuali per Core2
  M5.Display.fillRect(10, 210, 140, 25, DARKGREY);
  M5.Display.fillRect(170, 210, 140, 25, DARKGREY);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(1);
  M5.Display.drawString("< PREV", 60, 220);
  M5.Display.drawString("NEXT >", 220, 220);
  M5.Display.setTextSize(2);
}
#endif

void mainScreen() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastOBDQueryTime >= OBDQueryInterval) {
    lastOBDQueryTime = currentMillis;
    dataRequestOBD();
    handleOBDResponse();
    updateDisplay();
  }
}

void runningScreen() {
   static float lastLoad = -999, lastCoolant = -999;

  sendOBDCommand(PID_RPM);  delay(50); handleOBDResponse();  // engineLoad (0–100)
  sendOBDCommand(PID_COOLANT_TEMP); delay(50); handleOBDResponse();  // coolantTemp (0–130)

// Parametri gauge
  const int cx       = 160;    // centro X
  const int cxRpm    = cx + 40;
  const int cxCool   = cx - 40;
  const int cy       = 140;    // centro Y
  const int radius   = 100;    // raggio esterno
  const int maxRpmValue = 8000;    // valore massimo rpm
  const int maxCoolantValue = 140; // valore massimo temperatura acqua

  // Lancette precedenti
  static int lastRpmX = cxRpm, lastRpmY = cy - (radius - 15);
  static int lastCoolantX = cxCool, lastCoolantY = cy - (radius - 15);
  // Buffer scia (ultime 5 posizioni) RPM
  const int TRAIL_LEN = 5;
  static int histRpmX[TRAIL_LEN], histRpmY[TRAIL_LEN];
  static int histRpmCnt = 0, histRpmIdx = 0;
  static int histCoolantX[TRAIL_LEN], histCoolantY[TRAIL_LEN];
  static int histCoolantCnt = 0, histCoolantIdx = 0;

  // Disegno iniziale del gauge
  if (firstRunningScreen) {
    M5.Lcd.fillScreen(BLACK);
  #ifdef USE_TOUCH_BUTTONS
    drawTouchButtons();
  #endif

    // Titolo
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(DARKGREY);
    M5.Lcd.setCursor(cx - 120, 5);
    M5.Lcd.print("Temp");
    M5.Lcd.setCursor(cx + 80, 5);
    M5.Lcd.print("RPM");

    // Semicerchio colorato da -90° a +90°
    for (int v = 0; v <= maxRpmValue; v++) {
      int angDeg = map(v, 0, maxRpmValue, -90, 90);
      float ang  = angDeg * PI / 180.0f;
      uint16_t col;
      if (v <= 3000)       col = GREEN;   // normale
      else if (v <= 5000) col = YELLOW;  // warning
      else               col = RED;     // alto

      int x1 = cxRpm + cos(ang) * radius;
      int y1 = cy + sin(ang) * radius;
      int x2 = cxRpm + cos(ang) * (radius - 12);
      int y2 = cy + sin(ang) * (radius - 12);
      M5.Lcd.drawLine(x1, y1, x2, y2, col);
    }

    for (int v = 0; v <= maxCoolantValue; v++) {
      int angDeg = map(v, 0, maxCoolantValue, -270, -90);
      float ang  = angDeg * PI / 180.0f;
      uint16_t col;
      if (v <= 40)       col = BLUE;   // normale
      else if (v <= 85) col = CYAN;  // warning
      else if (v <= 100) col = GREEN;  // warning
      else if (v <= 108) col = ORANGE;  // warning
      else               col = RED;     // alto

      int x1 = cxCool + cos(ang) * radius;
      int y1 = cy + sin(ang) * radius;
      int x2 = cxCool + cos(ang) * (radius - 12);
      int y2 = cy + sin(ang) * (radius - 12);
      M5.Lcd.drawLine(x1, y1, x2, y2, col);
    }    
    // Interno nero
    M5.Lcd.fillCircle(cx, cy, radius - 12, BLACK);

    // Tacche ogni 2000 
    for (int v = 0; v <= maxRpmValue; v += 2000) {
      float ang = map(v, 0, maxRpmValue, -90, 90) * PI / 180.0f;
      int x1 = cxRpm + cos(ang) * (radius - 12);
      int y1 = cy + sin(ang) * (radius - 12);
      int x2 = cxRpm + cos(ang) * radius;
      int y2 = cy + sin(ang) * radius;
      M5.Lcd.drawLine(x1, y1, x2, y2, WHITE);
    }

  // Numeri ogni 2000 RPM
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE);
    for (int v = 0; v <= maxRpmValue; v += 2000) {
      float ang = map(v, 0, maxRpmValue, -90, 90) * PI / 180.0f;
      int xt = cxRpm + cos(ang) * (radius - 28) - 10;
      int yt = cy + sin(ang) * (radius - 28) - 8;
      M5.Lcd.drawNumber(v, xt, yt);
    }
//CERCHIO PER COOLANT
    for (int v = 0; v <= maxCoolantValue; v += 20) {
      float ang = map(v, 0, maxCoolantValue, -270, -90) * PI / 180.0f;
      int xt = cxCool + cos(ang) * (radius - 28) - 10;
      int yt = cy + sin(ang) * (radius - 28) - 8;
      M5.Lcd.drawNumber(v, xt, yt);
    }

    // Reset flag globali
    firstMainScreen    = true;
    firstRunningScreen = false;
    firstEngineScreen  = true;
    firstBarScreen     = true;
    firstMafScreen     = true;
    firstRpmScreen     = true;
    firstCoolantScreen = true;
  }
/* - - - - RPM - - - -*/
  // Cancella scia e lancetta precedenti RPM

  for (int i = 0; i < histRpmCnt; i++) {
    int idx = (histRpmIdx + i) % TRAIL_LEN;
    M5.Lcd.drawLine(cxRpm, cy, histRpmX[idx], histRpmY[idx], BLACK);
  }
  M5.Lcd.drawLine(cxRpm, cy, lastRpmX, lastRpmY, BLACK);

    // Calcola nuova posizione lancetta
  int angleDeg = map((int)rpm, 0, maxRpmValue, -90, 90);
  float rad    = angleDeg * PI / 180.0f;
  int nx = cxRpm + cos(rad) * (radius - 15);
  int ny = cy + sin(rad) * (radius - 15);

  // Aggiorna buffer scia
  histRpmX[histRpmIdx] = nx;
  histRpmY[histRpmIdx] = ny;
  if (histRpmCnt < TRAIL_LEN) histRpmCnt++;
  histRpmIdx = (histRpmIdx + 1) % TRAIL_LEN;

  // Disegna scia (5 tratti verdi sfumati)
  for (int i = 0; i < histRpmCnt; i++) {
    int idx = (histRpmIdx + i) % TRAIL_LEN;
    float t = float(i + 1) / histRpmCnt;         // più vecchio → t piccolo
    uint8_t g = uint8_t(t * 255);             // intensità verde
    uint16_t col = M5.Lcd.color565(0, g, 0);
    M5.Lcd.drawLine(cxRpm, cy, histRpmX[idx], histRpmY[idx], col);
  }

  // Disegna lancetta attuale (rossa)
  M5.Lcd.drawLine(cxRpm, cy, nx, ny, RED);
  lastRpmX = nx;
  lastRpmY = ny;

/* - - - - COOLANT - - - -*/
  // Cancella scia e lancetta precedenti

  for (int i = 0; i < histCoolantCnt; i++) {
    int idx = (histCoolantIdx + i) % TRAIL_LEN;
    M5.Lcd.drawLine(cxCool, cy, histCoolantX[idx], histCoolantY[idx], BLACK);
  }
  M5.Lcd.drawLine(cxCool, cy, lastCoolantX, lastCoolantY, BLACK);

    // Calcola nuova posizione lancetta
  int angleCoolantDeg = map((int)coolantTemp, 0, maxCoolantValue, -270, -90);
  float radCoolant    = angleCoolantDeg * PI / 180.0f;
  int nxCoolant = cxCool + cos(radCoolant) * (radius - 15);
  int nyCoolant = cy + sin(radCoolant) * (radius - 15);

  // Aggiorna buffer scia
  histCoolantX[histCoolantIdx] = nxCoolant;
  histCoolantY[histCoolantIdx] = nyCoolant;
  if (histCoolantCnt < TRAIL_LEN) histCoolantCnt++;
  histCoolantIdx = (histCoolantIdx + 1) % TRAIL_LEN;

  // Disegna scia (5 tratti verdi sfumati)
  for (int i = 0; i < histCoolantCnt; i++) {
    int idx = (histCoolantIdx + i) % TRAIL_LEN;
    float t = float(i + 1) / histCoolantCnt;         // più vecchio → t piccolo
    uint8_t g = uint8_t(t * 255);             // intensità verde
    uint16_t col = M5.Lcd.color565(0, g, 0);
    M5.Lcd.drawLine(cxCool, cy, histCoolantX[idx], histCoolantY[idx], col);
  }

  // Disegna lancetta attuale (rossa)
  M5.Lcd.drawLine(cxCool, cy, nxCoolant, nyCoolant, RED);
  lastRpmX = nx;
  lastRpmY = ny;
  lastCoolantX = nxCoolant;
  lastCoolantY = nyCoolant;



}


bool BTconnect() {
  ELM_PORT.begin(m5Name, true);  // Avvia la connessione Bluetooth
#ifdef DEBUG
  displayDebugMessage("Connessione BT...", 0, 200, WHITE);
#endif
  int retries = 0;  // Tentativo connessione bluetooth ELM327 (5 try)
  bool connected = false;
  while (retries < 2 && !connected) {
    connected = ELM_PORT.connect(BLEAddress1);
    if (!connected) {
#ifdef DEBUG
      displayDebugMessage("BT Conn FAIL", 0, 200, WHITE);
#endif
      retries++;
      delay(500);
    }
  }

if (!connected) {
#ifdef DEBUG
    displayDebugMessage("ELM BT NOT FOUND", 0, 200, WHITE);
#endif
    return false;  // Loop infinito se non riesce a connettersi ( NON PRORPIO :-) )
  } else {
#ifdef DEBUG
    displayDebugMessage("Connessione BT OK!", 0, 200, WHITE);
#endif
    return true;
  }
}

bool sendAndReadCommand(const char* cmd, String& response, int delayTime) {
  response = "";
  unsigned long startTime = millis();
  ELM_PORT.print(cmd);
  ELM_PORT.print("\r\n");

  while (millis() - startTime < delayTime) {
    if (ELM_PORT.available()) {
      char c = ELM_PORT.read();
      response += c;
    }
    delay(30); // Breve delay per non sovraccaricare la CPU
  }
  response.trim(); // Rimuove spazi bianchi

  if (response.length() > 0) {
#ifdef DEBUG
    displayDebugMessage(response.c_str(), 0, 200, GREEN);
#endif
  }

  if (response.length() == 0) {
#ifdef DEBUG
    Serial.println("No RCV");
#endif
    return false;
  }

  if (response.indexOf("OK") >= 0 || response.length() > 0) {
    return true;
  } else {
    Serial.println("Err: " + response);
    return false;
  }
}

void sendOBDCommand(const char* cmd) {
  ELM_PORT.print(cmd);
  ELM_PORT.print("\r\n");
}

String bufferSerialData(int timeout, int numChars) {
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    while (ELM_PORT.available()) {
      char c = ELM_PORT.read();
      writeToCircularBuffer(c);
    }
    delay(30); // Breve delay per non sovraccaricare la CPU
  }
  return readFromCircularBuffer(numChars);
}

void dataRequestOBD() {
  static unsigned long lastCommandTime = 0;
  static int commandIndex = 0;
  unsigned long currentMillis = millis();

  const char* commands[] = {PID_COOLANT_TEMP, PID_AIR_INTAKE_TEMP, PID_RPM, PID_ENGINE_LOAD, PID_MAF};
  int numCommands = sizeof(commands) / sizeof(commands[0]);

  if (currentMillis - lastCommandTime >= OBDQueryInterval / numCommands) {
    if (commandIndex < numCommands) {
      sendOBDCommand(commands[commandIndex]);
      delay(40);
      commandIndex++;
    } else {
      commandIndex = 0;
      if (millis() - lastVoltageQueryTime >= voltageQueryInterval) {
        sendOBDCommand("ATRV");
        lastVoltageQueryTime = millis();
      }
    }
    lastCommandTime = currentMillis;
  }
}

void handleOBDResponse() {
  String response = bufferSerialData(250, 100);  // Riempimento del buffer
  parseOBDData(response);  // Parsing del buffer
}

void parseOBDData(const String& response) {
  if (response.indexOf("4105") >= 0) {
    coolantTemp = parseCoolantTemp(response);
  }
  else if (response.indexOf("410C") >= 0) {
    rpm = parseRPM(response);
  }
  else if (response.indexOf("410F") >= 0) {
    intakeTemp = parseIntakeTemp(response);
  }
  else if (response.indexOf("V") >= 0) {
    obdVoltage = parseOBDVoltage(response);
  }
  else if (response.indexOf("4104") >= 0) {
    engineLoad = parseEngineLoad(response);
  }
  else if (response.indexOf("4110") >= 0) {
    MAF = parseMAF(response);
  }
}

float parseCoolantTemp(const String& response) {
  if (response.indexOf("4105") == 0) {
    byte tempByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    return tempByte - 40;
  }
  return lastCoolantTemp;
}

float parseSpeed(const String& response) {
  if (response.indexOf("010D") == 0) {
    byte tempByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    return tempByte - 40;
  }
  return lastSpeed;
}
float parseIntakeTemp(const String& response) {
  if (response.indexOf("410F") == 0) {
    byte tempByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    return tempByte - 40;
  }
  return lastIntakeTemp;
}

float parseRPM(const String& response) {
  if (response.indexOf("410C") == 0) {
    byte highByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    byte lowByte = strtoul(response.substring(6, 8).c_str(), NULL, 16);
    return (highByte * 256 + lowByte) / 4;
  }
  return 0.0;
}

float parseEngineLoad(const String& response) {
  if (response.indexOf("4104") == 0) {
    byte loadByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    return ((loadByte * 100.0) / 255.0)*1.2;          // 1.2 per questioni grafiche DA RIVEDERE
  }
  return lastEngineLoad;
}

float parseMAF(const String& response) {
  if (response.indexOf("4110") == 0) {
    int A = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    int B = strtoul(response.substring(6, 8).c_str(), NULL, 16);
    return ((A * 256) + B) / 100.0;
  }
  return lastMAFvalue;
}

float parseBarometricPressure(const String& response) {
  if (response.indexOf("4133") == 0) {
    byte pressureByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    return pressureByte;
  }
  return 0.0;
}

float parseOBDVoltage(const String& response) {
  int indexV = response.indexOf('V');
  if (indexV >= 0) {
    String voltageStr = response.substring(0, indexV);
    return voltageStr.toFloat();
  }
  return 0.0;
}

String parseDTCStatus(const String& response) {
  if (response.indexOf("4101") == 0) {
    return response.substring(4);  // Estrae il messaggio DTC completo
  }
  return "";
}

void updateDisplay() {
  static float lastCoolantTemp = -999.0;
  static float lastObdVoltage = -999.0;
  static float lastRpm = -999.0;
  static float lastIntakeTemp = -999.0;
  static float lastEngineLoad = -999.0;
  static float lastMAF = -999.0;

  if (firstMainScreen) {
    M5.Display.fillScreen(BLACK);
    for(int i = 15; i < 160; i += 20) { 
      M5.Display.drawFastHLine(0, i, 320, OLIVE); 
    }
    M5.Display.drawFastVLine(240, 0, 155, OLIVE);
    M5.Display.setCursor(0, 0);
    M5.Display.setTextSize(2);
    
#ifdef USE_TOUCH_BUTTONS
    drawTouchButtons();
#endif
    
    // Reset flag globali
    firstMainScreen    = false;
    firstRunningScreen = true;
    firstEngineScreen  = true;
    firstBarScreen     = true;
    firstMafScreen     = true;
    firstRpmScreen     = true;
    firstCoolantScreen = true;
  }

  if (coolantTemp != lastCoolantTemp) {
    M5.Display.fillRect(0, 0, 320, 20, BLACK);  // Aggiorna solo la parte della temperatura del liquido refrigerante
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor((coolantTemp < 50) ? LIGHTGREY : (coolantTemp >= 40 && coolantTemp <= 65) ? BLUE : (coolantTemp > 65 && coolantTemp <= 80) ? GREENYELLOW : (coolantTemp >= 81 && coolantTemp <= 100) ? GREEN : (coolantTemp <= 102) ? ORANGE : RED);
    M5.Display.printf("Acqua: %.1f C\n", coolantTemp);
    lastCoolantTemp = coolantTemp;
  }

  if (obdVoltage != lastObdVoltage) {
    M5.Display.fillRect(0, 20, 320, 20, BLACK);  // Aggiorna solo la parte della tensione OBD
    M5.Display.setCursor(0, 20);
    M5.Display.setTextColor((obdVoltage < 11.8) ? RED : (obdVoltage > 11.8 && obdVoltage <= 12.1) ? YELLOW : (obdVoltage > 12.1 && obdVoltage <= 13.8) ? GREEN : (obdVoltage > 13.8 && obdVoltage <= 14.5) ? GREENYELLOW : ORANGE);
    M5.Display.printf("V OBD : %.2f V\n", obdVoltage);
    lastObdVoltage = obdVoltage;
  }

  if (rpm != lastRpm) {
    M5.Display.fillRect(0, 40, 320, 20, BLACK);  // Aggiorna solo la parte del RPM
    M5.Display.setCursor(0, 40);
    M5.Display.setTextColor(WHITE);
    M5.Display.printf("RPM: %.0f\n", rpm);
    lastRpm = rpm;
  }

  if (intakeTemp != lastIntakeTemp) {
    M5.Display.fillRect(0, 60, 320, 20, BLACK);  // Aggiorna solo la parte della temperatura di aspirazione
    M5.Display.setCursor(0, 60);
    M5.Display.setTextColor(WHITE);
    M5.Display.printf("Intake Temp: %.1f C\n", intakeTemp);
    lastIntakeTemp = intakeTemp;
  }

  if (MAF != lastMAF) {
    M5.Display.fillRect(0, 100, 320, 20, BLACK);  // Aggiorna solo la parte del MAF
    M5.Display.setCursor(0, 100);
    M5.Display.setTextColor(WHITE);
    M5.Display.printf("MAF: %.1f%%\n", MAF);
    lastMAF = MAF;
  }

  if (engineLoad != lastEngineLoad) {
    M5.Display.fillRect(0, 80, 320, 20, BLACK);  // Aggiorna solo la parte del carico del motore
    M5.Display.setCursor(0, 80);
    M5.Display.setTextColor(WHITE);
    M5.Display.printf("Load: %.1f%%\n", engineLoad);
    lastEngineLoad = engineLoad;
  }
}

void displayDebugMessage(const char* message, int x, int y, uint16_t textColour) {
  M5.Display.setTextColor(textColour);
  M5.Display.fillRect(x, y, 320, 40, BLACK);  // Pulisce la parte bassa del display
  M5.Display.setCursor(x, y);
  M5.Display.print(message);
  Serial.println(message);
}

bool ELMinit() {
  String response;

#ifdef DEBUG
  displayDebugMessage("ELM init...", 0, 200, WHITE);
#endif

  if (!sendAndReadCommand("ATZ", response, 1500)) {
#ifdef DEBUG
    displayDebugMessage("Err ATZ", 0, 20, WHITE);
#endif
    return false;
  }
#ifdef DEBUG
  displayDebugMessage(response.c_str(), 0, 20, WHITE);
#endif

  if (!sendAndReadCommand("ATE0", response, 1500)) {
#ifdef DEBUG
    displayDebugMessage("Err ATE0", 0, 40, WHITE);
#endif
    return false;
  }
#ifdef DEBUG
  displayDebugMessage(response.c_str(), 0, 40, WHITE);
#endif

  if (!sendAndReadCommand("ATL0", response, 1500)) {
#ifdef DEBUG
    displayDebugMessage("Err ATL0", 0, 60, WHITE);
#endif
    return false;
  }
#ifdef DEBUG
  displayDebugMessage(response.c_str(), 0, 60, WHITE);
#endif

  if (!sendAndReadCommand("ATS0", response, 1500)) {
#ifdef DEBUG
    displayDebugMessage("Err ATS0", 0, 80, WHITE);
#endif
    return false;
  }
#ifdef DEBUG
  displayDebugMessage(response.c_str(), 0, 80, WHITE);
#endif

  if (!sendAndReadCommand("ATST0A", response, 1500)) {
#ifdef DEBUG
    displayDebugMessage("Err ATST0A", 0, 100, WHITE);
#endif
    return false;
  }
#ifdef DEBUG
  displayDebugMessage(response.c_str(), 0, 100, WHITE);
#endif

  if (!sendAndReadCommand("ATSP0", response, 15000)) {  // Imposta protocollo automatico SP 0 e gestire la risposta "SEARCHING..."
#ifdef DEBUG
    displayDebugMessage("Err ATSP0", 0, 120, WHITE);
#endif
    return false;
  }
#ifdef DEBUG
  displayDebugMessage(response.c_str(), 0, 120, WHITE);
#endif

  return true;
}

void writeToCircularBuffer(char c) {
  circularBuffer[writeIndex] = c;
  writeIndex = (writeIndex + 1) % BUFFER_SIZE;
  if (writeIndex == readIndex) {
    readIndex = (readIndex + 1) % BUFFER_SIZE; // Sovrascrivi i dati più vecchi
  }
}

String readFromCircularBuffer(int numChars) {
  String result = "";
  int charsRead = 0;

  // Leggi dal buffer finché ci sono caratteri da leggere
  // e non si è raggiunto il numero massimo di caratteri richiesti
  while (readIndex != writeIndex && charsRead < numChars) {
    result += circularBuffer[readIndex];
    readIndex = (readIndex + 1) % BUFFER_SIZE;
    charsRead++;
  }

  result.trim();  // Rimuove gli spazi bianchi extra all'inizio e alla fine
  return result;
}

void coolantScreen() {

  sendOBDCommand(PID_COOLANT_TEMP);
  delay(20);
  handleOBDResponse();

  // Parametri gauge
  const int cx       = 160;    // centro X
  const int cy       = 140;    // centro Y
  const int radius   = 100;    // raggio esterno
  const int maxValue = 140;    // valore massimo (°C)

  // Lancetta precedente
  static int lastX = cx, lastY = cy - (radius - 15);

  // Buffer scia (ultime 5 posizioni)
  const int TRAIL_LEN = 5;
  static int histX[TRAIL_LEN], histY[TRAIL_LEN];
  static int histCnt = 0, histIdx = 0;

  // Disegno iniziale del gauge
  if (firstCoolantScreen) {
    M5.Lcd.fillScreen(BLACK);
  #ifdef USE_TOUCH_BUTTONS
    drawTouchButtons();
  #endif

    // Titolo
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(LIGHTGREY);
    M5.Lcd.setCursor(cx - 70, 10);
    M5.Lcd.print("Coolant");

    // Semicerchio colorato da -90° a +90°
    for (int v = 0; v <= maxValue; v++) {
      int angDeg = map(v, 0, maxValue, -225, 55);
      float ang  = angDeg * PI / 180.0f;
      uint16_t col;
      if      (v <=  30) col = BLUE;
      else if (v <=  60) col = CYAN;
      else if (v <= 100) col = GREEN;
      else if (v <= 107) col = GREENYELLOW;
      else if (v <= 111) col = YELLOW;
      else               col = RED;

      int x1 = cx + cos(ang) * radius;
      int y1 = cy + sin(ang) * radius;
      int x2 = cx + cos(ang) * (radius - 12);
      int y2 = cy + sin(ang) * (radius - 12);
      M5.Lcd.drawLine(x1, y1, x2, y2, col);
    }

    // Interno del gauge
    M5.Lcd.fillCircle(cx, cy, radius - 12, BLACK);

    // Tacche ogni 5 °C
    for (int v = 0; v <= maxValue; v += 5) {
      float ang = map(v, 0, maxValue, -225, 55) * PI / 180.0f;
      int x1 = cx + cos(ang) * (radius - 12);
      int y1 = cy + sin(ang) * (radius - 12);
      int x2 = cx + cos(ang) * radius;
      int y2 = cy + sin(ang) * radius;
      M5.Lcd.drawLine(x1, y1, x2, y2, WHITE);
    }

    // Numeri ogni 20 °C
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    for (int v = 0; v <= maxValue; v += 20) {
      float ang = map(v, 0, maxValue, -225, 55) * PI / 180.0f;
      int xt = cx + cos(ang) * (radius - 28) - 10;
      int yt = cy + sin(ang) * (radius - 28) - 8;
      M5.Lcd.drawNumber(v, xt, yt);
    }

    // Reset dei flag globali
    firstMainScreen    = true;
    firstRunningScreen = true;
    firstEngineScreen  = true;
    firstBarScreen     = true;
    firstMafScreen     = true;
    firstRpmScreen     = true;
    firstCoolantScreen = false;
  }

  // Cancella scia precedente
  for (int i = 0; i < histCnt; i++) {
    int idx = (histIdx + i) % TRAIL_LEN;
    M5.Lcd.drawLine(cx, cy, histX[idx], histY[idx], BLACK);
  }
  // Cancella vecchia lancetta
  M5.Lcd.drawLine(cx, cy, lastX, lastY, BLACK);

  // Calcola nuova posizione lancetta
  int angleDeg = map((int)coolantTemp, 0, maxValue, -225, 45);
  float rad    = angleDeg * PI / 180.0f;
  int nx = cx + cos(rad) * (radius - 15);
  int ny = cy + sin(rad) * (radius - 15);

  // Aggiorna buffer scia
  histX[histIdx] = nx;
  histY[histIdx] = ny;
  if (histCnt < TRAIL_LEN) histCnt++;
  histIdx = (histIdx + 1) % TRAIL_LEN;

  // Disegna scia (5 tratti verdi sfumati)
  for (int i = 0; i < histCnt; i++) {
    int idx = (histIdx + i) % TRAIL_LEN;
    float t = float(i + 1) / histCnt;            // più vecchio → t piccolo
    uint8_t g = uint8_t(t * 255);                // intensità verde
    uint16_t col = M5.Lcd.color565(0, g, 0);
    M5.Lcd.drawLine(cx, cy, histX[idx], histY[idx], col);
  }

  // Disegna lancetta attuale (rossa)
  M5.Lcd.drawLine(cx, cy, nx, ny, RED);
  lastX = nx;
  lastY = ny;

  // Valore numerico 
  M5.Lcd.setTextSize(3);
  
  if(coolantTemp <= 30)        M5.Lcd.setTextColor(BLUE, BLACK);
  else if (coolantTemp <= 60)  M5.Lcd.setTextColor(CYAN, BLACK);
  else if (coolantTemp <= 100) M5.Lcd.setTextColor(GREEN, BLACK);
  else if (coolantTemp <= 107) M5.Lcd.setTextColor(GREENYELLOW, BLACK);
  else if (coolantTemp <= 111) M5.Lcd.setTextColor(YELLOW, BLACK);
  else                         M5.Lcd.setTextColor(RED, BLACK);

  M5.Lcd.drawNumber(coolantTemp, cx - 30, cy + 70);
}

void engineLoadScreen() {

  sendOBDCommand(PID_ENGINE_LOAD);
  delay(20);
  handleOBDResponse();

  // Parametri gauge
  const int cx     = 160;    // centro X
  const int cy     = 140;    // centro Y
  const int radius = 100;    // raggio esterno

  // Stato primo disegno gauge e lancetta precedente
  static bool firstDrawGauge = true;
  static int  lastX = cx, lastY = cy - (radius - 15);

  // Buffer per effetto “scia”
  const int TRAIL_LEN = 5;
  static int histX[TRAIL_LEN], histY[TRAIL_LEN];
  static int histCnt = 0, histIdx = 0;

  if (firstEngineScreen) {
    // Inizializza display
    M5.Lcd.fillScreen(BLACK);
  #ifdef USE_TOUCH_BUTTONS
    drawTouchButtons();
  #endif

    // Titolo
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(cx - 30, 10);
    M5.Lcd.print("Load %");

    // Disegna zone colorate (semicerchio –90°…+90°)
    for (int v = 0; v <= 100; v++) {
      int angDeg = map(v, 0, 100, -90, 90);
      float ang  = angDeg * PI / 180.0f;
      uint16_t col;
      if (v <= 33)        col = GREEN;
      else if (v <= 66)   col = YELLOW;
      else                col = RED;

      int x1 = cx + cos(ang) * radius;
      int y1 = cy + sin(ang) * radius;
      int x2 = cx + cos(ang) * (radius - 12);
      int y2 = cy + sin(ang) * (radius - 12);
      M5.Lcd.drawLine(x1, y1, x2, y2, col);
    }

    // Interno gauge (sfondo nero)
    M5.Lcd.fillCircle(cx, cy, radius - 12, BLACK);

    // Tacche ogni 10 unità
    for (int v = 0; v <= 100; v += 10) {
      float ang = map(v, 0, 100, -90, 90) * PI / 180.0f;
      int x1 = cx + cos(ang) * (radius - 12);
      int y1 = cy + sin(ang) * (radius - 12);
      int x2 = cx + cos(ang) * radius;
      int y2 = cy + sin(ang) * radius;
      M5.Lcd.drawLine(x1, y1, x2, y2, WHITE);
    }

    // Numeri ogni 20 unità
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    for (int v = 0; v <= 100; v += 20) {
      float ang = map(v, 0, 100, -90, 90) * PI / 180.0f;
      int xt = cx + cos(ang) * (radius - 28) - 10;
      int yt = cy + sin(ang) * (radius - 28) - 8;
      M5.Lcd.drawNumber(v, xt, yt);
    }

    // Reset global flags
    firstMainScreen    = true;
    firstRunningScreen = true;
    firstEngineScreen  = false;
    firstBarScreen     = true;
    firstMafScreen     = true;
    firstRpmScreen     = true;
    firstCoolantScreen = true;
  }

  // Cancella scia precedente
  for (int i = 0; i < histCnt; i++) {
    int idx = (histIdx + i) % TRAIL_LEN;
    M5.Lcd.drawLine(cx, cy, histX[idx], histY[idx], BLACK);
  }
  // Cancella vecchia lancetta
  M5.Lcd.drawLine(cx, cy, lastX, lastY, BLACK);

  // Calcola nuova posizione lancetta
  int angleDeg = map((int)engineLoad, 0, 100, -90, 90);
  float rad    = angleDeg * PI / 180.0f;
  int nx = cx + cos(rad) * (radius - 15);
  int ny = cy + sin(rad) * (radius - 15);

  // Aggiorna buffer scia
  histX[histIdx] = nx;
  histY[histIdx] = ny;
  if (histCnt < TRAIL_LEN) histCnt++;
  histIdx = (histIdx + 1) % TRAIL_LEN;

  // Disegna scia (verde sfumato)
  for (int i = 0; i < histCnt; i++) {
    int idx = (histIdx + i) % TRAIL_LEN;
    float t = float(i + 1) / histCnt;           // t piccolo = più vecchio
    uint8_t g = uint8_t(t * 255);               // intensità verde
    uint16_t col = M5.Lcd.color565(0, g, 0);
    M5.Lcd.drawLine(cx, cy, histX[idx], histY[idx], col);
  }

  // Disegna lancetta attuale (rosso)
  M5.Lcd.drawLine(cx, cy, nx, ny, RED);
  lastX = nx;
  lastY = ny;

  // Mostra valore numerico al centro
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.drawNumber(engineLoad, cx - 30, cy - 15);
}
void mafScreen() {

  sendOBDCommand(PID_MAF);
  delay(20);
  handleOBDResponse();

  // Parametri gauge
  const int cx       = 160;    // centro X
  const int cy       = 140;    // centro Y
  const int radius   = 100;    // raggio esterno
  const int maxValue = 200;    // portata massima (g/s)

  // Lancetta precedente
  static int lastX = cx, lastY = cy - (radius - 15);

  // Buffer scia (ultime 5 posizioni)
  const int TRAIL_LEN = 5;
  static int histX[TRAIL_LEN], histY[TRAIL_LEN];
  static int histCnt = 0, histIdx = 0;

  // Primo avvio: disegno del quadrante
  if (firstMafScreen) {
    M5.Lcd.fillScreen(BLACK);
  #ifdef USE_TOUCH_BUTTONS
    drawTouchButtons();
  #endif

    // Titolo
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(cx - 30, 10);
    M5.Lcd.print("MAF");

    // Semicerchio colorato (-90° … +90°)
    for (int v = 0; v <= maxValue; v++) {
      int angDeg = map(v, 0, maxValue, -90, 90);
      float ang  = radians(angDeg);
      uint16_t col;
      if      (v <=  66)   col = GREEN;   // 0–⅓
      else if (v <= 133)   col = YELLOW;  // ⅓–⅔
      else                 col = RED;     // ⅔–1

      int x1 = cx + cos(ang) * radius;
      int y1 = cy + sin(ang) * radius;
      int x2 = cx + cos(ang) * (radius - 12);
      int y2 = cy + sin(ang) * (radius - 12);
      M5.Lcd.drawLine(x1, y1, x2, y2, col);
    }

    // Interno nero
    M5.Lcd.fillCircle(cx, cy, radius - 12, BLACK);

    // Tacche ogni 20 g/s
    for (int v = 0; v <= maxValue; v += 20) {
      float ang = radians(map(v, 0, maxValue, -90, 90));
      int x1 = cx + cos(ang) * (radius - 12);
      int y1 = cy + sin(ang) * (radius - 12);
      int x2 = cx + cos(ang) * radius;
      int y2 = cy + sin(ang) * radius;
      M5.Lcd.drawLine(x1, y1, x2, y2, WHITE);
    }

    // Numeri ogni 40 g/s
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    for (int v = 0; v <= maxValue; v += 40) {
      float ang = radians(map(v, 0, maxValue, -90, 90));
      int xt = cx + cos(ang) * (radius - 28) - 10;
      int yt = cy + sin(ang) * (radius - 28) - 8;
      M5.Lcd.setCursor(xt, yt);
      M5.Lcd.printf("%d", v);
    }

    firstMafScreen = false;
  }

  // Cancella scia e lancetta precedenti
  for (int i = 0; i < histCnt; i++) {
    int idx = (histIdx + i) % TRAIL_LEN;
    M5.Lcd.drawLine(cx, cy, histX[idx], histY[idx], BLACK);
  }
  M5.Lcd.drawLine(cx, cy, lastX, lastY, BLACK);

  // Nuova posizione della lancetta
  int angleDeg = map((int)MAF, 0, maxValue, -90, 90);
  float rad    = radians(angleDeg);
  int nx = cx + cos(rad) * (radius - 15);
  int ny = cy + sin(rad) * (radius - 15);

  // Aggiorna buffer scia
  histX[histIdx] = nx;
  histY[histIdx] = ny;
  if (histCnt < TRAIL_LEN) histCnt++;
  histIdx = (histIdx + 1) % TRAIL_LEN;

  // Disegna scia (5 tratti verdi sfumati)
  for (int i = 0; i < histCnt; i++) {
    int idx = (histIdx + i) % TRAIL_LEN;
    float t = float(i + 1) / histCnt;           // t piccolo = tratto più vecchio
    uint8_t g = uint8_t(t * 255);               // intensità verde
    uint16_t col = M5.Lcd.color565(0, g, 0);
    M5.Lcd.drawLine(cx, cy, histX[idx], histY[idx], col);
  }

  // Disegna lancetta attuale (rossa)
  M5.Lcd.drawLine(cx, cy, nx, ny, RED);
  lastX = nx;
  lastY = ny;

  // Stampa valore numerico con drawNumber
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.drawNumber((int)MAF, cx - 20, cy - 12);
}

void barometricScreen() {

  sendOBDCommand(PID_BAROMETRIC_PRESSURE);
  delay(20);
  handleOBDResponse();

  // Parametri gauge
  const int cx       = 160;    // centro X
  const int cy       = 140;    // centro Y
  const int radius   = 100;    // raggio esterno
  const int maxValue = 150;    // valore massimo (kPa)

  // Lancetta precedente
  static int lastX = cx, lastY = cy - (radius - 15);
  // Buffer scia (ultime 5 posizioni)
  const int TRAIL_LEN = 5;
  static int histX[TRAIL_LEN], histY[TRAIL_LEN];
  static int histCnt = 0, histIdx = 0;

  // Disegno iniziale del gauge
  if (firstBarScreen) {
    M5.Lcd.fillScreen(BLACK);
  #ifdef USE_TOUCH_BUTTONS
    drawTouchButtons();
  #endif

    // Titolo
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(cx - 60, 10);
    M5.Lcd.print("Baro kPa");

    // Semicerchio colorato da -90° a +90°
    for (int v = 0; v <= maxValue; v++) {
      int angDeg = map(v, 0, maxValue, -90, 90);
      float ang  = angDeg * PI / 180.0f;
      uint16_t col;
      if (v <= 80)       col = GREEN;   // normale
      else if (v <= 100) col = YELLOW;  // warning
      else               col = RED;     // alto

      int x1 = cx + cos(ang) * radius;
      int y1 = cy + sin(ang) * radius;
      int x2 = cx + cos(ang) * (radius - 12);
      int y2 = cy + sin(ang) * (radius - 12);
      M5.Lcd.drawLine(x1, y1, x2, y2, col);
    }

    // Interno nero
    M5.Lcd.fillCircle(cx, cy, radius - 12, BLACK);

    // Tacche ogni 10 kPa
    for (int v = 0; v <= maxValue; v += 10) {
      float ang = map(v, 0, maxValue, -90, 90) * PI / 180.0f;
      int x1 = cx + cos(ang) * (radius - 12);
      int y1 = cy + sin(ang) * (radius - 12);
      int x2 = cx + cos(ang) * radius;
      int y2 = cy + sin(ang) * radius;
      M5.Lcd.drawLine(x1, y1, x2, y2, WHITE);
    }

    // Numeri ogni 25 kPa
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    for (int v = 0; v <= maxValue; v += 25) {
      float ang = map(v, 0, maxValue, -90, 90) * PI / 180.0f;
      int xt = cx + cos(ang) * (radius - 28) - 10;
      int yt = cy + sin(ang) * (radius - 28) - 8;
      M5.Lcd.drawNumber(v, xt, yt);
    }

    // Reset flag globali
    firstMainScreen    = true;
    firstRunningScreen = true;
    firstEngineScreen  = true;
    firstBarScreen     = false;
    firstMafScreen     = true;
    firstRpmScreen     = true;
    firstCoolantScreen = true;
  }

  // Cancella scia e lancetta precedenti
  for (int i = 0; i < histCnt; i++) {
    int idx = (histIdx + i) % TRAIL_LEN;
    M5.Lcd.drawLine(cx, cy, histX[idx], histY[idx], BLACK);
  }
  M5.Lcd.drawLine(cx, cy, lastX, lastY, BLACK);

  // Calcola nuova posizione lancetta
  int angleDeg = map((int)barometricPressure, 0, maxValue, -90, 90);
  float rad    = angleDeg * PI / 180.0f;
  int nx = cx + cos(rad) * (radius - 15);
  int ny = cy + sin(rad) * (radius - 15);

  // Aggiorna buffer scia
  histX[histIdx] = nx;
  histY[histIdx] = ny;
  if (histCnt < TRAIL_LEN) histCnt++;
  histIdx = (histIdx + 1) % TRAIL_LEN;

  // Disegna scia (5 tratti verdi sfumati)
  for (int i = 0; i < histCnt; i++) {
    int idx = (histIdx + i) % TRAIL_LEN;
    float t = float(i + 1) / histCnt;         // più vecchio → t piccolo
    uint8_t g = uint8_t(t * 255);             // intensità verde
    uint16_t col = M5.Lcd.color565(0, g, 0);
    M5.Lcd.drawLine(cx, cy, histX[idx], histY[idx], col);
  }

  // Disegna lancetta attuale (rossa)
  M5.Lcd.drawLine(cx, cy, nx, ny, RED);
  lastX = nx;
  lastY = ny;

  // Valore numerico al centro
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.drawNumber(barometricPressure, cx - 40, cy - 15);
}

void rpmScreen() {

  sendOBDCommand(PID_RPM);
  delay(20);
  handleOBDResponse();

  // Parametri gauge
  const int cx       = 160;    // centro X
  const int cy       = 140;    // centro Y
  const int radius   = 100;    // raggio esterno
  const int maxValue = 8000;   // valore massimo (rpm)

  // Lancetta precedente
  static int lastX = cx, lastY = cy - (radius - 15);
  // Buffer scia (ultime 10 posizioni)
  const int TRAIL_LEN = 10;
  static int histX[TRAIL_LEN], histY[TRAIL_LEN];
  static int histCnt = 0, histIdx = 0;

  // Disegno iniziale del gauge
  if (firstRpmScreen) {
    M5.Lcd.fillScreen(BLACK);
  #ifdef USE_TOUCH_BUTTONS
    drawTouchButtons();
  #endif

    // Titolo
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(LIGHTGREY);
    M5.Lcd.setCursor(cx - 100, 10);
    M5.Lcd.print("RPM/min x100");

    // Semicerchio colorato da -90° a +90°
    for (int v = 0; v <= maxValue; v += 50) {
      int angDeg = map(v, 0, maxValue, -225, 45);
      float ang  = angDeg * PI / 180.0f;
      uint16_t col;
      if (v <= 3000)     col = GREEN;
      else if (v <= 4000)col = GREENYELLOW;
      else if (v <= 5000)col = YELLOW;
      else if (v <= 6000)col = ORANGE;
      else               col = RED;

      int x1 = cx + cos(ang) * radius;
      int y1 = cy + sin(ang) * radius;
      int x2 = cx + cos(ang) * (radius - 12);
      int y2 = cy + sin(ang) * (radius - 12);
      M5.Lcd.drawLine(x1, y1, x2, y2, col);
    }

    // Interno nero
    M5.Lcd.fillCircle(cx, cy, radius - 12, BLACK);

    // Tacche ogni 1000 rpm
    for (int v = 0; v <= maxValue; v += 1000) {
      float ang = map(v, 0, maxValue, -225, 45) * PI / 180.0f;
      int x1 = cx + cos(ang) * (radius - 12);
      int y1 = cy + sin(ang) * (radius - 12);
      int x2 = cx + cos(ang) * radius;
      int y2 = cy + sin(ang) * radius;
      M5.Lcd.drawLine(x1, y1, x2, y2, WHITE);
    }

    // Numeri ogni 1000 rpm
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    for (int v = 0; v <= maxValue; v += 1000) {
      float ang = map(v, 0, maxValue, -225, 45) * PI / 180.0f;
      int xt = cx + cos(ang) * (radius - 28) - 10;
      int yt = cy + sin(ang) * (radius - 28) - 8;
      M5.Lcd.drawNumber(v/100, xt, yt);
    }

    // Reset flag globali
    firstMainScreen    = true;
    firstRunningScreen = true;
    firstEngineScreen  = true;
    firstBarScreen     = true;
    firstMafScreen     = true;
    firstRpmScreen     = false;
    firstCoolantScreen = true;
  }

  // Cancella scia e lancetta precedenti
  for (int i = 0; i < histCnt; i++) {
    int idx = (histIdx + i) % TRAIL_LEN;
    M5.Lcd.drawLine(cx, cy, histX[idx], histY[idx], BLACK);
  }
  M5.Lcd.drawLine(cx, cy, lastX, lastY, BLACK);

  // Calcola nuova posizione lancetta
  int angleDeg = map((int)rpm, 0, maxValue, -225, 45);
  float rad    = angleDeg * PI / 180.0f;
  int nx = cx + cos(rad) * (radius - 15);
  int ny = cy + sin(rad) * (radius - 15);

  // Aggiorna buffer scia
  histX[histIdx] = nx;
  histY[histIdx] = ny;
  if (histCnt < TRAIL_LEN) histCnt++;
  histIdx = (histIdx + 1) % TRAIL_LEN;

  // Disegna scia (5 tratti verdi sfumati)
  for (int i = 0; i < histCnt; i++) {
    int idx = (histIdx + i) % TRAIL_LEN;
    float t = float(i + 1) / histCnt;
    uint8_t g = uint8_t(t * 255);
    uint16_t col = M5.Lcd.color565(0, g, 0);
    M5.Lcd.drawLine(cx, cy, histX[idx], histY[idx], col);
  }

  // Disegna lancetta attuale (rossa)
  M5.Lcd.drawLine(cx, cy, nx, ny, RED);
  lastX = nx;
  lastY = ny;

  // Valore numerico al centro
  M5.Lcd.setTextSize(3);
  if(rpm <= 3000)      M5.Lcd.setTextColor(GREEN, BLACK);
  else if(rpm <= 4000) M5.Lcd.setTextColor(GREENYELLOW, BLACK);
  else if(rpm <= 5000) M5.Lcd.setTextColor(YELLOW, BLACK);
  else if(rpm <= 6000) M5.Lcd.setTextColor(ORANGE, BLACK);
  else                 M5.Lcd.setTextColor(RED, BLACK);

  M5.Lcd.drawNumber(rpm, cx - 30, cy + 70);
}


/*DA IMPLEMENTARE*/
void dtcStatusScreen() {
  sendOBDCommand(PID_DTC_STATUS);
  delay(20);
  handleOBDResponse();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.print("DTC Status: ");
  M5.Lcd.println(dtcStatus);
}


void IRAM_ATTR indexUp(){
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) >  debounceDelay){
    buttonPressed = true;
    lastDebounceTime = currentTime;
  }
  else{
    buttonPressed = false;
  }
  Serial.println("indexUP");
  z++;
  if(z >= 7) z = 0;
}

void IRAM_ATTR indexDown(){
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) >  debounceDelay){
    buttonPressed = true;
    
    lastDebounceTime = currentTime;
  }
  else{
    buttonPressed = false;
  }
  Serial.println("indexDOWN");
  z--;
  if(z < 0) z = 6;

}