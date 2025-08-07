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
#define PID_COOLANT_TEMP "0105"
#define PID_AIR_INTAKE_TEMP "010F"
#define PID_RPM "010C"
#define PID_ENGINE_LOAD "0104"
#define PID_MAF "0110"
#define PID_FUEL_SYSTEM_STATUS "0103"
#define PID_BAROMETRIC_PRESSURE "0133"
#define PID_BATTERY_VOLTAGE "0142"
#define PID_DTC_STATUS "0101"
#define PID_VEHICLE_SPEED "010D"

#define m5Name "M5Stack_OBD"

BluetoothSerial ELM_PORT;
bool counter = false;
volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

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
uint8_t BLEAddress1[6] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xBA};  // Indirizzo Bluetooth del modulo ELM327
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
int screenIndex[5] = {0, 1, 2, 3, 4};
int z = 1;
int zLast = -1;

void setup() {
  // Init M5Unified
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
  //pinMode(ButtonA, INPUT);
  pinMode(ButtonB, INPUT);
  pinMode(ButtonC, INPUT);
  
  //attachInterrupt(digitalPinToInterrupt(ButtonA), indexUp, RISING);
  attachInterrupt(digitalPinToInterrupt(ButtonB), indexUp, RISING);
  attachInterrupt(digitalPinToInterrupt(ButtonC), indexDown, RISING);
#endif

  BTconnect();
  delay(1000);
  ELMinit();
  delay(500);
}

void loop() {
  M5.update();  
  if(z > 4) z = 0;
  if(z < 0) z = 4;

  // Gestione input universale
  handleInput();

  if (z != zLast) {
    M5.Display.setTextSize(2);
    M5.Display.fillScreen(BLACK);
#ifdef USE_TOUCH_BUTTONS
    drawTouchButtons();
#endif
  }

  switch (screenIndex[z]) {
    case 0: mainScreen(); break;
    case 1: runningScreen(); break;
    case 2: engineLoadScreen(); break;
    case 3: barometricScreen(); break;
    case 4: mafScreen(); break;
  }

coolantTemp = coolantTemp + 1;
intakeTemp = intakeTemp + 1;
rpm = rpm + 1;
obdVoltage = obdVoltage + 1;
engineLoad = engineLoad + 1;
MAF = MAF + 1;

  zLast = z;
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
  // Per Core classico, gli interrupt gestiranno i pulsanti
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
  // stato primo disegno e ultimi valori
//  static bool  firstDraw        = true;
  static float lastLoad         = -999, lastCoolant = -999, lastIntake = -999;

  // soglie di tacca per ciascuna barra
  static const int loadTicks[]     = { 20, 38 };
  static const int coolantTicks[]  = { 60, 80, 100, 107, 111 };
  static const int intakeTicks[]   = { 15, 30 };
  static const uint8_t loadCount   = sizeof(loadTicks)    / sizeof(loadTicks[0]);
  static const uint8_t coolCount   = sizeof(coolantTicks) / sizeof(coolantTicks[0]);
  static const uint8_t intakeCount = sizeof(intakeTicks)  / sizeof(intakeTicks[0]);

  // dimensioni schermo e layout
  const int screenW     = 320;
  const int marginX     = 10;
  const int titleH      = 20;
  const int barH        = 50;
  const int spacingY    = 20;
  const int barW        = screenW - 2 * marginX;  // 300 px

  // ordinate Y delle barre
  const int yLoad    = titleH + marginX;
  const int yCoolant = yLoad + barH + spacingY;
  const int yIntake  = yCoolant + barH + spacingY;

  if (firstRunningScreen) {
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);

    // etichette
    M5.Display.setCursor(marginX,    yLoad    - titleH); M5.Display.print("Load   %");
    M5.Display.setCursor(marginX,    yCoolant - titleH); M5.Display.print("Coolant C");
    M5.Display.setCursor(marginX,    yIntake  - titleH); M5.Display.print("Air   \u00B0C");

    // cornici
    M5.Display.drawRect(marginX, yLoad,    barW, barH, LIGHTGREY);
    M5.Display.drawRect(marginX, yCoolant, barW, barH, LIGHTGREY);
    M5.Display.drawRect(marginX, yIntake,  barW, barH, LIGHTGREY);

    firstMainScreen = true;
    firstRunningScreen = false;
    firstEngineScreen = true;
    firstBarScreen = true;
    firstMafScreen = true;
  }

  // letture OBD
  sendOBDCommand(PID_ENGINE_LOAD);     handleOBDResponse();  // engineLoad (0–100)
  sendOBDCommand(PID_COOLANT_TEMP);    handleOBDResponse();  // coolantTemp (0–130)
  sendOBDCommand(PID_AIR_INTAKE_TEMP); handleOBDResponse();  // intakeTemp (0–50)

  // ----- 1) ENGINE LOAD -----
  if (engineLoad != lastLoad) {
    int w = map(engineLoad, 0, 100, 0, barW);
    uint16_t c = (engineLoad <= 20 ? GREEN
                   : engineLoad <= 38 ? GREENYELLOW
                   : RED);

    // 1) riempi colore
    M5.Display.fillRect(marginX+1, yLoad+1, w,           barH-2, c);
    // 2) pulisci resto in nero
    M5.Display.fillRect(marginX+1 + w, yLoad+1, barW-w-1, barH-2, BLACK);
    // 3) disegna solo le tacche coperte dal riempimento
    for (uint8_t i = 0; i < loadCount; i++) {
      int x = marginX + map(loadTicks[i], 0, 100, 0, barW);
      if (x <= marginX + w) {
        M5.Display.drawFastVLine(x, yLoad+1, barH-2, BLACK);
      }
    }
    // testo
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.drawNumber(engineLoad, marginX + (barW>>1) - 16, yLoad + (barH>>1) - 10);

    lastLoad = engineLoad;
  }

  // ----- 2) COOLANT TEMP -----
  if (coolantTemp != lastCoolant) {
    int w = map(coolantTemp, 0, 130, 0, barW);
    uint16_t c = (coolantTemp <=  60 ? BLUE
                   : coolantTemp <=  80 ? CYAN
                   : coolantTemp <= 100 ? GREEN
                   : coolantTemp <= 107 ? GREENYELLOW
                   : coolantTemp <= 111 ? YELLOW
                   : RED);

    M5.Display.fillRect(marginX+1, yCoolant+1, w,           barH-2, c);
    M5.Display.fillRect(marginX+1 + w, yCoolant+1, barW-w-1, barH-2, BLACK);
    for (uint8_t i = 0; i < coolCount; i++) {
      int x = marginX + map(coolantTicks[i], 0, 130, 0, barW);
      if (x <= marginX + w) {
        M5.Display.drawFastVLine(x, yCoolant+1, barH-2, BLACK);
      }
    }
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.drawNumber(coolantTemp, marginX + (barW>>1) - 16, yCoolant + (barH>>1) - 10);

    lastCoolant = coolantTemp;
  }

  // ----- 3) AIR INTAKE TEMP -----
  if (intakeTemp != lastIntake) {
    int w = map(intakeTemp, 0, 50, 0, barW);
    uint16_t c = (intakeTemp <= 15 ? BLUE
                   : intakeTemp <= 30 ? GREEN
                   : YELLOW);

    M5.Display.fillRect(marginX+1, yIntake+1, w,           barH-2, c);
    M5.Display.fillRect(marginX+1 + w, yIntake+1, barW-w-1, barH-2, BLACK);
    for (uint8_t i = 0; i < intakeCount; i++) {
      int x = marginX + map(intakeTicks[i], 0, 50, 0, barW);
      if (x <= marginX + w) {
        M5.Display.drawFastVLine(x, yIntake+1, barH-2, BLACK);
      }
    }
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.drawNumber( intakeTemp, marginX + (barW>>1) - 16, yIntake + (barH>>1) - 10);

    lastIntake = intakeTemp;
  }
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
    return false;  // Loop infinito se non riesce a connettersi
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
    return (loadByte * 100.0) / 255.0;
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
    
    firstMainScreen = false;
    firstRunningScreen = true;
    firstEngineScreen = true;
    firstBarScreen = true;
    firstMafScreen = true;
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

void rpmScreen() {
  sendOBDCommand(PID_RPM);
  delay(20);
  handleOBDResponse();
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextSize(3);
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(10, 10);
  M5.Display.printf("RPM: %.0f", rpm);
#ifdef USE_TOUCH_BUTTONS
  drawTouchButtons();
#endif
}

void engineLoadScreen() {
  sendOBDCommand(PID_ENGINE_LOAD);
  delay(20);
  handleOBDResponse();
  if(firstEngineScreen) {
    M5.Display.fillScreen(BLACK);
#ifdef USE_TOUCH_BUTTONS
    drawTouchButtons();
#endif
    firstMainScreen = true;
    firstRunningScreen = true;
    firstEngineScreen = false;
    firstBarScreen = true;
    firstMafScreen = true;
  }
  M5.Display.setTextSize(3);
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(10, 10);
  M5.Display.printf("Engine Load: %.1f%%", engineLoad);
}

void mafScreen() {
  sendOBDCommand(PID_MAF);
  delay(20);
  handleOBDResponse();

  if(firstMafScreen){
     M5.Lcd.fillScreen(BLACK);
    firstMainScreen = true;
    firstRunningScreen = true;
    firstEngineScreen = true;
    firstBarScreen = true;
    firstMafScreen = false;
  }
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.printf("MAF: %.1f g/s", MAF);
}

void barometricScreen() {
  sendOBDCommand(PID_BAROMETRIC_PRESSURE);
  delay(20);
  handleOBDResponse();
  if(firstBarScreen){
    M5.Lcd.fillScreen(DARKGREY);
    firstMainScreen = true;
    firstRunningScreen = true;
    firstEngineScreen = true;
    firstBarScreen = false;
    firstMafScreen = true;
  }
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.printf("Bar kPa: %.1f kPa", barometricPressure);
}

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

}