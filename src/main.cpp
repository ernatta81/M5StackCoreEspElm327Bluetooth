#include <M5Stack.h>
#include <BluetoothSerial.h>
//#include <Free_Fonts.h>

#define DEBUG
#define ButtonC GPIO_NUM_37
#define ButtonB GPIO_NUM_38
#define ButtonA GPIO_NUM_39
//define PID
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
bool counter = false ;
volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Dichiarazione funzioni
bool ELMinit();
bool BTconnect();
bool sendAndReadCommand(const char* cmd, String& response, int delayTime);
void updateDisplay();
void dataRequestOBD();
void displayDebugMessage(const char* message, int x , int y, uint16_t textColour);
void sendOBDCommand(const char* cmd);
void writeToCircularBuffer(char c);
void handleOBDResponse();
// funzioni lcd
void mainScreen();
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

uint8_t BLEAddress[6] = {0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03};  // Indirizzo Bluetooth del modulo ELM327

float coolantTemp = 0.0;
float oilTemp = 0.0;
float intakeTemp = 0.0;
float rpm = 0.0;
float obdVoltage = 0.0;
float engineLoad = 0.0;
float MAF = 0.0;
float lastCoolantTemp = -999.0; 
float lastIntakeTemp = -999.0; 
float lastOilTemp = -999.0;
float lastEngineLoad = -999.0;
float lastMAFvalue = -999.0;
float barometricPressure = 0.0;
float dtcStatus = 0.0;

bool firstMainScreen = true;
bool firstCoolantScreen = true;
bool firstEngineScreen = true;
bool firstBarScreen = true;
bool firstMafScreen = true;

const unsigned long voltageQueryInterval = 1000; // Intervallo di 4 secondi

unsigned long lastVoltageQueryTime = 0;
unsigned long lastOBDQueryTime = 0;
unsigned long OBDQueryInterval = 150; // Intervallo di query OBD in millisecondi

const int BUFFER_SIZE = 256;
char circularBuffer[BUFFER_SIZE];
int writeIndex = 0;
int readIndex = 0;
int screenIndex[5] = { 0, 1, 2, 3, 4 };
int z = 1;
int zLast = -1;

void setup() {
  M5.begin();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.fillScreen(BLACK);
  Serial.begin(115200);
  Serial.println("Setup in corso...");
  //pinMode(ButtonA, INPUT);
  pinMode(ButtonB, INPUT);
  pinMode(ButtonC, INPUT);
  BTconnect();
  delay(1000);
  ELMinit();
  delay(500);
 
  //attachInterrupt(digitalPinToInterrupt(ButtonA), indexUp, RISING); // Se il bluetooth è abilitato non è possibile utilizzare interrupt su GPIO39
  attachInterrupt(digitalPinToInterrupt(ButtonB), indexUp, RISING);
  attachInterrupt(digitalPinToInterrupt(ButtonC), indexDown, RISING);
}

void loop() {
 
  if(z>4) z=0;
  if(z<0) z=4;

  if (z != zLast){
    M5.Lcd.setTextSize(2);
    M5.Lcd.clearDisplay();
  }
  
  switch (screenIndex[z]){
    case 0: mainScreen(); break;
    case 1: coolantScreen(); break;
    case 2: engineLoadScreen(); break;
    case 3: barometricScreen(); break;
    case 4: mafScreen(); break;
  }
 zLast = z;
}

void mainScreen(){
  unsigned long currentMillis = millis();
  if (currentMillis - lastOBDQueryTime >= OBDQueryInterval) {
    lastOBDQueryTime = currentMillis;
    dataRequestOBD();
    handleOBDResponse();
    updateDisplay();
  }
}

void coolantScreen() {
  int fontLCD = 3;
  static float lastCoolantTempMenu = -999.0;
//  static float lastBarometricPressure = -999.0;
//  static float lastIntakeTemp = -999.0;
//  static float lastMAF = -999.0;
  
  if(firstCoolantScreen){
    M5.Lcd.fillScreen(BLACK);
    /*
    M5.Lcd.drawFastHLine(0, 120, 320 , YELLOW);
    M5.Lcd.drawFastVLine(150, 0, 120, GREEN);
    */
    M5.Lcd.fillRect(0, 120, 320 ,5, OLIVE);
    M5.Lcd.fillRect(150, 0, 5, 290, OLIVE);
    M5.Lcd.setCursor(0, 0);
    //M5.Lcd.setFreeFont(FF1);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(LIGHTGREY);
    M5.Lcd.drawString("Temp.Acqua",5, 130, 2);
    M5.Lcd.drawString("Pot.Mot %", 10 , 0, 2);
    M5.Lcd.drawString("Temp.Aria C°", 180 , 0, 2);
    M5.Lcd.drawString("In.Aria %" , 180, 130 , 2 );
    firstMainScreen = true;
    firstCoolantScreen = false;
    firstEngineScreen = true;
    firstBarScreen = true;
    firstMafScreen = true;
 }
  
  sendOBDCommand(PID_COOLANT_TEMP);
  handleOBDResponse();
  sendOBDCommand(PID_ENGINE_LOAD);
  handleOBDResponse();
  sendOBDCommand(PID_AIR_INTAKE_TEMP);
  handleOBDResponse();
  sendOBDCommand(PID_MAF);
  handleOBDResponse();

  /*
  if (coolantTemp >120){
    counter = true;
  }
  if (counter == false){
    coolantTemp++;
    MAF++;
    engineLoad++;
    intakeTemp++;
  }
  else if (counter == true){
    coolantTemp--;
    MAF--;
    engineLoad--;
    intakeTemp--;
  }
  */

 // Aggiorna solo se i valori sono cambiati
  if (coolantTemp != lastCoolantTempMenu) {
    M5.Lcd.fillRect(5, 175, 140, 62, BLACK); // Cancella la vecchia area
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor((coolantTemp <= 65) ? BLUE : (coolantTemp > 65 && coolantTemp <= 80) ? GREENYELLOW : (coolantTemp >= 81 && coolantTemp <= 100) ? GREEN : (coolantTemp <= 102) ? ORANGE : RED);
    M5.Lcd.drawNumber(coolantTemp, 58, 195);
    
    if (coolantTemp <100){
         M5.Lcd.fillCircle(35, 190, 10, BLACK);
      }
    else if (coolantTemp >=110){
         M5.Lcd.fillCircle(35, 190, 10, RED);
    }
    else if(coolantTemp >= 101 ){
      M5.Lcd.fillCircle(35, 190, 10, YELLOW);
    } 

    lastCoolantTemp = coolantTemp; // Aggiorna lastCoolantTemp solo quando il valore cambia
  }

  if (MAF != lastMAFvalue) {
    M5.Lcd.fillRect(165, 175, 170, 62, BLACK); // Cancella la vecchia area
    M5.Lcd.setTextColor(LIGHTGREY);
    M5.Lcd.drawNumber(MAF, 235, 195);
    lastMAFvalue = MAF;
  }

  if (engineLoad != lastEngineLoad) {
    M5.Lcd.fillRect(5, 55, 140, 62, BLACK); // Cancella la vecchia area
    M5.Lcd.setTextSize(3);
    //M5.Lcd.setTextColor(LIGHTGREY);
    M5.Lcd.setTextColor((engineLoad <= 65) ? DARKGREEN : (engineLoad >65 && engineLoad <= 78) ? GREENYELLOW : (engineLoad > 78 && engineLoad <= 88) ? ORANGE : (engineLoad > 88) ? RED : RED);
    M5.Lcd.drawNumber(engineLoad, 58 , 70);

    if (engineLoad <=55){
      M5.Lcd.fillCircle(35, 65, 10, BLACK);
    }
    else if (engineLoad > 88){
      M5.Lcd.fillCircle(35, 65, 10, RED);
    }
    else if (engineLoad > 78 && engineLoad <= 88){
      M5.Lcd.fillCircle(35, 65, 10, ORANGE);
    }
    else if (engineLoad >65 && engineLoad <= 78){
         M5.Lcd.fillCircle(35, 65, 10, GREENYELLOW);
    }
    else if(engineLoad >= 101 ){
      M5.Lcd.fillCircle(35, 65, 10, YELLOW);
   }

   lastEngineLoad = engineLoad; // Aggiorna lastBarometricPressure solo quando il valore cambia
  }

  if (intakeTemp != lastIntakeTemp) {
    M5.Lcd.fillRect(165, 55, 170, 62, BLACK); // Cancella la vecchia area
    M5.Lcd.setTextColor(LIGHTGREY);
    M5.Lcd.drawNumber(intakeTemp, 235 , 62);
    lastIntakeTemp = intakeTemp;
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
    connected = ELM_PORT.connect(BLEAddress);
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
        displayDebugMessage(response.c_str(), 0 , 200, GREEN);
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

// Funzione per analizzare la temperatura del liquido di raffreddamento
float parseCoolantTemp(const String& response) {
  if (response.indexOf("4105") == 0) {
    byte tempByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    return tempByte - 40;
  }
  return lastCoolantTemp;
}

// Funzione per analizzare la temperatura dell'aria di aspirazione
float parseIntakeTemp(const String& response) {
  if (response.indexOf("410F") == 0) {
    byte tempByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    return tempByte - 40;
  }
  return lastIntakeTemp;
}

// Funzione per analizzare i giri del motore (RPM)
float parseRPM(const String& response) {
  if (response.indexOf("410C") == 0) {
    byte highByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    byte lowByte = strtoul(response.substring(6, 8).c_str(), NULL, 16);
    return (highByte * 256 + lowByte) / 4;
  }
  return 0.0;
}

// Funzione per analizzare il carico del motore
float parseEngineLoad(const String& response) {
  if (response.indexOf("4104") == 0) {
    byte loadByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    return (loadByte * 100.0) / 255.0;
  }
  return lastEngineLoad;
}

// Funzione per analizzare il flusso di massa d'aria (MAF)
float parseMAF(const String& response) {
  if (response.indexOf("4110") == 0) {
    int A = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    int B = strtoul(response.substring(6, 8).c_str(), NULL, 16);
    return ((A * 256) + B) / 100.0;
  }
  return lastMAFvalue;
}

// Funzione per analizzare la pressione barometrica
float parseBarometricPressure(const String& response) {
  if (response.indexOf("4133") == 0) {
    byte pressureByte = strtoul(response.substring(4, 6).c_str(), NULL, 16);
    return pressureByte;
  }
  return 0.0;
}

// Funzione per analizzare la tensione OBD
float parseOBDVoltage(const String& response) {
  int indexV = response.indexOf('V');
  if (indexV >= 0) {
    String voltageStr = response.substring(0, indexV);
    return voltageStr.toFloat();
  }
  return 0.0;
}

// Funzione per analizzare lo stato dei DTC
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
  
  if (firstMainScreen){
     M5.Lcd.fillScreen(BLACK);
    for(int i=15; i<160; i+=20){ M5.Lcd.drawFastHLine(0, i, 320 , OLIVE); }
     M5.Lcd.drawFastVLine(240, 0, 155, OLIVE);
     M5.Lcd.setCursor(0, 0);
     M5.Lcd.setTextSize(2);
    firstMainScreen = false;
    firstCoolantScreen = true;
    firstEngineScreen = true;
    firstBarScreen = true;
    firstMafScreen = true;
  }
  
 
 if (coolantTemp != lastCoolantTemp) {
    M5.Lcd.fillRect(0, 0, 320, 20, BLACK);  // Aggiorna solo la parte della temperatura del liquido refrigerante
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextColor((coolantTemp < 50) ? LIGHTGREY : (coolantTemp >= 40 && coolantTemp <= 65) ? BLUE : (coolantTemp > 65 && coolantTemp <= 80) ? GREENYELLOW : (coolantTemp >= 81 && coolantTemp <= 100) ? GREEN : (coolantTemp <= 102) ? ORANGE : RED);
    M5.Lcd.printf("Coolant Temp: %.1f C\n", coolantTemp);
    lastCoolantTemp = coolantTemp;
  }

  if (obdVoltage != lastObdVoltage) {
    M5.Lcd.fillRect(0, 20, 320, 20, BLACK);  // Aggiorna solo la parte della tensione OBD
    M5.Lcd.setCursor(0, 20);
    M5.Lcd.setTextColor((obdVoltage < 11.8) ? RED : (obdVoltage > 11.8 && obdVoltage <= 12.1) ? YELLOW : (obdVoltage > 12.1 && obdVoltage <= 13.8) ? GREEN : (obdVoltage > 13.8 && obdVoltage <= 14.5) ? GREENYELLOW : ORANGE);
    M5.Lcd.printf("OBD Voltage: %.2f V\n", obdVoltage);
    lastObdVoltage = obdVoltage;
  }

  if (rpm != lastRpm) {
    M5.Lcd.fillRect(0, 40, 320, 20, BLACK);  // Aggiorna solo la parte del RPM
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.printf("RPM: %.0f\n", rpm);
    lastRpm = rpm;
  }

  if (intakeTemp != lastIntakeTemp) {
    M5.Lcd.fillRect(0, 60, 320, 20, BLACK);  // Aggiorna solo la parte della temperatura di aspirazione
    M5.Lcd.setCursor(0, 60);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.printf("Intake Temp: %.1f C\n", intakeTemp);
    lastIntakeTemp = intakeTemp;
  }

  if (engineLoad != lastEngineLoad) {
    M5.Lcd.fillRect(0, 80, 320, 20, BLACK);  // Aggiorna solo la parte del carico del motore
    M5.Lcd.setCursor(0, 80);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.printf("Engine Load: %.1f%%\n", engineLoad);
    lastEngineLoad = engineLoad;
  }

  if (MAF != lastMAF) {
    M5.Lcd.fillRect(0, 100, 320, 20, BLACK);  // Aggiorna solo la parte del MAF
    M5.Lcd.setCursor(0, 100);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.printf("MAF: %.1f%%\n", MAF);
    lastMAF = MAF;
  }
}

void displayDebugMessage(const char* message, int x , int y, uint16_t textColour) {
  M5.Lcd.setTextColor(textColour);
  M5.Lcd.fillRect(x, y, 320, 40, BLACK);  // Pulisce la parte bassa del display
  M5.Lcd.setCursor(x, y);
  M5.Lcd.print(message);
  Serial.println(message);  // Mostra anche nel monitor seriale
}

bool ELMinit() {
  String response;

  #ifdef DEBUG
    displayDebugMessage("ELM init...", 0 ,200, WHITE);
  #endif

  if (!sendAndReadCommand("ATZ", response, 1500)) {
    #ifdef DEBUG
      displayDebugMessage("Err ATZ", 0 , 20, WHITE);
    #endif
    return false;
  }
  #ifdef DEBUG
    displayDebugMessage(response.c_str(), 0 , 20, WHITE);
  #endif

  if (!sendAndReadCommand("ATE0", response, 1500)) {
    #ifdef DEBUG
      displayDebugMessage("Err ATE0", 0 , 40, WHITE);
    #endif
    return false;
  }
  #ifdef DEBUG
    displayDebugMessage(response.c_str(), 0 , 40, WHITE);
  #endif

  if (!sendAndReadCommand("ATL0", response, 1500)) {
    #ifdef DEBUG
      displayDebugMessage("Err ATL0", 0 , 60, WHITE);
    #endif
    return false;
  }
  #ifdef DEBUG
    displayDebugMessage(response.c_str(), 0 , 60, WHITE);
  #endif

  if (!sendAndReadCommand("ATS0", response, 1500)) {
    #ifdef DEBUG
      displayDebugMessage("Err ATS0", 0 , 80, WHITE);
    #endif
    return false;
  }
  #ifdef DEBUG
    displayDebugMessage(response.c_str(), 0 , 80, WHITE);
  #endif

  if (!sendAndReadCommand("ATST0A", response, 1500)) {
    #ifdef DEBUG
      displayDebugMessage("Err ATST0A", 0 , 100, WHITE);
    #endif
    return false;
  }
  #ifdef DEBUG
    displayDebugMessage(response.c_str(), 0 , 100, WHITE);
  #endif

  if (!sendAndReadCommand("ATSP0", response, 15000)) {  // Imposta protocollo automatico SP 0 e gestire la risposta "SEARCHING..."
    #ifdef DEBUG
      displayDebugMessage("Err ATSP0", 0 , 120, WHITE);
    #endif
    return false;
  }
  #ifdef DEBUG
    displayDebugMessage(response.c_str(), 0 , 120, WHITE);
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
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.printf("RPM: %.0f", rpm);
}

void engineLoadScreen() {
  sendOBDCommand(PID_ENGINE_LOAD);
  delay(20);
  handleOBDResponse();
  if(firstEngineScreen){
    M5.Lcd.fillScreen(BLACK);
    firstMainScreen = true;
    firstCoolantScreen = true;
    firstEngineScreen = false;
    firstBarScreen = true;
    firstMafScreen = true;
  }
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.printf("Engine Load: %.1f%%", engineLoad);
}

void mafScreen() {
  sendOBDCommand(PID_MAF);
  delay(20);
  handleOBDResponse();

  if(firstMafScreen){
     M5.Lcd.fillScreen(BLACK);
    firstMainScreen = true;
    firstCoolantScreen = true;
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
    firstCoolantScreen = true;
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