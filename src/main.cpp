//         _.-'-``-._           ______  ______   ______   ______  _______ _______  ______  81
//       ,'::::::''  `.        | |     | |  | \ | |  \ \ | |  | |   | |     | |   | |  | |
//      ::::::'        :       | |---- | |__| | | |  | | | |__| |   | |     | |   | |__| |
//      |:::'          |       |_|____ |_|  \_\ |_|  |_| |_|  |_|   |_|     |_|   |_|  |_|
//      ;:::  .  ,     :        2026-09-24
//     :-------::-------:       ernattaMaker
//     `.__O__.;:.__O__.'       M5stack OBD-II Dashboard
//     (.:::::(:_)_____.)

#include <M5Unified.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

#define DEBUG

#define ButtonB GPIO_NUM_38
#define ButtonC GPIO_NUM_37

// Temi grafici (RGB565)

struct Theme {
  const char* name;
  uint16_t text;   // titoli e testo principale
  uint16_t accent; // valori normali, evidenziazioni
  uint16_t label;  // etichette, testo secondario
  uint16_t track;  // sfondo di barre e archi, linee
  uint16_t panel;  // sfondo card e riga selezionata
  uint16_t header; // barra del titolo
  uint16_t ok;     // valore nella norma
  uint16_t mid;    // valore intermedio (es. motore in temperatura)
  uint16_t cold;   // motore freddo
  uint16_t warn;   // attenzione
  uint16_t alarm;  // allarme
};
const Theme THEMES[] = {
  //  name          text    accent  label   track   panel   header  ok      mid          cold    warn    alarm
  { "Default",    WHITE,  0x05FF, 0xAD55, 0x2945, 0x18E3, 0x10A2, GREEN,  GREENYELLOW, 0x3D7F, ORANGE, RED    },
  { "Amber",      0xFD80, 0xFD80, 0xA360, 0x3940, 0x20C0, 0x1880, 0xFD80, 0xFD80,      0xA360, 0xFB00, RED    },
  { "Red",        0xFBCD, 0xF943, 0xA9E6, 0x3861, 0x2041, 0x1820, 0xF943, 0xF943,      0xA9E6, 0xFD80, WHITE  },
  { "Blue",       0xCF1F, 0x2C7F, 0x5BD5, 0x1107, 0x08A4, 0x0863, 0x2C7F, 0x2C7F,      0x5BD5, ORANGE, RED    },
  { "Ice Blue",   0xE7DF, 0x8EFF, 0x7D16, 0x1966, 0x10E4, 0x08A3, 0x8EFF, 0x8EFF,      0x7D16, ORANGE, RED    },
  { "Green",      0xCFFA, 0x3FEC, 0x5D4D, 0x11A3, 0x0902, 0x08A1, 0x3FEC, 0x3FEC,      0x5D4D, ORANGE, RED    },
  { "Warm White", 0xFF59, 0xFF59, 0xAD11, 0x3165, 0x18E3, 0x1082, 0xFF59, 0xFF59,      0x3D7F, ORANGE, RED    },
  { "Cool White", WHITE,  0xE79F, 0x9515, 0x2945, 0x18E3, 0x10A2, 0xE79F, 0xE79F,      0x3D7F, ORANGE, RED    },
  { "Violet",     0xE69F, 0xBBDF, 0x8B75, 0x28E6, 0x1884, 0x1043, 0xBBDF, 0xBBDF,      0x8B75, ORANGE, RED    }
};
const uint8_t THEME_COUNT = sizeof(THEMES) / sizeof(THEMES[0]);

// Colori correntiapplyTheme()
const uint16_t COL_BG = 0x0000; // nero in tutti i temi
uint16_t COL_TEXT, COL_ACCENT, COL_LABEL, COL_TRACK, COL_PANEL, COL_HEADER;
uint16_t COL_OK, COL_MID, COL_COLD, COL_WARN, COL_ALARM;

void applyTheme(uint8_t i) {
  const Theme& t = THEMES[i < THEME_COUNT ? i : 0];
  COL_TEXT   = t.text;   COL_ACCENT = t.accent; COL_LABEL = t.label;
  COL_TRACK  = t.track;  COL_PANEL  = t.panel;  COL_HEADER = t.header;
  COL_OK     = t.ok;     COL_MID    = t.mid;    COL_COLD  = t.cold;
  COL_WARN   = t.warn;   COL_ALARM  = t.alarm;
}

// Layout 320x240
const int HEADER_H = 26;
const int FOOTER_Y = 222;
const int CARD_W   = 154;
const int CARD_H   = 66;
const int VAL_W    = 130; // area valore al centro
const int VAL_H    = 64;

// Sprite piccoli
lgfx::LGFX_Sprite cardSpr(&M5.Display);
lgfx::LGFX_Sprite valSpr(&M5.Display);

// OBD-II PIDs (modo 01, SAE J1979): formule di conversione dei byte dati A, B
float fA(uint8_t a, uint8_t)          { return a; }
float fTemp(uint8_t a, uint8_t)       { return a - 40; }                     // °C
float fPct(uint8_t a, uint8_t)        { return a * 100.0f / 255; }           // %
float fTrim(uint8_t a, uint8_t)       { return (a - 128) * 100.0f / 128; }   // correzione carburante %
float fA3(uint8_t a, uint8_t)         { return a * 3; }                      // pressione carburante kPa
float fRpm(uint8_t a, uint8_t b)      { return (a * 256 + b) / 4.0f; }
float fWord(uint8_t a, uint8_t b)     { return a * 256 + b; }
float fMaf(uint8_t a, uint8_t b)      { return (a * 256 + b) / 100.0f; }     // g/s
float fTiming(uint8_t a, uint8_t)     { return a / 2.0f - 64; }              // gradi prima del PMS
float fMinutes(uint8_t a, uint8_t b)  { return (a * 256 + b) / 60.0f; }      // s -> min
float fRailBar(uint8_t a, uint8_t b)  { return (a * 256 + b) / 10.0f; }      // 10 kPa/bit -> bar
float fCatTemp(uint8_t a, uint8_t b)  { return (a * 256 + b) / 10.0f - 40; } // °C
float fVolt(uint8_t a, uint8_t b)     { return (a * 256 + b) / 1000.0f; }    // V
float fAbsLoad(uint8_t a, uint8_t b)  { return (a * 256 + b) * 100.0f / 255; }
float fLambda(uint8_t a, uint8_t b)   { return (a * 256 + b) * 2.0f / 65536; }
float fFuelRate(uint8_t a, uint8_t b) { return (a * 256 + b) / 20.0f; }      // L/h
float fTorque(uint8_t a, uint8_t)     { return a - 125; }                    // %

struct PidDef {
  const char* cmd;                     // comando inviato all'ELM
  uint8_t     bytes;                   // byte dati nella risposta
  float     (*calc)(uint8_t a, uint8_t b);
};

// Stesso ordine dell'enum sotto
const PidDef PIDS[] = {
  { "0105", 1, fTemp     }, // coolant temp
  { "010F", 1, fTemp     }, // intake temp
  { "010C", 2, fRpm      }, // RPM
  { "0104", 1, fPct      }, // engine load
  { "0110", 2, fMaf      }, // MAF
  { "0133", 1, fA        }, // barometric pressure
  { "ATRV", 0, nullptr   }, // battery voltage ELM sul pin 16
  { "0101", 1, fA        }, // DTC status (byte A grezzo: MIL + numero DTC)
  { "010D", 1, fA        }, // vehicle speed
  { "0106", 1, fTrim     }, // short term fuel trim bank 1
  { "0107", 1, fTrim     }, // long term fuel trim bank 1
  { "0108", 1, fTrim     }, // short term fuel trim bank 2
  { "0109", 1, fTrim     }, // long term fuel trim bank 2
  { "010A", 1, fA3       }, // fuel pressure
  { "010B", 1, fA        }, // intake manifold absolute pressure
  { "010E", 1, fTiming   }, // timing advance
  { "0111", 1, fPct      }, // throttle position
  { "011F", 2, fMinutes  }, // run time since engine start
  { "0121", 2, fWord     }, // distance traveled with MIL on
  { "0123", 2, fRailBar  }, // fuel rail gauge pressure (diesel, iniezione diretta)
  { "012C", 1, fPct      }, // commanded EGR
  { "012E", 1, fPct      }, // commanded evaporative purge
  { "012F", 1, fPct      }, // fuel tank level
  { "0131", 2, fWord     }, // distance since codes cleared
  { "013C", 2, fCatTemp  }, // catalyst temperature bank 1 sensor 1
  { "0142", 2, fVolt     }, // control module voltage
  { "0143", 2, fAbsLoad  }, // absolute load
  { "0144", 2, fLambda   }, // commanded air-fuel equivalence ratio (lambda)
  { "0145", 1, fPct      }, // relative throttle position
  { "0146", 1, fTemp     }, // ambient air temperature
  { "0149", 1, fPct      }, // accelerator pedal position D
  { "014C", 1, fPct      }, // commanded throttle actuator
  { "0152", 1, fPct      }, // ethanol fuel %
  { "015B", 1, fPct      }, // hybrid battery pack remaining life
  { "015C", 1, fTemp     }, // engine oil temperature
  { "015E", 2, fFuelRate }, // engine fuel rate
  { "0161", 1, fTorque   }, // driver's demand engine torque
  { "0162", 1, fTorque   }, // actual engine torque
  { "0163", 2, fWord     }  // engine reference torque
};
enum {
  IDX_COOLANT = 0,
  IDX_INTAKE,
  IDX_RPM,
  IDX_LOAD,
  IDX_MAF,
  IDX_BARO,
  IDX_VOLT,
  IDX_DTC,
  IDX_SPEED,
  IDX_STFT1,
  IDX_LTFT1,
  IDX_STFT2,
  IDX_LTFT2,
  IDX_FUEL_PRESS,
  IDX_MAP,
  IDX_TIMING,
  IDX_THROTTLE,
  IDX_RUNTIME,
  IDX_MIL_DIST,
  IDX_RAIL_PRESS,
  IDX_EGR,
  IDX_PURGE,
  IDX_FUEL_LEVEL,
  IDX_CLR_DIST,
  IDX_CAT_TEMP,
  IDX_ECU_VOLT,
  IDX_ABS_LOAD,
  IDX_LAMBDA,
  IDX_REL_THROTTLE,
  IDX_AMBIENT,
  IDX_PEDAL,
  IDX_THROTTLE_CMD,
  IDX_ETHANOL,
  IDX_HYBRID_BATT,
  IDX_OIL,
  IDX_FUEL_RATE,
  IDX_TORQUE_DEMAND,
  IDX_TORQUE_ACTUAL,
  IDX_TORQUE_REF,
  PID_COUNT
};
static_assert(sizeof(PIDS) / sizeof(PIDS[0]) == PID_COUNT, "PIDS[] e enum IDX_* non allineati");

// Detail screens (screen 0 = dashboard), mostrate solo se il PID è supportato.
// Unit "*C" = gradi Celsius, "*" = gradi
struct Gauge {
  uint8_t     idx;
  const char* title;
  const char* unit;
  float       minV, maxV;
  uint8_t     decimals;
};
const Gauge GAUGES[] = {
  // Motore
  { IDX_RPM,           "RPM",            "rpm",   0, 8000, 0 },
  { IDX_LOAD,          "Engine Load",    "%",     0,  100, 0 },
  { IDX_ABS_LOAD,      "Absolute Load",  "%",     0,  100, 0 },
  { IDX_COOLANT,       "Coolant Temp",   "*C",   40,  130, 0 },
  { IDX_OIL,           "Oil Temp",       "*C",   40,  150, 0 },
  { IDX_INTAKE,        "Intake Temp",    "*C",  -20,   80, 0 },
  { IDX_AMBIENT,       "Ambient Temp",   "*C",  -20,   50, 0 },
  { IDX_MAF,           "MAF",            "g/s",   0,  150, 1 },
  { IDX_MAP,           "Intake Press.",  "kPa",   0,  255, 0 },
  { IDX_THROTTLE,      "Throttle",       "%",     0,  100, 0 },
  { IDX_REL_THROTTLE,  "Rel. Throttle",  "%",     0,  100, 0 },
  { IDX_THROTTLE_CMD,  "Throttle Cmd",   "%",     0,  100, 0 },
  { IDX_PEDAL,         "Pedal",          "%",     0,  100, 0 },
  { IDX_TIMING,        "Timing Adv.",    "*",   -20,   60, 1 },
  { IDX_SPEED,         "Speed",          "km/h",  0,  200, 0 },
  // Carburante e scarico
  { IDX_FUEL_LEVEL,    "Fuel Level",     "%",     0,  100, 0 },
  { IDX_FUEL_RATE,     "Fuel Rate",      "L/h",   0,   30, 1 },
  { IDX_FUEL_PRESS,    "Fuel Pressure",  "kPa",   0,  765, 0 },
  { IDX_RAIL_PRESS,    "Rail Pressure",  "bar",   0, 2000, 0 },
  { IDX_STFT1,         "STFT Bank 1",    "%",   -25,   25, 1 },
  { IDX_LTFT1,         "LTFT Bank 1",    "%",   -25,   25, 1 },
  { IDX_STFT2,         "STFT Bank 2",    "%",   -25,   25, 1 },
  { IDX_LTFT2,         "LTFT Bank 2",    "%",   -25,   25, 1 },
  { IDX_LAMBDA,        "Lambda Cmd",     "",    0.7,  1.3, 2 },
  { IDX_ETHANOL,       "Ethanol",        "%",     0,  100, 0 },
  { IDX_CAT_TEMP,      "Catalyst Temp",  "*C",    0, 1000, 0 },
  { IDX_EGR,           "EGR Cmd",        "%",     0,  100, 0 },
  { IDX_PURGE,         "Evap Purge",     "%",     0,  100, 0 },
  // Elettrico
  { IDX_VOLT,          "Battery",        "V",    10,   16, 1 },
  { IDX_ECU_VOLT,      "ECU Voltage",    "V",    10,   16, 1 },
  { IDX_HYBRID_BATT,   "Hybrid Battery", "%",     0,  100, 0 },
  // Coppia
  { IDX_TORQUE_DEMAND, "Torque Demand",  "%",     0,  100, 0 },
  { IDX_TORQUE_ACTUAL, "Torque Actual",  "%",     0,  100, 0 },
  { IDX_TORQUE_REF,    "Torque Ref.",    "Nm",    0, 1000, 0 },
  // Varie
  { IDX_BARO,          "Barometric",     "kPa",  70,  110, 0 },
  { IDX_RUNTIME,       "Run Time",       "min",   0,  120, 0 },
  { IDX_MIL_DIST,      "Dist. MIL On",   "km",    0, 1000, 0 },
  { IDX_CLR_DIST,      "Dist. Cleared",  "km",    0, 10000, 0 }
};
const uint8_t GAUGE_COUNT  = sizeof(GAUGES) / sizeof(GAUGES[0]);
const uint8_t SCREEN_DTC      = 1 + GAUGE_COUNT; // stato DTC
const uint8_t SCREEN_SETTINGS = SCREEN_DTC + 1;  // ultima schermata: impostazioni
const uint8_t SCREEN_COUNT    = SCREEN_SETTINGS + 1;

// PID letti a rotazione sulla dashboard, uno per ciclo di loop.
// RPM e velocità ripetuti perché cambiano più in fretta.
const uint8_t DASH_POLL[] = {
  IDX_RPM, IDX_SPEED, IDX_LOAD,
  IDX_RPM, IDX_SPEED, IDX_MAF,
  IDX_RPM, IDX_SPEED, IDX_COOLANT
};
const uint8_t DASH_POLL_COUNT = sizeof(DASH_POLL) / sizeof(DASH_POLL[0]);

// Bluetooth & ELM327
BluetoothSerial ELM;
const char*     ELM_NAME   = "M5Stack_OBD";
const int       BT_SCAN_MS = 6 * 1280; // durata ricerca (~7.7 s, unità di inquiry BT = 1.28 s)
const int       MAX_FOUND  = 10;       // dispositivi mostrati dopo la ricerca
bool            btStackOn  = false;    // stack BT avviato (solo quando serve: in demo no)
bool            elmOnline  = false;    // adattatore connesso e inizializzato

// ELM327 preimpostato : connessione immediata
uint8_t          ELM_DEFAULT_ADDR[6] = { 0x01,0x23,0x45,0x67,0x89,0xBA };
//uint8_t          ELM_DEFAULT_ADDR[6] = { 0x00,0x10,0xCC,0x4F,0x36,0x03 };
const char*      ELM_DEFAULT_NAME    = "ELM327 default";

// Stile delle pagine
enum GaugeStyle : uint8_t { STYLE_ARC, STYLE_ANALOG, STYLE_DIGIT, STYLE_GRAPH, STYLE_BAR, STYLE_COUNT };
const char* STYLE_NAMES[STYLE_COUNT] = { "Arc", "Analog", "Digit", "Graph", "Bar" };

// Impostazioni salvate in NVS
struct Settings {
  bool    hasDevice;
  bool    isDefault;  // dispositivo = ELM_DEFAULT_ADDR (non salvato in NVS)
  uint8_t addr[6];
  char    name[32];
  bool    demo;       // simula adattatore e valori, per provare senza elm
  uint8_t gaugeStyle; // GaugeStyle
  uint8_t theme;      // indice in THEMES[]
};
Settings    settings;
Preferences prefs;
const char* PREFS_NS = "obd";

// Circular buffer for incoming data
const int BUF_SIZE = 256;
char circBuf[BUF_SIZE];
int  writeIdx = 0, readIdx = 0;

// Posizione corrente in DASH_POLL
int pidIdx = 0;

// PID supportati dalla centralina, letti all'avvio a blocchi di 32:
// "0100" (PID 01-20), "0120" (21-40), "0140" (41-60), "0160" (61-80).
// Finché non sono noti (demo, offline) si mostra tutto.
const int PID_BLOCKS          = 4;
uint32_t  pidMask[PID_BLOCKS] = { 0, 0, 0, 0 };
bool      pidsKnown           = false;

// Stato della schermata DTC (lettura solo su richiesta con il tasto A)
enum DtcState : uint8_t { DTC_NOT_READ, DTC_READ, DTC_NO_RESPONSE };
DtcState dtcState = DTC_NOT_READ;

// Data storage arrays
float values[PID_COUNT];

// Display navigation
int  screenIndex = 0, prevScreen = -1;
bool forceRedraw = false; // ridisegna la schermata corrente

// Buttons B/C (interrupt): l'ISR conta le pressioni, loop e menu le usano
struct Button {
  uint8_t           pin;
  volatile uint32_t lastEdge; // ms dell'ultimo fronte (pressione o rilascio)
  volatile uint8_t  presses;  // pressioni non ancora usate
};
const uint32_t DEBOUNCE_MS = 50;
Button         btnB = { ButtonB, 0, 0 };
Button         btnC = { ButtonC, 0, 0 };
portMUX_TYPE   btnMux = portMUX_INITIALIZER_UNLOCKED;

// --- Function Prototypes ---
bool   BTconnect();
bool   ELMinit();
bool   sendAndRead(const char* cmd, String& resp, int timeout = 250);
String readBuffer(int timeout, int maxChars);
void   writeBuf(char c);
void   requestAndParse(uint8_t idx, int timeout = 250);
void   parseOBD(uint8_t idx, const String& resp);
bool   obdData(const String& r, const char* pid, uint8_t* out, int n);
bool   pidSupported(uint8_t idx);
bool   readPidMask(const String& r, const char* pid, uint32_t& mask);
void   mainScreen(bool full);
void   gaugeScreen(const Gauge& g, bool full);
void   dtcScreen(bool full);
void   settingsScreen(bool full);
void   drawHeader(const char* title, bool pageDots = true);
void   drawFooter(const char* a, const char* b, const char* c);
void   drawBtStatus(bool full);
void   onButton(void* arg);
int    takePresses(Button& b);
void   discardPresses();
void   bootScreen();
void   bootLog(const char* msg, uint16_t color = COL_TEXT);
void   loadSettings();
void   saveSettings();
void   clearSettings();
void   connectFlow();
bool   settingsMenu();
String demoResponse(uint8_t idx);
float  demoValue(uint8_t idx);

// --- Setup & Loop ---
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  applyTheme(0); // colori validi fin da subito; quello salvato arriva con loadSettings()
  Serial.begin(115200);

  // CHANGE: servono anche i rilasci per scartare i rimbalzi
  pinMode(ButtonB, INPUT);
  pinMode(ButtonC, INPUT);
  attachInterruptArg(digitalPinToInterrupt(ButtonB), onButton, &btnB, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(ButtonC), onButton, &btnC, CHANGE);

  // Sprite allocati prima del Bluetooth, che occupa molta RAM
  cardSpr.setColorDepth(16);
  valSpr.setColorDepth(16);
  if (!cardSpr.createSprite(CARD_W, CARD_H) || !valSpr.createSprite(VAL_W, VAL_H)) {
    Serial.println("Sprite alloc FAIL");
  }

  loadSettings();
  connectFlow();
}

// Indice sempre in 0..n-1, anche con passi negativi
int wrapIndex(int v, int n) {
  return ((v % n) + n) % n;
}

// Pagine visibili: quelle di dettaglio dei PID non supportati sono nascoste
bool screenAvailable(int s) {
  if (s == 0 || s == SCREEN_SETTINGS) return true;
  if (s == SCREEN_DTC)                return pidSupported(IDX_DTC);
  return pidSupported(GAUGES[s - 1].idx);
}

// Avanza di n pagine (negativo = indietro) saltando quelle nascoste
int stepScreen(int from, int n) {
  int s   = from;
  int dir = n > 0 ? 1 : -1;
  for (int k = 0; k < abs(n); ++k) {
    do { s = wrapIndex(s + dir, SCREEN_COUNT); } while (!screenAvailable(s));
  }
  return s;
}

void loop() {
  M5.update();
  int step = takePresses(btnB) - takePresses(btnC);
  if (!screenAvailable(screenIndex)) screenIndex = 0; // es. dopo una riconnessione
  screenIndex = stepScreen(screenIndex, step);

  bool full = (screenIndex != prevScreen) || forceRedraw;
  forceRedraw = false;
  if (full) {
    M5.Display.fillScreen(COL_BG);
    const char* a = (screenIndex == SCREEN_DTC)      ? "READ"
                  : (screenIndex == SCREEN_SETTINGS) ? "OPEN" : nullptr;
    drawFooter(a, "NEXT >", "< PREV");
  }

  if (screenIndex == 0) {
    mainScreen(full);
  } else if (screenIndex == SCREEN_DTC) {
    dtcScreen(full);
  } else if (screenIndex == SCREEN_SETTINGS) {
    settingsScreen(full);
  } else {
    gaugeScreen(GAUGES[screenIndex - 1], full);
  }
  drawBtStatus(full);
  prevScreen = screenIndex;
}

// --- Input Handling ---
// Tasti attivi bassi. Conta una pressione solo se il pin va a LOW dopo
// almeno DEBOUNCE_MS senza fronti: i rimbalzi di pressione e rilascio sono scartati.
void IRAM_ATTR onButton(void* arg) {
  Button*  b   = static_cast<Button*>(arg);
  uint32_t now = millis();
  bool     low = digitalRead(b->pin) == LOW;

  portENTER_CRITICAL_ISR(&btnMux);
  if (low && now - b->lastEdge >= DEBOUNCE_MS && b->presses < 100) {
    b->presses = b->presses + 1;
  }
  b->lastEdge = now;
  portEXIT_CRITICAL_ISR(&btnMux);
}

int takePresses(Button& b) {
  portENTER_CRITICAL(&btnMux);
  int n = b.presses;
  b.presses = 0;
  portEXIT_CRITICAL(&btnMux);
  return n;
}

// Scarta le pressioni accumulate (es. entrando in un menu)
void discardPresses() {
  takePresses(btnB);
  takePresses(btnC);
}

// --- Bluetooth & ELM Initialization ---
// Avvia lo stack BT (master) solo al primo uso: in modalità demo non serve
void startBluetooth() {
  if (btStackOn) return;
  ELM.begin(ELM_NAME, true);
  btStackOn = true;
}

// Collegamento attivo: demo, oppure adattatore inizializzato e ancora connesso.
// btStackOn va controllato prima: ELM.connected() senza stack BT avviato va in crash.
bool linkUp() {
  if (settings.demo) return true;
  return btStackOn && elmOnline && ELM.connected(0);
}

bool BTconnect() {
  startBluetooth();
  if (ELM.connected(0)) ELM.disconnect(); // es. riconnessione a un altro dispositivo
  if (!settings.hasDevice) {
    bootLog("Nessun dispositivo salvato", COL_WARN);
    return false;
  }
#ifdef DEBUG
  bootLog((String("Connessione a ") + settings.name + "...").c_str());
#endif
  bool ok = false;
  for (int i = 0; i < 2 && !ok; ++i) {
    ok = ELM.connect(settings.addr);
    if (!ok) {
#ifdef DEBUG
      bootLog("BT Conn FAIL", COL_WARN);
#endif
      delay(500);
    }
  }
#ifdef DEBUG
  bootLog(ok ? "Connessione BT OK!" : "ELM BT NOT FOUND", ok ? COL_OK : COL_ALARM);
#endif
  return ok;
}

bool ELMinit() {
  String resp;
#ifdef DEBUG
  bootLog("ELM init...");
#endif
  // Timeout massimi: ogni comando termina appena l'ELM risponde con il prompt '>'
  const char* cmds[]   = { "ATZ", "ATE0", "ATL0", "ATS0", "ATST0A", "ATSP0" };
  int         timeouts[] = { 2500, 1000, 1000, 1000, 1000, 1000 };
  for (int i = 0; i < 6; ++i) {
    if (!sendAndRead(cmds[i], resp, timeouts[i])) {
#ifdef DEBUG
      bootLog((String("Err ") + cmds[i]).c_str(), COL_ALARM);
#endif
      return false;
    }
#ifdef DEBUG
    bootLog((String(cmds[i]) + ": " + resp).c_str(), COL_LABEL);
#endif
  }

  // Prima richiesta OBD: con ATSP0 l'ELM cerca qui il protocollo dell'auto (può
  // richiedere qualche secondo). Se la centralina non risponde (quadro spento)
  // l'adattatore resta comunque utilizzabile.
#ifdef DEBUG
  bootLog("Ricerca protocollo auto...");
#endif
  // La stessa risposta dice anche quali PID sono supportati. L'ultimo bit di ogni
  // blocco (PID 20, 40, 60) indica se esiste il blocco successivo.
  const char* blocks[PID_BLOCKS] = { "0100", "0120", "0140", "0160" };
  pidsKnown  = false;
  bool ecuOk = sendAndRead(blocks[0], resp, 10000) && readPidMask(resp, blocks[0], pidMask[0]);
  if (ecuOk) {
    for (int b = 1; b < PID_BLOCKS; ++b) {
      pidMask[b] = 0;
      if ((pidMask[b - 1] & 1) && sendAndRead(blocks[b], resp, 1000)) {
        readPidMask(resp, blocks[b], pidMask[b]);
      }
    }
    pidsKnown = true;
  }
#ifdef DEBUG
  if (ecuOk) {
    int n = 0, total = 0;
    for (int i = 0; i < PID_COUNT; ++i) {
      if (i == IDX_VOLT) continue; // ATRV è un comando dell'ELM, non un PID
      ++total;
      n += pidSupported(i);
    }
    bootLog((String("Centralina OK - PID supportati ") + n + "/" + total).c_str(), COL_OK);
  } else {
    bootLog("Centralina non risponde", COL_WARN);
  }
#endif
  return true;
}

// Maschera PID supportati da una risposta "0100"/"0120": 4 byte, il bit più alto
// è il primo PID del blocco. Con più centraline le risposte si uniscono (OR).
bool readPidMask(const String& r, const char* pid, uint32_t& mask) {
  char hdr[5] = { '4', '1', pid[2], pid[3], 0 };
  bool found  = false;
  mask = 0;
  for (int p = r.indexOf(hdr); p >= 0; p = r.indexOf(hdr, p + 4)) {
    uint8_t b[4];
    if (obdData(r.substring(p), pid, b, 4)) {
      mask |= (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
      found = true;
    }
  }
  return found;
}

bool pidSupported(uint8_t idx) {
  if (!pidsKnown || idx == IDX_VOLT) return true; // ATRV non dipende dalla centralina
  int p = strtol(PIDS[idx].cmd + 2, nullptr, 16);
  if (p < 1 || p > PID_BLOCKS * 32) return true;  // fuori dai blocchi letti: non filtrato
  return pidMask[(p - 1) / 32] & (1UL << (31 - (p - 1) % 32));
}

// Invia un comando e legge la risposta fino al prompt '>' (max 'timeout' ms)
bool sendAndRead(const char* cmd, String& resp, int timeout) {
  resp = "";
  while (ELM.available()) ELM.read(); // scarta dati vecchi
  ELM.print(cmd);
  ELM.print("\r\n");

  unsigned long start  = millis();
  bool          prompt = false;
  while (!prompt && millis() - start < timeout) {
    while (ELM.available()) {
      char c = ELM.read();
      if (c == '>') { prompt = true; break; }
      resp += c;
    }
    if (!prompt) delay(5);
  }
  resp.trim();
  return resp.length() > 0;
}

// --- Circular Buffer ---
String readBuffer(int timeout, int maxChars) {
  // Esce appena l'ELM manda il prompt '>' (risposta completa), max 'timeout' ms
  unsigned long start  = millis();
  bool          prompt = false;
  while (!prompt && millis() - start < timeout) {
    while (ELM.available()) {
      char c = ELM.read();
      writeBuf(c);
      if (c == '>') prompt = true;
    }
    if (!prompt) delay(5);
  }
  String out;
  while (readIdx != writeIdx && out.length() < maxChars) {
    out += circBuf[readIdx++];
    if (readIdx >= BUF_SIZE) readIdx = 0;
  }
  readIdx = writeIdx; // scarta l'eccedenza: non deve finire nella risposta successiva
  out.trim();
  return out;
}

void writeBuf(char c) {
  circBuf[writeIdx++] = c;
  if (writeIdx >= BUF_SIZE) writeIdx = 0;
  if (writeIdx == readIdx) {
    if (++readIdx >= BUF_SIZE) readIdx = 0;
  }
}

// --- Request & Parse ---
void requestAndParse(uint8_t idx, int timeout) {
  if (settings.demo) {
    delay(40); // latenza simile a un ELM327 vero
    String r = demoResponse(idx);
    if (r.length()) parseOBD(idx, r);
    else            values[idx] = demoValue(idx);
    return;
  }
  if (!linkUp()) return; // offline: niente attese inutili

  while (ELM.available()) ELM.read(); // via eventuali risposte arrivate in ritardo
  ELM.print(PIDS[idx].cmd);
  ELM.print("\r\n");
  String resp = readBuffer(timeout, 64);
  parseOBD(idx, resp);
}

// Estrae n byte dati dalla risposta di un PID modo 01 (es. "010C" -> "410C1AF8" -> 1A F8).
// L'intestazione è cercata ovunque: la risposta può iniziare con "SEARCHING..." o simili.
bool obdData(const String& r, const char* pid, uint8_t* out, int n) {
  char hdr[5] = { '4', '1', pid[2], pid[3], 0 };
  int  p      = r.indexOf(hdr);
  if (p < 0 || (int)r.length() < p + 4 + n * 2) return false;
  for (int i = 0; i < n; ++i) {
    char  hex[3] = { r[p + 4 + i * 2], r[p + 5 + i * 2], 0 };
    char* end;
    out[i] = strtoul(hex, &end, 16);
    if (*end) return false; // non esadecimale
  }
  return true;
}

void parseOBD(uint8_t idx, const String& r) {
  // ATRV risponde con la tensione in chiaro, es. "12.6V"
  if (idx == IDX_VOLT) {
    int end = r.indexOf('V');
    int start = end;
    while (start > 0 && (isdigit(r[start - 1]) || r[start - 1] == '.')) --start;
    float v = (end > start) ? r.substring(start, end).toFloat() : 0;
    if (v > 0) values[idx] = v;
    return;
  }

  // PID modo 01: byte dati convertiti con la formula della tabella PIDS[]
  const PidDef& p    = PIDS[idx];
  uint8_t       d[2] = { 0, 0 };
  if (p.calc && obdData(r, p.cmd, d, p.bytes)) values[idx] = p.calc(d[0], d[1]);
}

// --- UI helpers ---
bool sameValue(float a, float b) {
  return (isnan(a) && isnan(b)) || a == b;
}

void formatValue(char* buf, size_t len, float v, uint8_t decimals) {
  if (isnan(v)) snprintf(buf, len, "--");
  else          snprintf(buf, len, "%.*f", decimals, v);
}

// Colore del valore in base alle soglie (motore freddo / normale / caldo)
uint16_t valueColor(uint8_t idx, float v) {
  if (isnan(v)) return COL_LABEL;
  switch (idx) {
    case IDX_COOLANT:
      if (v < 65)   return COL_COLD;
      if (v < 80)   return COL_MID;
      if (v <= 100) return COL_OK;
      if (v <= 105) return COL_WARN;
      return COL_ALARM;
    case IDX_LOAD:
      if (v <= 65) return COL_OK;
      if (v <= 78) return COL_MID;
      if (v <= 88) return COL_WARN;
      return COL_ALARM;
    case IDX_VOLT:     // a motore spento ~12.6 V, in carica ~13.8-14.4 V
    case IDX_ECU_VOLT:
      if (v < 11.8)  return COL_ALARM;
      if (v < 12.2)  return COL_WARN;
      if (v <= 14.8) return COL_OK;
      return COL_ALARM;
    case IDX_OIL:
      if (v < 60)   return COL_COLD;
      if (v <= 120) return COL_OK;
      if (v <= 130) return COL_WARN;
      return COL_ALARM;
    case IDX_FUEL_LEVEL: // riserva
      if (v < 10) return COL_ALARM;
      if (v < 20) return COL_WARN;
      return COL_ACCENT;
    case IDX_STFT1: // correzioni carburante: oltre ±10% c'è qualcosa da controllare
    case IDX_LTFT1:
    case IDX_STFT2:
    case IDX_LTFT2:
      if (fabsf(v) <= 10) return COL_OK;
      if (fabsf(v) <= 20) return COL_WARN;
      return COL_ALARM;
    default:
      return COL_ACCENT;
  }
}

// Unità di misura (bottom-left in x,y); "*" iniziale = simbolo gradi, assente nei font ASCII
int unitWidth(lgfx::LovyanGFX& g, const char* unit) {
  g.setFont(&fonts::Font2);
  g.setTextSize(1);
  if (*unit == '*') return 7 + g.textWidth(unit + 1);
  return g.textWidth(unit);
}

void drawUnit(lgfx::LovyanGFX& g, const char* unit, int x, int y, uint16_t color) {
  g.setFont(&fonts::Font2);
  g.setTextSize(1);
  g.setTextColor(color);
  g.setTextDatum(BL_DATUM);
  if (*unit == '*') {
    g.drawCircle(x + 2, y - 12, 2, color);
    x += 7;
    ++unit;
  }
  g.drawString(unit, x, y);
}

// --- Header / Footer ---
void drawHeader(const char* title, bool pageDots) {
  auto& d = M5.Display;
  d.fillRect(0, 0, d.width(), HEADER_H, COL_HEADER);
  d.setFont(&fonts::FreeSansBold9pt7b);
  d.setTextSize(1);
  d.setTextColor(COL_TEXT);
  d.setTextDatum(ML_DATUM);
  d.drawString(title, 8, HEADER_H / 2);

  // Indicatore pagina "n/N" sulle pagine visibili (non nei menu): con decine di
  // gauge i pallini non starebbero nell'header
  if (!pageDots) return;
  int count = 0, pos = 0;
  for (int i = 0; i < SCREEN_COUNT; ++i) {
    if (!screenAvailable(i)) continue;
    ++count;
    if (i == screenIndex) pos = count;
  }
  char buf[8];
  snprintf(buf, sizeof(buf), "%d/%d", pos, count);
  d.setFont(&fonts::Font2);
  d.setTextColor(COL_LABEL);
  d.setTextDatum(MR_DATUM);
  d.drawString(buf, d.width() - 6, HEADER_H / 2);
}

// Stato collegamento, a sinistra del numero di pagina: "BT" verde/rosso oppure "DEMO"
void drawBtStatus(bool full) {
  static int shown = -1;
  int state = settings.demo ? 2 : linkUp() ? 1 : 0;
  if (!full && state == shown) return;

  auto& d = M5.Display;
  int right = d.width() - 6 - 40 - 8; // 40 px riservati a "nn/nn"
  d.fillRect(right - 44, 4, 46, HEADER_H - 8, COL_HEADER);
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(state == 2 ? COL_ACCENT : state == 1 ? COL_OK : COL_ALARM);
  d.setTextDatum(MR_DATUM);
  d.drawString(state == 2 ? "DEMO" : "BT", right, HEADER_H / 2);
  shown = state;
}

// Etichette sopra i tasti fisici A (x=65), B (x=160) e C (x=255); nullptr = nessuna.
// L'azione del tasto A è evidenziata in azzurro.
void drawFooter(const char* a, const char* b, const char* c) {
  auto& d = M5.Display;
  d.fillRect(0, FOOTER_Y, d.width(), d.height() - FOOTER_Y, COL_BG);
  d.drawFastHLine(0, FOOTER_Y - 1, d.width(), COL_TRACK);
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextDatum(TC_DATUM);
  if (a) {
    d.setTextColor(COL_ACCENT);
    d.drawString(a, 65, FOOTER_Y + 2);
  }
  d.setTextColor(COL_LABEL);
  if (b) d.drawString(b, 160, FOOTER_Y + 2);
  if (c) d.drawString(c, 255, FOOTER_Y + 2);
}

// --- Main Screen (dashboard) ---
void drawCard(const char* title, const char* unit, uint8_t idx, uint8_t decimals, int x, int y) {
  float    v   = values[idx];
  bool     na  = !pidSupported(idx);
  uint16_t col = na ? COL_TRACK : valueColor(idx, v);
  char     buf[12];
  formatValue(buf, sizeof(buf), v, decimals);

  cardSpr.fillScreen(COL_BG);
  cardSpr.fillRoundRect(0, 0, CARD_W, CARD_H, 8, COL_PANEL);
  cardSpr.fillRoundRect(0, 0, 5, CARD_H, 2, col);

  cardSpr.setFont(&fonts::Font2);
  cardSpr.setTextSize(1);
  cardSpr.setTextColor(COL_LABEL);
  cardSpr.setTextDatum(TL_DATUM);
  cardSpr.drawString(title, 12, 5);
  drawUnit(cardSpr, unit, CARD_W - 8 - unitWidth(cardSpr, unit), 21, COL_LABEL);

  if (na) { // PID non supportato dalla centralina (Font7 ha solo cifre)
    cardSpr.setFont(&fonts::Font4);
    cardSpr.setTextSize(1);
    cardSpr.setTextColor(COL_LABEL);
    cardSpr.setTextDatum(BL_DATUM);
    cardSpr.drawString("N/A", 12, CARD_H - 8);
  } else {
    cardSpr.setFont(&fonts::Font7);
    cardSpr.setTextSize(0.75f);
    cardSpr.setTextColor(col);
    cardSpr.setTextDatum(BL_DATUM);
    cardSpr.drawString(buf, 12, CARD_H - 5);
  }

  cardSpr.pushSprite(x, y);
}

void drawLoadBar(float v, int x, int y, int w) {
  auto& d = M5.Display;
  const int barY = y + 20, barH = 18;
  uint16_t col = valueColor(IDX_LOAD, v);
  char buf[12];
  formatValue(buf, sizeof(buf), v, 0);

  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(COL_LABEL, COL_BG);
  d.setTextDatum(TL_DATUM);
  d.drawString("Engine Load", x, y);
  bool na = !pidSupported(IDX_LOAD);
  d.setTextColor(na ? COL_LABEL : col, COL_BG);
  d.setTextDatum(TR_DATUM);
  d.setTextPadding(50);
  d.drawString(na ? "N/A" : (String(buf) + " %").c_str(), x + w, y);
  d.setTextPadding(0);

  // Parte piena + parte vuota: nessuna cancellazione, nessun flicker
  float pct = isnan(v) ? 0 : constrain(v, 0.0f, 100.0f);
  int   fw  = (int)(pct * w / 100.0f);
  d.fillRect(x, barY, fw, barH, col);
  d.fillRect(x + fw, barY, w - fw, barH, COL_TRACK);
}

void mainScreen(bool full) {
  struct Card { uint8_t idx; const char* title; const char* unit; uint8_t dec; int x, y; };
  static const Card cards[] = {
    { IDX_COOLANT, "Coolant", "*C",   0,   4,  32 },
    { IDX_RPM,     "RPM",     "rpm",  0, 162,  32 },
    { IDX_SPEED,   "Speed",   "km/h", 0,   4, 104 },
    { IDX_MAF,     "MAF",     "g/s",  1, 162, 104 }
  };
  static float shown[5];

  if (full) {
    drawHeader("Dashboard");
    for (auto& s : shown) s = NAN;
  }

  // Prossimo PID della rotazione, saltando quelli non supportati
  for (int k = 0; k < DASH_POLL_COUNT; ++k) {
    uint8_t idx = DASH_POLL[pidIdx];
    pidIdx = (pidIdx + 1) % DASH_POLL_COUNT;
    if (pidSupported(idx)) {
      requestAndParse(idx);
      break;
    }
  }

  // Ridisegno solo ciò che è cambiato
  for (int i = 0; i < 4; ++i) {
    float v = values[cards[i].idx];
    if (full || !sameValue(v, shown[i])) {
      drawCard(cards[i].title, cards[i].unit, cards[i].idx, cards[i].dec, cards[i].x, cards[i].y);
      shown[i] = v;
    }
  }
  float load = values[IDX_LOAD];
  if (full || !sameValue(load, shown[4])) {
    drawLoadBar(load, 6, 178, 308);
    shown[4] = load;
  }
}

// --- Gauge Screens ---
// Stile scelto in Impostazioni (settings.gaugeStyle). Ogni stile tiene la propria
// cache del valore mostrato e ridisegna solo ciò che cambia.
const int   GAUGE_CX = 160, GAUGE_CY = 124;
const int   GAUGE_R0 = 92,  GAUGE_R1 = 74;
const float GAUGE_START = 135, GAUGE_SWEEP = 270; // apertura in basso

// Decimali dei numeri di scala: solo per scale strette (es. Lambda 0.7-1.3, batteria 10-16 V)
uint8_t scaleDecimals(const Gauge& g) {
  return (g.maxV - g.minV) < 10 ? g.decimals : 0;
}

// Posizione del valore nella scala del gauge, 0..1 (0 se il valore manca)
float gaugeFrac(const Gauge& g, float v) {
  return isnan(v) ? 0 : constrain((v - g.minV) / (g.maxV - g.minV), 0.0f, 1.0f);
}

// Punto a distanza r dal centro del gauge, angolo in gradi (0 = ore 3, senso orario)
void polarPoint(float r, float angle, int& x, int& y) {
  float rad = angle * DEG_TO_RAD;
  x = GAUGE_CX + (int)lroundf(r * cosf(rad));
  y = GAUGE_CY + (int)lroundf(r * sinf(rad));
}

// Arc: anello colorato fino al valore, numero al centro
void drawArcGauge(const Gauge& g, float v, bool full) {
  static float shown = NAN;
  auto& d = M5.Display;

  if (full) {
    d.fillArc(GAUGE_CX, GAUGE_CY, GAUGE_R0, GAUGE_R1, GAUGE_START, GAUGE_START + GAUGE_SWEEP, COL_TRACK);

    char buf[12];
    d.setFont(&fonts::Font2);
    d.setTextSize(1);
    d.setTextColor(COL_LABEL);
    d.setTextDatum(TC_DATUM);
    formatValue(buf, sizeof(buf), g.minV, scaleDecimals(g));
    d.drawString(buf, GAUGE_CX - 60, GAUGE_CY + 72);
    formatValue(buf, sizeof(buf), g.maxV, scaleDecimals(g));
    d.drawString(buf, GAUGE_CX + 60, GAUGE_CY + 72);
    shown = NAN;
  }
  if (!full && sameValue(v, shown)) return;
  shown = v;

  // Arco: tratto colorato fino al valore, grigio per il resto
  uint16_t col  = valueColor(g.idx, v);
  float    frac = gaugeFrac(g, v);
  float    a    = GAUGE_START + frac * GAUGE_SWEEP;
  if (frac > 0) d.fillArc(GAUGE_CX, GAUGE_CY, GAUGE_R0, GAUGE_R1, GAUGE_START, a, col);
  if (frac < 1) d.fillArc(GAUGE_CX, GAUGE_CY, GAUGE_R0, GAUGE_R1, a, GAUGE_START + GAUGE_SWEEP, COL_TRACK);

  // Valore centrale
  char buf[12];
  formatValue(buf, sizeof(buf), v, g.decimals);
  valSpr.fillScreen(COL_BG);
  valSpr.setFont(&fonts::Font7);
  valSpr.setTextSize(1);
  if (valSpr.textWidth(buf) > VAL_W - 4) valSpr.setTextSize(0.8f); // es. "123.4"
  valSpr.setTextColor(col);
  valSpr.setTextDatum(TC_DATUM);
  valSpr.drawString(buf, VAL_W / 2, 0);
  drawUnit(valSpr, g.unit, (VAL_W - unitWidth(valSpr, g.unit)) / 2, VAL_H - 1, COL_LABEL);
  valSpr.pushSprite(GAUGE_CX - VAL_W / 2, GAUGE_CY - VAL_H / 2);
}

// Analog: quadrante con tacche e lancetta.
// La lancetta (lunga NEEDLE_LEN) non raggiunge mai numeri e valore digitale:
// si cancella ridisegnandola nel colore di sfondo, senza toccare il resto.
const int DIAL_R     = 92;
const int DIAL_LBL_R = 70;
const int NEEDLE_LEN = 52;

void drawNeedle(float angle, uint16_t color) {
  int tx, ty, b1x, b1y, b2x, b2y;
  polarPoint(NEEDLE_LEN, angle, tx, ty);
  polarPoint(5, angle + 90, b1x, b1y);
  polarPoint(5, angle - 90, b2x, b2y);
  M5.Display.fillTriangle(tx, ty, b1x, b1y, b2x, b2y, color);
}

void drawAnalogGauge(const Gauge& g, float v, bool full) {
  static float shown    = NAN;
  static float needleA  = 0;
  static bool  needleOn = false;
  auto& d = M5.Display;

  if (full) {
    // 50 tacche: lunghe e numerate ogni 10, medie ogni 5
    d.setFont(&fonts::Font2);
    d.setTextSize(1);
    d.setTextDatum(MC_DATUM);
    for (int i = 0; i <= 50; ++i) {
      float a     = GAUGE_START + GAUGE_SWEEP * i / 50;
      bool  major = (i % 10 == 0);
      int   len   = major ? 14 : (i % 5 == 0) ? 9 : 5;
      int   x0, y0, x1, y1;
      polarPoint(DIAL_R - len, a, x0, y0);
      polarPoint(DIAL_R, a, x1, y1);
      if (major) d.drawWideLine(x0, y0, x1, y1, 1.5f, COL_TEXT);
      else       d.drawLine(x0, y0, x1, y1, COL_LABEL);
      if (major) {
        char buf[8];
        formatValue(buf, sizeof(buf), g.minV + (g.maxV - g.minV) * i / 50, scaleDecimals(g));
        polarPoint(DIAL_LBL_R, a, x0, y0);
        d.setTextColor(COL_LABEL);
        d.drawString(buf, x0, y0);
      }
    }
    d.fillArc(GAUGE_CX, GAUGE_CY, DIAL_R + 4, DIAL_R + 2, GAUGE_START, GAUGE_START + GAUGE_SWEEP, COL_TRACK);
    drawUnit(d, g.unit, GAUGE_CX - unitWidth(d, g.unit) / 2, GAUGE_CY + 88, COL_LABEL);
    shown    = NAN;
    needleOn = false;
  }
  if (!full && sameValue(v, shown)) return;
  shown = v;

  uint16_t col = valueColor(g.idx, v);
  if (needleOn) drawNeedle(needleA, COL_BG);
  needleA = GAUGE_START + gaugeFrac(g, v) * GAUGE_SWEEP;
  drawNeedle(needleA, col);
  needleOn = true;
  d.fillCircle(GAUGE_CX, GAUGE_CY, 7, COL_LABEL);
  d.fillCircle(GAUGE_CX, GAUGE_CY, 3, COL_BG);

  // Valore digitale sotto il perno
  char buf[12];
  formatValue(buf, sizeof(buf), v, g.decimals);
  d.setFont(&fonts::Font4);
  d.setTextSize(1);
  d.setTextColor(col, COL_BG);
  d.setTextDatum(MC_DATUM);
  d.setTextPadding(90);
  d.drawString(buf, GAUGE_CX, GAUGE_CY + 58);
  d.setTextPadding(0);
}

// Digit: numero grande a 7 segmenti + minimo/massimo visti su questa pagina
void drawDigitGauge(const Gauge& g, float v, bool full) {
  static float shown = NAN, minSeen = NAN, maxSeen = NAN;
  auto&     d  = M5.Display;
  const int cx = d.width() / 2;

  if (full) {
    drawUnit(d, g.unit, cx - unitWidth(d, g.unit) / 2, 158, COL_LABEL);
    d.setFont(&fonts::Font2);
    d.setTextColor(COL_LABEL);
    d.setTextDatum(TC_DATUM);
    d.drawString("MIN", 80, 172);
    d.drawString("MAX", 240, 172);
    d.drawFastHLine(20, 166, d.width() - 40, COL_TRACK);
    shown = minSeen = maxSeen = NAN;
  }
  if (!full && sameValue(v, shown)) return;
  shown = v;
  if (!isnan(v)) {
    if (isnan(minSeen) || v < minSeen) minSeen = v;
    if (isnan(maxSeen) || v > maxSeen) maxSeen = v;
  }

  char buf[12];
  formatValue(buf, sizeof(buf), v, g.decimals);
  d.setFont(&fonts::Font7);
  d.setTextSize(1.5f);
  d.setTextColor(valueColor(g.idx, v), COL_BG);
  d.setTextDatum(MC_DATUM);
  d.setTextPadding(d.width() - 20);
  d.drawString(buf, cx, 90);

  d.setFont(&fonts::Font4);
  d.setTextSize(1);
  d.setTextColor(COL_TEXT, COL_BG);
  d.setTextPadding(110);
  formatValue(buf, sizeof(buf), minSeen, g.decimals);
  d.drawString(buf, 80, 204);
  formatValue(buf, sizeof(buf), maxSeen, g.decimals);
  d.drawString(buf, 240, 204);
  d.setTextPadding(0);
}

// Graph: andamento nel tempo, tracciato "a scansione" come un monitor:
// il pennino avanza a destra e cancella poco più avanti, senza ridisegnare tutto.
const int GRAPH_X0 = 36, GRAPH_W = 276;
const int GRAPH_Y0 = 62, GRAPH_H = 146;
const int GRAPH_STEP  = 2; // pixel per campione
const int GRAPH_ERASE = 8; // spazio vuoto davanti al pennino

void drawGraphGrid(int x, int w) {
  for (int i = 1; i < 4; ++i) {
    M5.Display.drawFastHLine(x, GRAPH_Y0 + GRAPH_H * i / 4, w, COL_TRACK);
  }
}

void drawGraphGauge(const Gauge& g, float v, bool full) {
  static float shown = NAN;
  static int   gx = 0, prevY = 0;
  static bool  havePrev = false;
  auto& d = M5.Display;

  if (full) {
    char buf[12];
    d.drawRect(GRAPH_X0 - 1, GRAPH_Y0 - 1, GRAPH_W + 2, GRAPH_H + 2, COL_TRACK);
    drawGraphGrid(GRAPH_X0, GRAPH_W);
    d.setFont(&fonts::Font2);
    d.setTextSize(1);
    d.setTextColor(COL_LABEL);
    d.setTextDatum(TR_DATUM);
    formatValue(buf, sizeof(buf), g.maxV, scaleDecimals(g));
    d.drawString(buf, GRAPH_X0 - 4, GRAPH_Y0 - 2);
    d.setTextDatum(BR_DATUM);
    formatValue(buf, sizeof(buf), g.minV, scaleDecimals(g));
    d.drawString(buf, GRAPH_X0 - 4, GRAPH_Y0 + GRAPH_H + 2);
    drawUnit(d, g.unit, 276, 52, COL_LABEL);
    shown    = NAN;
    gx       = 0;
    havePrev = false;
  }

  // Valore attuale in alto a destra
  if (full || !sameValue(v, shown)) {
    char buf[12];
    formatValue(buf, sizeof(buf), v, g.decimals);
    d.setFont(&fonts::Font4);
    d.setTextSize(1);
    d.setTextColor(valueColor(g.idx, v), COL_BG);
    d.setTextDatum(BR_DATUM);
    d.setTextPadding(120);
    d.drawString(buf, 270, 56);
    d.setTextPadding(0);
    shown = v;
  }
  if (isnan(v)) return;

  // Un campione per lettura, anche se il valore non cambia (asse del tempo)
  int x = GRAPH_X0 + gx;
  int y = GRAPH_Y0 + GRAPH_H - 1 - (int)(gaugeFrac(g, v) * (GRAPH_H - 1));
  int w = min(GRAPH_ERASE, GRAPH_X0 + GRAPH_W - x);
  d.fillRect(x, GRAPH_Y0, w, GRAPH_H, COL_BG);
  drawGraphGrid(x, w);
  if (havePrev) d.drawLine(x - GRAPH_STEP, prevY, x, y, valueColor(g.idx, v));
  else          d.drawPixel(x, y, valueColor(g.idx, v));
  prevY    = y;
  havePrev = true;

  gx += GRAPH_STEP;
  if (gx >= GRAPH_W) { // a capo: ricomincia da sinistra
    gx       = 0;
    havePrev = false;
  }
}

// Bar: numero grande e barra orizzontale con scala
void drawBarGauge(const Gauge& g, float v, bool full) {
  static float shown = NAN;
  auto&     d  = M5.Display;
  const int bx = 20, by = 128, bw = 280, bh = 36;
  const int cx = d.width() / 2;

  if (full) {
    drawUnit(d, g.unit, cx - unitWidth(d, g.unit) / 2, 114, COL_LABEL);
    d.setFont(&fonts::Font2);
    d.setTextSize(1);
    d.setTextColor(COL_LABEL);
    d.setTextDatum(TC_DATUM);
    for (int i = 0; i <= 4; ++i) {
      char buf[8];
      int  x = bx + bw * i / 4;
      d.drawFastVLine(x, by + bh + 3, 6, COL_LABEL);
      formatValue(buf, sizeof(buf), g.minV + (g.maxV - g.minV) * i / 4, scaleDecimals(g));
      d.drawString(buf, x, by + bh + 12);
    }
    shown = NAN;
  }
  if (!full && sameValue(v, shown)) return;
  shown = v;

  uint16_t col = valueColor(g.idx, v);
  char     buf[12];
  formatValue(buf, sizeof(buf), v, g.decimals);
  d.setFont(&fonts::Font7);
  d.setTextSize(1);
  d.setTextColor(col, COL_BG);
  d.setTextDatum(MC_DATUM);
  d.setTextPadding(240);
  d.drawString(buf, cx, 68);
  d.setTextPadding(0);

  int fw = (int)(gaugeFrac(g, v) * bw);
  d.fillRect(bx, by, fw, bh, col);
  d.fillRect(bx + fw, by, bw - fw, bh, COL_TRACK);
}

void gaugeScreen(const Gauge& g, bool full) {
  if (full) drawHeader(g.title);

  requestAndParse(g.idx);
  float v = values[g.idx];

  switch (settings.gaugeStyle) {
    case STYLE_ANALOG: drawAnalogGauge(g, v, full); break;
    case STYLE_DIGIT:  drawDigitGauge(g, v, full);  break;
    case STYLE_GRAPH:  drawGraphGauge(g, v, full);  break;
    case STYLE_BAR:    drawBarGauge(g, v, full);    break;
    default:           drawArcGauge(g, v, full);    break;
  }
}

// --- DTC Screen ---
// Lo stato DTC (PID 0101) si legge solo su richiesta, premendo il tasto A.
// Il tasto A (GPIO39) è letto in polling con M5.BtnA: un interrupt su GPIO39
// riceverebbe falsi impulsi con il Bluetooth attivo (limite hardware ESP32).
void drawDtcBody(const char* busyMsg = nullptr) {
  auto& d = M5.Display;
  const int cx = d.width() / 2;
  d.fillRect(0, HEADER_H, d.width(), FOOTER_Y - 1 - HEADER_H, COL_BG);
  d.setTextSize(1);
  d.setTextDatum(MC_DATUM);

  if (busyMsg) {
    d.setFont(&fonts::FreeSansBold12pt7b);
    d.setTextColor(COL_ACCENT);
    d.drawString(busyMsg, cx, 120);
    return;
  }

  switch (dtcState) {
    case DTC_NOT_READ:
      d.setFont(&fonts::FreeSans9pt7b);
      d.setTextColor(COL_LABEL);
      d.drawString("Press A to read", cx, 110);
      d.drawString("the DTC status", cx, 134);
      break;

    case DTC_NO_RESPONSE:
      d.setFont(&fonts::FreeSansBold12pt7b);
      d.setTextColor(COL_ALARM);
      d.drawString("No response", cx, 105);
      d.setFont(&fonts::FreeSans9pt7b);
      d.setTextColor(COL_LABEL);
      d.drawString("Press A to retry", cx, 145);
      break;

    case DTC_READ: {
      int  a   = (int)values[IDX_DTC];
      bool mil = a & 0x80; // bit 7 del byte A: spia MIL
      int  n   = a & 0x7F; // bit 0-6: numero di DTC memorizzati

      // Spia MIL
      d.fillRoundRect(cx - 90, 38, 180, 40, 8, mil ? COL_ALARM : COL_OK);
      d.setFont(&fonts::FreeSansBold12pt7b);
      d.setTextColor(COL_BG); // su riquadro pieno
      d.drawString(mil ? "MIL ON" : "MIL OFF", cx, 58);

      // Numero di DTC
      char buf[6];
      snprintf(buf, sizeof(buf), "%d", n);
      d.setFont(&fonts::Font7);
      d.setTextColor(n > 0 ? COL_WARN : COL_OK);
      d.drawString(buf, cx, 124);
      d.setFont(&fonts::Font2);
      d.setTextColor(COL_LABEL);
      d.drawString(n == 1 ? "stored DTC" : "stored DTCs", cx, 164);
      d.drawString("Press A to read again", cx, 200);
    } break;
  }
}

void dtcScreen(bool full) {
  if (full) {
    drawHeader("DTC Status");
    drawDtcBody();
  }

  // In attesa del tasto A: qui non si interroga l'ELM, il loop resta veloce
  if (!M5.BtnA.wasPressed()) return;

  if (!linkUp()) {
    drawDtcBody("Not connected");
    delay(1500);
    drawDtcBody();
    return;
  }

  drawDtcBody("Reading...");
  values[IDX_DTC] = NAN;
  requestAndParse(IDX_DTC, 2000); // timeout lungo: la prima richiesta può includere "SEARCHING..."
  dtcState = isnan(values[IDX_DTC]) ? DTC_NO_RESPONSE : DTC_READ;
  drawDtcBody();
}

// --- Boot Screen ---
const int BOOT_LOG_Y = 84;
int       bootLine   = 0;

void bootScreen() {
  auto& d = M5.Display;
  d.fillScreen(COL_BG);
  d.setFont(&fonts::FreeSansBold12pt7b);
  d.setTextSize(1);
  d.setTextColor(COL_TEXT);
  d.setTextDatum(TC_DATUM);
  d.drawString("OBD-II Monitor", d.width() / 2, 18);
  d.setFont(&fonts::Font2);
  d.setTextColor(COL_ACCENT);
  d.drawString("ELM327 Bluetooth", d.width() / 2, 50);
  d.drawFastHLine(20, 74, d.width() - 40, COL_TRACK);
  bootLine = 0;
}

// Log a righe sulla schermata di avvio (ricomincia dall'alto quando è piena).
// Si ferma sopra il footer, che serve per la scelta in caso di errore.
void bootLog(const char* msg, uint16_t color) {
  auto& d = M5.Display;
  const int lineH = 18, maxLines = (FOOTER_Y - 1 - BOOT_LOG_Y) / lineH;
  if (bootLine >= maxLines) {
    d.fillRect(0, BOOT_LOG_Y, d.width(), FOOTER_Y - 1 - BOOT_LOG_Y, COL_BG);
    bootLine = 0;
  }
  String s(msg);
  s.replace('\r', ' ');
  s.replace('\n', ' ');
  d.setFont(&fonts::Font2);
  d.setTextSize(1);
  d.setTextColor(color, COL_BG);
  d.setTextDatum(TL_DATUM);
  d.drawString(s.c_str(), 12, BOOT_LOG_Y + bootLine * lineH);
  ++bootLine;
  Serial.println(msg);
}

// --- Settings Storage (NVS) ---
void macToStr(const uint8_t* addr, char* out, size_t len) {
  snprintf(out, len, "%02X:%02X:%02X:%02X:%02X:%02X",
           addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

// Nessun dispositivo salvato: si usa l'ELM327 preimpostato
void applyDefaultDevice() {
  settings.hasDevice = true;
  settings.isDefault = true;
  memcpy(settings.addr, ELM_DEFAULT_ADDR, sizeof(settings.addr));
  strlcpy(settings.name, ELM_DEFAULT_NAME, sizeof(settings.name));
}

void loadSettings() {
  memset(&settings, 0, sizeof(settings));
  prefs.begin(PREFS_NS, false);
  settings.hasDevice = prefs.getBytes("addr", settings.addr, sizeof(settings.addr)) == sizeof(settings.addr);
  prefs.getString("name", settings.name, sizeof(settings.name));
  settings.demo       = prefs.getBool("demo", false);
  settings.gaugeStyle = prefs.getUChar("style", STYLE_ARC);
  settings.theme      = prefs.getUChar("theme", 0);
  prefs.end();
  if (settings.gaugeStyle >= STYLE_COUNT) settings.gaugeStyle = STYLE_ARC;
  if (settings.theme >= THEME_COUNT)      settings.theme = 0;
  if (!settings.hasDevice) applyDefaultDevice();
  applyTheme(settings.theme);
}

void saveSettings() {
  prefs.begin(PREFS_NS, false);
  prefs.putBool("demo", settings.demo);
  prefs.putUChar("style", settings.gaugeStyle);
  prefs.putUChar("theme", settings.theme);
  if (settings.hasDevice && !settings.isDefault) {
    prefs.putBytes("addr", settings.addr, sizeof(settings.addr));
    prefs.putString("name", settings.name);
  } else {
    prefs.remove("addr");
    prefs.remove("name");
  }
  prefs.end();
}

// Cancella dispositivo salvato, modalità demo e abbinamenti BT memorizzati dallo stack.
// Si torna all'ELM327 preimpostato.
void clearSettings() {
  prefs.begin(PREFS_NS, false);
  prefs.clear();
  prefs.end();
  memset(&settings, 0, sizeof(settings));
  applyDefaultDevice();
  applyTheme(settings.theme);

  startBluetooth();
  if (ELM.connected(0)) ELM.disconnect();
  elmOnline = false;

  // Abbinamenti BT: API ESP-IDF, il core 2.0.x non ha deleteAllBondedDevices()
  esp_bd_addr_t bonded[8];
  int           count = sizeof(bonded) / sizeof(bonded[0]);
  if (esp_bt_gap_get_bond_device_num() > 0 &&
      esp_bt_gap_get_bond_device_list(&count, bonded) == ESP_OK) {
    for (int i = 0; i < count; ++i) esp_bt_gap_remove_bond_device(bonded[i]);
  }
}

// --- Demo Mode ---
int clampInt(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Risposte ELM327 simulate: passano dallo stesso parser delle risposte vere.
// Valori che variano nel tempo (motore che si scalda, giri che salgono e scendono).
String demoResponse(uint8_t idx) {
  float t       = millis() / 1000.0f;
  float rpm     = 1800 + 1200 * sinf(t * 0.35f) + 150 * sinf(t * 2.1f);
  float speed   = (rpm - 800) / 25.0f;
  float coolant = min(40 + t * 0.5f, 90.0f) + 2 * sinf(t * 0.05f); // 40 -> 90 °C in ~100 s
  float load    = 20 + (rpm - 800) / 40.0f;
  float maf     = rpm / 250.0f;
  float intake  = 28 + 3 * sinf(t * 0.02f);
  float volt    = 14.1f + 0.15f * sinf(t * 0.3f);

  char        buf[24];
  const char* pid = PIDS[idx].cmd;
  auto one = [&](float a) {
    snprintf(buf, sizeof(buf), "41%s%02X", pid + 2, clampInt((int)a, 0, 255));
  };
  auto two = [&](float v) {
    int x = clampInt((int)v, 0, 65535);
    snprintf(buf, sizeof(buf), "41%s%02X%02X", pid + 2, x >> 8, x & 0xFF);
  };

  switch (idx) {
    case IDX_COOLANT: one(coolant + 40);       break;
    case IDX_INTAKE:  one(intake + 40);        break;
    case IDX_RPM:     two(rpm * 4);            break;
    case IDX_LOAD:    one(load * 255 / 100);   break;
    case IDX_MAF:     two(maf * 100);          break;
    case IDX_BARO:    one(101);                break;
    case IDX_SPEED:   one(speed);              break;
    case IDX_VOLT:    snprintf(buf, sizeof(buf), "%.1fV", volt); break;
    case IDX_DTC:     snprintf(buf, sizeof(buf), "4101%02X076500", 0x82); break; // MIL accesa, 2 DTC
    default:          return String(); // altri PID del catalogo: vedi demoValue()
  }
  return String(buf) + "\r\r>";
}

// Valore simulato per i PID del catalogo senza risposta demo dedicata:
// oscilla dentro la scala della sua pagina di dettaglio
float demoValue(uint8_t idx) {
  for (const Gauge& g : GAUGES) {
    if (g.idx != idx) continue;
    float t = millis() / 1000.0f;
    return g.minV + (g.maxV - g.minV) * (0.5f + 0.35f * sinf(t * 0.3f + idx));
  }
  return NAN;
}

// --- Connection Flow ---
void resetReadings() {
  for (auto& v : values) v = NAN;
  dtcState  = DTC_NOT_READ;
  pidIdx    = 0;
  pidsKnown = false; // si rileggono in ELMinit(); in demo e offline si mostra tutto
}

// Connessione all'avvio (e dopo le impostazioni). Se non riesce chiede cosa fare:
// A = impostazioni, B = riprova, C = continua senza adattatore.
void connectFlow() {
  resetReadings();
  while (true) {
    bootScreen();

    if (settings.demo) {
      if (btStackOn && ELM.connected(0)) ELM.disconnect();
      elmOnline = false;
      bootLog("Modalita demo: ELM327 simulato", COL_ACCENT);
      delay(1000);
      break;
    }

    if (BTconnect()) {
      delay(500);
      elmOnline = ELMinit();
      if (elmOnline) {
        delay(1000);
        break;
      }
    }
    elmOnline = false;

    // Tasto A in polling (niente interrupt su GPIO39), B e C dai loro interrupt
    drawFooter("SETUP", "RETRY", "SKIP");
    discardPresses();
    int choice = 0;
    while (!choice) {
      M5.update();
      if (M5.BtnA.wasPressed())   choice = 1;
      else if (takePresses(btnB)) choice = 2;
      else if (takePresses(btnC)) choice = 3;
      delay(10);
    }
    if (choice == 1) settingsMenu(); // poi riprova con le nuove impostazioni
    if (choice == 3) break;          // continua offline
  }
  discardPresses();
  forceRedraw = true;
}

// --- Menu UI (lista a righe, usata da impostazioni e ricerca BT) ---
const int ROW_H     = 36;
const int LIST_Y    = HEADER_H + 4;
const int LIST_ROWS = 5;

void clearBody() {
  auto& d = M5.Display;
  d.fillRect(0, HEADER_H, d.width(), FOOTER_Y - 1 - HEADER_H, COL_BG);
}

// Riga di menu: etichetta a sinistra, valore a destra, eventuale seconda riga piccola
void drawRow(int row, bool selected, const char* label, const char* value = nullptr,
             const char* sub = nullptr, uint16_t valueCol = COL_ACCENT) {
  auto& d = M5.Display;
  int   y = LIST_Y + row * ROW_H;
  d.fillRect(0, y, d.width(), ROW_H, COL_BG);
  if (selected) {
    d.fillRoundRect(4, y + 1, d.width() - 8, ROW_H - 2, 6, COL_PANEL);
    d.fillRoundRect(4, y + 1, 5, ROW_H - 2, 2, COL_ACCENT);
  }

  int labelY = sub ? y + 11 : y + ROW_H / 2;
  d.setTextSize(1);
  d.setFont(&fonts::FreeSans9pt7b);
  d.setTextColor(selected ? COL_TEXT : COL_LABEL);
  d.setTextDatum(ML_DATUM);
  d.drawString(label, 16, labelY);

  if (sub) {
    d.setFont(&fonts::Font2);
    d.setTextColor(COL_LABEL);
    d.setTextDatum(TL_DATUM);
    d.drawString(sub, 16, y + 19);
  }
  if (value) {
    d.setFont(&fonts::Font2);
    d.setTextColor(valueCol);
    d.setTextDatum(MR_DATUM);
    d.drawString(value, d.width() - 14, labelY);
  }
}

// Frecce di scorrimento a destra della lista: ci sono voci sopra / sotto
void drawScrollHint(bool up, bool down) {
  auto& d = M5.Display;
  const int x = d.width() - 8;
  d.fillRect(x - 5, LIST_Y - 4, 11, 4, COL_BG);
  d.fillRect(x - 5, LIST_Y + LIST_ROWS * ROW_H, 11, FOOTER_Y - 1 - (LIST_Y + LIST_ROWS * ROW_H), COL_BG);
  if (up)   d.fillTriangle(x - 4, LIST_Y - 1, x + 4, LIST_Y - 1, x, LIST_Y - 4, COL_LABEL);
  if (down) d.fillTriangle(x - 4, LIST_Y + LIST_ROWS * ROW_H + 2, x + 4, LIST_Y + LIST_ROWS * ROW_H + 2,
                           x, LIST_Y + LIST_ROWS * ROW_H + 8, COL_LABEL);
}

// Richiesta di conferma: A = sì, B/C = no
bool confirmDialog(const char* title, const char* line1, const char* line2) {
  auto& d = M5.Display;
  drawHeader(title, false);
  clearBody();
  d.setTextSize(1);
  d.setFont(&fonts::FreeSans9pt7b);
  d.setTextColor(COL_TEXT);
  d.setTextDatum(MC_DATUM);
  d.drawString(line1, d.width() / 2, 100);
  d.setTextColor(COL_LABEL);
  d.drawString(line2, d.width() / 2, 126);
  drawFooter("YES", "NO", "NO");

  discardPresses();
  while (true) {
    M5.update();
    if (M5.BtnA.wasPressed()) return true;
    if (takePresses(btnB) + takePresses(btnC) > 0) return false;
    delay(10);
  }
}

// --- Bluetooth Device Selection ---
// Cerca i dispositivi Bluetooth Classic vicini e salva quello scelto.
// Ritorna true se è stato salvato un nuovo dispositivo.
bool selectDevice() {
  struct Found { uint8_t addr[6]; char name[32]; char mac[18]; };
  static Found found[MAX_FOUND]; // static: evita ~600 byte sullo stack

  auto& d = M5.Display;
  while (true) {
    // 1) Ricerca
    drawHeader("Select device", false);
    clearBody();
    drawFooter(nullptr, nullptr, nullptr);
    d.setTextSize(1);
    d.setTextDatum(MC_DATUM);
    d.setFont(&fonts::FreeSansBold12pt7b);
    d.setTextColor(COL_ACCENT);
    d.drawString("Scanning...", d.width() / 2, 105);
    d.setFont(&fonts::Font2);
    d.setTextColor(COL_LABEL);
    d.drawString("Adapter on, not connected to a phone", d.width() / 2, 140);

    startBluetooth();
    elmOnline = false;                          // discover() chiude la connessione attiva
    BTScanResults* res = ELM.discover(BT_SCAN_MS);
    int n = 0;
    for (int i = 0; res && i < res->getCount() && n < MAX_FOUND; ++i) {
      BTAdvertisedDevice* dev = res->getDevice(i);
      BTAddress           a   = dev->getAddress();
      memcpy(found[n].addr, *a.getNative(), 6);
      macToStr(found[n].addr, found[n].mac, sizeof(found[n].mac));
      strlcpy(found[n].name, dev->haveName() ? dev->getName().c_str() : found[n].mac, sizeof(found[n].name));
      ++n;
    }

    // 2) Scelta: dispositivi trovati + "Rescan" + "Cancel"
    const int count = n + 2;
    int  sel = 0, top = 0;
    bool redraw = true;
    drawHeader(n ? "Select device" : "No devices found", false);
    clearBody();
    drawFooter("OK", "DOWN", "UP");
    discardPresses();

    while (true) {
      M5.update();
      int step = takePresses(btnB) - takePresses(btnC);
      if (step) {
        sel    = wrapIndex(sel + step, count);
        redraw = true;
      }
      if (redraw) {
        if (sel < top)              top = sel;
        if (sel >= top + LIST_ROWS) top = sel - LIST_ROWS + 1;
        for (int r = 0; r < LIST_ROWS; ++r) {
          int i = top + r;
          if (i >= count)  d.fillRect(0, LIST_Y + r * ROW_H, d.width(), ROW_H, COL_BG);
          else if (i < n)  drawRow(r, i == sel, found[i].name, nullptr, found[i].mac);
          else             drawRow(r, i == sel, i == n ? "Rescan" : "Cancel");
        }
        drawScrollHint(top > 0, top + LIST_ROWS < count);
        redraw = false;
      }
      if (M5.BtnA.wasPressed()) {
        if (sel < n) {
          settings.hasDevice = true;
          settings.isDefault = false;
          memcpy(settings.addr, found[sel].addr, 6);
          strlcpy(settings.name, found[sel].name, sizeof(settings.name));
          saveSettings();
          return true;
        }
        if (sel == n) break; // Rescan
        return false;        // Cancel
      }
      delay(10);
    }
  }
}

// --- Settings Menu ---
// Menu modale: B/C spostano la selezione, A conferma.
// Ritorna true se serve riconnettersi (impostazioni cambiate o "Reconnect").
bool settingsMenu() {
  enum { ITEM_DEVICE, ITEM_DEMO, ITEM_STYLE, ITEM_THEME, ITEM_RECONNECT, ITEM_CLEAR, ITEM_EXIT, ITEM_COUNT };
  bool changed = false;
  int  sel = 0, top = 0;
  bool full = true;

  // Disegna la voce i nella riga visibile row
  auto drawItem = [&](int i, int row) {
    bool s = (i == sel);
    switch (i) {
      case ITEM_DEVICE: {
        char mac[18] = "";
        if (settings.hasDevice) macToStr(settings.addr, mac, sizeof(mac));
        drawRow(row, s, "Device", settings.hasDevice ? settings.name : "none",
                settings.hasDevice ? mac : "Press A to search", settings.hasDevice ? COL_ACCENT : COL_WARN);
      } break;
      case ITEM_DEMO:
        drawRow(row, s, "Demo mode", settings.demo ? "ON" : "OFF", nullptr, settings.demo ? COL_ACCENT : COL_LABEL);
        break;
      case ITEM_STYLE:
        drawRow(row, s, "Gauge style", STYLE_NAMES[settings.gaugeStyle], "Press A to change");
        break;
      case ITEM_THEME:
        drawRow(row, s, "Theme", THEMES[settings.theme].name, "Press A to change");
        break;
      case ITEM_RECONNECT: drawRow(row, s, "Reconnect");    break;
      case ITEM_CLEAR:     drawRow(row, s, "Clear memory"); break;
      case ITEM_EXIT:      drawRow(row, s, "Exit");         break;
    }
  };

  discardPresses();
  while (true) {
    M5.update();
    int step = takePresses(btnB) - takePresses(btnC);
    if (step) sel = wrapIndex(sel + step, ITEM_COUNT);

    if (full) {
      drawHeader("Settings", false);
      clearBody();
      drawFooter("OK", "DOWN", "UP");
    }
    if (full || step) {
      if (sel < top)              top = sel;
      if (sel >= top + LIST_ROWS) top = sel - LIST_ROWS + 1;
      for (int r = 0; r < LIST_ROWS; ++r) drawItem(top + r, r);
      drawScrollHint(top > 0, top + LIST_ROWS < ITEM_COUNT);
      full = false;
    }

    if (M5.BtnA.wasPressed()) {
      switch (sel) {
        case ITEM_DEVICE:
          if (selectDevice()) changed = true;
          full = true;
          break;
        case ITEM_DEMO:
          settings.demo = !settings.demo;
          saveSettings();
          changed = true;
          drawItem(sel, sel - top);
          break;
        case ITEM_STYLE: // non serve riconnettersi: vale dalla prossima pagina di dettaglio
          settings.gaugeStyle = (settings.gaugeStyle + 1) % STYLE_COUNT;
          saveSettings();
          drawItem(sel, sel - top);
          break;
        case ITEM_THEME: // applicato subito: il menu ridisegnato fa da anteprima
          settings.theme = (settings.theme + 1) % THEME_COUNT;
          applyTheme(settings.theme);
          saveSettings();
          full = true;
          break;
        case ITEM_RECONNECT:
          return true;
        case ITEM_CLEAR:
          if (confirmDialog("Clear memory", "Erase saved device and settings?", "(back to default ELM327)")) {
            clearSettings();
            changed = true;
          }
          full = true;
          break;
        case ITEM_EXIT:
          return changed;
      }
      discardPresses();
    }
    delay(10);
  }
}

// --- Settings Screen (pagina nella rotazione) ---
// Riepilogo delle impostazioni; con A si apre il menu.
void settingsScreen(bool full) {
  if (full) {
    char mac[18] = "";
    if (settings.hasDevice) macToStr(settings.addr, mac, sizeof(mac));

    drawHeader("Settings");
    drawRow(0, false, "Device", settings.hasDevice ? settings.name : "none",
            settings.hasDevice ? mac : nullptr, settings.hasDevice ? COL_ACCENT : COL_WARN);
    drawRow(1, false, "Mode", settings.demo ? "Demo" : "Bluetooth");
    drawRow(2, false, "Link", linkUp() ? "Connected" : "Offline", nullptr, linkUp() ? COL_OK : COL_ALARM);
    drawRow(3, false, "Gauge style", STYLE_NAMES[settings.gaugeStyle]);
    drawRow(4, false, "Theme", THEMES[settings.theme].name); // A = "OPEN" nel footer apre il menu
  }

  if (!M5.BtnA.wasPressed()) return;

  if (settingsMenu()) connectFlow();
  discardPresses();
  forceRedraw = true;
}
