#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// --- PIN DEFINITIONS ---
#define ESTOP_BTN_PIN 13
#define GREEN_BTN_PIN 25
#define RED_BTN_PIN 26
#define YELLOW_BTN_PIN 27
#define BLUE_BTN_PIN 14
#define LED_PIN 2

// --- ENCODER ---
#define ENC_CLK 22
#define ENC_DT 23
#define ENC_SW 21

// --- SERIALE NEXTION ---
#define RXD2 16
#define TXD2 17

// --- JOYSTICK ANALOGICO ---
#define JOY_X_PIN 36
#define JOY_Y_PIN 39
#define JOY_BTN_PIN 18

// --- PAGINE NEXTION ---
#define NEXTION_MAIN_PAGE "page0"  // Pagina principale (DRO e pulsanti)
#define NEXTION_TOOL_PAGE "page2"  // Keypad per il numero utensile (INIT TOOL)
#define NEXTION_AUTO_PAGE "page3"  // Pagina AUTO (esecuzione programma)
#define PAGE_ID_MAIN 0             // Numeri inviati dagli eventi di caricamento pagina ("#p0", "#p2", "#p3")
#define PAGE_ID_TOOL 2
#define PAGE_ID_AUTO 3
#define TOOL_TEXT_MAX_LEN 16       // Lunghezza massima del testo inviato dal keypad

// --- COLORI NEXTION ---
#define COLOR_GREEN 2032   // Verde standard Nextion
#define COLOR_WHITE 63455  // Il tuo bianco
#define COLOR_CYAN 2047    // Ciano
#define COLOR_BLACK 0      // Nero
#define COLOR_GRAY 1040    // Grigione

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile long encoderCount = 0;
volatile int lastEncoded = 0;
long encoderAccumulator = 0;
const int PULSES_PER_CLICK = 4;

volatile char currentAxis = '0';  // In page3: 'L' = scelta riga per RUN LINE, 'F' = feed override
volatile int currentPage = PAGE_ID_MAIN;
unsigned long sendtomega = 0;

// VARIABILI PER E-STOP
int lastEstopState = HIGH;
unsigned long lastDebounceEstop = 0;

// VARIABILI PER HOMING-ASSI
int lastGreenBtnState = HIGH;
unsigned long lastDebounceGreenBtn = 0;

// VARIABILI PER CYCLE START
int lastRedBtnState = HIGH;
unsigned long lastDebounceRedBtn = 0;

// VARIABILI PER MANDRINO START-STOP
int lastYellowBtnState = HIGH;
unsigned long lastDebounceYellowBtn = 0;

//VARIABILI PER PULSANTE MODE AUTO MDI MAN
int lastBlueBtnState = HIGH;
unsigned long lastDebounceBlueBtn = 0;

// --- VARIABILI SHIFT / MODE ---
bool isShiftActive = false;          // Diventa true finché tieni premuto il Mode
bool shiftActionUsed = false;        // Diventa true se premi un altro tasto mentre Shift è attivo
unsigned long shiftPressedTime = 0;  // Registra quando hai iniziato a premere il tasto

// --- VARIABILI JOYSTICK & MODI ---
bool isJoyMode = false;   // 0 = Wheel, 1 = Joy (da bt13)
bool joyPlaneZA = false;  // 0 = XY, 1 = ZA
int lastJoyBtnState = HIGH;
unsigned long lastDebounceJoyBtn = 0;
float lastJoyX = 0.0;
float lastJoyY = 0.0;
unsigned long lastJoySend = 0;

// --- ISR ENCODER ---
void IRAM_ATTR isrEncoder() {
  int MSB = digitalRead(ENC_CLK);
  int LSB = digitalRead(ENC_DT);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount--;
  lastEncoded = encoded;
}

void serialListenerTask(void* parameter);

// --- HELPER NEXTION ---
void nextionSetText(String objName, float value, int decimals) {
  Serial2.print(objName);
  Serial2.print(".txt=\"");
  Serial2.print(value, decimals);
  Serial2.print("\"");
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

void nextionSetTextStr(String objName, String text) {
  Serial2.print(objName);
  Serial2.print(".txt=\"");
  Serial2.print(text);
  Serial2.print("\"");
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

void nextionSetColor(String objName, int color) {
  Serial2.print(objName);
  Serial2.print(".pco=");
  Serial2.print(color);  // CAMBIA IN .bco= SE USI LO SFONDO
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

void nextionCmd(String cmd) {
  Serial2.print(cmd);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

// Pagina attiva: la comunica anche a LinuxCNC, che reinvia i dati della pagina
void setCurrentPage(int pageId) {
  currentPage = pageId;
  Serial.print("PAGEID:");
  Serial.println(pageId);
}

void nextionGotoPage(const char* pageName, int pageId) {
  nextionCmd(String("page ") + pageName);
  setCurrentPage(pageId);
}

void nextionSetVal(String objName, int val) {
  Serial2.print(objName);
  Serial2.print(".val=");
  Serial2.print(val);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

// --- GESTIONE GRAFICA UI ---
void updateNextionUI() {
  if (currentPage != PAGE_ID_MAIN) return;  // Gli oggetti tX..tA esistono solo in page0
  if (!isJoyMode) {
    // MODALITÀ WHEEL
    nextionSetColor("tX", (currentAxis == 'X') ? COLOR_WHITE : COLOR_GREEN);
    nextionSetColor("tY", (currentAxis == 'Y') ? COLOR_WHITE : COLOR_GREEN);
    nextionSetColor("tZ", (currentAxis == 'Z') ? COLOR_WHITE : COLOR_GREEN);
    nextionSetColor("tA", (currentAxis == 'A') ? COLOR_WHITE : COLOR_GREEN);
    nextionSetColor("tF", (currentAxis == 'F') ? COLOR_BLACK : COLOR_GRAY);
  } else {
    // MODALITÀ JOY
    if (!joyPlaneZA) {
      // Piano XY (Joy abilitato, Z di default su Encoder)
      nextionSetColor("tX", COLOR_CYAN);
      nextionSetColor("tY", COLOR_CYAN);
      nextionSetColor("tZ", (currentAxis == 'Z') ? COLOR_WHITE : COLOR_GREEN);
      nextionSetColor("tA", (currentAxis == 'A') ? COLOR_WHITE : COLOR_GREEN);
    } else {
      // Piano ZA (Joy abilitato, Encoder per X o Y selezionati manualmente)
      nextionSetColor("tZ", COLOR_CYAN);
      nextionSetColor("tA", COLOR_CYAN);
      nextionSetColor("tX", (currentAxis == 'X') ? COLOR_WHITE : COLOR_GREEN);
      nextionSetColor("tY", (currentAxis == 'Y') ? COLOR_WHITE : COLOR_GREEN);
    }
  }
}

// Stato di sicurezza quando si entra o si esce dalla page2 (INIT TOOL):
// joystick e encoder disattivati. Tornando su page0 il Nextion ricarica i
// valori di default (modo DRO, nessun asse selezionato), quindi ci si allinea.
void resetMainPageState() {
  isJoyMode = false;
  joyPlaneZA = false;
  currentAxis = '0';
  portENTER_CRITICAL(&timerMux);
  encoderCount = 0;
  portEXIT_CRITICAL(&timerMux);
}

// PAGE3: stato del programma da LinuxCNC (stessi colori degli eventi del progetto .HMI).
// LinuxCNC e' l'unica fonte dello stato: vale anche per avvii da QtDragon o dai pulsanti fisici.
void applyAutoState(String state) {
  bool running = (state == "RUNNING");
  bool paused = (state == "PAUSED");
  bool stopped = !running && !paused;  // STOPPED o ESTOP

  String label = (state == "ESTOP") ? String("E-STOP") : state;
  nextionSetTextStr("t4", label);
  nextionCmd(paused ? "t4.pco=63456" : "t4.pco=65535");
  nextionCmd(stopped ? "t4.picc=5" : "t4.picc=6");
  nextionSetVal("va0", stopped ? 0 : 1);

  nextionSetVal("bt0", running ? 1 : 0);                      // START: premuto mentre gira
  nextionCmd((running || state == "ESTOP") ? "tsw bt0,0" : "tsw bt0,1");
  nextionSetVal("bt1", 0);                                    // STOP: attivo solo con programma in corso
  nextionCmd(stopped ? "tsw bt1,0" : "tsw bt1,1");
  nextionSetVal("bt2", paused ? 1 : 0);                       // PAUSE: premuto = in pausa
  nextionCmd(stopped ? "tsw bt2,0" : "tsw bt2,1");
}

// INTERPRETE DEI DATI TRASMESSI DA LINUXCNC
void parsePCData() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    bool onAuto = (currentPage == PAGE_ID_AUTO);
    if (line.startsWith("POS:")) {
      // DRO: tX..tA in page0, tX_p3..tA_p3 in page3
      if (currentPage == PAGE_ID_MAIN || onAuto) {
        char axisChar = line.charAt(4);
        String valStr = line.substring(6);
        float newVal = valStr.toFloat();
        String nextionObj = "t" + String(axisChar) + (onAuto ? "_p3" : "");
        nextionSetText(nextionObj, newVal, 3);
      }
    } else if (line.startsWith("MOD:")) {
      if (currentPage != PAGE_ID_TOOL) nextionSetTextStr("tmode", line.substring(4));
    } else if (line.startsWith("OVR:")) {
      if (currentPage != PAGE_ID_TOOL) nextionSetTextStr("tF", line.substring(4));  // "F100%"
    } else if (line.startsWith("SEL:")) {
      char axis = line.charAt(4);
      if (onAuto) currentAxis = (axis == '0') ? 'L' : axis;  // Fine programma: torna alla scelta riga
      else currentAxis = axis;
      updateNextionUI();
    } else if (line == "PAGE:MAIN") {
      // INIT TOOL annullato o rifiutato da LinuxCNC: torna alla pagina principale
      nextionGotoPage(NEXTION_MAIN_PAGE, PAGE_ID_MAIN);
      resetMainPageState();
      delay(50);  // Attende il caricamento di page0
      updateNextionUI();
    } else if (line == "PAGE:TOOL") {
      // Mandrino arrivato in posizione di cambio (o numero non valido): apre il keypad
      resetMainPageState();
      nextionGotoPage(NEXTION_TOOL_PAGE, PAGE_ID_TOOL);
      nextionSetTextStr("t0", "");
    } else if (line == "PAGE:AUTO") {
      // Partenza di un programma: apre la pagina AUTO
      resetMainPageState();
      nextionGotoPage(NEXTION_AUTO_PAGE, PAGE_ID_AUTO);
      currentAxis = 'L';
    } else if (onAuto) {
      // --- Contenuti della page3 ---
      if (line.startsWith("ST:")) {
        applyAutoState(line.substring(3));
      } else if (line.startsWith("GL:") && line.length() >= 5) {
        // Righe G-code: GL:0 precedente, GL:1 corrente/selezionata, GL:2 successiva
        nextionSetTextStr("t" + String(line.charAt(3)), line.substring(5));
      } else if (line.startsWith("MSG:")) {
        nextionSetTextStr("t3", line.substring(4));
      } else if (line.startsWith("OPT:")) {
        nextionSetVal("bt5", line.charAt(4) == '1' ? 1 : 0);
      } else if (line.startsWith("BLK:")) {
        nextionSetVal("bt6", line.charAt(4) == '1' ? 1 : 0);
      } else if (line.startsWith("SLD:F:")) {
        nextionSetVal("h1", line.substring(6).toInt());
      } else if (line.startsWith("SLD:S:")) {
        nextionSetVal("h0", line.substring(6).toInt());
      }
    }
  }
}

// CALIBRAZIONE ASIMMETRICA JOYSTICK
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float getNormalizedJoy(int raw) {
  float norm = 0.0;
  // Calibrazione: centro fisico dell'ADC ESP32 è circa 2048, il minimo è ~250.
  if (raw >= 2048) {
    norm = mapFloat(raw, 2048, 4095, 0.0, 1.0);
  } else {
    norm = mapFloat(raw, 250, 2048, -1.0, 0.0);
  }

  // Limiti di sicurezza hardware
  if (norm > 1.0) norm = 1.0;
  if (norm < -1.0) norm = -1.0;

  // Deadzone al 10%
  if (abs(norm) < 0.15) norm = 0.0;

  return norm;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(ESTOP_BTN_PIN, INPUT_PULLUP);
  pinMode(GREEN_BTN_PIN, INPUT_PULLUP);
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  pinMode(RED_BTN_PIN, INPUT_PULLUP);
  pinMode(YELLOW_BTN_PIN, INPUT_PULLUP);
  pinMode(BLUE_BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_CLK), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), isrEncoder, CHANGE);

  xTaskCreatePinnedToCore(serialListenerTask, "serialListenerTask", 10000, NULL, 1, NULL, 0);

  Serial.println("ESP32 Ready. Cycle Start Enabled.");
  updateNextionUI();
}

void loop() {
  parsePCData();

  // 1. PULSANTI FISICI (E-STOP, HOME, CYCLE, SPINDLE, MODE)
  
  // E-STOP & Machine ON
  int currentEstopState = digitalRead(ESTOP_BTN_PIN);
  if (currentEstopState != lastEstopState) {
    if (millis() - lastDebounceEstop > 50) {
      if (currentEstopState == HIGH) {
        Serial.println("ESTOP:ON");
        digitalWrite(LED_PIN, HIGH);
      } else {
        Serial.println("ESTOP:OFF");
        digitalWrite(LED_PIN, LOW);
      }
      lastEstopState = currentEstopState;
      lastDebounceEstop = millis();
    }
  }

  // BUTTON BLUE (Pin 14): MODE / ↑SHIFT (Pin 14)
  int currentBlueBtnState = digitalRead(BLUE_BTN_PIN);
  if (currentBlueBtnState != lastBlueBtnState) {
    if (millis() - lastDebounceBlueBtn > 50) {
       
       if (currentBlueBtnState == LOW) {
          // TASTO PREMUTO: Attiva la modalità Shift e registra il tempo
          isShiftActive = true;
          shiftActionUsed = false; // Resetta il flag
          shiftPressedTime = millis();
       } 
       else {
          // TASTO RILASCIATO: Disattiva lo Shift
          isShiftActive = false;
          
          // Se lo hai rilasciato velocemente (es. sotto i 400ms) E non hai premuto altri tasti nel mentre, 
          // allora era un semplice click per cambiare modalità!
          if (!shiftActionUsed && (millis() - shiftPressedTime < 400)) {
             Serial.println("CMD:MODE_TOGGLE");
          }
       }
       lastBlueBtnState = currentBlueBtnState; 
       lastDebounceBlueBtn = millis();
    }
  }

  // BUTTON GREEN (Pin 25): CYCLE-START / ↑HOME-ALL
  int currentGreenBtnState = digitalRead(GREEN_BTN_PIN);
  if (currentGreenBtnState != lastGreenBtnState) {
    if (millis() - lastDebounceGreenBtn > 50) {
       if (currentGreenBtnState == LOW) {
          
          // CONTROLLA SE SHIFT E' PREMUTO
          if (isShiftActive) {
             // ---> FUNZIONE SHIFT (Mode + Home)
             Serial.println("CMD:HOMEALL");             
             // Diciamo al tasto Mode che è stato usato come Shift, 
             // così quando lo rilasci NON invierà il MODE_TOGGLE
             shiftActionUsed = true; 
          } 
          else {
             // ---> FUNZIONE NORMALE (Solo Home)
             // Qui ora il tasto è libero! Puoi fargli fare quello che vuoi,
             // ad esempio mandare la macchina allo zero pezzo.
             Serial.println("CMD:CYCLE_START"); 
          }
       }
       lastGreenBtnState = currentGreenBtnState; 
       lastDebounceGreenBtn = millis();
    }
  }
   
  // RED BUTTON (Pin 26): CYCLE-STOP / SPINDLE_ON-OFF
  int currentRedBtnState = digitalRead(RED_BTN_PIN);
  if (currentRedBtnState != lastRedBtnState) {
    if (millis() - lastDebounceRedBtn > 50) {
      if (currentRedBtnState == LOW) 
      {          
          // CONTROLLA SE SHIFT E' PREMUTO
          if (isShiftActive) {
             // ---> FUNZIONE SHIFT (Mode + Red)
             Serial.println("CMD:SPINDLE_TOGGLE");
             // Diciamo al tasto Mode che è stato usato come Shift, 
             // così quando lo rilasci NON invierà il MODE_TOGGLE
             shiftActionUsed = true; 
          }
          else {
             // ---> FUNZIONE NORMALE (Solo Red)
             // Qui ora il tasto è libero! Puoi fargli fare quello che vuoi,
             // ad esempio mandare la macchina allo zero pezzo.
            Serial.println("CMD:CYCLE_STOP");             
          }
      }
      lastRedBtnState = currentRedBtnState;
      lastDebounceRedBtn = millis();
    }
  }
  
  // YELLOW BUTTON (Pin 27): CYCLE_PAUSE_FULL / ↑CYCLE_PAUSE
  int currentYellowBtnState = digitalRead(YELLOW_BTN_PIN);
  if (currentYellowBtnState != lastYellowBtnState) {
    if (millis() - lastDebounceYellowBtn > 50) {
      if (currentYellowBtnState == LOW) {          
          // CONTROLLA SE SHIFT E' PREMUTO
          if (isShiftActive) {
             // ---> FUNZIONE SHIFT (Mode + Yellow) -> Pausa Semplice
             Serial.println("CMD:CYCLE_PAUSE");
             shiftActionUsed = true; 
          }
          else {
             // ---> FUNZIONE NORMALE (Solo Yellow) -> Pausa + Spindle Off
             Serial.println("CMD:CYCLE_PAUSE_FULL");             
          }
      }
      lastYellowBtnState = currentYellowBtnState;
      lastDebounceYellowBtn = millis();
    }
  }
  
  // 2. GESTIONE JOYSTICK E MOUSE (50ms refresh rate)
  if (millis() - lastJoySend > 50) {
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);

    float normX = getNormalizedJoy(rawX);
    float normY = getNormalizedJoy(rawY);
    int currentJoyBtnState = digitalRead(JOY_BTN_PIN);

    if (isJoyMode) {
      // ==========================================
      // MODALITÀ CNC JOYSTICK
      // ==========================================
      if (currentJoyBtnState != lastJoyBtnState) {
        if (millis() - lastDebounceJoyBtn > 50) {
          if (currentJoyBtnState == LOW) {
            if (normX == 0.0 && normY == 0.0) {
              joyPlaneZA = !joyPlaneZA;
              if (!joyPlaneZA) {
                currentAxis = 'Z';
                nextionSetVal("bt5", 0);
                nextionSetVal("bt6", 0);
                nextionSetVal("bt7", 1);
                nextionSetVal("bt8", 0);
              } else {
                currentAxis = '0';
                nextionSetVal("bt5", 0);
                nextionSetVal("bt6", 0);
                nextionSetVal("bt7", 0);
                nextionSetVal("bt8", 0);
              }
              updateNextionUI();
            }
          }
          lastJoyBtnState = currentJoyBtnState;
          lastDebounceJoyBtn = millis();
        }
      }

      if (abs(normX - lastJoyX) > 0.05 || (normX == 0.0 && lastJoyX != 0.0)) {
        String mode = (abs(normX) >= 0.98) ? "G0" : "G1";
        char axis = joyPlaneZA ? 'A' : 'X';
        Serial.print("JOY:");
        Serial.print(axis);
        Serial.print(":");
        Serial.print(normX);
        Serial.print(":");
        Serial.println(mode);
        lastJoyX = normX;
      }

      if (abs(normY - lastJoyY) > 0.05 || (normY == 0.0 && lastJoyY != 0.0)) {
        String mode = (abs(normY) >= 0.98) ? "G0" : "G1";
        char axis = joyPlaneZA ? 'Z' : 'Y';
        Serial.print("JOY:");
        Serial.print(axis);
        Serial.print(":");
        Serial.print(normY);
        Serial.print(":");
        Serial.println(mode);
        lastJoyY = normY;
      }
    } else {
      // ==========================================
      // MODALITÀ WHEEL -> JOYSTICK DIVENTA MOUSE
      // ==========================================
      // 1. Sicurezza: Se stiamo uscendo dalla modalità Joy, fermiamo i motori CNC
      if (lastJoyX != 0.0 || lastJoyY != 0.0) {
        Serial.println("JOY:X:0.0:G1");
        Serial.println("JOY:Y:0.0:G1");
        Serial.println("JOY:Z:0.0:G1");
        Serial.println("JOY:A:0.0:G1");
        lastJoyX = 0.0;
        lastJoyY = 0.0;
      }

      // 2. Click Sinistro del Mouse
      if (currentJoyBtnState != lastJoyBtnState) {
        if (millis() - lastDebounceJoyBtn > 50) {
          if (currentJoyBtnState == LOW) {
            Serial.println("MOUSE:CLICK");
          }
          lastJoyBtnState = currentJoyBtnState;
          lastDebounceJoyBtn = millis();
        }
      }

      // 3. Movimento del Mouse
      if (normX != 0.0 || normY != 0.0) {
        Serial.print("MOUSE:MOV:");
        Serial.print(normX);
        Serial.print(":");
        Serial.println(normY);
      }
    }

    lastJoySend = millis();
  }

  // 3. GESTIONE ENCODER
  if (millis() - sendtomega > 50) {
    if (currentAxis != '0') {
      long countsRead = 0;
      portENTER_CRITICAL(&timerMux);
      countsRead = encoderCount;
      encoderCount = 0;
      portEXIT_CRITICAL(&timerMux);

      encoderAccumulator += countsRead;
      int cleanCounts = encoderAccumulator / PULSES_PER_CLICK;

      if (cleanCounts != 0) {
        encoderAccumulator -= (cleanCounts * PULSES_PER_CLICK);
        Serial.print("JOG:");
        Serial.print(currentAxis);
        Serial.print(":");
        Serial.println(cleanCounts);
      }
    } else {
      portENTER_CRITICAL(&timerMux);
      encoderCount = 0;
      portEXIT_CRITICAL(&timerMux);
      encoderAccumulator = 0;
    }
    sendtomega = millis();
  }
}

// --- TASK: NEXTION -> ESP32 -> PC ---
bool expectingCmd = false;
bool readingToolText = false;  // Dopo "#T": testo del keypad fino a ';'
String toolText = "";
bool expectingPageId = false;  // Dopo "#p": numero della pagina appena caricata
char sliderTarget = 0;         // Dopo "#V" (SPD) o "#W" (FED): 1 byte con il valore 0..100

void serialListenerTask(void* parameter) {
  for (;;) {
    while (Serial2.available()) {
      char rawChar = (char)Serial2.read();
      if (sliderTarget) {
        // Valore binario dello slider (prints h0.val,1): puo' valere anche '#', va letto per primo
        Serial.print("SLD:");
        Serial.print(sliderTarget);
        Serial.print(":");
        Serial.println((uint8_t)rawChar);
        sliderTarget = 0;
      } else if (expectingPageId) {
        expectingPageId = false;
        if (rawChar >= '0' && rawChar <= '9') {
          int pageId = rawChar - '0';
          // Pagina caricata (anche da touch): joystick ed encoder ripartono da uno stato sicuro
          resetMainPageState();
          if (pageId == PAGE_ID_AUTO) currentAxis = 'L';
          setCurrentPage(pageId);
          updateNextionUI();
        }
      } else if (rawChar == '#') {
        readingToolText = false;  // Un nuovo comando interrompe un testo incompleto
        expectingCmd = true;
      } else if (readingToolText) {
        if (rawChar == ';') {
          // Enter sul keypad: il Nextion e' gia' tornato alla pagina principale
          readingToolText = false;
          Serial.print("TOOL:SET:");
          Serial.println(toolText);
          resetMainPageState();
          vTaskDelay(50 / portTICK_PERIOD_MS);  // Attende il caricamento di page0
          setCurrentPage(PAGE_ID_MAIN);
          updateNextionUI();
        } else if (rawChar >= ' ' && rawChar <= '~' && toolText.length() < TOOL_TEXT_MAX_LEN) {
          // Solo caratteri stampabili: scarta eventuali codici di risposta del Nextion
          toolText += rawChar;
        }
      } else if (expectingCmd) {
        expectingCmd = false;
        char cmd = rawChar;

        // --- NUOVI COMANDI MODO JOY/WHEEL ---
        if (cmd == 'M') {
          // Aspetta 1ms per leggere lo stato 0 o 1
          delay(2);
          if (Serial2.available()) {
            char modeState = (char)Serial2.read();
            isJoyMode = (modeState == '1');
            if (isJoyMode) {
              joyPlaneZA = false;  // Reset al piano default
              currentAxis = 'Z';   // Default encoder su Z
              nextionSetVal("bt5", 0);
              nextionSetVal("bt6", 0);
              nextionSetVal("bt7", 1);
              nextionSetVal("bt8", 0);
            }
            updateNextionUI();
          }
        }
        // SELEZIONE ASSI ENCODER - F PER IL FEED OVERRIDE
        else if (cmd == 'X' || cmd == 'Y' || cmd == 'Z' || cmd == 'A' || cmd == 'F' || cmd == '0') {
          // In page3 togliere F (tF) riporta l'encoder sulla scelta della riga
          currentAxis = (cmd == '0' && currentPage == PAGE_ID_AUTO) ? 'L' : cmd;
          portENTER_CRITICAL(&timerMux);
          encoderCount = 0;
          portEXIT_CRITICAL(&timerMux);
          updateNextionUI();  // Aggiorna i colori al tocco
        } else if (cmd == 'o') Serial.println("SCALE:0.0");
        else if (cmd == 'i') Serial.println("SCALE:0.01");
        else if (cmd == 'j') Serial.println("SCALE:0.10");
        else if (cmd == 'k') Serial.println("SCALE:1.00");
        else if (cmd == 'l') Serial.println("SCALE:10.0");
        else if (cmd == 'q') Serial.println("CMD:G53");
        else if (cmd == 'w') Serial.println("CMD:G54");
        else if (cmd == 'e') Serial.println("CMD:G55");
        else if (cmd == 'r') Serial.println("CMD:G56");
        else if (cmd == 't') Serial.println("CMD:G57");
        else if (cmd == 'x') Serial.println("CMD:ZERO:X");
        else if (cmd == 'y') Serial.println("CMD:ZERO:Y");
        else if (cmd == 'z') Serial.println("CMD:ZERO:Z");
        else if (cmd == 'a') Serial.println("CMD:ZERO:A");
        else if (cmd == '*') Serial.println("CMD:ZERO:ALL");
        else if (cmd == 'h') Serial.println("CMD:MACRO_1");
        else if (cmd == 'c') Serial.println("CMD:MACRO_2");
        else if (cmd == 'd') {
          // INIT TOOL: LinuxCNC aprira' la page2 (PAGE:TOOL) quando il mandrino
          // e' in posizione di cambio
          Serial.println("CMD:INIT_TOOL");
        }
        else if (cmd == 'T') {
          readingToolText = true;
          toolText = "";
        }
        else if (cmd == 'K') {
          // Indietro dal keypad senza confermare
          Serial.println("TOOL:CANCEL");
          resetMainPageState();
          vTaskDelay(50 / portTICK_PERIOD_MS);
          setCurrentPage(PAGE_ID_MAIN);
          updateNextionUI();
        }
        else if (cmd == 'f') Serial.println("CMD:MACRO_4");
        // --- PAGE3 (AUTO) ---
        else if (cmd == 'S') Serial.println("CMD:CYCLE_START");
        else if (cmd == 'E') Serial.println("CMD:CYCLE_STOP");
        else if (cmd == 'P') Serial.println("CMD:PAUSE_TOGGLE");
        else if (cmd == 'N') Serial.println("CMD:STEP");
        else if (cmd == 'R') Serial.println("CMD:RUN_LINE");
        else if (cmd == 'O') Serial.println("CMD:OPT_STOP");
        else if (cmd == 'B') Serial.println("CMD:BLK_DEL");
        else if (cmd == 'V') sliderTarget = 'S';  // Slider SPD (h0)
        else if (cmd == 'W') sliderTarget = 'F';  // Slider FED (h1)
        else if (cmd == 'p') expectingPageId = true;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
