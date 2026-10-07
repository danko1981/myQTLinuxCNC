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

volatile char currentAxis = '0';
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

// INTERPRETE DEI DATI TRASMESSI DA LINUXCNC
void parsePCData() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("POS:")) {
      char axisChar = line.charAt(4);
      String valStr = line.substring(6);
      float newVal = valStr.toFloat();
      String nextionObj = "t" + String(axisChar);
      nextionSetText(nextionObj, newVal, 3);
    } else if (line.startsWith("MOD:")) {
      String modeStr = line.substring(4);
      nextionSetTextStr("tmode", modeStr);
    } else if (line.startsWith("OVR:")) {
      String valStr = line.substring(4);  // Prende "F100%"
      nextionSetTextStr("tF", valStr);    // Lo invia alla textbox
    } else if (line.startsWith("SEL:")) {
      currentAxis = line.charAt(4);
      updateNextionUI();
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

void serialListenerTask(void* parameter) {
  for (;;) {
    while (Serial2.available()) {
      char rawChar = (char)Serial2.read();
      if (rawChar == '#') {
        expectingCmd = true;
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
          currentAxis = cmd;
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
        else if (cmd == 'd') Serial.println("CMD:MACRO_3");
        else if (cmd == 'f') Serial.println("CMD:MACRO_4");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
