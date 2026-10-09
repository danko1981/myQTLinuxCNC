# myQTLinuxCNC
This is my linuxcnc configuration for my Machinator

---

## Pendant ESP32 (MPG)

Il pendant è un volantino elettronico basato su **ESP32** con display touch **Nextion**, encoder rotativo, joystick analogico, 4 pulsanti e un fungo di emergenza. Comunica con LinuxCNC tramite USB (seriale a 115200 baud).

### Architettura

```
 Nextion (touch)  --Serial2-->  ESP32  --USB/seriale-->  esp32_mpg.py  --HAL / API linuxcnc-->  LinuxCNC
                 <--Serial2--          <--USB/seriale--
```

| File | Ruolo |
|------|-------|
| [ESP32_Pendant/esp32_mpg/esp32_mpg.ino](ESP32_Pendant/esp32_mpg/esp32_mpg.ino) | Firmware ESP32: legge pulsanti, encoder, joystick e touch Nextion e li invia al PC come righe di testo. Riceve dal PC posizioni e stato e aggiorna il display. |
| [ESP32_Pendant/esp32_cncmpg_v2.HMI](ESP32_Pendant/esp32_cncmpg_v2.HMI) | Progetto grafico del display (Nextion Editor). |
| [esp32_mpg.py](esp32_mpg.py) | Componente HAL userspace `esp_mpg`: traduce i messaggi seriali in pin HAL e comandi LinuxCNC e rimanda al pendant DRO, modo e feed override. |
| [custom.hal](custom.hal) | Carica `esp32_mpg.py` e collega i suoi pin a jog, E-Stop e joystick. |

### Installazione

1. **Firmware ESP32**: aprire `ESP32_Pendant/esp32_mpg/esp32_mpg.ino` con Arduino IDE (core ESP32 installato) e caricarlo sulla scheda.
2. **Display Nextion**: aprire `ESP32_Pendant/esp32_cncmpg_v2.HMI` con Nextion Editor e caricarlo sul display (tramite SD o USB-TTL).
3. **PC LinuxCNC**:
   - pacchetti necessari: `python-serial` / `python3-serial` e `xdotool` (per la modalità mouse);
   - l'utente deve poter accedere alla porta seriale: `sudo usermod -a -G dialout $USER` (poi logout/login);
   - `qtdragon.ini` deve caricare `custom.hal` (`HALFILE = custom.hal`, già presente).

Non serve configurare la porta: lo script cerca automaticamente `/dev/ttyUSB*` e `/dev/ttyACM*` e, se il cavo viene scollegato, ferma il joystick e riprova a connettersi ogni 2 secondi.

### Collegamenti ESP32

| Funzione | GPIO |
|----------|------|
| Fungo E-Stop (NC verso GND, pull-up interno) | 13 |
| Pulsante verde | 25 |
| Pulsante rosso | 26 |
| Pulsante giallo | 27 |
| Pulsante blu (MODE / SHIFT) | 14 |
| LED stato E-Stop | 2 |
| Encoder CLK / DT / SW | 22 / 23 / 21 |
| Nextion RX2 / TX2 | 16 / 17 |
| Joystick X / Y (analogici) | 36 / 39 |
| Pulsante joystick | 18 |

Tutti gli ingressi digitali sono attivi bassi (pull-up interno, il pulsante chiude verso GND).

### Uso del pendant

#### Pulsanti fisici

Il pulsante **blu** funziona anche da **SHIFT**: tenendolo premuto, gli altri pulsanti eseguono la funzione alternativa.

| Pulsante | Pressione semplice | Con SHIFT (blu tenuto premuto) |
|----------|--------------------|--------------------------------|
| **Blu** | Click breve (< 400 ms): cambia modo MANUALE → AUTO → MDI → MANUALE | – |
| **Verde** | **CYCLE START**: avvia il file caricato oppure riprende da una pausa (anche dalla pausa `M0` del cambio utensile M6) | **HOME ALL**: homing di tutti gli assi |
| **Giallo** | **PAUSA COMPLETA**: salva gli RPM, mette in pausa il programma e dopo 1 s spegne il mandrino | **PAUSA SEMPLICE**: pausa senza spegnere il mandrino |
| **Rosso** | **STOP**: abort del programma e mandrino spento | **MANDRINO ON/OFF**: `M3 S12000` / `M5` (solo a macchina ferma) |
| **Fungo E-Stop** | Premuto: E-Stop immediato | Rilasciato: reset E-Stop **e macchina ON** |

> **Attenzione:** rilasciando il fungo la macchina viene riabilitata automaticamente (`halui.estop.reset` + `halui.machine.on`).

**Ripresa dopo la pausa completa:** premendo il verde dopo una pausa fatta con il giallo, il mandrino viene riacceso agli RPM salvati e il programma riparte dopo 3 secondi. Se la pausa è di sistema (es. `M0` dentro M6) il mandrino non viene toccato.

#### Encoder (volantino)

- Sul display si seleziona l'asse da muovere: **X, Y, Z, A**, oppure **F** per il feed override, oppure **0** per disattivare l'encoder.
- Il jog con encoder funziona **solo in modo MANUALE**, con macchina abilitata e senza E-Stop.
- Il passo per scatto si sceglie dal display: **0 / 0.01 / 0.1 / 1 / 10 mm**.
- Con **F** selezionato, ogni scatto cambia il feed override dell'1%, tra 50% e 200%.
- Durante l'esecuzione di un G-code in AUTO il pendant deseleziona l'asse (`SEL:F`) e lo rilascia (`SEL:0`) a fine programma, per evitare movimenti accidentali.

#### Joystick

Il modo del joystick si sceglie dal display (pulsante Wheel/Joy):

- **Modo WHEEL**: il joystick fa da **mouse** sul PC (movimento puntatore, pulsante = click sinistro). Richiede `xdotool`.
- **Modo JOY**: il joystick muove la macchina in velocità continua.
  - Piano predefinito **XY** (l'encoder resta su Z).
  - Premendo il pulsante del joystick **con lo stick al centro** si passa al piano **ZA** (asse verticale = Z, orizzontale = A) e viceversa; nel piano ZA l'encoder può essere assegnato a X o Y dal display.
  - Velocità proporzionale all'inclinazione, fino a `[TRAJ] DEFAULT_LINEAR_VELOCITY`; a fondo corsa (≥ 98%) passa in rapido con `[TRAJ] MAX_LINEAR_VELOCITY`.
  - Zona morta centrale del 15%.
  - Uscendo dal modo JOY tutti gli assi vengono fermati.

Nota: in `custom.hal` il joystick per l'asse A è commentato; va abilitato se si configura il 4° motore.

#### Pulsanti sul display Nextion

| Funzione | Azione |
|----------|--------|
| **G53** | Il DRO del pendant mostra le coordinate macchina (solo visualizzazione). |
| **G54 … G57** | Attiva il sistema di coordinate scelto (MDI) e torna al DRO in coordinate pezzo. |
| **Zero X / Y / Z** | `G10 L20 P0 <asse>0`: azzera l'asse nel sistema di coordinate attivo. Lo zero A non è gestito. |
| **Zero ALL** | `G10 L20 P0 X0 Y0 Z0` |
| **Macro 1** | `G90 G53 G0 Z0`, poi `G90 G53 G1 X0 Y0 F2000`: va allo zero macchina. |
| **Macro 2** | `G90 G53 G0 Z0`, poi `G90 G1 X0 Y0 F2000`: va sopra lo zero pezzo XY restando a Z macchina 0 (non scende a G54 Z0, così c'è spazio per il touch plate). |
| **INIT TOOL** | Va in posizione di cambio; all'arrivo LinuxCNC apre il keypad (page2) per il numero utensile, poi `M61 Qn` e misura sul tool sensor. Vedi [INIT TOOL](#init-tool-dal-pendant). |
| **Macro 4** | `O<touch_plate> call`: esegue [probe/basic_probe/macros/touch_plate.ngc](probe/basic_probe/macros/touch_plate.ngc). Richiede un utensile misurato con TOUCH SENSOR o M6; partire al massimo `[PROBE] TOUCH_PLATE_MAXPROBE` mm (40) sopra il piattello. |

Le macro e i comandi MANDRINO, MODE e HOME vengono eseguiti solo se la macchina è abilitata, senza E-Stop e con l'interprete fermo. I comandi MDI passano temporaneamente in modo MDI e poi tornano in MANUALE; durante l'attesa lo script continua a leggere l'E-Stop.

#### INIT TOOL dal pendant

Serve a montare e misurare il primo utensile, dichiarandone il numero dal display invece che con `M61 Qn`.

1. Macchina accesa e in homing. Premi **INIT TOOL**: il mandrino sale a G53 Z0 e va in `[CHANGE_POSITION]`.
2. Quando il mandrino è arrivato, LinuxCNC mostra su QtDragon il messaggio **CHANGE TOOL** e fa aprire al Nextion la page2 (keypad), tramite `PAGE:TOOL`.
3. Monta l'utensile, scrivi il numero sul keypad (`5`, `T5`, `Q5` o `M61 Q5`) e premi **Enter**: il display torna alla pagina principale.
4. `esp32_mpg.py` esegue `M61 Qn` e poi [tool_sensor.ngc](probe/basic_probe/macros/tool_sensor.ngc): il mandrino va sul sensore, misura l'utensile e scrive l'offset in tabella.

- **Indietro** sul keypad annulla: la macchina resta in posizione di cambio e non viene dichiarato nessun utensile.
- Se la macchina non è pronta o non è in homing, oppure il movimento viene interrotto (STOP, E-Stop), il keypad non si apre.
- Se il numero non è valido il keypad si riapre; se l'utensile non è in tabella, `M61` fallisce e la misura non parte.
- Un E-Stop durante l'attesa annulla la procedura.
- Mentre il keypad è aperto joystick ed encoder sono disattivati; tornando alla pagina principale vanno riselezionati l'asse e il modo JOY.

Lato Nextion (progetto `.HMI`) servono questi eventi:

| Oggetto | Evento Touch Release |
|---------|----------------------|
| page0 `b8` (INIT TOOL) | solo `prints "#d",0` (la page2 la apre LinuxCNC) |
| page2 Enter | `prints "#T",0`, `prints t0.txt,0`, `prints ";",0`, `t0.txt=""`, `page page0` |
| page2 `b20` (Indietro) | `prints "#K",0`, `t0.txt=""`, `page page0` |

I nomi delle pagine sono definiti in `NEXTION_MAIN_PAGE` e `NEXTION_TOOL_PAGE` nel firmware.

Anche il cambio utensile `M6` nei programmi mostra su QtDragon il messaggio **CHANGE TOOL: monta Tn e premi CYCLE START** quando il mandrino è in posizione di cambio (vedi [python/remap.py](python/remap.py)).

### Interfaccia con `esp32_mpg.py`

#### Pin HAL del componente `esp_mpg`

| Pin | Tipo | Collegato a (custom.hal) |
|-----|------|--------------------------|
| `esp_mpg.count-x/y/z/a` | s32 out | `axis.<n>.jog-counts` |
| `esp_mpg.scale` | float out | `axis.<n>.jog-scale` (default 0.1) |
| `esp_mpg.joy-vel-x/y/z` | float out | `halui.axis.<n>.analog` e `halui.joint.<n>.analog` |
| `esp_mpg.joy-vel-a` | float out | non collegato |
| `esp_mpg.estop` | bit out | `halui.estop.activate`; negato su `halui.estop.reset` e `halui.machine.on` |

Oltre ai pin HAL, lo script usa direttamente l'API Python `linuxcnc` (`command` / `stat`) per start, pausa, abort, modi, homing, MDI, feed override e per leggere le posizioni. Su `ESTOP:ON` invia anche `STATE_ESTOP` via API, così l'arresto non dipende solo dal pin HAL.

Lo script legge da `qtdragon.ini` le velocità del joystick (sezione `[TRAJ]`); se l'INI non è leggibile usa i valori di default.

#### Protocollo seriale ESP32 → PC

Una riga di testo per messaggio, terminata da `\n`.

| Messaggio | Significato |
|-----------|-------------|
| `ESTOP:ON` / `ESTOP:OFF` | Fungo premuto / rilasciato |
| `JOG:<asse>:<scatti>` | Scatti encoder (asse `X`,`Y`,`Z`,`A`, oppure `F` per il feed override) |
| `JOY:<asse>:<valore>:<G0\|G1>` | Joystick, valore normalizzato da -1.0 a 1.0 |
| `MOUSE:MOV:<x>:<y>` / `MOUSE:CLICK` | Modo mouse |
| `SCALE:<mm>` | Passo encoder (`0.0`, `0.01`, `0.10`, `1.00`, `10.0`) |
| `CMD:CYCLE_START` / `CMD:CYCLE_PAUSE_FULL` / `CMD:CYCLE_PAUSE` / `CMD:CYCLE_STOP` | Controllo ciclo |
| `CMD:SPINDLE_TOGGLE` / `CMD:MODE_TOGGLE` / `CMD:HOMEALL` | Mandrino, modo, homing |
| `CMD:G53` … `CMD:G57` | Sistema di coordinate / visualizzazione DRO |
| `CMD:ZERO:<X\|Y\|Z\|A\|ALL>` | Azzeramento assi |
| `CMD:MACRO_1` / `CMD:MACRO_2` / `CMD:MACRO_4` | Macro |
| `CMD:INIT_TOOL` | Avvio INIT TOOL (posizione di cambio e attesa del numero utensile) |
| `TOOL:SET:<testo>` / `TOOL:CANCEL` | Numero utensile dal keypad / keypad annullato |

#### Protocollo seriale PC → ESP32

| Messaggio | Significato |
|-----------|-------------|
| `POS:<asse>:<valore>` | DRO (3 decimali), inviato solo se cambia di più di 0.0005 |
| `MOD:<MAN\|AUTO\|MDI>` | Modo macchina |
| `OVR:F<n>%` | Feed override |
| `SEL:<asse>` | Forza l'asse selezionato sull'encoder (`F` / `0` durante i programmi AUTO) |
| `PAGE:MAIN` / `PAGE:TOOL` | Torna alla pagina principale (INIT TOOL annullato da E-Stop) / apre il keypad (mandrino in posizione di cambio, o numero non valido) |

Il DRO viene aggiornato ogni 50 ms. Il valore mostrato è in coordinate pezzo (macchina − G5x − G92 − offset utensile), oppure in coordinate macchina dopo **G53**.

#### Protocollo Nextion → ESP32

Il display invia `#` seguito da un carattere di comando:

| Carattere | Comando | Carattere | Comando |
|-----------|---------|-----------|---------|
| `M0` / `M1` | Modo Wheel / Joy | `q` | G53 |
| `X` `Y` `Z` `A` | Selezione asse encoder | `w` `e` `r` `t` | G54 / G55 / G56 / G57 |
| `F` | Encoder su feed override | `x` `y` `z` `a` | Zero X / Y / Z / A |
| `0` | Encoder disattivato | `*` | Zero ALL |
| `o` `i` `j` `k` `l` | Passo 0 / 0.01 / 0.1 / 1 / 10 | `h` `c` `f` | Macro 1 / 2 / 4 |
| `d` | INIT TOOL | `T<testo>;` / `K` | Numero utensile dal keypad / Indietro |

Per aggiungere un nuovo comando: assegnare un carattere libero nel progetto `.HMI`, aggiungere la riga `else if (cmd == '…') Serial.println("CMD:…");` in `serialListenerTask()` del firmware e gestire `CMD:…` in `process_serial_data()` di `esp32_mpg.py`.

### Debug

- `DEBUG_MODE = True` in `esp32_mpg.py` stampa ogni comando ricevuto sul terminale da cui è stato avviato LinuxCNC (`[DEBUG] …`); errori e connessioni compaiono come `[ERROR/INFO] …`.
- Per controllare i pin HAL: `halshow`, oppure `halcmd show pin esp_mpg`.
- Per vedere i messaggi grezzi del pendant (con LinuxCNC spento): `screen /dev/ttyUSB0 115200`.
