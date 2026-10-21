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
| **Blu** | Click breve (< 400 ms): cambia modo MANUALE → AUTO → MDI → MANUALE; in AUTO il Nextion apre la [pagina AUTO](#pagina-auto-page3) | – |
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

#### Pagina AUTO (page3)

La page3 corrisponde al modo **AUTO** della macchina, la page0 ai modi MANUALE e MDI:

- **Il Nextion segue la macchina:** a ogni passaggio in AUTO si apre la page3, qualunque sia la causa: pulsante blu, QtDragon (anche caricando un file), CYCLE START, partenza di un programma. Uscendo da AUTO (per esempio pulsante blu AUTO → MDI, o jog da QtDragon) la page3 torna alla page0. Il keypad dell'INIT TOOL (page2) non viene mai interrotto.
- **La macchina segue il Nextion:** toccando `tmode` in page0 si apre la page3 e la macchina passa in AUTO; toccando `tmode` in page3 si torna alla page0 e la macchina passa in MANUALE, così l'encoder torna a muovere gli assi. Il cambio di modo avviene solo a interprete fermo: durante un programma la page0 si può aprire per consultarla, ma la macchina resta in AUTO.

A fine programma la page3 resta aperta. Lo stato mostrato viene sempre da LinuxCNC.

| Elemento | Funzione |
|----------|----------|
| `t4` | Stato: **RUNNING**, **PAUSED** (lampeggia), **STOPPED**, **E-STOP** |
| `tX_p3` … `tA_p3` | DRO, come in page0 |
| `bt7` ABS / `t5` | `bt7` premuto: DRO in coordinate assolute (macchina), `t5.picc=5`. `bt7` rilasciato: DRO nel sistema di coordinate attivo, `t5.picc=6`. `t5` mostra sempre il sistema attivo (`G54`…`G59.3`). Il sistema attivo non cambia; l'impostazione vale anche per il DRO della page0, dove `bt0` (G53) e `bt1`…`bt4` (G54…G57) si allineano |
| `t0` / `t1` / `t2` | Riga G-code precedente / corrente / successiva, con numero di riga. Durante il programma `t1` è la riga in esecuzione; a programma fermo è la riga scelta per RUN LINE |
| `t3` | Messaggi operatore (CHANGE TOOL dell'M6, probe fallito, INIT TOOL, errori del pendant); senza messaggi mostra il nome del file |
| `h0` SPD / `h1` FED | Override mandrino (da `MIN_SPINDLE_0_OVERRIDE` a `MAX_SPINDLE_0_OVERRIDE`) e feed (da 0 a `MAX_FEED_OVERRIDE`), letti da `[DISPLAY]` in `qtdragon.ini`. Seguono anche le modifiche fatte da QtDragon o dall'encoder |
| **START** | Avvia il programma o riprende da una pausa (anche la pausa dell'M6) |
| **STOP** | Abort del programma e mandrino spento. A macchina ferma resta rosso (premuto) |
| **PAUSE** | Pausa completa (come il pulsante giallo); premuto di nuovo riprende come START |
| **STEP** | Da fermo avvia il programma una riga alla volta; in pausa esegue la riga successiva |
| **RUN LINE** | Avvia il programma dalla riga mostrata in `t1` |
| **OPT STOP** / **BLK DEL** | Attiva/disattiva l'arresto opzionale (`M1`) e la cancellazione blocchi (`/`) |

**Encoder in page3:** a programma fermo scorre le righe (`t1`) per scegliere da dove ripartire con RUN LINE; dopo uno STOP propone l'ultima riga eseguita. Durante il programma regola il feed override; toccando `tF` si passa dall'una all'altra funzione.

##### PAUSE: pausa e ripresa

PAUSE ha doppia funzione e decide in base allo stato reale di LinuxCNC:

- **programma in esecuzione:** pausa completa, come il pulsante giallo: il programma si ferma, gli RPM vengono salvati e dopo 1 s il mandrino si spegne;
- **programma in pausa:** ripresa, come START. Se la pausa aveva spento il mandrino, lo riaccende agli RPM salvati, attende 3 s e riprende. Vale anche per la pausa dell'M6 e per `M1`.

Durante le attese di 1 s e 3 s viene accettato solo STOP (l'E-Stop è sempre attivo); gli altri comandi, compreso un secondo tocco su PAUSE, vengono ignorati. Alla fine dell'attesa la sequenza ricontrolla lo stato: riprende solo se il programma è ancora in pausa e spegne il mandrino solo se è davvero in pausa. Così un doppio tocco non può far ripartire il programma con il mandrino spento.

##### STEP, RUN LINE, OPT STOP e BLK DEL

Sono funzioni standard di LinuxCNC (le stesse di QtDragon) per controllare come viene eseguito il programma.

**STEP (esecuzione riga per riga)**: ogni pressione esegue la riga successiva del programma e poi si ferma.
- Serve per provare un programma nuovo con calma, per esempio il primo avvicinamento al pezzo o la prima discesa in Z, controllando ogni movimento prima di dare il successivo.
- A programma fermo avvia il programma in modalità STEP; in pausa esegue la riga successiva. Per tornare all'esecuzione normale premi START o PAUSE.

**RUN LINE (avvio da una riga)**: avvia il programma dalla riga scelta invece che dall'inizio.
- Serve per riprendere un lavoro interrotto (fresa rotta, STOP, mancanza di corrente) senza rifare le parti già lavorate.
- A programma fermo la riga si sceglie con l'encoder ed è quella mostrata in `t1`; dopo uno STOP viene proposta l'ultima riga eseguita.

> **Attenzione con RUN LINE:** LinuxCNC salta le righe prima di quella scelta, quindi cambio utensile, accensione del mandrino e quote di sicurezza scritti all'inizio del programma non vengono eseguiti. Prima di avviare da una riga a metà programma porta la macchina in una posizione sicura e accendi il mandrino.

**OPT STOP (arresto opzionale, `M1`)**: decide se il comando `M1` nel programma ferma la macchina.
- Attivo: a ogni `M1` il programma va in pausa come con `M0` e riparte con START (o PAUSE). Disattivo: gli `M1` vengono ignorati.
- Serve per mettere nel programma punti di controllo da usare solo quando servono, per esempio misurare il pezzo dopo la sgrossatura o pulire i trucioli. In Fusion 360 si può inserire un `M1` tra un'operazione e l'altra.

**BLK DEL (cancellazione blocchi, `/`)**: decide se le righe che iniziano con `/` vengono eseguite.
- Attivo: le righe con `/` davanti vengono saltate. Disattivo: vengono eseguite normalmente.
- Serve per avere parti di programma escludibili senza modificare il file, per esempio una passata di finitura in più, un ciclo di pulizia o il refrigerante (`/M8`) da usare solo su alcuni materiali.

OPT STOP e BLK DEL restano premuti quando la funzione è attiva; il loro stato è quello reale di LinuxCNC, quindi coincide con QtDragon. Se nei programmi non ci sono `M1` o righe con `/`, non hanno effetto.

##### Abilitazione dei pulsanti

Lo stato dei pulsanti non è gestito dal Nextion ma da `esp32_mpg.py` e dall'ESP32:

1. `esp32_mpg.py` legge lo stato reale da LinuxCNC e invia `ST:RUNNING`, `ST:PAUSED`, `ST:STOPPED` o `ST:ESTOP`: all'apertura della page3, a ogni cambio di stato (controllo ogni 200 ms) e subito dopo ogni pulsante della page3. Vale anche per programmi avviati da QtDragon o dai pulsanti fisici.
2. L'ESP32 (`applyAutoState()` nel firmware) applica lo stato al display: abilita o disabilita i pulsanti con `tsw`, imposta il loro `.val` e aggiorna testo e colori di `t4` e `va0`.

| Stato | START (`bt0`) | STOP (`bt1`) | PAUSE (`bt2`) |
|-------|---------------|--------------|---------------|
| STOPPED | abilitato | disabilitato (premuto, rosso) | disabilitato |
| RUNNING | disabilitato (premuto) | abilitato | abilitato |
| PAUSED | abilitato | abilitato | abilitato (premuto) |
| E-STOP | disabilitato | disabilitato (premuto, rosso) | disabilitato |

Il Nextion fa da solo solo due cose: i `tsw bt1,0` / `tsw bt2,0` del Postinitialize di page3, sovrascritti dal primo `ST:`, e il cambio di stato dei pulsanti dual-state al tocco, che `esp32_mpg.py` corregge subito.

`esp32_mpg.py` inoltre verifica ogni comando ricevuto, indipendentemente da come appare il pulsante: se il display fosse fuori sincrono, un comando non valido viene ignorato.

| Comando | Accettato solo se |
|---------|-------------------|
| START | senza E-Stop e macchina abilitata; avvia se fermo con un programma caricato, riprende se in pausa |
| PAUSE | programma in esecuzione (pausa) o in pausa (ripresa) |
| STEP | senza E-Stop, macchina abilitata e programma caricato; programma fermo o in pausa |
| RUN LINE | programma fermo, macchina pronta e programma caricato; altrimenti messaggio in `t3` |
| OPT STOP / BLK DEL | sempre |
| tutti tranne STOP | non durante le attese di pausa completa o ripresa con riavvio mandrino |

STEP, RUN LINE, OPT STOP e BLK DEL non vengono disabilitati sul display: li protegge solo questo controllo.

Gli errori di LinuxCNC (per esempio limiti superati) restano solo su QtDragon: lo script non legge il canale errori per non sottrarli alla GUI.

Eventi da impostare nel progetto `.HMI`:

| Oggetto | Evento | Codice |
|---------|--------|--------|
| page0, page2, page3 | Postinitialize | rispettivamente `prints "#p0",0`, `prints "#p2",0`, `prints "#p3",0` (in page3 dopo le righe `tsw` esistenti) |
| page3 `bt0` START | Touch Release | `prints "#S",0` (la vecchia logica locale di `bt0`, `bt1` e `bt2` è commentata: lo stato lo gestisce LinuxCNC) |
| page3 `bt1` STOP | Touch Release | `prints "#E",0` |
| page3 `bt2` PAUSE | Touch Release | `prints "#P",0` |
| page3 `bt3` STEP | Touch Release | `prints "#N",0` |
| page3 `bt4` RUN LINE | Touch Release | `prints "#R",0` |
| page3 `bt5` OPT STOP | Touch Release | `prints "#O",0` |
| page3 `bt6` BLK DEL | Touch Release | `prints "#B",0` |
| page3 `h0` SPD | Touch Release | `prints "#V",0` e `prints h0.val,1` (slider da 0 a 100) |
| page3 `h1` FED | Touch Release | `prints "#W",0` e `prints h1.val,1` (slider da 0 a 100) |
| page3 `bt7` ABS (dual-state) | Touch Release | `prints "#q",0`: in page3 il firmware lo traduce in `CMD:VIEW_TOGGLE` invece di `CMD:G53` |

`tmode` (`page page0` in page3, `page page3` in page0), `tF` e il timer `tm0` restano come sono: il cambio di modo macchina lo fa `esp32_mpg.py` quando riceve il `PAGEID`. Il nome della pagina è definito in `NEXTION_AUTO_PAGE` nel firmware.

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
| `CMD:VIEW_TOGGLE` | `bt7` ABS della page3: alterna DRO assoluto / sistema attivo, senza cambiare il sistema |
| `CMD:ZERO:<X\|Y\|Z\|A\|ALL>` | Azzeramento assi |
| `CMD:MACRO_1` / `CMD:MACRO_2` / `CMD:MACRO_4` | Macro |
| `CMD:INIT_TOOL` | Avvio INIT TOOL (posizione di cambio e attesa del numero utensile) |
| `TOOL:SET:<testo>` / `TOOL:CANCEL` | Numero utensile dal keypad / keypad annullato |
| `PAGEID:<n>` | Pagina caricata sul Nextion (0, 2, 3): LinuxCNC reinvia i dati di quella pagina; con page3 passa in AUTO, con page0 da AUTO passa in MANUALE (solo a interprete fermo) |
| `CMD:PAUSE_TOGGLE` / `CMD:STEP` / `CMD:RUN_LINE` / `CMD:OPT_STOP` / `CMD:BLK_DEL` | Pulsanti della page3 (START e STOP usano `CMD:CYCLE_START` / `CMD:CYCLE_STOP`) |
| `SLD:<S\|F>:<0-100>` | Slider SPD / FED della page3 |
| `JOG:L:<scatti>` | Encoder in page3 a programma fermo: scelta della riga per RUN LINE |

#### Protocollo seriale PC → ESP32

| Messaggio | Significato |
|-----------|-------------|
| `POS:<asse>:<valore>` | DRO (3 decimali), inviato solo se cambia di più di 0.0005 |
| `MOD:<MAN\|AUTO\|MDI>` | Modo macchina |
| `OVR:F<n>%` | Feed override |
| `SEL:<asse>` | Forza l'asse selezionato sull'encoder (`F` / `0` durante i programmi AUTO) |
| `PAGE:MAIN` / `PAGE:TOOL` | Torna alla pagina principale (INIT TOOL annullato da E-Stop, uscita dal modo AUTO) / apre il keypad (mandrino in posizione di cambio, o numero non valido) |
| `PAGE:AUTO` | Apre la pagina AUTO (passaggio in modo AUTO o partenza di un programma) |
| `VIEW:<0\|1>:<G5x>` | DRO nel sistema attivo (`0`) o assoluto (`1`) e sistema attivo: `bt7`/`t5` in page3, `bt0`…`bt4` in page0 |
| `ST:<RUNNING\|PAUSED\|STOPPED\|ESTOP>` | Stato del programma per la page3 (`t4`, START/STOP/PAUSE) |
| `GL:<0\|1\|2>:<testo>` | Righe G-code precedente / corrente / successiva (`t0`..`t2`) |
| `MSG:<testo>` | Messaggio operatore o nome del file (`t3`) |
| `OPT:<0\|1>` / `BLK:<0\|1>` | Stato di OPT STOP (`bt5`) e BLK DEL (`bt6`) |
| `SLD:<S\|F>:<0-100>` | Posizione degli slider SPD (`h0`) e FED (`h1`) |

I messaggi della page3 vengono inviati solo quando la page3 è aperta, al massimo ogni 200 ms e solo se cambiano.

Il DRO viene aggiornato ogni 50 ms. Il valore mostrato è in coordinate pezzo (macchina − G5x − G92 − offset utensile), oppure in coordinate macchina dopo **G53**.

#### Protocollo Nextion → ESP32

Il display invia `#` seguito da un carattere di comando:

| Carattere | Comando | Carattere | Comando |
|-----------|---------|-----------|---------|
| `M0` / `M1` | Modo Wheel / Joy | `q` | G53 (page0) / ABS on-off (page3 `bt7`) |
| `X` `Y` `Z` `A` | Selezione asse encoder | `w` `e` `r` `t` | G54 / G55 / G56 / G57 |
| `F` | Encoder su feed override | `x` `y` `z` `a` | Zero X / Y / Z / A |
| `0` | Encoder disattivato | `*` | Zero ALL |
| `o` `i` `j` `k` `l` | Passo 0 / 0.01 / 0.1 / 1 / 10 | `h` `c` `f` | Macro 1 / 2 / 4 |
| `d` | INIT TOOL | `T<testo>;` / `K` | Numero utensile dal keypad / Indietro |
| `p<n>` | Pagina caricata (`p0`, `p2`, `p3`) | `S` `E` `P` | START / STOP / PAUSE (page3) |
| `N` `R` | STEP / RUN LINE (page3) | `O` `B` | OPT STOP / BLK DEL (page3) |
| `V<byte>` | Slider SPD, 1 byte 0..100 | `W<byte>` | Slider FED, 1 byte 0..100 |

Per aggiungere un nuovo comando: assegnare un carattere libero nel progetto `.HMI`, aggiungere la riga `else if (cmd == '…') Serial.println("CMD:…");` in `serialListenerTask()` del firmware e gestire `CMD:…` in `process_serial_data()` di `esp32_mpg.py`.

### Debug

- `DEBUG_MODE = True` in `esp32_mpg.py` stampa ogni comando ricevuto sul terminale da cui è stato avviato LinuxCNC (`[DEBUG] …`); errori e connessioni compaiono come `[ERROR/INFO] …`.
- Per controllare i pin HAL: `halshow`, oppure `halcmd show pin esp_mpg`.
- Per vedere i messaggi grezzi del pendant (con LinuxCNC spento): `screen /dev/ttyUSB0 115200`.
