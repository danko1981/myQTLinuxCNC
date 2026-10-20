#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import time
import serial
import glob
import subprocess
import re
import linuxcnc

# ==========================================
# IMPOSTAZIONI DI DEBUG
# ==========================================
DEBUG_MODE = True 

def dprint(msg):
    if DEBUG_MODE: print("[DEBUG] " + str(msg))

def eprint(msg):
    print("[ERROR/INFO] " + str(msg))

try:
    import hal
except ImportError:
    eprint("ERRORE CRITICO: Modulo 'hal' non trovato. Lo script deve girare sotto LinuxCNC.")
    sys.exit(1)

BAUD_RATE = 115200

# ==========================================
# LETTURA DINAMICA DAL FILE .INI
# ==========================================
ini_path = os.environ.get('INI_FILE_NAME')

ini_sensor_x = 0.0
ini_sensor_y = 0.5
ini_max_probe = -10.0
ini_search_vel = 200.0
ini_probe_vel = 50.0
ini_touch_height = 30.0
ini_change_x = 189.5
ini_change_y = 10.0
ini_max_feed_ovr = 2.0
ini_min_spindle_ovr = 0.5
ini_max_spindle_ovr = 1.5

JOY_G1_MAX_VELOCITY = 4200.0
JOY_G0_RAPID_VELOCITY = 2400.0
MOUSE_SPEED_MULTIPLIER = 30.0 

if ini_path:
    try:
        inidata = linuxcnc.ini(ini_path)
        ini_sensor_x = float(inidata.find("TOOLSENSOR", "X") or ini_sensor_x)
        ini_sensor_y = float(inidata.find("TOOLSENSOR", "Y") or ini_sensor_y)
        ini_max_probe = float(inidata.find("TOOLSENSOR", "MAXPROBE") or ini_max_probe)
        ini_search_vel = float(inidata.find("TOOLSENSOR", "SEARCH_VEL") or ini_search_vel)
        ini_probe_vel = float(inidata.find("TOOLSENSOR", "PROBE_VEL") or ini_probe_vel)
        ini_touch_height = float(inidata.find("TOOLSENSOR", "TOUCH_HEIGHT") or ini_touch_height)
        ini_change_x = float(inidata.find("CHANGE_POSITION", "X") or ini_change_x)
        ini_change_y = float(inidata.find("CHANGE_POSITION", "Y") or ini_change_y)
        ini_max_feed_ovr = float(inidata.find("DISPLAY", "MAX_FEED_OVERRIDE") or ini_max_feed_ovr)
        ini_min_spindle_ovr = float(inidata.find("DISPLAY", "MIN_SPINDLE_0_OVERRIDE")
                                    or inidata.find("DISPLAY", "MIN_SPINDLE_OVERRIDE") or ini_min_spindle_ovr)
        ini_max_spindle_ovr = float(inidata.find("DISPLAY", "MAX_SPINDLE_0_OVERRIDE")
                                    or inidata.find("DISPLAY", "MAX_SPINDLE_OVERRIDE") or ini_max_spindle_ovr)
        
        traj_def_vel = inidata.find("TRAJ", "DEFAULT_LINEAR_VELOCITY")
        if traj_def_vel: JOY_G1_MAX_VELOCITY = float(traj_def_vel) * 60.0
            
        traj_max_vel = inidata.find("TRAJ", "MAX_LINEAR_VELOCITY")
        if traj_max_vel: JOY_G0_RAPID_VELOCITY = float(traj_max_vel) * 60.0
        eprint("ESP32 MPG: Parametri .INI caricati con successo.")
    except Exception as e:
        eprint("ESP32 MPG: Errore lettura INI, uso parametri di default. Dettaglio: " + str(e))

# ==========================================
# SETUP HAL E VARIABILI GLOBALI
# ==========================================
emc = linuxcnc.command()
stat = linuxcnc.stat()

c = hal.component("esp_mpg")
c.newpin("count-x", hal.HAL_S32, hal.HAL_OUT)
c.newpin("count-y", hal.HAL_S32, hal.HAL_OUT)
c.newpin("count-z", hal.HAL_S32, hal.HAL_OUT)
c.newpin("count-a", hal.HAL_S32, hal.HAL_OUT)
c.newpin("joy-vel-x", hal.HAL_FLOAT, hal.HAL_OUT)
c.newpin("joy-vel-y", hal.HAL_FLOAT, hal.HAL_OUT)
c.newpin("joy-vel-z", hal.HAL_FLOAT, hal.HAL_OUT)
c.newpin("joy-vel-a", hal.HAL_FLOAT, hal.HAL_OUT)
c.newpin("scale", hal.HAL_FLOAT, hal.HAL_OUT)
c.newpin("estop", hal.HAL_BIT, hal.HAL_OUT)
c.ready()

c['scale'] = 0.1
c['estop'] = False
c['joy-vel-x'] = 0.0; c['joy-vel-y'] = 0.0; c['joy-vel-z'] = 0.0; c['joy-vel-a'] = 0.0

counts = {'X': 0, 'Y': 0, 'Z': 0, 'A': 0}
last_send_time = 0
last_reconnect_time = 0
ser = None

view_abs_mode = False 
last_sent_pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'A': 0.0}
last_sent_mode = -1 
last_sent_feed_override = -1
last_interp_state = -1
saved_spindle_speed = 12000.0
manual_spindle_paused = False
cycle_sequence_busy = False     # True durante l'attesa di pausa completa / ripresa con riavvio mandrino

# INIT TOOL da Nextion: la page2 (keypad) e' aperta e si attende il numero utensile
waiting_tool = False
pending_tool = None

# PAGINA AUTO (page3 Nextion)
NEXTION_TEXT_MAX = 55           # t0..t3 hanno txt_maxl 60 nel progetto .HMI
AUTO_PAGE_PERIOD = 0.2          # Aggiornamento righe/stato della page3 (secondi)
esp_page = 0                    # Pagina mostrata dal Nextion (comunicata con PAGEID:n)
selected_line = 1               # Riga per RUN LINE, scelta con l'encoder a programma fermo
last_motion_line = 0            # Ultima riga eseguita: dopo uno STOP RUN LINE riparte da qui
auto_sent = {}                  # Ultimi valori inviati alla page3 (si invia solo cio' che cambia)
last_auto_update = 0
py_message = ""                 # Messaggio operatore generato da questo script
py_message_until = 0            # 0 = finche' non viene cancellato
gcode_cache = {'path': None, 'mtime': None, 'lines': []}

def force_refresh_all():
    global last_sent_pos, last_sent_mode, last_sent_feed_override, manual_spindle_paused
    last_sent_pos = {'X': -999999.0, 'Y': -999999.0, 'Z': -999999.0, 'A': -999999.0}
    last_sent_mode = -1 
    last_sent_feed_override = -1
    manual_spindle_paused = False

def is_machine_idle_and_ready():
    stat.poll()
    return (not stat.estop and stat.enabled and stat.interp_state == linuxcnc.INTERP_IDLE)

def is_all_homed():
    stat.poll()
    return all(stat.homed[i] for i in range(stat.joints))

def is_at_change_position():
    # Verifica che i movimenti verso la posizione di cambio siano davvero finiti
    # (non interrotti da E-Stop o abort): quote macchina XY e Z0
    stat.poll()
    if stat.estop or not stat.enabled: return False
    pos = stat.actual_position
    return (abs(pos[0] - ini_change_x) < 0.05 and abs(pos[1] - ini_change_y) < 0.05
            and abs(pos[2]) < 0.05)

def send_to_esp(msg):
    if ser is None: return
    try: ser.write((msg + "\n").encode('utf-8'))
    except Exception as e: eprint("Errore invio a ESP32: " + str(e))

def cancel_tool_init(reason):
    global waiting_tool, pending_tool
    if waiting_tool or pending_tool is not None:
        dprint("INIT TOOL annullato: " + reason)
    waiting_tool = False
    pending_tool = None

def parse_tool_number(text):
    # Il keypad accetta anche lettere: valgono "5", "T5", "Q5" e "M61 Q5"
    m = re.match(r'^(?:M61Q|T|Q)?(\d+)$', text.replace(" ", "").upper())
    if not m: return None
    n = int(m.group(1))
    return n if n > 0 else None

def run_tool_init(tool):
    # Eseguito dal loop principale, a macchina ferma sulla posizione di cambio
    dprint("INIT TOOL: dichiarazione utensile T{} e misura su tool sensor".format(tool))
    send_mdi("M61 Q{}".format(tool))
    stat.poll()
    if stat.tool_in_spindle != tool:
        eprint("INIT TOOL: M61 Q{} non accettato (utensile non in tabella?). Misura annullata.".format(tool))
        set_message("M61 Q{} rifiutato: utensile non in tabella?".format(tool), 8)
        return
    send_mdi("O<tool_sensor> call")
    clear_message()
    force_refresh_all()

# ==========================================
# PAGINA AUTO (page3 Nextion)
# ==========================================
def set_message(text, timeout=None):
    # Messaggio operatore per la casella t3 (timeout in secondi, None = fisso)
    global py_message, py_message_until
    py_message = text
    py_message_until = (time.time() + timeout) if timeout else 0

def clear_message():
    set_message("")

def nextion_text(text):
    # Testo sicuro per una stringa Nextion: solo ASCII, niente doppi apici, lunghezza limitata
    text = "".join(ch for ch in text if 32 <= ord(ch) < 127)
    text = text.replace('"', "'").replace("\\", "/").strip()
    return text[:NEXTION_TEXT_MAX]

def get_gcode_lines():
    # Righe del programma caricato, rilette solo se cambia il file
    global selected_line
    path = stat.file
    if not path:
        gcode_cache['path'] = None; gcode_cache['lines'] = []
        return []
    try: mtime = os.path.getmtime(path)
    except OSError: mtime = None
    if path != gcode_cache['path'] or mtime != gcode_cache['mtime']:
        try:
            with open(path, 'rb') as f:
                gcode_cache['lines'] = f.read().decode('utf-8', 'ignore').splitlines()
        except Exception as e:
            eprint("Lettura G-code fallita: " + str(e))
            gcode_cache['lines'] = []
        if path != gcode_cache['path']: selected_line = 1
        gcode_cache['path'] = path; gcode_cache['mtime'] = mtime
    return gcode_cache['lines']

def format_gcode_line(lines, n):
    if n < 1 or n > len(lines): return ""
    return nextion_text("{}: {}".format(n, lines[n - 1]))

def is_program_running():
    return stat.interp_state in (linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING)

def is_near(x, y):
    pos = stat.actual_position
    return abs(pos[0] - x) < 0.05 and abs(pos[1] - y) < 0.05

def m6_pause_message():
    # Durante l'M6 (python/remap.py) il programma e' in pausa (M0) con un utensile
    # preparato: in posizione di cambio si attende il montaggio, sul sensore la
    # tastatura e' fallita
    if stat.interp_state != linuxcnc.INTERP_PAUSED or stat.pocket_prepped < 0:
        return None
    try: tool = stat.tool_table[stat.pocket_prepped].id
    except Exception: tool = "?"
    if is_near(ini_change_x, ini_change_y):
        return "CHANGE TOOL: monta T{} e premi START".format(tool)
    if is_near(ini_sensor_x, ini_sensor_y):
        return "PROBE FALLITO: controlla il sensore e premi START"
    return None

def auto_message():
    # Priorita': messaggio M6, messaggio di questo script, nome del file
    msg = m6_pause_message()
    if msg: return msg
    if py_message and (py_message_until == 0 or time.time() < py_message_until):
        return py_message
    return os.path.basename(stat.file) if stat.file else "Nessun programma caricato"

def to_slider(value, vmin, vmax):
    if vmax <= vmin: return 0
    return max(0, min(100, int(round((value - vmin) / (vmax - vmin) * 100.0))))

def from_slider(value, vmin, vmax):
    return vmin + max(0, min(100, value)) / 100.0 * (vmax - vmin)

def spindle_override():
    try: return stat.spindle[0]['override']
    except Exception: return stat.spindlerate

def send_auto_value(key, msg):
    if auto_sent.get(key) != msg:
        send_to_esp(msg)
        auto_sent[key] = msg

def update_auto_page(force=False):
    # Stato, righe G-code, messaggi, OPT/BLK e slider della page3
    global last_auto_update
    if esp_page != 3 or ser is None: return
    now = time.time()
    if not force and (now - last_auto_update) < AUTO_PAGE_PERIOD: return
    last_auto_update = now
    stat.poll()

    if stat.estop: state = "ESTOP"
    elif stat.interp_state == linuxcnc.INTERP_PAUSED: state = "PAUSED"
    elif is_program_running(): state = "RUNNING"
    else: state = "STOPPED"
    send_auto_value('st', "ST:" + state)

    lines = get_gcode_lines()
    if state in ("RUNNING", "PAUSED") and stat.motion_line > 0:
        current = stat.motion_line
    else:
        current = selected_line
    send_auto_value('gl0', "GL:0:" + format_gcode_line(lines, current - 1))
    send_auto_value('gl1', "GL:1:" + format_gcode_line(lines, current))
    send_auto_value('gl2', "GL:2:" + format_gcode_line(lines, current + 1))

    send_auto_value('msg', "MSG:" + nextion_text(auto_message()))
    send_auto_value('opt', "OPT:" + ("1" if stat.optional_stop else "0"))
    send_auto_value('blk', "BLK:" + ("1" if stat.block_delete else "0"))
    send_auto_value('sldf', "SLD:F:{}".format(to_slider(stat.feedrate, 0.0, ini_max_feed_ovr)))
    send_auto_value('slds', "SLD:S:{}".format(to_slider(spindle_override(), ini_min_spindle_ovr, ini_max_spindle_ovr)))

def refresh_auto_page():
    # Forza il reinvio di tutto (dopo un cambio pagina o un pulsante che cambia da solo il proprio stato)
    auto_sent.clear()
    update_auto_page(force=True)

def ensure_auto_mode():
    if stat.task_mode != linuxcnc.MODE_AUTO:
        emc.mode(linuxcnc.MODE_AUTO)
        emc.wait_complete()

# ==========================================
# ELABORAZIONE SERIALE E COMANDI (CUORE DEL SISTEMA)
# ==========================================
def process_serial_data():
    global counts, saved_spindle_speed, manual_spindle_paused, view_abs_mode, ser
    global waiting_tool, pending_tool, esp_page, selected_line, cycle_sequence_busy
    if ser is None: return

    while ser.inWaiting() > 0:
        try:
            line = ser.readline().decode('utf-8', 'ignore').strip()
            if not line: continue
            
            # --- SICUREZZE ASSOLUTE (PRIORITÀ 1) ---
            if line == "ESTOP:ON":
                c['estop'] = True
                dprint("E-STOP: FORZATO TRAMITE API")
                emc.state(linuxcnc.STATE_ESTOP) # Interruzione Istantanea Hard-Coded
                if waiting_tool or pending_tool is not None:
                    cancel_tool_init("E-STOP")
                    send_to_esp("PAGE:MAIN")
                continue
            elif line == "ESTOP:OFF": 
                c['estop'] = False
                dprint("E-STOP: OFF (Rilasciato su hardware)")
                continue
            
            # --- ENCODER ROTATIVO E FEED ---
            if line.startswith("JOG:"):
                stat.poll()
                parts = line.split(":")
                if len(parts) == 3:
                    axis = parts[1]
                    delta = int(parts[2])
                    
                    if axis == 'L':
                        # Page3: a programma fermo l'encoder sceglie la riga per RUN LINE
                        if stat.interp_state == linuxcnc.INTERP_IDLE:
                            n_lines = len(get_gcode_lines())
                            if n_lines > 0:
                                selected_line = max(1, min(n_lines, selected_line + delta))
                                update_auto_page(force=True)

                    elif axis == 'F':
                        current_override = stat.feedrate
                        new_override = current_override + (delta * 0.01) 
                        if new_override < 0.5: new_override = 0.5
                        if new_override > 2.0: new_override = 2.0
                        emc.feedrate(new_override)
                        dprint("Feed Override: {:.0f}%".format(new_override*100))
                        
                    elif stat.task_mode == linuxcnc.MODE_MANUAL and not stat.estop and stat.enabled:
                        if axis in counts:
                            counts[axis] += delta
                            if axis == 'X': c['count-x'] = counts['X']
                            if axis == 'Y': c['count-y'] = counts['Y']
                            if axis == 'Z': c['count-z'] = counts['Z']
                            if axis == 'A': c['count-a'] = counts['A']
                            
            # --- JOYSTICK ASSI ---
            elif line.startswith("JOY:"):
                parts = line.split(":")
                if len(parts) == 4:
                    axis_str = parts[1]
                    val = float(parts[2]) 
                    mode = parts[3]       
                    speed = 0.0
                    if val != 0.0:
                        if mode == "G0": speed = JOY_G0_RAPID_VELOCITY * (1.0 if val > 0 else -1.0)
                        else: speed = JOY_G1_MAX_VELOCITY * val

                    if axis_str == 'X': c['joy-vel-x'] = speed
                    elif axis_str == 'Y': c['joy-vel-y'] = speed
                    elif axis_str == 'Z': c['joy-vel-z'] = speed
                    elif axis_str == 'A': c['joy-vel-a'] = speed
            
            # --- MOUSE MODE ---
            elif line.startswith("MOUSE:"):
                parts = line.split(":")
                if len(parts) == 2 and parts[1] == "CLICK":
                    subprocess.Popen(["xdotool", "click", "1"])
                elif len(parts) == 4 and parts[1] == "MOV":
                    nx = float(parts[2]); ny = float(parts[3])
                    dx = int(nx * MOUSE_SPEED_MULTIPLIER); dy = int(ny * -MOUSE_SPEED_MULTIPLIER) 
                    if dx != 0 or dy != 0: subprocess.Popen(["xdotool", "mousemove_relative", "--", str(dx), str(dy)])

            # --- INIT TOOL: numero utensile dal keypad Nextion (page2) ---
            elif line.startswith("TOOL:SET:"):
                text = line[len("TOOL:SET:"):]
                tool = parse_tool_number(text)
                if not waiting_tool:
                    dprint("TOOL:SET '{}' ignorato: nessun INIT TOOL in corso".format(text))
                elif tool is None:
                    eprint("INIT TOOL: numero utensile non valido '{}'. Riprova.".format(text))
                    set_message("Numero utensile non valido: riprova")
                    send_to_esp("PAGE:TOOL")
                else:
                    # Eseguito dal loop principale, a macchina ferma
                    waiting_tool = False
                    pending_tool = tool
                    dprint("INIT TOOL: ricevuto utensile T{}".format(tool))
                    set_message("Misura utensile T{} in corso".format(tool), 30)

            elif line == "TOOL:CANCEL":
                cancel_tool_init("annullato da Nextion")
                clear_message()
                force_refresh_all()

            # --- PAGINA MOSTRATA DAL NEXTION (evento di caricamento pagina) ---
            elif line.startswith("PAGEID:"):
                try: esp_page = int(line.split(":")[1])
                except ValueError: continue
                dprint("Nextion: pagina {} attiva".format(esp_page))
                force_refresh_all()
                if esp_page == 3:
                    stat.poll()
                    # Encoder: override feed durante il programma, scelta riga a programma fermo
                    if is_program_running() or stat.interp_state == linuxcnc.INTERP_PAUSED:
                        send_to_esp("SEL:F")
                    refresh_auto_page()

            # --- SLIDER PAGE3: SPD (h0) e FED (h1), valore 0..100 ---
            elif line.startswith("SLD:"):
                parts = line.split(":")
                if len(parts) == 3:
                    try: val = int(parts[2])
                    except ValueError: continue
                    if parts[1] == 'F':
                        emc.feedrate(from_slider(val, 0.0, ini_max_feed_ovr))
                        auto_sent['sldf'] = "SLD:F:{}".format(val)
                    elif parts[1] == 'S':
                        emc.spindleoverride(from_slider(val, ini_min_spindle_ovr, ini_max_spindle_ovr))
                        auto_sent['slds'] = "SLD:S:{}".format(val)

            elif line.startswith("SCALE:"):
                parts = line.split(":")
                if len(parts) == 2: c['scale'] = float(parts[1])
            
            # --- LOGICA PULSANTI E CICLO MACCHINA ---
            elif line.startswith("CMD:"):
                cmd_part = line.split(":")[1]
                dprint("CMD Ricevuto: " + str(cmd_part))

                # Durante l'attesa di una pausa completa o di una ripresa con riavvio
                # mandrino si accetta solo STOP (l'E-Stop e' gestito sopra)
                if cycle_sequence_busy and cmd_part != "CYCLE_STOP":
                    dprint("CMD {} ignorato: pausa/ripresa in corso".format(cmd_part))
                    continue

                # PAUSE della page3: mette in pausa (come il pulsante giallo) o riprende (come START)
                if cmd_part == "PAUSE_TOGGLE":
                    stat.poll()
                    cmd_part = "CYCLE_START" if stat.interp_state == linuxcnc.INTERP_PAUSED else "CYCLE_PAUSE_FULL"

                # 1. GREEN BUTTON: START / RESUME 
                if cmd_part == "CYCLE_START":
                    stat.poll()
                    if not stat.estop and stat.enabled:
                        
                        # Partenza da zero (Richiede stat.file)
                        if stat.interp_state == linuxcnc.INTERP_IDLE and stat.file:
                            dprint("CYCLE_START: Avvio File (AUTO_RUN)")
                            if stat.task_mode != linuxcnc.MODE_AUTO:
                                emc.mode(linuxcnc.MODE_AUTO)
                                emc.wait_complete()
                            manual_spindle_paused = False
                            emc.auto(linuxcnc.AUTO_RUN, 0)
                            
                        # Ripresa da pausa (Qualunque sia l'origine: WCS, MDI, M6)
                        elif stat.interp_state == linuxcnc.INTERP_PAUSED:
                            if manual_spindle_paused:
                                dprint("CYCLE_START: Ripresa da pausa manuale. Accensione Mandrino a {} RPM...".format(saved_spindle_speed))
                                is_spinning = False
                                try: is_spinning = stat.spindle[0]['enabled'] or (stat.spindle[0]['direction'] != 0)
                                except AttributeError: is_spinning = stat.spindle_enabled
                                
                                if not is_spinning:
                                    emc.spindle(linuxcnc.SPINDLE_FORWARD, saved_spindle_speed)
                                    # Attesa 3 secondi NON BLOCCANTE (Continua a leggere E-Stop e STOP;
                                    # gli altri comandi, es. un nuovo PAUSE, sono ignorati)
                                    cycle_sequence_busy = True
                                    try:
                                        for _ in range(30):
                                            time.sleep(0.1)
                                            stat.poll()
                                            if stat.estop: break # Esci se premi E-Stop nel frattempo
                                            process_serial_data()
                                    finally:
                                        cycle_sequence_busy = False
                                manual_spindle_paused = False
                            else:
                                dprint("CYCLE_START: Pausa di Sistema (es. M6). Salto controllo mandrino.")

                            # Durante l'attesa possono essere arrivati STOP o E-Stop: si riprende solo se ancora in pausa
                            stat.poll()
                            if not stat.estop and stat.interp_state == linuxcnc.INTERP_PAUSED:
                                dprint("CYCLE_START: Eseguo AUTO_RESUME")
                                emc.auto(linuxcnc.AUTO_RESUME)
                            else:
                                dprint("CYCLE_START: ripresa annullata (STOP o E-Stop durante l'attesa)")
                            
                # 2. YELLOW BUTTON: PAUSA COMPLETA
                elif cmd_part == "CYCLE_PAUSE_FULL":
                    stat.poll()
                    if stat.interp_state in [linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING]:
                        try:
                            current_rpm = abs(stat.spindle[0]['speed'])
                            if current_rpm > 0: saved_spindle_speed = current_rpm
                        except: pass
                        dprint("CYCLE_PAUSE_FULL: Salvataggio RPM ({}) e arresto.".format(saved_spindle_speed))
                            
                        emc.auto(linuxcnc.AUTO_PAUSE)
                        cycle_sequence_busy = True  # Un resume durante l'attesa verrebbe annullato dallo spegnimento
                        try:
                            for _ in range(10): # 1 secondo di attesa non bloccante
                                time.sleep(0.1)
                                process_serial_data()
                        finally:
                            cycle_sequence_busy = False
                        # Mandrino spento solo se il programma e' davvero fermo in pausa
                        # (con uno STOP durante l'attesa l'abort lo ha gia' spento)
                        stat.poll()
                        if stat.interp_state == linuxcnc.INTERP_PAUSED:
                            emc.spindle(linuxcnc.SPINDLE_OFF)
                            manual_spindle_paused = True

                # 3. SHIFT + YELLOW: PAUSA SEMPLICE
                elif cmd_part == "CYCLE_PAUSE":
                    stat.poll()
                    if stat.interp_state in [linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING]:
                        dprint("CYCLE_PAUSE: Pausa semplice")
                        emc.auto(linuxcnc.AUTO_PAUSE)
                        
                # 4. RED BUTTON: ABORT
                elif cmd_part == "CYCLE_STOP":
                    stat.poll()
                    dprint("CYCLE_STOP: Abort Programma")
                    emc.abort()
                    emc.spindle(linuxcnc.SPINDLE_OFF) 
                    manual_spindle_paused = False 
                    
                # 5. SHIFT + RED: SPINDLE TOGGLE
                elif cmd_part == "SPINDLE_TOGGLE":
                    if is_machine_idle_and_ready():
                        stat.poll()
                        is_spinning = False
                        try: is_spinning = stat.spindle[0]['enabled'] or (stat.spindle[0]['direction'] != 0)
                        except AttributeError: is_spinning = stat.spindle_enabled
                        if is_spinning: send_mdi("M5")
                        else: send_mdi("M3 S12000")
                        
                # 6. MDI MACRO
                elif cmd_part == "MODE_TOGGLE":
                    if is_machine_idle_and_ready():
                        current_task = stat.task_mode
                        if current_task == linuxcnc.MODE_MANUAL: emc.mode(linuxcnc.MODE_AUTO)
                        elif current_task == linuxcnc.MODE_AUTO: emc.mode(linuxcnc.MODE_MDI)
                        elif current_task == linuxcnc.MODE_MDI: emc.mode(linuxcnc.MODE_MANUAL)
                        emc.wait_complete()
                        
                elif cmd_part == "HOMEALL":
                    if is_machine_idle_and_ready():
                        if stat.task_mode != linuxcnc.MODE_MANUAL:
                            emc.mode(linuxcnc.MODE_MANUAL); emc.wait_complete()
                        emc.home(-1)
                elif cmd_part == "G53":
                    view_abs_mode = True; force_refresh_all() 
                elif cmd_part in ["G54", "G55", "G56", "G57", "G58", "G59"]:
                    view_abs_mode = False; send_mdi(cmd_part); force_refresh_all() 
                elif cmd_part == "ZERO":
                     if len(line.split(":")) > 2:
                        target = line.split(":")[2]
                        if target == "ALL": send_mdi("G10 L20 P0 X0 Y0 Z0")
                        elif target != "A": send_mdi("G10 L20 P0 " + target + "0")
                        force_refresh_all()
                        
                elif cmd_part == "MACRO_1":
                    if is_machine_idle_and_ready():
                        send_mdi("G90 G53 G0 Z0")
                        send_mdi("G90 G53 G1 X0 Y0 F2000")

                elif cmd_part == "MACRO_2":
                    # Solo XY sullo zero pezzo, Z resta a G53 Z0: scendere a G54 Z0
                    # porterebbe la punta sul pezzo, senza spazio per il touch plate
                    if is_machine_idle_and_ready():
                        send_mdi("G90 G53 G0 Z0")
                        send_mdi("G90 G1 X0 Y0 F2000")
                        
                elif cmd_part == "INIT_TOOL":
                    # Si va in posizione di cambio; il keypad (page2) si apre solo
                    # quando il mandrino e' arrivato
                    if is_machine_idle_and_ready() and is_all_homed():
                        dprint("INIT TOOL: movimento in posizione di cambio")
                        cancel_tool_init("nuovo INIT TOOL")
                        send_mdi("G90 G53 G0 Z0")
                        send_mdi("G90 G53 G0 X{} Y{}".format(ini_change_x, ini_change_y))
                        if is_at_change_position():
                            send_mdi("(MSG, CHANGE TOOL: monta l'utensile e inserisci il numero sul pendant)")
                            set_message("CHANGE TOOL: monta l'utensile e inserisci il numero")
                            waiting_tool = True
                            send_to_esp("PAGE:TOOL")
                            dprint("INIT TOOL: in posizione di cambio, keypad aperto")
                        else:
                            eprint("INIT TOOL: posizione di cambio non raggiunta, procedura annullata.")
                    else:
                        eprint("INIT TOOL rifiutato: macchina non pronta o non in homing.")
                        set_message("INIT TOOL: macchina non pronta o non in homing", 5)
               
                elif cmd_part == "MACRO_4":
                    if is_machine_idle_and_ready():
                            dprint("Lancio Macro di Sistema: touch_plate")
                            # Chiama il file touch_plate.ngc
                            send_mdi("O<touch_plate> call")

                # --- PAGE3 (AUTO): STEP, RUN LINE, OPT STOP, BLK DEL ---
                elif cmd_part == "STEP":
                    # Da fermo avvia il programma una riga alla volta, in pausa esegue la riga successiva
                    stat.poll()
                    if not stat.estop and stat.enabled and stat.file:
                        if stat.interp_state == linuxcnc.INTERP_IDLE:
                            ensure_auto_mode()
                            emc.auto(linuxcnc.AUTO_STEP)
                        elif stat.interp_state == linuxcnc.INTERP_PAUSED:
                            emc.auto(linuxcnc.AUTO_STEP)

                elif cmd_part == "RUN_LINE":
                    stat.poll()
                    if is_machine_idle_and_ready() and stat.file:
                        dprint("RUN LINE: avvio dalla riga {}".format(selected_line))
                        ensure_auto_mode()
                        manual_spindle_paused = False
                        emc.auto(linuxcnc.AUTO_RUN, selected_line)
                    else:
                        set_message("RUN LINE: macchina non pronta o nessun programma", 5)

                elif cmd_part == "OPT_STOP":
                    stat.poll()
                    emc.set_optional_stop(not stat.optional_stop)

                elif cmd_part == "BLK_DEL":
                    stat.poll()
                    emc.set_block_delete(not stat.block_delete)

                # I pulsanti dual-state della page3 cambiano stato da soli al tocco:
                # si ripristina subito lo stato reale di LinuxCNC
                if esp_page == 3:
                    refresh_auto_page()

        except Exception as e:
            eprint("Eccezione durante lettura/esecuzione Seriale: " + str(e))
            break

# ==========================================
# AGGIORNAMENTO DRO
# ==========================================
def update_esp_dros():
    global last_send_time, last_sent_pos, ser, view_abs_mode, last_sent_mode
    global last_sent_feed_override, last_interp_state, selected_line, last_motion_line
    if ser is None: return
        
    current_time = time.time()
    if (current_time - last_send_time) > 0.05:
        try:
            stat.poll() 
            current_state = stat.interp_state
            current_mode = stat.task_mode
            
            # -------------------------------------------------------------
            # FIX RESPONSIVITÀ VOLANTINO E QTDRAGON
            # QTDragon usa l'MDI per task in background. Per non resettare
            # l'asse selezionato, limitiamo l'invio di SEL:F / SEL:0 al Nextion
            # SOLO alle reali esecuzioni di file G-code (MODE_AUTO).
            # -------------------------------------------------------------
            if current_state in [linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING] and last_interp_state == linuxcnc.INTERP_IDLE:
                if current_mode == linuxcnc.MODE_AUTO:
                    ser.write("SEL:F\n".encode('utf-8'))
                    dprint("STATO: Esecuzione G-Code. Blocco Nextion (SEL:F).")
                    # Apre la pagina AUTO (non interrompe il keypad dell'INIT TOOL)
                    if esp_page not in (2, 3):
                        send_to_esp("PAGE:AUTO")

            elif current_state == linuxcnc.INTERP_IDLE and last_interp_state in [linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING, linuxcnc.INTERP_PAUSED]:
                if current_mode == linuxcnc.MODE_AUTO:
                    ser.write("SEL:0\n".encode('utf-8'))
                    dprint("STATO: Fine G-Code. Sblocco Nextion (SEL:0).")
                    # Dopo uno STOP, RUN LINE propone l'ultima riga eseguita
                    if last_motion_line > 0:
                        selected_line = last_motion_line

            if current_mode == linuxcnc.MODE_AUTO and current_state != linuxcnc.INTERP_IDLE and stat.motion_line > 0:
                last_motion_line = stat.motion_line
            last_interp_state = current_state

            # Sblocco di sicurezza se usciamo in MANUALE (ignorato se proveniamo da MDI)
            if current_mode == linuxcnc.MODE_MANUAL and last_sent_mode != linuxcnc.MODE_MANUAL:
                if last_sent_mode == linuxcnc.MODE_AUTO:
                    ser.write("SEL:0\n".encode('utf-8'))
                    dprint("MODALITA: Ritorno in Manuale da Auto. Sblocco Nextion (SEL:0).")

            # -------------------------------------------------------------
            # AGGIORNAMENTO FEED OVERRIDE E MODALITA' MACCHINA
            # -------------------------------------------------------------
            current_feed = int(stat.feedrate * 100)
            if current_feed != last_sent_feed_override:
                msg = "OVR:F" + str(current_feed) + "%\n"
                ser.write(msg.encode('utf-8'))
                last_sent_feed_override = current_feed

            if current_mode != last_sent_mode:
                mod_str = "???"
                if current_mode == linuxcnc.MODE_MANUAL: mod_str = "MAN"
                elif current_mode == linuxcnc.MODE_AUTO: mod_str = "AUTO"
                elif current_mode == linuxcnc.MODE_MDI: mod_str = "MDI"
                
                msg = "MOD:" + mod_str + "\n"
                ser.write(msg.encode('utf-8'))
                last_sent_mode = current_mode

            # -------------------------------------------------------------
            # CALCOLO E INVIO DELLE POSIZIONI ASSI
            # -------------------------------------------------------------
            machine_pos = stat.actual_position 
            g5x_off = stat.g5x_offset
            g92_off = stat.g92_offset
            tool_off = stat.tool_offset
            display_pos = [0.0, 0.0, 0.0, 0.0]
            num_axes = len(machine_pos)
            
            for i in range(min(num_axes, 4)):
                if view_abs_mode: display_pos[i] = machine_pos[i]
                else:
                    offset_total = g5x_off[i] + g92_off[i] + tool_off[i]
                    display_pos[i] = machine_pos[i] - offset_total
                    
            final_dict = {'X': display_pos[0], 'Y': display_pos[1], 'Z': display_pos[2], 'A': display_pos[3] if num_axes > 3 else 0.0}
            
            for axis, val in final_dict.items():
                if abs(val - last_sent_pos[axis]) > 0.0005:
                    msg = "POS:" + axis + ":" + "{:.3f}".format(val) + "\n"
                    ser.write(msg.encode('utf-8'))
                    last_sent_pos[axis] = val
            last_send_time = current_time

            # Pagina AUTO: stato, righe G-code, messaggi, OPT/BLK, slider
            update_auto_page()
            
        # MODIFICA SICUREZZA: Esposizione in chiaro degli errori di loop
        except Exception as e: 
            eprint("ERRORE FATALE in update_esp_dros: " + str(e))

# ==========================================
# ESECUZIONE MDI ASINCRONA
# ==========================================
def send_mdi(cmd):
    dprint("---- INIZIO MDI: " + str(cmd) + " ----")
    try:
        stat.poll()
        if not stat.estop and stat.enabled:
            if stat.task_mode != linuxcnc.MODE_MDI:
                emc.mode(linuxcnc.MODE_MDI)
                emc.wait_complete()
            
            emc.mdi(cmd)
            time.sleep(0.1) 
            
            stat.poll()
            while stat.interp_state != linuxcnc.INTERP_IDLE and not stat.estop and stat.enabled:
                time.sleep(0.05)
                stat.poll()
                update_esp_dros() 
                process_serial_data() # <-- FIX: CONTINUA A LEGGERE E-STOP MENTRE ASPETTA!
            
            emc.mode(linuxcnc.MODE_MANUAL)
            emc.wait_complete()
            dprint("---- FINE MDI ----")
    except Exception as e:
        eprint("Errore MDI Imprevisto: " + str(e))

def connect_serial():
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    for p in ports:
        try:
            s = serial.Serial(p, BAUD_RATE, timeout=0.01)
            eprint("ESP32 MPG: Connesso su " + str(p))
            return s
        except: pass
    return None

eprint("ESP32 MPG - Controller Avviato")

# ==========================================
# LOOP PRINCIPALE
# ==========================================
try:
    while True:
        current_time = time.time()

        if ser is None:
            if current_time - last_reconnect_time > 2.0:
                ser = connect_serial()
                if ser: force_refresh_all()
                last_reconnect_time = current_time
            time.sleep(0.1); continue

        if not os.path.exists(ser.port):
            eprint("ATTENZIONE: Cavo seriale disconnesso.")
            c['joy-vel-x'] = 0.0; c['joy-vel-y'] = 0.0; c['joy-vel-z'] = 0.0; c['joy-vel-a'] = 0.0
            try: ser.close() 
            except: pass
            ser = None; continue

        # IL CUORE DEL PROGRAMMA
        process_serial_data()
        update_esp_dros()

        # INIT TOOL: numero ricevuto, si procede appena la macchina e' ferma
        if pending_tool is not None and is_machine_idle_and_ready():
            tool = pending_tool
            pending_tool = None
            run_tool_init(tool)

        time.sleep(0.01)

except KeyboardInterrupt:
    eprint("Uscita in corso.")
    raise SystemExit
