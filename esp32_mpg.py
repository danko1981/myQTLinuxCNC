#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import time
import serial
import glob
import subprocess
import linuxcnc 

# ==========================================
# IMPOSTAZIONI DI DEBUG
# ==========================================
# True  = Stampa TUTTI i passaggi, la raw-serial, gli stati e le transizioni
# False = Stampa SOLO errori, eccezioni gestite e messaggi di avvio critici
DEBUG_MODE = True 

def dprint(msg):
    if DEBUG_MODE:
        print("[DEBUG] " + str(msg))

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

# Valori di default
ini_sensor_x = 0.0
ini_sensor_y = 0.5
ini_max_probe = -10.0
ini_search_vel = 200.0
ini_probe_vel = 50.0
ini_touch_height = 30.0

JOY_G1_MAX_VELOCITY = 4200.0
JOY_G0_RAPID_VELOCITY = 2400.0
MOUSE_SPEED_MULTIPLIER = 30.0 

if ini_path:
    dprint("Trovato percorso file INI: " + str(ini_path))
    try:
        inidata = linuxcnc.ini(ini_path)
        
        ini_sensor_x = float(inidata.find("TOOLSENSOR", "X") or ini_sensor_x)
        ini_sensor_y = float(inidata.find("TOOLSENSOR", "Y") or ini_sensor_y)
        ini_max_probe = float(inidata.find("TOOLSENSOR", "MAXPROBE") or ini_max_probe)
        ini_search_vel = float(inidata.find("TOOLSENSOR", "SEARCH_VEL") or ini_search_vel)
        ini_probe_vel = float(inidata.find("TOOLSENSOR", "PROBE_VEL") or ini_probe_vel)
        ini_touch_height = float(inidata.find("TOOLSENSOR", "TOUCH_HEIGHT") or ini_touch_height)
        
        traj_def_vel = inidata.find("TRAJ", "DEFAULT_LINEAR_VELOCITY")
        if traj_def_vel: JOY_G1_MAX_VELOCITY = float(traj_def_vel) * 60.0
            
        traj_max_vel = inidata.find("TRAJ", "MAX_LINEAR_VELOCITY")
        if traj_max_vel: JOY_G0_RAPID_VELOCITY = float(traj_max_vel) * 60.0
            
        dprint("Parametri INI: Sensor({}, {}), MaxProbe({}), Search({}), Probe({}), Height({})".format(
            ini_sensor_x, ini_sensor_y, ini_max_probe, ini_search_vel, ini_probe_vel, ini_touch_height))
        dprint("Velocita G1_MAX={}, G0_RAPID={}".format(JOY_G1_MAX_VELOCITY, JOY_G0_RAPID_VELOCITY))
        eprint("ESP32 MPG: Parametri .INI caricati con successo.")
    except Exception as e:
        eprint("ESP32 MPG: Errore lettura INI, uso parametri di default. Dettaglio: " + str(e))
else:
    dprint("Nessun file INI trovato nell'ambiente, uso parametri hardcoded di default.")

# ==========================================
# SETUP HAL E VARIABILI GLOBALI
# ==========================================
dprint("Inizializzazione comandi LinuxCNC e HAL component 'esp_mpg'...")
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
dprint("Componente HAL registrato e pronto.")

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

def force_refresh_all():
    global last_sent_pos, last_sent_mode, last_sent_feed_override
    dprint("Forzatura aggiornamento completo display (Refresh All)")
    last_sent_pos = {'X': -999999.0, 'Y': -999999.0, 'Z': -999999.0, 'A': -999999.0}
    last_sent_mode = -1 
    last_sent_feed_override = -1

def is_machine_idle_and_ready():
    stat.poll()
    is_ready = (not stat.estop and stat.enabled and stat.interp_state == linuxcnc.INTERP_IDLE)
    if not is_ready:
        dprint("Controllo IDLE fallito: Macchina NON pronta (Estop/Disabled/Occupata)")
    return is_ready

# ==========================================
# AGGIORNAMENTO DRO E OVERRIDE
# ==========================================
def update_esp_dros():
    global last_send_time, last_sent_pos, ser, view_abs_mode, last_sent_mode
    global last_sent_feed_override, last_interp_state
    if ser is None: return
        
    current_time = time.time()
    if (current_time - last_send_time) > 0.05:
        try:
            stat.poll() 
            current_state = stat.interp_state
            current_mode = stat.task_mode
            
            # 1. AUTO-ATTIVAZIONE FEED OVERRIDE
            if current_state in [linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING] and last_interp_state == linuxcnc.INTERP_IDLE:
                dprint("Macchina avviata in AUTO: Abilito Override (SEL:F)")
                ser.write("SEL:F\n".encode('utf-8'))
                
            elif current_state == linuxcnc.INTERP_IDLE and last_interp_state in [linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING]:
                dprint("Macchina fermata/idle: Disabilito Override (SEL:0)")
                ser.write("SEL:0\n".encode('utf-8')) 
                
            last_interp_state = current_state

            if current_mode == linuxcnc.MODE_MANUAL and last_sent_mode != linuxcnc.MODE_MANUAL:
                dprint("Transizione a MANUALE: Disabilito Override (SEL:0)")
                ser.write("SEL:0\n".encode('utf-8'))

            # 2. AGGIORNAMENTO TESTO OVERRIDE
            current_feed = int(stat.feedrate * 100)
            if current_feed != last_sent_feed_override:
                msg = "OVR:F" + str(current_feed) + "%\n"
                ser.write(msg.encode('utf-8'))
                last_sent_feed_override = current_feed

            # 3. SINCRONIZZAZIONE MODALITÀ
            if current_mode != last_sent_mode:
                mod_str = "???"
                if current_mode == linuxcnc.MODE_MANUAL: mod_str = "MAN"
                elif current_mode == linuxcnc.MODE_AUTO: mod_str = "AUTO"
                elif current_mode == linuxcnc.MODE_MDI: mod_str = "MDI"
                
                msg = "MOD:" + mod_str + "\n"
                dprint("Aggiornamento Label Modalita Nextion: " + msg.strip())
                ser.write(msg.encode('utf-8'))
                last_sent_mode = current_mode

            # 4. AGGIORNAMENTO DRO
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
                    # dprint("Aggiorno DRO {}: {:.3f}".format(axis, val)) # Decommenta per log continui
                    ser.write(msg.encode('utf-8'))
                    last_sent_pos[axis] = val
            last_send_time = current_time
        except Exception as e:
            eprint("Eccezione in update_esp_dros (gestita): " + str(e))

def send_mdi(cmd):
    dprint("---- INIZIO MACRO MDI: " + str(cmd) + " ----")
    try:
        stat.poll()
        if not stat.estop and stat.enabled:
            if stat.task_mode != linuxcnc.MODE_MDI:
                dprint("MDI: Richiesta passaggio in MODE_MDI...")
                emc.mode(linuxcnc.MODE_MDI)
                emc.wait_complete()
            
            dprint("MDI: Inserimento comando -> " + str(cmd))
            emc.mdi(cmd)
            time.sleep(0.1) 
            
            stat.poll()
            dprint("MDI: In attesa di INTERP_IDLE...")
            while stat.interp_state != linuxcnc.INTERP_IDLE and not stat.estop and stat.enabled:
                time.sleep(0.05)
                stat.poll()
                update_esp_dros() 
            
            dprint("MDI: Movimento terminato, ritorno in MODE_MANUAL...")
            emc.mode(linuxcnc.MODE_MANUAL)
            emc.wait_complete()
            dprint("---- FINE MACRO MDI ----")
        else:
            dprint("MDI Ignorato: Macchina in E-Stop o non abilitata.")
    except Exception as e:
        eprint("Errore MDI Imprevisto: " + str(e))

def connect_serial():
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    for p in ports:
        try:
            s = serial.Serial(p, BAUD_RATE, timeout=0.01)
            eprint("ESP32 MPG: Connesso con successo su " + str(p))
            return s
        except Exception as e:
            dprint("Tentativo fallito su " + str(p) + ": " + str(e))
    return None

eprint("ESP32 MPG - Controller Avviato")
dprint("Avvio Loop Principale")

# ==========================================
# LOOP PRINCIPALE
# ==========================================
try:
    while True:
        current_time = time.time()

        if ser is None:
            if current_time - last_reconnect_time > 2.0:
                dprint("Scansione porte seriali in corso...")
                ser = connect_serial()
                if ser: force_refresh_all()
                last_reconnect_time = current_time
            time.sleep(0.1); continue

        if not os.path.exists(ser.port):
            eprint("ATTENZIONE: Cavo seriale rimosso fisicamente. Disconnessione forzata.")
            c['joy-vel-x'] = 0.0; c['joy-vel-y'] = 0.0; c['joy-vel-z'] = 0.0; c['joy-vel-a'] = 0.0
            try: ser.close() 
            except: pass
            ser = None; continue

        try:
            if ser.inWaiting() > 0:
                line = ser.readline().decode('utf-8', 'ignore').strip()
                if line != "":
                    # Rimuovi questa riga dprint se i movimenti JOG inquinano troppo il log
                    # dprint("Serial RX -> " + str(line))
                    pass
                
                if line.startswith("JOG:"):
                    stat.poll()
                    parts = line.split(":")
                    if len(parts) == 3:
                        axis = parts[1]
                        delta = int(parts[2])
                        
                        if axis == 'F':
                            current_override = stat.feedrate
                            new_override = current_override + (delta * 0.01) 
                            if new_override < 0.5: new_override = 0.5
                            if new_override > 2.0: new_override = 2.0
                            emc.feedrate(new_override)
                            dprint("Feed Override Aggiornato: {:.0f}%".format(new_override*100))
                            
                        elif stat.task_mode == linuxcnc.MODE_MANUAL and not stat.estop and stat.enabled:
                            if axis in counts:
                                counts[axis] += delta
                                if axis == 'X': c['count-x'] = counts['X']
                                if axis == 'Y': c['count-y'] = counts['Y']
                                if axis == 'Z': c['count-z'] = counts['Z']
                                if axis == 'A': c['count-a'] = counts['A']
                                
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
                
                elif line.startswith("MOUSE:"):
                    parts = line.split(":")
                    if len(parts) == 2 and parts[1] == "CLICK":
                        dprint("Esecuzione xdotool: Click Mouse")
                        subprocess.Popen(["xdotool", "click", "1"])
                    elif len(parts) == 4 and parts[1] == "MOV":
                        nx = float(parts[2]); ny = float(parts[3])
                        dx = int(nx * MOUSE_SPEED_MULTIPLIER); dy = int(ny * -MOUSE_SPEED_MULTIPLIER) 
                        if dx != 0 or dy != 0: subprocess.Popen(["xdotool", "mousemove_relative", "--", str(dx), str(dy)])

                elif line.startswith("SCALE:"):
                    parts = line.split(":")
                    if len(parts) == 2: 
                        c['scale'] = float(parts[1])
                        dprint("Scala Jog impostata a: " + str(c['scale']))
                elif line == "ESTOP:ON": 
                    c['estop'] = True
                    dprint("E-STOP: ON (Premuto)")
                elif line == "ESTOP:OFF": 
                    c['estop'] = False
                    dprint("E-STOP: OFF (Rilasciato)")
                
                # --- LOGICA PULSANTI E CICLO MACCHINA ---
                elif line.startswith("CMD:"):
                    cmd_part = line.split(":")[1]                    
                    dprint("Richiesta Azione CMD Ricevuta: " + str(cmd_part))
                    
                    # 1. GREEN BUTTON: START / RESUME INTELLIGENTE
                    if cmd_part == "CYCLE_START":
                        stat.poll()
                        if not stat.estop and stat.enabled and stat.file:
                            if stat.task_mode != linuxcnc.MODE_AUTO:
                                dprint("CYCLE_START: Passaggio in MODE_AUTO")
                                emc.mode(linuxcnc.MODE_AUTO)
                                emc.wait_complete()
                            
                            if stat.interp_state == linuxcnc.INTERP_IDLE:
                                dprint("CYCLE_START: Avvio File da zero (AUTO_RUN)")
                                emc.auto(linuxcnc.AUTO_RUN, 0)
                            elif stat.interp_state == linuxcnc.INTERP_PAUSED:
                                dprint("CYCLE_START: Programma in pausa. Valutazione stato Mandrino...")
                                is_spinning = False
                                try: is_spinning = stat.spindle[0]['enabled'] or (stat.spindle[0]['direction'] != 0)
                                except AttributeError: is_spinning = stat.spindle_enabled
                                
                                if not is_spinning:
                                    # Usa la velocità salvata in memoria dal tasto Pausa
                                    dprint("CYCLE_START: Accensione Mandrino a {} RPM. Attesa 3 secondi...".format(saved_spindle_speed))
                                    emc.spindle(linuxcnc.SPINDLE_FORWARD, saved_spindle_speed)
                                    time.sleep(3.0) 
                                    
                                dprint("CYCLE_START: Ripresa esecuzione (AUTO_RESUME)")
                                emc.auto(linuxcnc.AUTO_RESUME)
                                
                    # 2. YELLOW BUTTON: PAUSA COMPLETA
                    elif cmd_part == "CYCLE_PAUSE_FULL":
                        stat.poll()
                        if stat.interp_state in [linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING]:
                            dprint("CYCLE_PAUSE_FULL: Salvataggio RPM e arresto mandrino")
                            
                            # Fotografa gli RPM attuali prima di spegnere
                            try:
                                current_rpm = abs(stat.spindle[0]['speed'])
                                if current_rpm > 0:
                                    saved_spindle_speed = current_rpm
                                    dprint("Velocita salvata in memoria: {}".format(saved_spindle_speed))
                            except Exception as e:
                                dprint("Impossibile leggere RPM: {}".format(e))
                                
                            emc.auto(linuxcnc.AUTO_PAUSE)
                            time.sleep(1.0)
                            emc.spindle(linuxcnc.SPINDLE_OFF)

                    # 3. SHIFT + YELLOW: PAUSA SEMPLICE
                    elif cmd_part == "CYCLE_PAUSE":
                        stat.poll()
                        if stat.interp_state in [linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING]:
                            dprint("CYCLE_PAUSE: Pausa semplice")
                            emc.auto(linuxcnc.AUTO_PAUSE)
                            
                    # 4. RED BUTTON: ABORT
                    elif cmd_part == "CYCLE_STOP":
                        stat.poll()
                        dprint("CYCLE_STOP: Abort Programma ed arresto mandrino")
                        emc.abort()
                        emc.spindle(linuxcnc.SPINDLE_OFF) 
                        
                    # 5. SHIFT + RED: SPINDLE TOGGLE
                    elif cmd_part == "SPINDLE_TOGGLE":
                        if is_machine_idle_and_ready():
                            stat.poll()
                            is_spinning = False
                            try: is_spinning = stat.spindle[0]['enabled'] or (stat.spindle[0]['direction'] != 0)
                            except AttributeError: is_spinning = stat.spindle_enabled
                                
                            if is_spinning: 
                                dprint("SPINDLE_TOGGLE: Spegnimento")
                                send_mdi("M5")
                            else: 
                                dprint("SPINDLE_TOGGLE: Accensione a 12000 RPM")
                                send_mdi("M3 S12000")
                            
                    elif cmd_part == "MODE_TOGGLE":
                        if is_machine_idle_and_ready():
                            current_task = stat.task_mode
                            dprint("MODE_TOGGLE: Richiesto salto modalita")
                            if current_task == linuxcnc.MODE_MANUAL: emc.mode(linuxcnc.MODE_AUTO)
                            elif current_task == linuxcnc.MODE_AUTO: emc.mode(linuxcnc.MODE_MDI)
                            elif current_task == linuxcnc.MODE_MDI: emc.mode(linuxcnc.MODE_MANUAL)
                            emc.wait_complete()
                            
                    # G-CODE & WCS
                    elif cmd_part == "HOMEALL":
                        if is_machine_idle_and_ready():
                            dprint("HOMEALL: Homing generale di tutti gli assi")
                            if stat.task_mode != linuxcnc.MODE_MANUAL:
                                emc.mode(linuxcnc.MODE_MANUAL); emc.wait_complete()
                            emc.home(-1)
                    elif cmd_part == "G53":
                        dprint("WCS: Cambio vista a coordinate Assolute (G53)")
                        view_abs_mode = True; force_refresh_all() 
                    elif cmd_part in ["G54", "G55", "G56", "G57", "G58", "G59"]:
                        dprint("WCS: Cambio vista WCS (" + str(cmd_part) + ")")
                        view_abs_mode = False; send_mdi(cmd_part); force_refresh_all() 
                    elif cmd_part == "ZERO":
                         if len(line.split(":")) > 2:
                            target = line.split(":")[2]
                            dprint("ZERO: Azzeramento Asse " + str(target))
                            if target == "ALL": send_mdi("G10 L20 P0 X0 Y0 Z0")
                            elif target != "A": send_mdi("G10 L20 P0 " + target + "0")
                            force_refresh_all()
                            
                    # MACRO MDI
                    elif cmd_part == "GOTO_ZERO":
                        if is_machine_idle_and_ready():
                            send_mdi("G53 G0 Z0")
                            if view_abs_mode: send_mdi("G53 G1 X0 Y0 F2000")
                            else:
                                send_mdi("G1 X0 Y0 F2000")
                                send_mdi("G1 Z0 F1000")
                    
                    elif cmd_part == "GOTO_CHANGE":
                        if is_machine_idle_and_ready():
                            send_mdi("G53 G0 Z0")
                            send_mdi("G53 G1 X190 Y10 F2000")
                            send_mdi("G53 G0 Z-10")
                            
                    elif cmd_part == "GOTO_REAR":
                        if is_machine_idle_and_ready():
                            send_mdi("G53 G0 Z0")
                            send_mdi("G53 G1 X5 Y615 F2000")
                   
                    elif cmd_part == "TOOL_PROBE":
                        if is_machine_idle_and_ready():
                            send_mdi("G53 G0 Z0")
                            send_mdi("G53 G0 X" + str(ini_sensor_x) + " Y" + str(ini_sensor_y))
                            send_mdi("G38.2 Z" + str(ini_max_probe) + " F" + str(ini_search_vel))
                            send_mdi("G91 G1 Z2 F1000")
                            send_mdi("G90") 
                            send_mdi("G38.2 Z" + str(ini_max_probe) + " F" + str(ini_probe_vel))
                            send_mdi("G10 L10 P#5400 Z" + str(ini_touch_height))
                            send_mdi("G43")
                            send_mdi("G53 G0 Z0")

            update_esp_dros()

        except (serial.SerialException, OSError) as e:
            eprint("Eccezione di IO Seriale (Cavo disconnesso o interferenza EMI): " + str(e))
            try: ser.close() 
            except: pass
            ser = None; continue
        except Exception as e:
            eprint("Eccezione Imprevista nel loop: " + str(e))
            pass 
                
        time.sleep(0.01)

except KeyboardInterrupt:
    eprint("Script interrotto dall'utente (Ctrl+C). Uscita in corso.")
    raise SystemExit
