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

def force_refresh_all():
    global last_sent_pos, last_sent_mode, last_sent_feed_override, manual_spindle_paused
    last_sent_pos = {'X': -999999.0, 'Y': -999999.0, 'Z': -999999.0, 'A': -999999.0}
    last_sent_mode = -1 
    last_sent_feed_override = -1
    manual_spindle_paused = False

def is_machine_idle_and_ready():
    stat.poll()
    return (not stat.estop and stat.enabled and stat.interp_state == linuxcnc.INTERP_IDLE)

# ==========================================
# ELABORAZIONE SERIALE E COMANDI (CUORE DEL SISTEMA)
# ==========================================
def process_serial_data():
    global counts, saved_spindle_speed, manual_spindle_paused, view_abs_mode, ser
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
                    
                    if axis == 'F':
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

            elif line.startswith("SCALE:"):
                parts = line.split(":")
                if len(parts) == 2: c['scale'] = float(parts[1])
            
            # --- LOGICA PULSANTI E CICLO MACCHINA ---
            elif line.startswith("CMD:"):
                cmd_part = line.split(":")[1]                    
                dprint("CMD Ricevuto: " + str(cmd_part))
                
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
                                    # Attesa 3 secondi NON BLOCCANTE (Continua a leggere E-Stop)
                                    for _ in range(30):
                                        time.sleep(0.1)
                                        stat.poll()
                                        if stat.estop: break # Esci se premi E-Stop nel frattempo
                                        process_serial_data()
                                manual_spindle_paused = False
                            else:
                                dprint("CYCLE_START: Pausa di Sistema (es. M6). Salto controllo mandrino.")
                                
                            dprint("CYCLE_START: Eseguo AUTO_RESUME")
                            emc.auto(linuxcnc.AUTO_RESUME)
                            
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
                        for _ in range(10): # 1 secondo di attesa non bloccante
                            time.sleep(0.1)
                            process_serial_data()
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

        except Exception as e:
            eprint("Eccezione durante lettura/esecuzione Seriale: " + str(e))
            break

# ==========================================
# AGGIORNAMENTO DRO 
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
            
            if current_state in [linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING] and last_interp_state == linuxcnc.INTERP_IDLE:
                ser.write("SEL:F\n".encode('utf-8'))
            elif current_state == linuxcnc.INTERP_IDLE and last_interp_state in [linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING]:
                ser.write("SEL:0\n".encode('utf-8')) 
            last_interp_state = current_state

            if current_mode == linuxcnc.MODE_MANUAL and last_sent_mode != linuxcnc.MODE_MANUAL:
                ser.write("SEL:0\n".encode('utf-8'))

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
        except Exception: pass

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
        
        time.sleep(0.01)

except KeyboardInterrupt:
    eprint("Uscita in corso.")
    raise SystemExit
