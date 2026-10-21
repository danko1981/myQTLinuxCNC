#!/usr/bin/env python
# -*- coding: utf-8 -*-
# SONDA SIMULATA per qtdragon_sim.ini: caricata SOLO da sim_machine.hal.
# La macchina reale (qtdragon.ini / qtdragonMac.hal) non usa questo file.
#
# Calcola la quota macchina Z del mandrino alla quale la punta dell'utensile
# tocca la superficie sotto di lei (pin contact-z):
#   - sopra il tool sensor ([TOOLSENSOR] X/Y, entro SENSOR_RADIUS): piano + SENSOR_HEIGHT
#   - altrove (touch plate sul pezzo): piano + STOCK_HEIGHT + [PROBE]PLATE_THICKNESS
# Il confronto con la Z reale e l'abilitazione durante i G38.x avvengono in
# realtime in sim_machine.hal, quindi il punto di tocco e' preciso.
#
# Lunghezza simulata dell'utensile: fissa per ogni numero utensile (misure
# ripetibili), tra TOOL_LENGTH_BASE e TOOL_LENGTH_BASE + TOOL_LENGTH_SPREAD.
# Offset atteso in tabella dopo la misura sul tool sensor = BED_Z + lunghezza.
import os
import sys
import math
import time
import hal
import linuxcnc

def eprint(msg):
    sys.stderr.write("SIM PROBE: " + msg + "\n")
    sys.stderr.flush()

ini = None
try:
    ini = linuxcnc.ini(os.environ['INI_FILE_NAME'])
except Exception as e:
    eprint("INI non leggibile ({}), uso i valori predefiniti".format(e))

def ini_float(section, key, default):
    try:
        value = ini.find(section, key) if ini else None
        return float(value) if value is not None else default
    except ValueError:
        return default

SENSOR_X = ini_float("TOOLSENSOR", "X", 1.0)
SENSOR_Y = ini_float("TOOLSENSOR", "Y", 0.0)
SENSOR_HEIGHT = ini_float("TOOLSENSOR", "SENSOR_HEIGHT", 43.5)
PLATE_THICKNESS = ini_float("PROBE", "PLATE_THICKNESS", 3.5)
BED_Z = ini_float("SIM_PROBE", "BED_Z", -130.0)
STOCK_HEIGHT = ini_float("SIM_PROBE", "STOCK_HEIGHT", 20.0)
SENSOR_RADIUS = ini_float("SIM_PROBE", "SENSOR_RADIUS", 10.0)
TOOL_LENGTH_BASE = ini_float("SIM_PROBE", "TOOL_LENGTH_BASE", 30.0)
TOOL_LENGTH_SPREAD = ini_float("SIM_PROBE", "TOOL_LENGTH_SPREAD", 5.0)

SENSOR_TOP_Z = BED_Z + SENSOR_HEIGHT
PLATE_TOP_Z = BED_Z + STOCK_HEIGHT + PLATE_THICKNESS

def tool_length(tool):
    # Sempre lo stesso valore per lo stesso utensile, diverso di qualche mm tra utensili
    if tool <= 0:
        return TOOL_LENGTH_BASE
    return round(TOOL_LENGTH_BASE + TOOL_LENGTH_SPREAD * ((tool * 0.6180339887) % 1.0), 3)

c = hal.component("sim_probe")
c.newpin("x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("y", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool-number", hal.HAL_S32, hal.HAL_IN)
c.newpin("tool-prep-number", hal.HAL_S32, hal.HAL_IN)
c.newpin("contact-z", hal.HAL_FLOAT, hal.HAL_OUT)
c.newpin("tool-length", hal.HAL_FLOAT, hal.HAL_OUT)
c.newpin("on-sensor", hal.HAL_BIT, hal.HAL_OUT)
c["contact-z"] = -1.0e6  # Nessun contatto finche' il ciclo non parte
c.ready()

eprint("piano Z {:.3f}, tool sensor Z {:.3f} (X{} Y{}), touch plate Z {:.3f}".format(
    BED_Z, SENSOR_TOP_Z, SENSOR_X, SENSOR_Y, PLATE_TOP_Z))

last_state = None
try:
    while True:
        # Durante l'M6 (python/remap.py) la misura avviene prima del CHANGE_TOOL:
        # l'utensile da misurare e' quello preparato con la parola T
        tool = c["tool-prep-number"] if c["tool-prep-number"] > 0 else c["tool-number"]
        length = tool_length(tool)
        on_sensor = math.hypot(c["x"] - SENSOR_X, c["y"] - SENSOR_Y) <= SENSOR_RADIUS
        surface = SENSOR_TOP_Z if on_sensor else PLATE_TOP_Z

        c["tool-length"] = length
        c["on-sensor"] = on_sensor
        c["contact-z"] = surface + length

        if (tool, on_sensor) != last_state:
            last_state = (tool, on_sensor)
            eprint("T{} lunghezza {:.3f} mm, sopra {}: contatto a Z macchina {:.3f}"
                   " (offset atteso dal tool sensor {:.3f})".format(
                       tool, length, "tool sensor" if on_sensor else "touch plate",
                       surface + length, BED_Z + length))
        time.sleep(0.01)
except KeyboardInterrupt:
    raise SystemExit
