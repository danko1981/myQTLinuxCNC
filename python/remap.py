# -*- coding: utf-8 -*-
import emccanon
import interpreter
import hal
import datetime
import os

# ==========================================
# CONFIGURAZIONE DEBUG E LOG
# ==========================================
DEBUG_MODE = True
LOG_FILE = "./m6_remap_debug.log" 

def log_debug(msg):
    """Funzione helper per scrivere i log su file e terminale."""
    if DEBUG_MODE:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_line = "[{}] [M6 REMAP] {}".format(timestamp, msg)
        print(log_line)
        try:
            with open(LOG_FILE, "a") as f:
                f.write(log_line + "\n")
        except Exception as e:
            pass 

# ==========================================
# LOGICA PRINCIPALE M6
# ==========================================
def change_tool(self, **words):
    log_debug("=== INIZIO PROCEDURA CAMBIO UTENSILE ===")
    
    try:
        # 1. Recupero parametri dal file .ini con validazione
        log_debug("Lettura parametri INI...")
        c_pos_x = float(self.ini.find("CHANGE_POSITION", "X") or 189.5)
        c_pos_y = float(self.ini.find("CHANGE_POSITION", "Y") or 10.0)
        s_pos_x = float(self.ini.find("TOOLSENSOR", "X") or 1.0)
        s_pos_y = float(self.ini.find("TOOLSENSOR", "Y") or 0.0)
        touch_z = float(self.ini.find("TOOLSENSOR", "TOUCH") or 29.7)
        max_probe = float(self.ini.find("TOOLSENSOR", "MAXPROBE") or 150.0)
        z_min_limit = float(self.ini.find("AXIS_Z", "MIN_LIMIT") or -150.0)
        
        log_debug("Parametri configurati -> Cambio: X{} Y{} | Sensore: X{} Y{}".format(c_pos_x, c_pos_y, s_pos_x, s_pos_y))
        log_debug("Parametri Probe -> Spessore: {}mm | Corsa Max: {}mm | Limite Z: {}mm".format(touch_z, max_probe, z_min_limit))

        # 2. Stato iniziale e sicurezza
        speed = self.params.get('_spindle_speed', 0)
        log_debug("Stato mandrino salvato: {} RPM".format(speed))
        
        log_debug("Fermata mandrino e sollevamento asse Z in sicurezza (G53 G0 Z0).")
        self.execute("M5 M9")
        self.execute("G21 G90 G53 G0 Z0")
        
        # 3. Spostamento in posizione di cambio
        log_debug("In movimento verso posizione di cambio...")
        self.execute("G53 G0 X{} Y{}".format(c_pos_x, c_pos_y))
        
        # Interazione Operatore
        self.set_errormsg("CAMBIO UTENSILE: Inserisci utensile e premi AVVIA")
        log_debug("Sospensione esecuzione. In attesa del comando AVVIA/RESUME da operatore.")
        yield interpreter.INTERP_EXECUTE_FINISH 
        log_debug("Operatore ha confermato la ripresa del programma.")
        
        # 4. Spostamento sopra il sensore
        log_debug("Spostamento asse XY verso la posizione del sensore.")
        self.execute("G53 G0 X{} Y{}".format(s_pos_x, s_pos_y))
        
        probe_success = False
        attempt = 1
        
        # 5. CICLO DI TASTATURA CON DEBUG
        while not probe_success:
            log_debug("--- Inizio tentativo tastatura n. {} ---".format(attempt))
            self.execute("G90")
            self.execute("G53 G0 Z0")
            
            # Ricalcolo dinamico sicurezza Z
            current_z_abs = self.params['_z'] 
            available_travel = abs(current_z_abs - z_min_limit) - 1.0 
            probe_dist = min(max_probe, available_travel)
            log_debug("Distanza max di ricerca impostata a {}mm (per evitare Extra-Corsa).".format(probe_dist))

            self.execute("G91")
            
            # --- Primo tocco (Ricerca) ---
            log_debug("Esecuzione primo tocco G38.2 (Velocita 200).")
            self.execute("G38.2 Z-{} F200".format(probe_dist))
            
            if self.params[5070] == 0:
                log_debug("ERRORE: Il primo tocco non ha rilevato il sensore.")
                self.execute("G90")
                self.execute("G53 G0 Z0")
                self.set_errormsg("PROBE FALLITO (Nessun tocco). Controlla e premi AVVIA per riprovare, o STOP.")
                yield interpreter.INTERP_EXECUTE_FINISH
                attempt += 1
                continue

            # --- Secondo tocco (Precisione) ---
            log_debug("Primo tocco avvenuto. Ritrazione di 2mm ed esecuzione tocco di precisione.")
            self.execute("G1 Z2 F1000")
            self.execute("G38.2 Z-4 F50")
            
            if self.params[5070] == 0:
                log_debug("ERRORE: Contatto perso durante il secondo tocco di precisione.")
                self.execute("G90")
                self.execute("G53 G0 Z0")
                self.set_errormsg("PROBE FALLITO (Errore precisione). Controlla e premi AVVIA per riprovare, o STOP.")
                yield interpreter.INTERP_EXECUTE_FINISH
                attempt += 1
                continue
            
            probe_success = True
            log_debug("Tastatura fisica completata con successo.")

        # 6. Calcolo e applicazione Offset
        last_probe_z = self.params[5063]
        new_offset = last_probe_z - touch_z
        log_debug("Dati Offset: Z Assoluta Letta = {:.4f}mm | Spessore noto = {}mm".format(last_probe_z, touch_z))
        log_debug("Applicazione nuovo Offset G43.1 Z = {:.4f}mm".format(new_offset))
        
        self.execute("G90")
        self.execute("G43.1 Z{}".format(new_offset))
        
        # Risalita finale
        log_debug("Risalita di sicurezza a Z0 G53.")
        self.execute("G53 G0 Z0")
        
        # 7. Ripristino Mandrino
        if speed > 0:
            log_debug("Comando di riaccensione mandrino a {} RPM. Attesa 2 secondi...".format(speed))
            self.set_errormsg("Riavvio mandrino a {} RPM...".format(speed))
            self.execute("S{} M3".format(speed))
            self.execute("G4 P2")

        log_debug("=== PROCEDURA CAMBIO UTENSILE TERMINATA CON SUCCESSO ===")
        
        yield interpreter.INTERP_OK
        return

    except Exception as e:
        error_str = "ECCEZIONE CRITICA PYTHON: {}".format(str(e))
        log_debug(error_str)
        self.set_errormsg(error_str)
        
        yield interpreter.INTERP_ERROR
        return