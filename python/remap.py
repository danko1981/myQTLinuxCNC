# -*- coding: utf-8 -*-
import emccanon
import interpreter
import hal
import datetime
import os

# ==========================================
# CONFIGURAZIONE DEBUG
# ==========================================
DEBUG_MODE = True
# Il file di log verrà salvato nella stessa cartella dei tuoi G-Code o config
LOG_FILE = "/home/andrea/linuxcnc/QTDragon/m6_remap_debug.log" 

def log_debug(msg):
    """Funzione helper per scrivere i log su file e terminale."""
    if DEBUG_MODE:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S,%f")[:-3]
        log_line = f"[{timestamp}] [M6 REMAP] {msg}"
        print(log_line) # Utile se avvii LinuxCNC da terminale
        try:
            with open(LOG_FILE, "a") as f:
                f.write(log_line + "\n")
        except Exception as e:
            pass # Evitiamo che un errore nel file di log blocchi la macchina

# ==========================================
# LOGICA PRINCIPALE M6
# ==========================================
def manual_change_with_probe(self, **words):
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
        
        log_debug(f"Parametri configurati -> Cambio: X{c_pos_x} Y{c_pos_y} | Sensore: X{s_pos_x} Y{s_pos_y}")
        log_debug(f"Parametri Probe -> Spessore: {touch_z}mm | Corsa Max: {max_probe}mm | Limite Z: {z_min_limit}mm")

        # 2. Stato iniziale e sicurezza
        speed = self.params.get('_spindle_speed', 0)
        log_debug(f"Stato mandrino salvato: {speed} RPM")
        
        log_debug("Fermata mandrino e sollevamento asse Z in sicurezza (G53 G0 Z0).")
        self.execute("M5 M9")
        self.execute("G21 G90 G53 G0 Z0")
        
        # 3. Spostamento in posizione di cambio
        log_debug(f"In movimento verso posizione di cambio...")
        self.execute(f"G53 G0 X{c_pos_x} Y{c_pos_y}")
        
        # Interazione Operatore
        self.set_errormsg("CAMBIO UTENSILE: Inserisci utensile e premi AVVIA")
        log_debug("Sospensione esecuzione. In attesa del comando AVVIA/RESUME da operatore.")
        yield interpreter.INTERP_EXECUTE_FINISH 
        log_debug("Operatore ha confermato la ripresa del programma.")
        
        # 4. Spostamento sopra il sensore
        log_debug("Spostamento asse XY verso la posizione del sensore.")
        self.execute(f"G53 G0 X{s_pos_x} Y{s_pos_y}")
        
        probe_success = False
        attempt = 1
        
        # 5. CICLO DI TASTATURA CON DEBUG
        while not probe_success:
            log_debug(f"--- Inizio tentativo tastatura n. {attempt} ---")
            self.execute("G90")
            self.execute("G53 G0 Z0")
            
            # Ricalcolo dinamico sicurezza Z
            current_z_abs = self.params['_z'] 
            available_travel = abs(current_z_abs - z_min_limit) - 1.0 
            probe_dist = min(max_probe, available_travel)
            log_debug(f"Distanza max di ricerca impostata a {probe_dist}mm (per evitare Extra-Corsa).")

            self.execute("G91")
            
            # --- Primo tocco (Ricerca) ---
            log_debug(f"Esecuzione primo tocco G38.2 (Velocità 200).")
            self.execute(f"G38.2 Z-{probe_dist} F200")
            
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
        log_debug(f"Dati Offset: Z Assoluta Letta = {last_probe_z:.4f}mm | Spessore noto = {touch_z}mm")
        log_debug(f"Applicazione nuovo Offset G43.1 Z = {new_offset:.4f}mm")
        
        self.execute("G90")
        self.execute(f"G43.1 Z{new_offset}")
        
        # Risalita finale
        log_debug("Risalita di sicurezza a Z0 G53.")
        self.execute("G53 G0 Z0")
        
        # 7. Ripristino Mandrino
        if speed > 0:
            log_debug(f"Comando di riaccensione mandrino a {speed} RPM. Attesa 2 secondi...")
            self.set_errormsg(f"Riavvio mandrino a {speed} RPM...")
            self.execute(f"S{speed} M3")
            self.execute("G4 P2")

        log_debug("=== PROCEDURA CAMBIO UTENSILE TERMINATA CON SUCCESSO ===")
        return interpreter.INTERP_OK

    except Exception as e:
        # Se c'è un errore fatale Python (es. sintassi G-Code sbagliata, variabile inesistente)
        error_str = f"ECCEZIONE CRITICA PYTHON: {str(e)}"
        log_debug(error_str)
        self.set_errormsg(error_str)
        return interpreter.INTERP_ERROR
