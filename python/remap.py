# -*- coding: utf-8 -*-
import emccanon
import interpreter
import hal
import datetime
import os
import linuxcnc 

# ==========================================
# CONFIGURAZIONE DEBUG E LOG
# ==========================================
DEBUG_MODE = True
LOG_FILE = "/home/andrea/linuxcnc/myQTLinuxCNC/m6_remap_debug.log" 

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
        # 1. Lettura nativa del file .ini
        ini_path = os.getenv('INI_FILE_NAME')
        inifile = linuxcnc.ini(ini_path)
        
        c_pos_x = float(inifile.find("CHANGE_POSITION", "X") or 189.5)
        c_pos_y = float(inifile.find("CHANGE_POSITION", "Y") or 10.0)
        s_pos_x = float(inifile.find("TOOLSENSOR", "X") or 1.0)
        s_pos_y = float(inifile.find("TOOLSENSOR", "Y") or 0.0)
        touch_z = float(inifile.find("TOOLSENSOR", "TOUCH") or 34.5)
        max_probe = float(inifile.find("TOOLSENSOR", "MAXPROBE") or 120.0)
        z_min_limit = float(inifile.find("AXIS_Z", "MIN_LIMIT") or -125.0)
        
        try:
            speed = self.params['_spindle_speed']
        except Exception:
            speed = 0
            
        log_debug("Stato mandrino salvato: {} RPM".format(speed))
        
        # 2. Spostamento in posizione di cambio
        log_debug("Fermata mandrino e movimento verso posizione di cambio...")
        self.execute("M5 M9")
        self.execute("G21 G90 G53 G0 Z0")
        self.execute("G53 G0 X{} Y{}".format(c_pos_x, c_pos_y))
        
        yield interpreter.INTERP_EXECUTE_FINISH
        
        # 3. Interazione Operatore
        self.set_errormsg("CAMBIO UTENSILE: Inserisci utensile e premi AVVIA")
        log_debug("M0: Macchina in pausa. In attesa del comando AVVIA/RESUME da operatore.")
        self.execute("M0")
        
        yield interpreter.INTERP_EXECUTE_FINISH 
        log_debug("Operatore ha confermato la ripresa del programma.")
        
        # 4. Spostamento sopra il sensore
        log_debug("Spostamento asse XY verso la posizione del sensore.")
        self.execute("G53 G0 X{} Y{}".format(s_pos_x, s_pos_y))
        # Annulla l'offset dell'utensile precedente per evitare misurazioni falsate
        self.execute("G49")
        yield interpreter.INTERP_EXECUTE_FINISH
        
        probe_success = False
        attempt = 1
        
        # 5. CICLO DI TASTATURA
        while not probe_success:
            log_debug("--- Inizio tentativo tastatura n. {} ---".format(attempt))
            self.execute("G90 G53 G0 Z0")
            
            yield interpreter.INTERP_EXECUTE_FINISH
            
            # Ricalcolo dinamico sicurezza Z
            current_z_abs = self.params['_z'] 
            available_travel = abs(current_z_abs - z_min_limit) - 1.0 
            probe_dist = min(max_probe, available_travel)

            self.execute("G91")
            
            # --- Primo tocco (Ricerca) con G38.3 per non abortire ---
            log_debug("Esecuzione primo tocco G38.3 (nessun aborto di sistema se fallisce)...")
            self.execute("G38.3 Z-{} F200".format(probe_dist))
            
            yield interpreter.INTERP_EXECUTE_FINISH
            
            if self.params[5070] == 0:
                log_debug("ERRORE: Il primo tocco non ha rilevato il sensore.")
                # Uniamo G90 e G53 assieme per sicurezza assoluta
                self.execute("G90 G53 G0 Z0")
                yield interpreter.INTERP_EXECUTE_FINISH
                
                self.set_errormsg("PROBE FALLITO. Controlla il sensore e premi AVVIA per riprovare.")
                self.execute("M0")
                yield interpreter.INTERP_EXECUTE_FINISH
                attempt += 1
                continue

            # --- Secondo tocco (Precisione) ---
            log_debug("Primo tocco avvenuto. Ritrazione di 2mm ed esecuzione tocco di precisione.")
            self.execute("G1 Z2 F1000")
            self.execute("G38.3 Z-4 F50")
            
            yield interpreter.INTERP_EXECUTE_FINISH
            
            if self.params[5070] == 0:
                log_debug("ERRORE: Contatto perso durante il secondo tocco di precisione.")
                self.execute("G90 G53 G0 Z0")
                yield interpreter.INTERP_EXECUTE_FINISH
                
                self.set_errormsg("PROBE FALLITO (Errore precisione). Controlla e premi AVVIA per riprovare.")
                self.execute("M0")
                yield interpreter.INTERP_EXECUTE_FINISH
                attempt += 1
                continue
            
            probe_success = True
            log_debug("Tastatura fisica completata con successo.")

      # 6. Calcolo e applicazione Offset Permanente in Tabella
        
        # Lettura istantanea e sincrona dalla memoria interna dell'interprete
        probed_z_wcs = self.params[5063]    # Quota di tocco
        current_cs = int(self.params[5220]) # Sistema di coordinate
        
        # Recupera l'offset dello Zero Pezzo (G54, G55, ecc)
        g5x_z = self.params[5203 + (20 * current_cs)]
        g92_z = self.params[5213]           # Eventuale offset G92 Z
        
        # Calcolo INFAILIBILE della vera quota Assoluta della Macchina.
        # N.B. Non si somma l'offset utensile perché la sonda scende sotto G49 (offset annullato).
        last_probe_z_abs = probed_z_wcs + g5x_z + g92_z
        
        # L'offset finale = Quota Assoluta Tocco - Spessore Sensore
        new_offset = last_probe_z_abs - touch_z
        
        tool_num = self.selected_tool
        pocket = self.selected_pocket
        
        log_debug("Z Tocco WCS={:.4f}, G5x={:.4f}, G92={:.4f}".format(probed_z_wcs, g5x_z, g92_z))
        log_debug("Quota tocco Assoluta Macchina calcolata = {:.4f}mm".format(last_probe_z_abs))
        log_debug("Registrazione Utensile {} in tabella con Z Offset = {:.4f}mm".format(tool_num, new_offset))
        
        emccanon.CHANGE_TOOL(pocket)
        
        self.execute("G90")
        
        # Scrittura in tabella e attivazione dell'offset
        self.execute("G10 L1 P{} Z{:.4f}".format(tool_num, new_offset))
        self.execute("G43 H{}".format(tool_num))
        
        # Risalita in Coordinate Macchina Assolute per sicurezza
        self.execute("G53 G0 Z0")
        
        yield interpreter.INTERP_EXECUTE_FINISH
        
        # 7. Ripristino Mandrino
        if speed > 0:
            log_debug("Comando di riaccensione mandrino a {} RPM. Attesa 2 secondi...".format(speed))
            self.set_errormsg("Riavvio mandrino a {} RPM...".format(speed))
            self.execute("S{} M3".format(speed))
            self.execute("G4 P2")
            yield interpreter.INTERP_EXECUTE_FINISH

        log_debug("=== PROCEDURA CAMBIO UTENSILE TERMINATA CON SUCCESSO ===")
        
        yield interpreter.INTERP_OK
        return

    except Exception as e:
        error_str = "ECCEZIONE CRITICA PYTHON: {}".format(str(e))
        log_debug(error_str)
        self.set_errormsg(error_str)
        
        # Ritorno forzato a G90 per evitare "Cannot use g53 incremental" in caso di panico
        try:
            self.execute("G90")
        except:
            pass
            
        yield interpreter.INTERP_ERROR
        return
