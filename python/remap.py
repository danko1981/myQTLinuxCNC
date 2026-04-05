import emccanon
import interpreter
import hal

def manual_change_with_probe(self, **words):
    try:
        # 1. Recupero parametri dal file .ini
        c_pos_x = float(self.ini.find("CHANGE_POSITION", "X") or 189.5)
        c_pos_y = float(self.ini.find("CHANGE_POSITION", "Y") or 10.0)
        s_pos_x = float(self.ini.find("TOOLSENSOR", "X") or 1.0)
        s_pos_y = float(self.ini.find("TOOLSENSOR", "Y") or 0.0)
        touch_z = float(self.ini.find("TOOLSENSOR", "TOUCH") or 29.7)
        max_probe = float(self.ini.find("TOOLSENSOR", "MAXPROBE") or 150.0)
        z_min_limit = float(self.ini.find("AXIS_Z", "MIN_LIMIT") or -150.0)
        
        # 2. Stato iniziale e sicurezza
        speed = self.params['_spindle_speed']
        self.execute("M5 M9")  # Ferma mandrino e refrigerante
        self.execute("G21 G90 G53 G0 Z0") # Z in sicurezza
        
        # 3. Spostamento in posizione di cambio
        self.execute(f"G53 G0 X{c_pos_x} Y{c_pos_y}")
        
        # Messaggio interattivo su QtDragon
        self.set_errormsg("CAMBIO UTENSILE: Inserisci utensile e premi AVVIA")
        yield interpreter.INTERP_EXECUTE_FINISH 
        
        # 4. Spostamento su sensore e calcolo limite Z
        self.execute(f"G53 G0 X{s_pos_x} Y{s_pos_y}")
        
        # Calcolo protezione "Joint limit exceed"
        # Spazio rimanente prima del limite -150mm
        current_z_abs = self.params['_z'] 
        available_travel = abs(current_z_abs - z_min_limit) - 1.0 
        probe_dist = min(max_probe, available_travel)

        # 5. Esecuzione tastatura (Probe)
        self.execute("G91") # Incrementale per probe
        self.execute(f"G38.2 Z-{probe_dist} F200")
        
        # Verifica se il probe ha toccato
        if self.params[5070] == 0:
            self.execute("G90")
            self.set_errormsg("ERRORE: Sensore non raggiunto!")
            return interpreter.INTERP_ERROR

        # 6. Secondo tocco di precisione
        self.execute("G1 Z2 F1000")
        self.execute("G38.2 Z-4 F50")
        
        # 7. Applicazione Offset G43.1
        last_probe_z = self.params[5063]
        new_offset = last_probe_z - touch_z
        self.execute("G90") # Torna in assoluto
        self.execute(f"G43.1 Z{new_offset}")
        
        # Risalita finale
        self.execute("G53 G0 Z0")
        
        # 8. Ripristino Mandrino se era acceso
        if speed > 0:
            self.set_errormsg(f"Riavvio mandrino a {speed} RPM...")
            self.execute(f"S{speed} M3")
            self.execute("G4 P2") # Attesa 2 secondi

        return interpreter.INTERP_OK

    except Exception as e:
        self.set_errormsg(f"ERRORE PYTHON: {str(e)}")
        return interpreter.INTERP_ERROR
