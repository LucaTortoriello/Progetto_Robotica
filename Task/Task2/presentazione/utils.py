#!/usr/bin/env python3
# Utility - Avvia logger e analisi

import subprocess
import sys
import time
from pathlib import Path


def find_latest_log():
    # Trova il file più recente
    log_dir = Path('/tmp/task2_logs')
    if not log_dir.exists():
        return None
    
    json_files = sorted(log_dir.glob('task2_data_*.json'), reverse=True)
    return json_files[0] if json_files else None


def start_logger():
    # Avvia data logger
    script = Path(__file__).parent / 'data_logger.py'
    print(f"✓ Avvio logger: {script}")
    subprocess.Popen(['python3', str(script)])


def analyze_latest():
    # Analizza il file più recente
    json_file = find_latest_log()
    
    if not json_file:
        print("✗ Nessun file di log trovato")
        return False
    
    print(f"✓ Analizzando: {json_file}")
    
    script = Path(__file__).parent / 'analysis.py'
    result = subprocess.run(['python3', str(script), str(json_file)], 
                          capture_output=False)
    
    return result.returncode == 0


def main():
    if len(sys.argv) < 2:
        print("Uso:")
        print("  python3 utils.py start      - Avvia il logger")
        print("  python3 utils.py analyze    - Analizza il file più recente")
        print("  python3 utils.py find       - Trova il file più recente")
        return
    
    command = sys.argv[1]
    
    if command == 'start':
        start_logger()
    
    elif command == 'analyze':
        success = analyze_latest()
        if success:
            print("\n✓ Analisi completata!")
            json_file = find_latest_log()
            output_dir = json_file.parent / 'analysis_output'
            print(f"Risultati in: {output_dir}")
    
    elif command == 'find':
        json_file = find_latest_log()
        if json_file:
            print(f"File trovato: {json_file}")
            print(f"\nEsegui l'analisi con:")
            print(f"  python3 analysis.py {json_file}")
        else:
            print("Nessun file trovato")
    
    else:
        print(f"Comando sconosciuto: {command}")


if __name__ == '__main__':
    main()
