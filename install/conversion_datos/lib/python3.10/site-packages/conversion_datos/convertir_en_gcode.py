#  Si no está instalado: 
#  https://www.python.org/downloads/
#  pip install pandas
#  Para lanzarlo: python convertir_en_gcode.py muelle_w3.csv muelle_w3.gcode --temp_extrusor 195 --temp_cama 60 --velocidad 80

import pandas as pd    
import argparse

def main(input_csv, output_gcode, temp_extrusor, temp_cama, velocidad):
    # Leer el CSV (sin cabecera)
    df = pd.read_csv(input_csv, header=None)
    df.columns = ['X', 'Y', 'Z', 'I', 'J', 'K', 'W']

    # Encabezado del G-code con temperaturas variables
    gcode_lines = [
		 "; Start G-code",
        f"M109 S{temp_extrusor} ; extruder temperature to {temp_extrusor}°C",
        f"M190 S{temp_cama} ; bed temperature to {temp_cama}°C",
        "M17 ; Start motors"
    ]

    # Bucle G1
    for _, row in df.iterrows():
        gcode_lines.append(
            f"G1 X{row['X']} Y{row['Y']} Z{row['Z']} I{row['I']} J{row['J']} K{row['K']} W{row['W']} F{velocidad}"
        )

    # End G-code
    gcode_lines.append("; End G-code")

    # Guardar archivo
    with open(output_gcode, 'w') as f:
        f.write('\n'.join(gcode_lines))
    print(f'G-code generado correctamente en: {output_gcode}')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Convierte un CSV a G-code para fabricación aditiva no planar.")
    parser.add_argument('input_csv', help='Ruta al archivo CSV de entrada')
    parser.add_argument('output_gcode', help='Ruta al archivo G-code de salida')
    parser.add_argument('--temp_extrusor', type=int, default=195, help='Temperatura del extrusor (M109), por defecto 195')
    parser.add_argument('--temp_cama', type=int, default=60, help='Temperatura de la cama (M190), por defecto 60')
    parser.add_argument('--velocidad', type=int, default=80, help='Valor F (velocidad) de deposición, por defecto 80')
    args = parser.parse_args()

    main(args.input_csv, args.output_gcode, args.temp_extrusor, args.temp_cama, args.velocidad)
