# DESCRIPCIÓN
#
# Este módulo permite interpretar archivos G-code y extraer información relevante para su uso en ROS 2.  
# Se encarga de analizar las líneas del G-code, extrayendo las siguientes características:

# - Posiciones (`X`, `Y`, `Z`): Coordenadas cartesianas.
# - Orientación (`I`, `J`, `K`, `W`): Representación de la orientación en formato cuaternión.
# - Velocidad de impresión (`F`): Ajuste de la velocidad del cabezal de impresión.
# - Temperatura del extrusor (`M109`): Define la temperatura del extrusor antes de iniciar la impresión.
# - Temperatura de la cama (`M190`): Establece la temperatura de la cama de impresión.
# - Estado de los motores (`M17`, `M18`, `M84`): Determina si los motores están activados o desactivados.

# La función principal `parseo_gcode(file_path)` recibe la ruta de un archivo G-code y devuelve las 
# posiciones extraídas, velocidades de impresión y parámetros térmicos.  


def parseo_gcode(file_path):

    poses = []  # Lista para almacenar las posicones y orientaciones
    poses_aux = {}
    Vel_impresion_aux = {}  # Variable para acumular posiciones/orientaciones en la misma entrada
    Vel_impresion = [ ] # Lista para almacenar las velocidades de impresión
    Temp_cama = 0  # Variable para almacenar la temperatura de la cama
    Temp_extrusor = 0   # Variable para almacenar la temperatura del extrusor
    motores_on = 0  # Variable para almacenar el estado de los motores

    current_line = 0  # Contador de líneas
    with open(file_path, 'r') as file:
        for line_number, line in enumerate(file):
           
            if line.startswith(';') or not line:
                continue

            if line.startswith('M109'):  # Configuración de temperatura del extrusor con espera
                parts = line.strip().split()
                for part in parts[1:]:  # Omitimos 'M109'
                    if len(part) > 1:
                        key, value = part[0], part[1:]
                        try:
                            if key == 'S':  # Verificamos si el parámetro es de temperatura
                                Temp_extrusor = float(value)
                                print(f"Temperatura del extrusor establecida a: {Temp_extrusor}°C")
                        except ValueError as e:  # Si no se puede convertir a float, mostramos un mensaje de advertencia
                            print(f"Advertencia: No se pudo convertir '{value}' en la línea: {line.strip()}")
                            continue


            elif line.startswith('M190'):  # Configuración de temperatura de la cama con espera
                parts = line.strip().split()
                for part in parts[1:]:  # Omitimos 'M190'
                    if len(part) > 1:
                        key, value = part[0], part[1:]
                        try:
                            if key == 'S':  # Verificamos si el parámetro es de temperatura
                                Temp_cama = float(value)
                                print(f"Temperatura de la cama establecida a: {Temp_cama}°C")
                        except ValueError as e:  # Si no se puede convertir a float, mostramos un mensaje de advertencia
                            print(f"Advertencia: No se pudo convertir '{value}' en la línea: {line.strip()}")
                            continue               
                            
            
            # Actualizar el estado de motores
            elif 'M17' in line:  # Encender motores
                motores_on = 1
                print(f"Estado motores: {motores_on}")
            elif 'M18' in line or 'M84' in line:  # Apagar motores
                motores_on = 0
                print(f"Estado motores: {motores_on}")

            elif line.startswith('G1'):  # Movimiento con extrusion
                poses_aux = [0] * 7
                parts = line.strip().split()
                for part in parts[1:]:  # Omitimos 'G1'
                    if len(part) > 1:
                        key, value = part[0], part[1:]
                        
                        try:
                            if key == 'X':
                                poses_aux[0] = float(value)
                            elif key == 'Y':
                                poses_aux[1] = float(value)
                            elif key == 'Z':
                                poses_aux[2] = float(value)
                            elif key == 'I':
                                poses_aux[3] = float(value)
                            elif key == 'J':
                                poses_aux[4] = float(value)
                            elif key == 'K':
                                poses_aux[5] = float(value)
                            elif key == 'W':
                                poses_aux[6] = float(value)
                            elif key == 'F':  
                                Vel_impresion_aux = float(value)
                                #print(f"Velocidad de impresión establecida a: {Vel_impresion_aux} mm/s")
                        except ValueError as e:   # Si no se puede convertir a float, se ignora el valor y muestra un mensaje de advertencia
                            print(f"Advertencia: No se pudo convertir '{value}' en la línea: {line.strip()}")
                            continue
                #print(f"Poses_aux: {poses_aux}")      
                if any(poses_aux[:3]): # Si se ha encontrado alguna posición, añadimos la entrada
                    poses.append(poses_aux)           # relleno con 0 los valores ausentes       
            
            elif poses_aux:  # Si se encuentra una nueva línea que no es G1, almacenamos la entrada acumulada
                poses.append(poses_aux)
                Vel_impresion.append(Vel_impresion_aux)
                poses_aux = {}  # Reiniciamos para la siguiente entrada
                Vel_impresion_aux = 0

        if poses_aux:  # Aseguramos añadir la última entrada procesada
            poses.append(poses_aux)

    return poses, Vel_impresion, Temp_cama, Temp_extrusor, motores_on # None indica que nse ha terminado todo el archivo
    

































# Mostrar las posiciones y velocidades una por una
# for idx, pose in enumerate(poses):
#     pos = {k: v for k, v in pose.items() if k.startswith('pos_')}
#     orient = {k: v for k, v in pose.items() if k.startswith('orient_')}
#     velocidad_impresion = pose.get('velocidad_impresion', 'No especificada')

#     #print(f"Entrada {idx + 1}:")
#     print(f"  Posición: {pos}")
#     print(f"  Orientación: {orient}")
#     print(f"  Velocidad de impresión: {velocidad_impresion}")
#     print("---")


#  if line.startswith('G4'):  # Comando para esperar un tiempo
#                 parts = line.strip().split()
#                 for part in parts[1:]:  # Omitimos 'G4'
#                     if len(part) > 1:
#                         key, value = part[0], part[1:]
#                         try:
#                             if key == 'S':  # Verificamos si el parámetro es de tiempo en segundos
#                                 Tiempo_espera = float(value)
#                                 print(f"Tiempo de espera establecido en: {Tiempo_espera} segundos")
#                         except ValueError as e:  # Si no se puede convertir a float, mostramos un mensaje de advertencia
#                             print(f"Advertencia: No se pudo convertir '{value}' en la línea: {line.strip()}")
#                             continue




# if line.startswith('GO'):  # Movimiento sin extrusion

