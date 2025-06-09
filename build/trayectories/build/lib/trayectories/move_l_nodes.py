#  DESCRIPCIÓN - Adela Jiménez 
#  Este script coordina la ejecución de trayectorias no planas en un UR10 a partir de un archivo G-code,
#  integrando el control de periféricos mediante ROS 2 y comunicación serie con microcontroladores.
#
#  Parámetros de entrada:
#  * trayectoria_dato: Nombre del archivo G-code con los puntos de la trayectoria.
#  * factor_escala: Ajusta la velocidad de ejecución de la trayectoria (0.01 - 2.00).
#  * arrancar_logger: Indica si se debe iniciar el sistema de logging de datos.
#
#  Flujo general:
#  1. Se parsea el archivo G-code para extraer trayectorias y parámetros de impresión.
#  2. Se envían las temperaturas objetivo a los nodos controladores de cama y extrusor.
#  3. Cada nodo establece la comunicación con su respectivo Arduino, que gestiona el calentamiento.
#  4. Se espera confirmación (`TEMP_OK`) de ambos nodos para continuar.
#  5. Se calcula la trayectoria cartesiana desde los puntos definidos en el G-code.
#  6. Se ejecuta la trayectoria con MoveIt!, mientras se monitorizan temperaturas.
#  7. Al finalizar, se publica una señal de `shutdown` que activa la parada de los sistemas térmicos.
#
#  
#  MODULARIZACIÓN DE NODOS:
#  - MasterNode(): Nodo principal que coordina la ejecución y publica órdenes globales.
#  - GcodeParserNode(): Lee el G-code y publica los parámetros extraídos.
#  - BedControllerNode(): Controla la temperatura de la cama y monitoriza el estado.
#  - ExtruderControllerNode(): Controla la temperatura del extrusor y los motores paso a paso.
#  - CartesianPathNode(): Calcula la trayectoria cartesiana.
#  - MyActionClientNode(): Ejecuta la trayectoria calculada a través de MoveIt!.

#  COMUNICACIÓN:
#  - `/gcode/positions` (String): Puntos de trayectoria [GcodeParserNode → MasterNode]
#  - `/gcode/temp_cama` (Float64): Temperatura cama [GcodeParserNode → MasterNode]
#  - `/gcode/temp_extrusor` (Float64): Temperatura extrusor [GcodeParserNode → MasterNode]
#  - `/gcode/motores_on` (Bool): Motores del extrusor [GcodeParserNode → MasterNode]
#  - `/gcode/vel_impresion` (String): Velocidad de impresión [GcodeParserNode → MasterNode]
#  - `/trajectory/temp_cama` (Float64): Temperatura consigna cama [MasterNode → BedControllerNode]
#  - `/trajectory/temp_extrusor` (Float64): Temperatura consigna extrusor [MasterNode → ExtruderControllerNode]
#  - `/trajectory/vel_impresion` (String): Velocidad de impresión [MasterNode → ExtruderControllerNode]
#  - `/check/cama_ok` (Bool): Confirmación de temperatura cama [BedControllerNode → MasterNode]
#  - `/check/ext_ok` (Bool): Confirmación de temperatura extrusor [ExtruderControllerNode → MasterNode]
#  - `/monitor/temp_cama` (Float64): Temperatura actual cama [BedControllerNode → MasterNode]
#  - `/monitor/temp_extrusor` (Float64): Temperatura actual extrusor [ExtruderControllerNode → MasterNode]
#  - `/shutdown` (Bool): Señal global de apagado [MasterNode → BedControllerNode & ExtruderControllerNode]
#  - `/arranca_logger_topic` (Bool): Señal para iniciar el logger [MasterNode → LoggerNode]
#  - `/execute_trajectory` (Action): Acción para ejecutar la trayectoria [MyActionClientNode → MoveIt!]
#  - `/compute_cartesian_path` (Service): Servicio para calcular la trayectoria [CartesianPathNode → MoveIt!]
#
#
#   GAP: Elapagado no se realiza en el flujo, porque no se detecta el finde la trayectoria. Para realizarlo por fuera: 
#   Abrir terminal +  ros2 topic pub /shutdown std_msgs/Bool "data: true"
#   o abrir los reail desde arduino y escribir: OFF

import sys
import os

# Agregar la ruta del directorio actual al path de Python
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Librerías de ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # Para manejar datos de ros 2 en formato String
from std_msgs.msg import Float64    # Ídem para datos tipo float de 64 bits.
from std_msgs.msg import Bool
from sensor_msgs.msg._joint_state import JointState  # Para leer el estado de las articulaciones del UR.
from ur_msgs.msg._io_states import IOStates # Para leer entradas y salidas digitales del UR.
from moveit_msgs.srv import GetPositionIK # Para leer los mensajes de moveit.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose # Para manejar datos de las poses adoptadas en coordenadas articulares.
from moveit_msgs.msg import Constraints # Control de restricciones impuestas por el urdf o el entorno de movit.
from moveit_msgs.srv import GetCartesianPath # Algoritmo de cálculo de trayectorias en un espacio vectorial cartesiano.
from rclpy.action import ActionClient # Cliente  para efecturar acciones de ros2.
from moveit_msgs.action import ExecuteTrajectory # Acción de ejecución de trayectorias adaptada de moveit.
from builtin_interfaces.msg import Duration
from parseo_gcode import parseo_gcode
from rclpy.executors import MultiThreadedExecutor

# Otras librerías útiles.
import pandas as pd     # Para manejar datos y crear tablas.
import matplotlib.pyplot as plt # Para hacer gráficos con los datos registrados.
import numpy as np
import re
import time
from tqdm import tqdm
import math 
import csv
import os
import datetime
import git
import serial
import threading     # esto permite que los nosos se ejecuten en paralelo.
# Esta clase es la respondable de calcular la trayectoria cartesiana.
# Dependiendo del número de puntos, complejidad de la trayectoria y pose inicial del robot puede demeorarse más o menos tiempo.
# Se recomienda realizar unos ensayos previos a un bajo factor de escala para conocer la velocidad de ejecución en cada caso.
class CartesianPathNode(Node):
    # Constructor de la clase de cálculo de trayectoria.
    def __init__(self):
        super().__init__('cartesian_path_node')
        # Se crea un cliente que calculará la trayectoria a partir de los datos cartesianos. El cliente es nativo de moveit y ros2.
        self.compute_cartesian_path_client= self.create_client(GetCartesianPath, '/compute_cartesian_path')

    # Método de cálculo de trayectoria
    def compute_cartesian_path(self, waypoints, factor_escala):
        # Se crea una solicitud para que el servicio calcule la trayectoria
        # Se asignan marcas de tiempo, ancho de paso, puntos de cálculo y que se limiten las colisiones con el entorno virtual definido por moveit.
        request= GetCartesianPath.Request()
        request.header.stamp= self.get_clock().now().to_msg()
        request.group_name= 'ur_manipulator'
        request.waypoints= waypoints
        request.max_step= 0.05
        request.avoid_collisions= True

        self.get_logger().info(f'Se mete como dato {len(waypoints)} waypoints a calcular')

        # Aviso de que el servicio puede tardar un poco en obtener un resultado final.
        while not self.compute_cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperado ...')

        # Solución de trayectoria calculada
        future= self.compute_cartesian_path_client.call_async(request)

        self.get_logger().info('Esperando servicio ...')
        rclpy.spin_until_future_complete(self, future)
        # self.get_logger.info('Servicio completado')
        self.get_logger().info('Servicio completado')
        self.get_logger().info(f'Future tiene un total de {len(future.result().solution.joint_trajectory.points)}')

        # Con una trayectoria previa calculada por el controlador, se define el control de velocidad mediante factor de escala.
        if future.result() is not None:
            
            # Copia de la trayectoria de moveit con la que se operará
            destino= future.result().solution
            # Modificación de velocidades y aceleraciones articulares de la trayectoria resultado.
            # ïdem para los tiempos relativo de ejecución. Se hace punto a punto para asegurar un buen resultado.
            for j in range(len(future.result().solution.joint_trajectory.points)):
                x_= future.result().solution.joint_trajectory.points[j]

                # Operación de velocidades y aceleraciones
                for i in range(len(x_.velocities)):
                    x_.velocities[i]*= factor_escala
                    x_.accelerations[i]*= factor_escala

                # print(x_.time_from_start)
                # Operación de tiempos de reloj
                aux_nanosec= x_.time_from_start.nanosec
                aux_sec= x_.time_from_start.sec
                aux_duration= aux_sec*1e9+aux_nanosec
                aux_duration/=factor_escala

                segundos= aux_duration//1e9
                nanosegundos= aux_duration%1e9
                # print(f'Tiempo con el factor de escala. Segundos {segundos} --- Nanosegundos {nanosegundos}')

                x_.time_from_start.sec= int(segundos)
                x_.time_from_start.nanosec= int(nanosegundos)

                # print(f"velocidades: {x_.velocities}")
                # print(f"aeleraciones: {x_.velocities}")

                # Asignación de los resultados modificados al resultado original. Se hace punto a punto.
                destino.joint_trajectory.points[j]= x_

                if j==len(future.result().solution.joint_trajectory.points)-1:
                    self.get_logger().info(f'Tiempo de ejecución aproximado: {segundos/60:.2f} minutos')
                    self.get_logger().info(f'Se calculan {len(future.result().solution.joint_trajectory.points)} puntos en la solución')
            
            
            # Depenediendo del factor de escala aplicado se pasa a ROS2 una solución a otra.
            # Solución original de moveit si =1. Solución modificada si >1 o <1.
            if factor_escala==1:
                self.get_logger().info('Se ejecuta la solucion de moveit')
                self.get_logger().info(f"Factor escala: {factor_escala}")
                trajectory= future.result().solution
            elif factor_escala<1:
                self.get_logger().info('Se ejecuta la solución del control de velocidad')
                self.get_logger().info(f"Factor escala < 1 : {factor_escala}")
                trajectory= destino
            else:
                self.get_logger().info('Se ejecuta la solución del control de velocidad')
                self.get_logger().info(f"Factor escala > 1 : {factor_escala}")
                trajectory= destino
            
            # Se guarda la trayectoria calculada por moveit en un excel.
            self.save_trajectory(trajectory=trajectory)

            return trajectory
        else:
            self.get_logger().info('Se ha fallado calculando la solución en IK.')
            return None
        

    def save_trajectory(self, trajectory):
        self.get_logger().info('Se procede a guardar la trayectoria calculada en un csv')
        joint_names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points

        data = {name: [] for name in joint_names}
        data['time_from_start'] = []

        for point in points:
            for i, name in enumerate(joint_names):
                data[name].append(point.positions[i])
            time_in_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            data['time_from_start'].append(time_in_sec)


        # Se guarda la trayectoria resultado del cálculo de moveit en la carpeta data/saved_trajectories
        # Se formatea el título para que incluya la fecha y hora correspondientes. Son las de creación de la gráfica.
        fecha_hora_actual=datetime.datetime.now()

        # Se guarda la imagen indicando la fecha de actualizacion.
        fecha_str=fecha_hora_actual.strftime("%Y%m%d_%H%M")
        titulo_tabla=f"{fecha_str}_trajectory.csv"

        # Obtener la ruta al directorio actual
        current_dir = os.path.abspath(os.path.dirname(__file__))

        # Obtener la ruta al directorio del repositorio Git
        git_repo_dir = git.Repo(current_dir, search_parent_directories=True).git.rev_parse("--show-toplevel")
        
        ruta_guardado= git_repo_dir + '/workspace/ros_ur_driver/src/trayectories/data/saved_trajectories/'


        
        ruta_guardado=os.path.join(ruta_guardado, titulo_tabla)

        # self.mi_tabla.to_csv(ruta_guardado)


        df = pd.DataFrame(data)
        df.to_csv(ruta_guardado)
        self.get_logger().info(f'Trayectoria guardada en {ruta_guardado}')

    def get_data_path(self):

        # Obtener la ruta al directorio actual
        current_dir = os.path.abspath(os.path.dirname(__file__))

        # Obtener la ruta al directorio del repositorio Git
        git_repo_dir = git.Repo(current_dir, search_parent_directories=True).git.rev_parse("--show-toplevel")
        
        data_path= git_repo_dir + '/workspace/ros_ur_driver/src/trayectories/data/'

        return data_path



# Clase responsable de comunicar las acciones de seguimiento y ejecución de trayectorias.
class MyActionClientNode(Node):
    
    # Método constructor
    def __init__(self):
        super().__init__('action_client_node') 

        self.execute_client= ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    # Método de ejecución de trayectorias calculadas por CartesianPathNode
    def execute_trajectory(self, trajectory_solution):
        # Se llama al servidor y se espera el tiempo necesario.
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('No está disponible el servidor de la acción /execute_trajectory')
            return
        
        # Se crea una orden de ejecución de trayectoria y se carga la solución de CartesianPathNode
        goal_msg=ExecuteTrajectory.Goal()
        goal_msg.trajectory=trajectory_solution

        future=self.execute_client.send_goal_async(goal=goal_msg)

        self.get_logger().info('Esperando resultado de la ejecución')

        # Repetir proceso hasta que se complelte la trayectoria o se paralice por moveit
        rclpy.spin_until_future_complete(self, future)
        result=future.result()



# Definir el nodo para la lectura del G-code. Se encarga de leer el archivo y publicar los datos necesarios
class GcodeParserNode(Node):
    def __init__(self):
        super().__init__('gcode_parser_node')
       
        # Crear los publicadores para enviar datos a otros nodos
        self.positions_pub = self.create_publisher(String, '/gcode/positions', 10)
        self.temp_cama_pub = self.create_publisher(Float64, '/gcode/temp_cama', 10)
        self.temp_extrusor_pub = self.create_publisher(Float64, '/gcode/temp_extrusor', 10)
        self.motores_on_pub = self.create_publisher(Bool, '/gcode/motores_on', 10)
        self.vel_impresion_pub = self.create_publisher(Float64, '/gcode/vel_impresion', 10)

        self.declare_parameter('trayectoria_dato', 'generated_gcode_medio_estrella_poses_v3.gcode')

        self.publicar_temp = True   # Se usa para detener la publicacion de la temperatura
        # Leer el archivo G-code y publicar los datos
        self.read_gcode_file()

    def read_gcode_file(self):
        self.get_logger().info("Leyendo archivo G-code...")
        trayectoria = self.get_parameter('trayectoria_dato').value
        file_path = self.get_data_path() + trayectoria

        # Parsear el archivo G-code
        positions, Vel_impresion, Temp_cama, Temp_extrusor, Motores_on = parseo_gcode(file_path)

        # Publicar los datos en los tópicos
        self.publish_data(positions,Vel_impresion, Temp_cama, Temp_extrusor, Motores_on)

    def publish_data(self, positions, vel_impresion, temp_cama, temp_extrusor, motores_on):
        self.get_logger().info("Publicando datos extraídos del G-code")
        pos_msg = String()
        if self.publicar_temp:

            pos_msg.data = str(positions)
            self.positions_pub.publish(pos_msg)
            self.get_logger
            
            vel_impresion_msg = Float64()   # por ahora es Float, porque vamos a velocidad constante. 
            vel_impresion_msg.data = float(vel_impresion)
            self.vel_impresion_pub.publish(vel_impresion_msg)
            self.get_logger().info(f"Velocidad: {vel_impresion} mm/s")
        
            temp_cama_msg = Float64()
            temp_cama_msg.data = temp_cama
            self.temp_cama_pub.publish(temp_cama_msg)
            self.get_logger().info(f"temp_cama: {temp_cama}°C")

            temp_extrusor_msg = Float64()
            temp_extrusor_msg.data = temp_extrusor
            self.temp_extrusor_pub.publish(temp_extrusor_msg)
            self.get_logger().info(f"temp_extrusor: {temp_extrusor}°C")

            motores_on_msg = Bool()    # Esto se enviara al extrusor justo antes de empezar a extruir
            motores_on_msg.data = bool(motores_on)
            self.motores_on_pub.publish(motores_on_msg)

            self.publicar_temp = False

    def get_data_path(self):
        import os
        import git

        current_dir = os.path.abspath(os.path.dirname(__file__))
        git_repo_dir = git.Repo(current_dir, search_parent_directories=True).git.rev_parse("--show-toplevel")
        return git_repo_dir + '/workspace/ros_ur_driver/src/trayectories/data/'



# Definir el nodo de control de temperatura. Gestiona la comunicación con Arduino y monitorea la temperatura de la cama.
class BedControllerNode(Node):
    def __init__(self):
        super().__init__('bed_controller_node')
        
        # Suscribirse al tópico de temperatura de la cama
        self.create_subscription(Float64, '/trajectory/temp_cama', self.temp_cama_callback, 10)
        self.create_subscription(Bool, '/shutdown', self.shutdown_callback, 10)

        # Publicador para la temperatura monitoreada y ack temp_ok
        self.cama_ok_pub = self.create_publisher(Bool, '/check/cama_ok', 10)
        self.temp_pub = self.create_publisher(Float64, '/monitor/temp_cama', 10)

        self.current_temp = 0.0
        self.timer_ = self.create_timer(10.0, self.publish_monitored_bed_temperature)  # Publicar temperatura cada X segundos

        try:
            self.serial_port = serial.Serial('/dev/ttyACM1', 2400, timeout=1)
            #self.serial_port = serial.Serial('/dev/ttyUSB0', 2400, timeout=1)
            self.get_logger().info('Puerto Serial abierto correctamente')
        except serial.SerialException as e:
            self.get_logger().error(f"Error abriendo puerto serie: {e}")
            rclpy.shutdown()
            sys.exit(1)

        self.publicar_temp_ok = True

    # En cuanto se recibe la temperatura de la cama, se envía a Arduino
    def temp_cama_callback(self, msg):
        self.get_logger().info(f'Iniciando  calentamiento de la cama a {msg.data}°C')
        self.serial_port.write(f"{int(msg.data)}\n".encode())
        self.serial_port.flush()
        self.wait_for_temperature_confirmation()

    def wait_for_temperature_confirmation(self):
        while True:
            if self.serial_port.in_waiting > 0:
                message = self.serial_port.readline().decode().strip()
                if message == "TEMP_OK" and self.publicar_temp_ok:
                    self.get_logger().info(f'{message}: Continuando ejecución.')
                    self.cama_ok_pub.publish(Bool(data=True))
                    self.publicar_temp_ok = False    # Solo se publica una vez
                    break
                else:
                    self.get_logger().info(f'Calentando cama .... Temperatura actual: {message} ºC')

    def shutdown_callback(self,msg):
        if msg.data:
            self.get_logger().info("Enfriando cama....")
            self.serial_port.write("OFF\n".encode())
            self.serial_port.flush()
            self.serial_port.close()

    def publish_monitored_bed_temperature(self):
        if self.serial_port.in_waiting > 0:
            message = self.serial_port.readline().decode().strip()
            try:
                self.current_temp = float(message)
                temp_msg = Float64()
                temp_msg.data = self.current_temp
                self.temp_pub.publish(temp_msg)
            except ValueError:
                pass  # Ignorar si no es float


# Definir el nodo encargado de la comunicación con el extrusor a Arduino por serial.
class ExtruderControllerNode(Node):
    def __init__(self):
        super().__init__('extruder_controller_node')
        
        # Suscribirse al tópico de 
        self.create_subscription(Float64, '/trajectory/temp_extrusor', self.temp_extrusor_callback, 10)
        self.create_subscription(Float64, '/trajectory/vel_impresion', self.vel_impresion_callback, 10)
        self.create_subscription(Bool, '/trajectory/motores_on', self.motores_on_callback,10)
        self.create_subscription(Bool, '/shutdown', self.shutdown_callback, 10)
        
        # Publicador para la temperatura monitoreada y ack temp_ok TBD
        self.extrusor_ok_pub = self.create_publisher(Bool, '/check/ext_ok', 10) 
        self.temp_pub = self.create_publisher(Float64, '/monitor/temp_extrusor', 10) 

        self.current_temp = 0.0
        self.timer_ = self.create_timer(10.0, self.publish_monitored_ext_temperature)  # Publicar temperatura cada X segundos

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 2400, timeout=1)
            #self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1) 
            self.get_logger().info('Puerto Serial abierto correctamente')
        except serial.SerialException as e:
            self.get_logger().error(f"Error abriendo puerto serie: {e}")
            rclpy.shutdown()
            sys.exit(1)

        
        self.publicar_temp_ok = True
        self.ready_to_send = False
        self.temp_extrusor = None
        self.vel_impresion = None

    def motores_on_callback(self,msg):
        # Se escribe enb el serial un 1
        self.get_logger().info('Se activan los motores')
        self.serial_port.write("EXT_OK\n".encode())
        self.serial_port.flush()

    # En cuanto se recibe la temperatura del extrusor, se envía a Arduino
    def temp_extrusor_callback(self, msg):
        self.temp_extrusor = float(msg.data)
        self.get_logger().info(f'Iniciando  calentamiento del extrusor a {msg.data}°C')
        #self.check_and_send_serial()

    def vel_impresion_callback(self, msg): 
        self.vel_impresion=float(msg.data)
        #self.get_logger().info(f'velociodad  {self.vel_impresion}')
        self.check_and_send_serial()

    def check_and_send_serial(self):
        # if self.temp_extrusor is not None and self.vel_impresion is not None and self.publicar_temp_ok:
            
            mensaje = f"{self.temp_extrusor},{self.vel_impresion}\n"
            self.serial_port.write(mensaje.encode())
            self.serial_port.flush()
            self.get_logger().info(f"Enviado al extrusor: {mensaje.strip()}")
            self.wait_for_temperature_confirmation()

    def wait_for_temperature_confirmation(self):
        while True:
            if self.serial_port.in_waiting > 0:
                message = self.serial_port.readline().decode().strip()
                if message == "TEMP_OK" and self.publicar_temp_ok:
                    self.get_logger().info(f'{message}: Continuando ejecución.')
                    self.extrusor_ok_pub.publish(Bool(data=True))
                    self.publicar_temp_ok = False    # Solo se publica una vez
                    break
                else:
                    self.get_logger().info(f'Temperatura actual del extrusor: {message} ºC.....Calentando')

    def shutdown_callback(self,msg):
        if msg.data:
            self.get_logger().info("Enfriando extrusor....")
            self.serial_port.write("OFF\n".encode())
            self.serial_port.flush()
            self.serial_port.close()

    def publish_monitored_ext_temperature(self):
        if self.serial_port.in_waiting > 0:
            message = self.serial_port.readline().decode().strip()
            try:
                self.current_temp = float(message)
                temp_msg = Float64()
                temp_msg.data = self.current_temp
                self.temp_pub.publish(temp_msg)
            except ValueError:
                pass  # Ignorar si no es float
  
    


# Clase principal: MasterNode
class MasterNode(Node):
    # Constructor
    def __init__(self):
        super().__init__('Master_node')
       
        # Parámetros de arranque para el nodo.
        self.declare_parameter('trayectoria_dato', 'generated_gcode_medio_estrella_poses_v3.gcode')
        self.declare_parameter('arrancar_logger', False)
        self.declare_parameter('factor_escala', 1.00)
        self.arranca_logger = self.get_parameter('arrancar_logger').value

        self.publisher_ = self.create_publisher(Bool, '/arranca_logger_topic', 10)
        # self.timer_ = self.create_timer(1.0, self.publish_arranca_logger)
         
        # Crear un Timer para la máquina de estados (cada 1s)
        self.timer = self.create_timer(1.0, self.gestionar_flujo)

        # Instanciar los nodos 
        self.cartesian_path_node = CartesianPathNode()
        self.action_client_node = MyActionClientNode()

        # Suscripciones a topics
        self.create_subscription(String, '/gcode/positions', self.positions_callback, 10)
        self.create_subscription(Float64, '/gcode/temp_cama', self.temp_cama_callback, 10)
        self.create_subscription(Float64, '/gcode/temp_extrusor', self.temp_extrusor_callback, 10)
        self.create_subscription(Bool, '/gcode/motores_on', self.motores_on_callback, 10)
        self.create_subscription(Float64, '/gcode/vel_impresion', self.vel_impresion_callback, 10)    #Aunque se lee en string, por ahora, me llega un float
        self.create_subscription(Bool, '/check/cama_ok', self.cama_ok_callback, 10)
        self.create_subscription(Float64, '/monitor/temp_cama', self.temp_cama_monitor_callback, 10)
        self.create_subscription(Bool, '/check/ext_ok', self.ext_ok_callback, 10)
        self.create_subscription(Float64,'/monitor/temp_extrusor', self.temp_extrusor_monitor_callback, 10)
        
        # Publshers de topics
        self.temp_cama_pub = self.create_publisher(Float64, '/trajectory/temp_cama', 10)
        self.shutdown_pub = self.create_publisher(Bool, '/shutdown', 10)
        self.temp_extrusor_pub = self.create_publisher(Float64, '/trajectory/temp_extrusor', 10)
        self.vel_impresion_pub = self.create_publisher(Float64, '/trajectory/vel_impresion', 10)
        self.motores_on_pub=self.create_publisher(Bool, '/trajectory/motores_on',10)

        # inicialización de variables de control
        self.estado = "INICIO"  # Estado inicial
        self.Temp_cama = Float64()
        self.temp_extrusor = Float64()
        self.cama_ok = False
        self.extrusor_ok = False 
        self.trayectoria_completada = False
        self.ejecutando = False
        self.publicar_temp = True # Se usa para detener la publicacion de la temperatura
        self.goal_names=[]
        self.factor_escala=self.get_parameter('factor_escala').value
    
    # Este método (que se ejecuta periodicamente), actua como una máquina de estados, dirigiendo el flujo del proceso
    def gestionar_flujo(self):
        
        if self.estado == "INICIO":
            if self.Temp_cama  and self.temp_extrusor :
                self.estado = "SETUP"

        elif self.estado == "SETUP":
            if self.publicar_temp:
                self.setup_process()
                self.publicar_temp = False

        elif self.estado == "EJECUTAR_TRAYECTORIA":
            if self.cama_ok and self.extrusor_ok:
                self.calculo_trayectoria()
                # esto es para que la maquina de estados solo entre una vez aqui.
                self.ejecutando = True
                self.cama_ok = False
                self.extrusor_ok = False 

        elif self.estado == "SHUTDOWN":
            self.shutdown_process()
    
    # falta gestionar la publicacion de las velocidades de impresion. de una en una? o todo el vector y el nodo se encarga de administrrlas

    def setup_process(self):
        self.get_logger().info('Iniciando proceso de setup...')
        # Enviamos la temperatura de la cama a su encargado
        temp_cama_msg = Float64()
        temp_cama_msg.data = self.Temp_cama
        self.temp_cama_pub.publish(temp_cama_msg)
        # Enviamos la temperatura del extrusor a su nodo controlador
        temp_extrusor_msg = Float64()
        temp_extrusor_msg.data = self.Temp_extrusor
        self.temp_extrusor_pub.publish(temp_extrusor_msg)
        # Enviamos la velocidad de impresion al extrusor
        vel_impresion_msg = Float64()
        vel_impresion_msg.data = self.Vel_impresion
        self.vel_impresion_pub.publish(vel_impresion_msg)
        
    def shutdown_process(self):
        if self.trayectoria_completada: # Se publica 1 topic, y todas las cosas que se tengan que apagar se suscriben a ese topic.
            self.get_logger().info("Iniciando proceso de apagado...")
            shutdown_msg = Bool()
            shutdown_msg.data = True
            self.shutdown_pub.publish(shutdown_msg) 
    
    def positions_callback(self, msg):
        self.positions = eval(msg.data)  # Convertir de string a lista
        # self.get_logger().info(f"Recibidas posiciones desde /gcode/positions: {len(self.positions)} puntos")

    def temp_cama_callback(self, msg):
        # self.get_logger().info(f"Temperatura de la cama: {msg.data}°C")
        self.Temp_cama = msg.data    
    
    def temp_cama_monitor_callback(self, msg):
        if self.ejecutando:
            self.get_logger().info(f"Monitoreo de temperatura de la cama: {msg.data}°C")

    def cama_ok_callback(self, msg):
        self.cama_ok = msg.data
        # self.get_logger().info('Temperatura de la cama alcanzada')
        if self.estado == "SETUP":
            self.estado = "EJECUTAR_TRAYECTORIA"
        
    def temp_extrusor_callback(self, msg):
        # self.get_logger().info(f"Temperatura del extrusor: {msg.data}°C")
        self.Temp_extrusor = msg.data

    def motores_on_callback(self, msg):
        self.Motores_on = msg.data

    def vel_impresion_callback(self, msg):
        self.Vel_impresion = msg.data

    def ext_ok_callback(self, msg):
        self.extrusor_ok = msg.data
        # self.get_logger().info('Temperatura del extrusor alcanzada')

    def temp_extrusor_callback(self, msg):
        self.Temp_extrusor = msg.data

    def temp_extrusor_monitor_callback(self, msg):
        if self.ejecutando:
            self.get_logger().info(f"Monitoreo de temperatura del extrusor: {msg.data}°C")
    
    # Método de cálculo de trayectoria. A continuacion, se ejecuta la trayectoria. 
    def calculo_trayectoria(self):
        self.get_logger().info("Calculando trayectoria...")
        # Asignación de posiciones dato desde el gcode
        for position in self.positions:
            poses=Pose()
            # Aquí abajo pongo orientatio porque es como estaba definido antes. Pero la documentación se refiere al campo orientation como quaternion
            poses.position.x, poses.position.y, poses.position.z, poses.orientation.x, poses.orientation.y, poses.orientation.z, poses.orientation.w = position
            self.goal_names.append(poses)
            #print(f'Posición añadida: {poses}')

        # Se solicita una trayectoria articular solución.
        #self.get_logger().info(f'El tamaño del vector de entrada es {len(self.goal_names)}')
        trajectory_solution= self.cartesian_path_node.compute_cartesian_path(self.goal_names, self.factor_escala)
        #self.get_logger().info(f'La trayectoria calculada tiene {len(trajectory_solution.joint_trajectory.points)} puntos')
        
        # Cuando se tiene la trayectoria solución se manda ejecutar.
        if trajectory_solution:
            self.get_logger().info('Se ha calculado la trayectoria con éxito, ejecutando ...')
            #justo antes de comenzar la ejecución, envio a los motores la orden de activarse.
            motores_on_msg = Bool()
            motores_on_msg.data = self.Motores_on
            self.motores_on_pub.publish(motores_on_msg)
            self.action_client_node.execute_trajectory(trajectory_solution)
            # self.trayectoria_completada = True   #NO SE SI VA AQUI. COMO DETECTO QUE SE HA FINALIZADO?
            # self.estado = "SHUTDOWN"
            
            # Mensaje interno de arrancar el logger.
            self.get_logger().info('Se arranca el logger')
            self.arranca_logger=True
            self.publish_arranca_logger()
            self.get_logger().info(f"Estoy corriendo el archivo de trayectoria {self.get_parameter('trayectoria_dato').value}")
        else:
            self.get_logger().info('Fallo en el cálculo de trayectoria')
        
        # self.get_logger().info('--- FIN DE TRAYECTORIA ---')
   
        
        

    # Método para indicar que se arranque el logger.
    def publish_arranca_logger(self):
        msg=Bool()
        msg.data=True
        self.publisher_.publish(msg)
        # print('Se publica las señal de arrancar logger')



    
        


def main(args=None):
    rclpy.init(args=args)

    # Esto lo necesito ara que existan los tres a la vez, y me salgan en ros2 node list
    master_node = MasterNode()
    gcode_parser_node = GcodeParserNode()
    bed_controller_node = BedControllerNode()
    extruder_controller_node = ExtruderControllerNode()
    #cartesian_path_node = CartesianPathNode()
    #action_client_node = MyActionClientNode()

    executor = MultiThreadedExecutor()
    executor.add_node(master_node)
    executor.add_node(gcode_parser_node)
    executor.add_node(bed_controller_node)
    executor.add_node(extruder_controller_node)
    #executor.add_node(cartesian_path_node)
    #executor.add_node(action_client_node)

    try:
        executor.spin()
    finally:
        master_node.destroy_node()
        gcode_parser_node.destroy_node()
        bed_controller_node.destroy_node()
        extruder_controller_node.destroy_node()
        #cartesian_path_node.destroy_node()
        #action_client_node.destroy_node()
        rclpy.shutdown()

if __name__== '__main__':
    main()