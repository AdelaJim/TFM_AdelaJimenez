# # DESCRIPCIÓN
# Este nodo permite define y ejecuta una rutina consistente en el cálculo de una trayectoria y la ejecución del super_logger. 
# Lo primero que hace este nodo es calcular la trayectoria y quedarse escuchando a la recepción de un mensaje de continuación, 
# en ese momento manda correr el nodo del super_logger con las instrucciones deseadas.
#
# Para poder correr ambos nodos con las instrucciones deseadas se recomienda abrir el código y modificar las strings que contienen 
# el comando de terminal equivalente. 
#
# Este nodo no requiere de argumentos previos.
# MODIFICACIONES ABRIL 2025:
# - Se ha habilitado la ejecución no bloqueante de los comandos usando hilos (threading).
# - Se puede modificar fácilmente el `factor_escala` y el `trayectoria_dato` desde las variables declaradas en el constructor.
# - El logger se lanza en un hilo independiente tras recibir la señal `/arranca_logger_topic`, sin bloquear el nodo principal.

import rclpy
from rclpy.node import Node         # Para emplear nodos de ros2 con python
from std_msgs.msg import String     # Para manejar datos de ros 2 en formato String
from std_msgs.msg import Float64    # Ídem para datos tipo float de 64 bits.
from std_msgs.msg import Bool       # ïdem para datos tipo Bool
from subprocess import Popen        # Biblioteca de python que permite correr procesos en paralelo co un aterminal virtualizada
import threading
import os
import subprocess
import time
import tqdm
import sys
import signal 

# El único nodo del fichero de nodo
class NodeLauncher(Node):
    # Constructor del nodo
    def __init__(self):
        super().__init__('super_logger_launcher')

        # Parametros de la rutina a ejecutar
        self.factor_escala=0.08
        self.trayectoria_dato='short_generated_gcode_medio_estrella_poses.gcode'

        # Se definen las variables que alojarań cada proceso de forma previa por motivos de seguridad
        self.logger_process = None
        self.mover_process = None

        # Se definen las instrucciones que entenderá nuestro nodo por correr 
        #self.string_trayectoria= 'ros2 run trayectories move_l_nodes --ros-args -p factor_escala:=0.08 -p trayectoria_dato:=generated_gcode_medio_estrella_poses_v3.gcode'
        #self.string_trayectoria= 'ros2 run trayectories move_l --ros-args -p factor_escala:=0.08 -p trayectoria_dato:=20240506_medio_estrella_posesROSquat_miguel_v3.csv'
        self.string_trayectoria = (f"ros2 run trayectories move_l_nodes "f"--ros-args -p factor_escala:={self.factor_escala} "f"-p trayectoria_dato:={self.trayectoria_dato}")
        self.string_super_logger= 'ros2 run data_logger super_logger --ros-args -p n_muestras:=225000 -p freq:=100 -p analog_input_pin:=0 -p io:=input'

        # Se arrancan los nodos partícipes
        self.start_nodes()  

    # Arranque de los nodos partícipes
    def start_nodes(self):
        # Se arrnca el nodo de mover el robot con la trayectoria y luego se carga en la variable que representa dicho proceso.
        #self.mover_process = Popen(self.string_trayectoria.split())

        
        # Se crea un nodo suscriptor a move_l que será responsable de actualizar la señal de arranque del nodo del logger gracias a su callback
        self.suscription_move_l= self.create_subscription(Bool,'/arranca_logger_topic', self.arranca_logger_callback, 10)

        # Se crea un hilo para el nodo de mover el robot con la trayectoria y luego se carga en la variable que representa dicho proceso.
        threading.Thread(target=self.launch_trayectoria, daemon=True).start()
        #self.suscription_move_l
        
        self.get_logger().info('Los nodos han arrancado')

    # Método para lanzar el nodo de mover el robot con la trayectoria
    def launch_trayectoria(self):
        self.get_logger().info("Se lanza el nodo de mover el robot con la trayectoria")
        #self.mover_process = Popen(self.string_trayectoria.split())
        self.mover_process = subprocess.Popen(self.string_trayectoria, stdout=sys.stdout, stderr=sys.stderr)


    # Callback para arrancar el proceso del nodo del logger
    def arranca_logger_callback(self, msg):
        if msg.data==True:
            # Se arranca el nodo de super_logger en cuanto el nodo de la trayectoria indica que la va a comenzar.
            #self.logger_process = Popen(self.string_super_logger.split())

            #self.logger_process
            self.get_logger().info('Señal recibida: Iniciando logger.')
            threading.Thread(target=self.launch_logger, daemon=True).start()

    def launch_logger(self):
        #self.get_logger().info(f"Lanzando logger con:\n{self.string_super_logger}")
        self.logger_process=Popen(self.string_super_logger, shell=True)


    # Método qenérico de detención de nodos.
    def stop_nodes(self):
        if self.logger_process:
            self.get_logger().info('Cerrando super_logger...')
            self.logger_process.terminate()
            self.logger_process.wait()
        if self.mover_process:
            self.get_logger().info('Cerrando move_l_nodes...')
            self.mover_process.terminate()
            self.mover_process.wait()
        self.get_logger().info('Los nodos han parado.')


# Función main del nodo
def main(args=None):
    rclpy.init(args=args)
    node_launcher = NodeLauncher()
    rclpy.spin(node_launcher)
    node_launcher.stop_nodes()
    node_launcher.destroy_node()
    rclpy.shutdown()

     

if __name__ == '__main__':
    main()
