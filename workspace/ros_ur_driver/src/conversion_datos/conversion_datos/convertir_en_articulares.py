# Tener instalado ROS2. 
# Para ejecutarlo:
# ros2 launch ur_bringup lanzar_simulacion.launch.py
# ros2 run trayectories convertir_en_articulares.py entrada.csv


# NOTA: si los csv están en otra carpeta diferente al script, es necesario indicarlo, poniendo el path completo.
# NOTA 2: el csv se guarda en esta misma carpeta.
# NOTA 3: para compilarlo: colcon build --packages-select conversion_datos

# ros2 run conversion_datos convertir_en_articulares /home/adela/TFM_AdelaJimenez/workspace/ros_ur_driver/src/trayectories/data/muelle_w3.csv muelle_w3_articulares.csv




import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
import csv
import sys
import os
import time

# ruta de salida
SALIDA_DIR = "/home/adela/TFM_AdelaJimenez/workspace/ros_ur_driver/src/conversion_datos/resultados/"

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /compute_ik ...')

    def cartesian_to_joints(self, pose):
        from geometry_msgs.msg import PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.avoid_collisions = True
        request.ik_request.pose_stamped = pose_stamped
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().solution.joint_state.position:
            return list(future.result().solution.joint_state.position)
        else:
            self.get_logger().error('IK solution failed para el punto dado')
            return None

def read_positions_from_file(file_path):
    positions = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            try:
                vals = [float(value) for value in row]
                if len(vals) == 7:
                    positions.append(vals)
            except Exception:
                continue
    return positions

def save_articular_positions(joints_list, output_path):
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
        for joint_set in joints_list:
            writer.writerow(joint_set)

def main(args=None):
    if len(sys.argv) != 2:
        print("USO: ros2 run conversion_datos convertir_en_articualres path_entrada.csv")
        sys.exit(1)

    input_csv = sys.argv[1]
    input_base = os.path.basename(input_csv)
    output_csv = os.path.join(SALIDA_DIR, 'articulares_' + input_base)


    rclpy.init(args=args)
    node = IKNode()

    positions = read_positions_from_file(input_csv)
    joint_positions = []

    print(f"Procesando {len(positions)} puntos del archivo {input_csv}...")
    start_time = time.time()  

    for idx, pos in enumerate(positions):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z, \
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = pos

        joints = node.cartesian_to_joints(pose)
        if joints:
            joint_positions.append(joints)
        else:
            print(f"[{idx+1}/{len(positions)}] Fallo IK en punto: {pos}")

    end_time = time.time()  # <-- Marca de fin
    elapsed = end_time - start_time
    
    node.destroy_node()
    rclpy.shutdown()

    save_articular_positions(joint_positions, output_csv)
    print(f"\nGuardado {len(joint_positions)} puntos de {len(positions)} en {output_csv}")
    
    if elapsed > 60:
        elapsed_sec = int(elapsed // 60)
        print(f"Tiempo total empleado: {elapsed_sec:.2f} minutos")
    else:
        print(f"Tiempo total empleado: {elapsed:.2f} segundos")


if __name__ == '__main__':
    main()




