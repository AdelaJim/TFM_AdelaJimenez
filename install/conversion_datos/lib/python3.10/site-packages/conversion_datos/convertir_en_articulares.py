# Tener instalado ROS2. 
# Para ejecutarlo:
# ros2 run trayectories convertir_en_articulares.py entrada.csv salida.csv
# NOTA: si los csv están en otra carpeta diferente al script, es necesario indicarlo.
# en ./trayectories/

# ros2 run conversion_datos convertir_en_articulares /home/adela/TFM_AdelaJimenez/workspace/ros_ur_driver/src/trayectories/data/muelle_w3.csv muelle_w3_articulares.csv




import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
import csv
import sys
import os

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
        print("USO: ros2 run <paquete> cartesian_to_articular_csv.py entrada.csv")
        sys.exit(1)

    input_csv = sys.argv[1]
    input_base = os.path.basename(input_csv)
    # Carpeta donde está el script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_csv = os.path.join(script_dir, 'articulares_' + input_base)


    rclpy.init(args=args)
    node = IKNode()

    positions = read_positions_from_file(input_csv)
    joint_positions = []

    print(f"Procesando {len(positions)} puntos del archivo {input_csv}...")

    for idx, pos in enumerate(positions):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z, \
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = pos

        joints = node.cartesian_to_joints(pose)
        if joints:
            joint_positions.append(joints)
        else:
            print(f"[{idx+1}/{len(positions)}] Fallo IK en punto: {pos}")

    node.destroy_node()
    rclpy.shutdown()

    save_articular_positions(joint_positions, output_csv)
    print(f"\nGuardado {len(joint_positions)} puntos de {len(positions)} en {output_csv}")

if __name__ == '__main__':
    main()





# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from moveit_msgs.srv import GetPositionIK
# import csv
# import sys
# import os

# class IKTransformNode(Node):
#     def __init__(self):
#         super().__init__('ik_transform_node')
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#         while not self.ik_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Esperando servicio /compute_ik ...')

#     def xyzijkw_to_joints(self, pose_stamped):
#         request = GetPositionIK.Request()
#         request.ik_request.group_name = "ur_manipulator"
#         request.ik_request.avoid_collisions = True
#         request.ik_request.pose_stamped = pose_stamped
#         future = self.ik_client.call_async(request)
#         rclpy.spin_until_future_complete(self, future)
#         if future.result() is not None and future.result().solution.joint_state.position:
#             return list(future.result().solution.joint_state.position)
#         else:
#             self.get_logger().error('IK solution failed para el punto dado')
#             return None

# def read_xyzijkw_csv(input_csv):
#     positions = []
#     with open(input_csv, 'r') as file:
#         reader = csv.reader(file)
#         for row in reader:
#             # Ignora filas con texto/no numéricas (cabecera)
#             try:
#                 vals = [float(val) for val in row[:7]]
#                 if len(vals) == 7:
#                     positions.append(vals)
#             except Exception:
#                 continue
#     return positions

# def write_joints_csv(joint_positions, output_csv):
#     with open(output_csv, 'w', newline='') as file:
#         writer = csv.writer(file)
#         writer.writerow(['q1','q2','q3','q4','q5','q6'])
#         writer.writerows(joint_positions)

# def main(args=None):
#     if len(sys.argv) != 3:
#         print("USO: ros2 run <paquete> csv_to_joint_positions.py input_xyzijkw.csv output_joints.csv")
#         sys.exit(1)

#     input_csv = sys.argv[1]
#     output_csv = sys.argv[2]

#     rclpy.init(args=args)
#     node = IKTransformNode()

#     # Nodo temporal para el timestamp ROS2
#     temp_node = rclpy.create_node("dummy_stamp")
#     positions = read_xyzijkw_csv(input_csv)
#     joint_positions = []

#     print(f"Procesando {len(positions)} puntos del archivo {input_csv}...")

#     for idx, pos in enumerate(positions):
#         pose = PoseStamped()
#         pose.header.frame_id = "base_link"
#         pose.header.stamp = temp_node.get_clock().now().to_msg()
#         pose.pose.position.x = pos[0]
#         pose.pose.position.y = pos[1]
#         pose.pose.position.z = pos[2]
#         pose.pose.orientation.x = pos[3]
#         pose.pose.orientation.y = pos[4]
#         pose.pose.orientation.z = pos[5]
#         pose.pose.orientation.w = pos[6]
#         joints = node.xyzijkw_to_joints(pose)
#         if joints:
#             joint_positions.append(joints)
#         else:
#             print(f"[{idx+1}/{len(positions)}] Fallo IK en punto: {pos}")

#     node.destroy_node()
#     temp_node.destroy_node()
#     rclpy.shutdown()
#     write_joints_csv(joint_positions, output_csv)
#     print(f"\nGuardado {len(joint_positions)} puntos de {len(positions)} en {output_csv}")

# if __name__ == '__main__':
#     main()
