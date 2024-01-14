from SimpleHandEye.interfaces.ros2 import ROS2ExecutorManager, ROS2CameraReader, ROS2TFInterface
import rclpy
import time
rclpy.init()    
executor_manager = ROS2ExecutorManager()
vicon_sensor = ROS2TFInterface('vicon/B1_BODY/B1_BODY', 'vicon/CALIB_BOARD/CALIB_BOARD', node_name='vicon_sensor_node')
executor_manager.add_node(vicon_sensor)
executor_manager.start()

while rclpy.ok():
    try:
        T = vicon_sensor.get_pose()
        if T is not None:
            print(T[0:3,-1])
        time.sleep(0.01)
    except KeyboardInterrupt:
        executor_manager.close()
        break

