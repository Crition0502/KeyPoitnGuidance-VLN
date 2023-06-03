#!/home/robot/anaconda3/envs/yolov5/bin/python3.8
import rospy
from yolo_bridge.yolo_bridge import Ros2Yolo
import sys
sys.path.insert(0,'/home/robot/ros_ws/src/ros-yolov5/model')
print(sys.path)
if __name__ == "__main__":
    yoloBridge = Ros2Yolo()
    rospy.spin()
