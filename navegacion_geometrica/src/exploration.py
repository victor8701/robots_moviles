#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from random import randint
#import numpy as np
#import cv2

def map_callback(map_msg):
    print('Mapa recibido')
    global map_data
    map_data = map_msg

def select_and_publish_goal():
    global map_data
    if map_data is not None:

        # mapa = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))
        # cv_image = mapa.astype(np.uint8)
        # # Create a custom colormap
        # colormap = np.zeros((256, 1, 3), dtype=np.uint8)
        # colormap[0] = [255, 255, 255]  # White for unknown cells (0)
        # colormap[100] = [0, 0, 0]  # Black for occupied cells (100)
        # colormap[-1] = [128, 128, 128]  # Gray for unoccupied cells (-1)

        # # Apply the custom colormap to the grayscale image
        # cv_image_colored = cv2.applyColorMap(cv_image, colormap)
        
        # cv2.imshow('Color image', cv_image_colored)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    
        print('Eligiendo destino...')
        width = map_data.info.width
        height = map_data.info.height

        goalFound = False
        while not goalFound:
            random_x = randint(0, width - 1)
            random_y = randint(0, height - 1)
            index = random_y * width + random_x

            if map_data.data[index] == 0:
                goalFound = True

        print('Destino elegido. Navegando hasta el punto...')

        goal_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        goal_client.wait_for_server()
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = random_x * map_data.info.resolution + map_data.info.origin.position.x
        goal.target_pose.pose.position.y = random_y * map_data.info.resolution + map_data.info.origin.position.y
        goal.target_pose.pose.orientation.w = 1.0

        goal_client.send_goal(goal)
        wait = goal_client.wait_for_result()
        print('Punto alcanzado!')
    
    map_data = None


if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True)
        map_data = None
        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback)
        
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            select_and_publish_goal()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration finished.")
