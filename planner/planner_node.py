#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

from visualization_msgs.msg import MarkerArray,Marker
from tf2_msgs.msg import TFMessage
import numpy as np
import json
import time
# import configparser

import graph_ltpl

# ----------------------------------------------------------------------------------------------------------------------
# IMPORT (should not change) -------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# top level path (module directory)
# import os
# import sys
# toppath = os.path.dirname(os.path.realpath(__file__))
# sys.path.append(toppath)

# track_param = configparser.ConfigParser()
# if not track_param.read(toppath + "/params/driving_task.ini"):
#     raise ValueError('Specified online parameter config file does not exist or is empty!')
toppath = "/sim_ws/src/planner"
track_specifier ="monteblanco"

# define all relevant paths
path_dict = {'globtraj_input_path': toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + track_specifier + ".csv",
             'graph_store_path': toppath + "/output/stored_graph.pckl",
             'ltpl_offline_param_path': toppath + "/params/ltpl_config_offline.ini",
             'ltpl_online_param_path': toppath + "/params/ltpl_config_online.ini"
             }





CAROFFSET = 0.3
L =2
KP = 0.3
class Quaternion():
    def __init__(self):
        x=0
        y=0
        z=0
        w=0

class EulerAngles():
    roll=0
    pitch=0
    yaw=0

def yaw2quaternion(yaw):
    cr = 1
    sr = 0
    cp = 1 #cos(pitch * 0.5);
    sp = 0 #sin(pitch * 0.5);
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    q = Quaternion()

    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    # print("{%.2f},{},{},{}".format(q.w,q.x,q.y,q.z))
    return q#[q.w,q.x,q.y,q.z]

def quaternion2yaw(q):
    # // roll (x-axis rotation)
    # double sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    # double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    # angles.roll = std::atan2(sinr_cosp, cosr_cosp)

    # // pitch (y-axis rotation)
    # double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
    # double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
    # angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2

    #  yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp).item()
    return yaw




class Planner(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('planner_node')
        # TODO: create ROS subscribers and publishers
        self.publisher_marker = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        # self.scan_sub = self.create_subscription(TFMessage,'tf',self.pose_callback_sim,10)
        # self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.L = L
        # self.wypts = np.loadtxt(FILENAME,delimiter=",", dtype=str)
        # self.get_logger().info("waypoints size=%d"%self.wypts.shape[0])
        # self.markerMsg = MarkerArray()
        self.kp = KP
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        
        # ----------------------------------------------------------------------------------------------------------------------
        # INITIALIZATION AND OFFLINE PART --------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------

        # intialize graph_ltpl-class
        self.ltpl_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=path_dict,
                                                    visual_mode=False,
                                                    log_to_file=False)

        # calculate offline graph
        self.ltpl_obj.graph_init()

        # set start pose based on first point in provided reference-line
        refline = graph_ltpl.imp_global_traj.src.\
            import_globtraj_csv.import_globtraj_csv(import_path=path_dict['globtraj_input_path'])[0]
        self.pos_est = refline[0, :]
        heading_est = np.arctan2(np.diff(refline[0:2, 1]), np.diff(refline[0:2, 0])) - np.pi / 2
        self.vel_est = 0.0

        # set start pos
        self.ltpl_obj.set_startpos(pos_est=self.pos_est,
                            heading_est=heading_est)
        self.traj_set = {'straight': None}
        self.tic = time.time()


    # def visualizeWaypoints(self):
    #     msg = MarkerArray()
    #     id = 0
    #     for wypt in  self.wypts:
    #         # self.get_logger().info("visualizeWaypoints")
    #         timestamp= self.get_clock().now().to_msg()
    #         marker = waypoint2Marker(wypt,timestamp,id)
    #         id+=1
    #         msg.markers.append(marker)
    #     self.publisher_marker.publish(msg)  

    """
    @param waypoint : a waypoint in map frame
    @param transform: map/base_link tf info(object of geometry_msgs)
    return : [x,y,yaw] waypoints location in car base_link frame 
    """
    # def target_transform(self,waypoint,transform,display=False):
    #     wypt_x = waypoint[0].astype(np.float64)
    #     wypt_y = waypoint[1].astype(np.float64)
    #     wypt_thta=waypoint[2].astype(np.float64)
    #     base_x = transform.translation.x
    #     base_y = transform.translation.y

    #     q_base= Quaternion()
    #     q_base.x = transform.rotation.x
    #     q_base.y = transform.rotation.y
    #     q_base.z = transform.rotation.z
    #     q_base.w = transform.rotation.w
    #     # distance = np.sqrt(np.square(wypt_x-base_x)+np.square(wypt_y-base_y))
    #     yaw = quaternion2yaw(q_base)
    #     # self.get_logger().info("wypt pos:%.2f,%.2f,%.3f"%(wypt_x,wypt_y,wypt_thta/np.pi*180))
    #     x_old = wypt_x-base_x
    #     y_old = wypt_y-base_y
    #     rotate_angle = -yaw
    #     x_new = x_old*np.cos(rotate_angle)-y_old*np.sin(rotate_angle)
    #     y_new = x_old*np.sin(rotate_angle)+y_old*np.cos(rotate_angle)

    #     if display==True:
    #         msg = MarkerArray()
    #         msg.markers.append(
    #         waypoint2Marker2([x_new,y_new,rotate_angle],
    #                         self.get_clock().now().to_msg(),
    #                         0,
    #                         "car",
    #                         1.0,
    #                         0.0,
    #                         0.0)
    #         )
    #         self.publisher_marker.publish(msg)
    #     return [x_new,y_new,(wypt_thta-yaw).astype(float)]

    """
    
    @param translation  : geometry_msgs/Vector3
    @param rotation     : geometry_msgs/Quaternion  
    return 
    """
    # def find_current_waypoint(self,transform):
    #     # self.get_logger().info("wypt[0][2]=%s"%self.wypts[0][2])

    #     car_x = transform.translation.x
    #     car_y = transform.translation.y

    #     # q_base= Quaternion()
    #     # q_base.x = transform.rotation.x
    #     # q_base.y = transform.rotation.y
    #     # q_base.z = transform.rotation.z
    #     # q_base.w = transform.rotation.w
    #     # yaw = quaternion2yaw(q_base)
    #     # self.get_logger().info("car  pos:%.2f,%.2f,%.3f"%(car_x,car_y,yaw/np.pi*180))

    #     np_square_distance =np.square(car_x-self.wypts[:,0].astype(np.float64))+np.square(car_y-self.wypts[:,1].astype(np.float64))
    #     square_L= self.L*self.L
    #     tmp = self.wypts[np_square_distance<=square_L]
    #     if tmp.shape[0] ==0:    # if no waypoint inside L distance, return 
    #         return None

    #     # possible_wypt=self.wypts[np_square_distance<=square_L][index]  # the possible wypts, within a circle with r=L, 
    #                                                                     # , the farthest point is the possible_wypt
    #     possible_wypt = None
    #     max_distance_sq = 0
    #     for wypt in tmp:
    #         [x,y,theta]=self.target_transform(wypt,transform)
    #         if x<0+CAROFFSET: # ignore all points behind the car 
    #             continue
    #         if x*x+y*y>max_distance_sq:
    #             possible_wypt=[x,y,theta]
    #             max_distance_sq=x*x+y*y
            
    #     if possible_wypt is not None:
    #         msg = MarkerArray()
    #         msg.markers.append(
    #         waypoint2Marker3([possible_wypt[0],possible_wypt[1],possible_wypt[2]],
    #                         self.get_clock().now().to_msg(),
    #                         0,
    #                         "car",
    #                         0.0,
    #                         1.0,
    #                         0.0)
    #         )
    #         self.publisher_marker.publish(msg)
    #         # self.get_logger().info("wypts:%.2f,%.2f"%(float(possible_wypt[0]),float(possible_wypt[1])))
    #         return possible_wypt
    #     return None
    


    # def pose_callback_sim(self, pose_msg):
    #     self.visualizeWaypoints()

    #     # TODO: find the current waypoint to track using methods mentioned in lecture
    #     waypoint = None
    #     base_link_tf_info=None
    #     for tf in pose_msg.transforms:
    #         if tf.header.frame_id=="map":
    #             waypoint = self.find_current_waypoint(tf.transform)
    #             base_link_tf_info=tf.transform
    #     if waypoint is None:
    #         return
        
    #     # self.get_logger().info("wypt pos:%.2f,%.2f,%.3f"%(waypoint[0],waypoint[1],waypoint[2]/np.pi*180))

    #     # TODO: transform goal point to vehicle frame of reference
    #     # basept=self.target_transform(waypoint,base_link_tf_info,False)
    #     distance_sq =(np.square(waypoint[0])+np.square(waypoint[1]))
    #     curvature = 2*abs(waypoint[1])/distance_sq
    #     # self.map_2_base_frame(waypoint,base_link_tf_info)

    #     # TODO: calculate curvature/steering angle

    #     # TODO: publish drive message, don't forget to limit the steering angle.
    #     drive_msg = AckermannDriveStamped()
    #     steering_angle = curvature*abs(waypoint[1])/waypoint[1]*self.kp
    #     drive_msg.drive.steering_angle =steering_angle 
    #     self.get_logger().info("steering angle=%.3f"%steering_angle)
    #     self.get_logger().info("\n")
    #     drive_msg.drive.speed = 2.0
    #     self.drive_pub.publish(drive_msg)

    def timer_callback(self):
        for sel_action in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
            if sel_action in self.traj_set.keys():
                break

        # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
        self.ltpl_obj.calc_paths(prev_action_id=sel_action,
                            object_list=[])

        # -- GET POSITION AND VELOCITY ESTIMATE OF EGO-VEHICLE -------------------------------------------------------------
        # (here: simulation dummy, replace with actual sensor readings)
        if self.traj_set[sel_action] is not None:
            self.pos_est, self.vel_est = graph_ltpl.testing_tools.src.vdc_dummy.\
                vdc_dummy(pos_est=self.pos_est,
                        last_s_course=(self.traj_set[sel_action][0][:, 0]),
                        last_path=(self.traj_set[sel_action][0][:, 1:3]),
                        last_vel_course=(self.traj_set[sel_action][0][:, 5]),
                        iter_time=time.time() - self.tic)
        self.tic = time.time()

        # -- CALCULATE VELOCITY PROFILE AND RETRIEVE TRAJECTORIES ----------------------------------------------------------
        # pos_est:[x, y]
        # vel_est:float
        self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.pos_est,
                                            vel_est=self.vel_est)[0]
        print(self.traj_set.keys())
        
        #[s, x, y, heading, curvature, vx, ax]
        print(self.traj_set["straight"][0][0])
        
        # print(traj_set)
        # -- LIVE PLOT (if activated) --------------------------------------------------------------------------------------
        self.ltpl_obj.visual()


def main(args=None):
    rclpy.init(args=args)
    print("planner Initialized")
    planner_node = Planner()
    # for i in range(0,1):
    #     planner_node.visualizeWaypoints()
    rclpy.spin(planner_node)

    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
