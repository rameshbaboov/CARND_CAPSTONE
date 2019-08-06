#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import TLStatus
import numpy as np
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
PUBLISHING_RATE = 50
MAX_SPEED_METERS_PER_SEC = 40*0.447
K_CONSTANT = MAX_SPEED_METERS_PER_SEC / LOOKAHEAD_WPS

class WaypointUpdater(object):
################################################################################################
#                               CLASS INITIATOR AND LOOPS
################################################################################################
    def __init__(self):
        rospy.init_node('waypoint_updater')
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        sub1 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub2 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub3 = rospy.Subscriber('/traffic_waypoint',TLStatus,self.traffic_cb)      
        sub4 = rospy.Subscriber('/current_velocity',TwistStamped,self.velocity_cb)
        rospy.loginfo('all modules subscirbed')
	self.final_waypoints = rospy.Publisher('final_waypoints', Lane, queue_size=1)
	rospy.loginfo("waypoint publishig initialized")
        # TODO: Add other member variables you need below
        # Waypoints
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
                
        # Traffic light related variables
        self.next_light_wp = -1
        self.next_light_state = -1  #0-red,1-yellow;2-green; -1 - no light
     
        # other variables
        self.current_vel = 0.0
        self.sim_testing = bool(rospy.get_param("~sim_testing", True))
        rospy.loginfo("all variables initiaized")
        self.loop()

    def loop(self):
        rate = rospy.Rate(PUBLISHING_RATE)
        while not rospy.is_shutdown():
        # do not start the loop unless all call back functions run and have the required data 
            rospy.loginfo("loop start")
 	    ros_start_time = rospy.get_time()
            if self.pose and self.base_waypoints and self.waypoints_tree and self.waypoints_2d:
                closest_waypoints_idx = self.get_closest_waypoint_idx()
                rospy.loginfo("Closest waypoint index is %s",closest_waypoints_idx)
		self.publish_waypoints(closest_waypoints_idx)
	    else:
		rospy.loginfo("some of the variables are not set by call back function")
            ros_end_time = rospy.get_time()
            rospy.loginfo("ros start time is %s",ros_start_time)
            rospy.loginfo("ros end time is %s",ros_end_time)
            rospy.loginfo("ros duratio is %s",ros_end_time - ros_start_time)
            rate.sleep()
################################################################################################
#                               CALL BACK FUNCTIONS
################################################################################################

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
	rospy.loginfo('self.pose received and is  %s ', self.pose.pose.position.x)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        if self.base_waypoints:
	   sub1.unregister()
	self.base_waypoints = waypoints
        if not self.waypoints_2d:
           self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
           self.waypoints_tree = KDTree(self.waypoints_2d)
        

    def traffic_cb(self, tl_status):
        # TODO: Callback for /traffic_waypoint message. Implement
	if self.pose:
	   self.next_light_wp = tl_status.waypoint
           self.next_light_state = tl_status.state
	   closest_waypoints_idx = self.get_closest_waypoint_idx()
	   self.publish_waypoints(closest_waypoints_idx)
        #rospy.loginfo('traffic_cb is called')
        pass
    
    def velocity_cb(self,msg):
        self.current_vel = msg.twist.linear.x
        # if self.current_vel > MAX_SPEED_METERS_PER_SEC:
        #   LOOKAHEAD_WPS = 20
        # else:
        #     LOOKAHEAD_WPS = 50
            

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
################################################################################################
#                               MAIN FUNCTIONS
################################################################################################
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoints_tree.query([x,y],1)[1]

        #check if closest waypoint is ahead or behind
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        
        val = np.dot(cl_vect - prev_vect,pos_vect - cl_vect)
        
        if val > 0 :
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx
            
    def publish_waypoints(self,closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        temp_waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        lane.header.frame_id = '/world'
        #lane.header.stamp = rospy.time(0)
        #rospy.loginfo('next light waypoint is %s',self.next_light_wp)
        #rospy.loginfo('next light waypoint status is %s',self.next_light_state)
        
        if (self.next_light_wp == -1 or self.next_light_wp >= closest_idx + LOOKAHEAD_WPS):
            #rospy.loginfo('there is no light ahead')
            lane.waypoints = self.set_velocity(temp_waypoints,1.)
        else:
            if (self.next_light_state == 0 or self.next_light_state == 1):
                lane.waypoints= self.decelerate_waypoints(temp_waypoints,closest_idx)
                #rospy.loginfo('there is a red/yellow light ahead')
            elif (self.next_light_state == 2):
                lane.waypoints = self.set_velocity(temp_waypoints, 0.8)
                #rospy.loginfo('there is a green light ahead') 
            else:
                #rospy.loginfo('traffic light status is unknown')
                lane.waypoints = self.set_velocity(temp_waypoints,1.)
            
        self.final_waypoints.publish(lane)
      
    def decelerate_waypoints(self,waypoints,closest_idx):
        temp = []
        stop_idx = max(self.next_light_wp - closest_idx -5,0)
        #rospy.loginfo("stop index is %s",stop_idx)
        #waypoints = self.set_velocity(waypoints,0.)
	#rospy.loginfo("i is %s", i)
	#rospy.loginfo("stop_idx is %s", stop_idx)
        #stop index reached. stop the vehicle.
        for i , wp in enumerate(waypoints):
            #waypoint_till_traffic_light  =  stop_idx - i 
            #rospy.loginfo("waypoint till traffic is %s", waypoint_till_traffic_light)
	    #rospy.loginfo(" i is %s" , i)
	    p = Waypoint()
            p.pose = wp.pose
            dist = self.distance(waypoints,i,stop_idx)
            #rospy.loginfo("i is less than stop index")
            #rospy.loginfo("dist is %s",dist)
            vel = K_CONSTANT * dist
            if vel < 1.:
               vel = 0.
            p.twist.twist.linear.x = min(vel,wp.twist.twist.linear.x)
            #rospy.loginfo("final velocity is %s", p.twist.twist.linear.x)
            temp.append(p)
        return temp
        
    def set_velocity(self,waypoints,reduction_rate):
        temp = []
        for i , wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            p.twist.twist.linear.x = MAX_SPEED_METERS_PER_SEC * reduction_rate
            temp.append(p)
        return temp
################################################################################################
#                               OTHER FUNCTIONS
################################################################################################
    def distance(self, waypoints, wp1, wp2):
        dist = 0       
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            if (i >= LOOKAHEAD_WPS):
		break
	    dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            #rospy.loginfo("wp1 is %s", wp1)
	    #rospy.loginfo("wp2 is %s",wp2)
	    #rospy.loginfo("parm1 is %s", waypoints[wp1].pose.pose.position)
	    #rospy.loginfo("i is %s", i)
	    #rospy.loginfo("parm2 is %s", waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
