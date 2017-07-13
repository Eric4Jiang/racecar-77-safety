#!/usr/bin/python

import math
import rospy

from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class SafetyControllerNode:
    def __init__(self):
        # subscribe to incomming Ackermann drive commands and laserscan measurements
        rospy.Subscriber("ackermann_cmd_input", AckermannDriveStamped,
                         self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laserscan_callback)

        # if the racecar was stopped or not
        self.EMERGENCY_STOPPED = False;
        
        # publisher for the safe Ackermann drive command
        self.cmd_pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=10)
	
        # publish if racecar has stopped because it's too close
        self.pubHandle = rospy.Publisher("racecar/emergency_stop", Bool, queue_size =1)

    def ackermann_cmd_input_callback(self, ackermann_input):
        # Stop racecar movement
        if self.EMERGENCY_STOPPED:
	    ackermann_input.drive.speed = 0;
	    ackermann_input.drive.steering_angle = 0;
	    ackermann_input.header.stamp = rospy.Time.now();

        # republish the input as output (not exactly "safe")
        self.cmd_pub.publish(ackermann_input)

    # Check surroundings of racecar in certain areas and confirm no collisions
    # 
    # Arguments:
    #   *msg - distances that the laser scanner collected (METERS)
    # 
    # msg[index] = distance at angle theta, where theta = (index * incr) + msg.min_angle
    # e.g index = 500, incr = 0.025 rads, min_angle = -2 rads
    #   - theta = (500 * 0.025) +  -2
    #   - msg[500] = distance at angle theta (in meters)
    def laserscan_callback(self, LaserScan_msg):
        # Limit of how close robot can get to objects in meters
        DISTANCE_THRESHOLD = 0.1016; # 4 inches

        # Gives some tolerance to how many points can be too close in case of errors. 
        MAX_CLOSEPOINTS = 20;

        # Number of laser points pased from laser scanner (270 degree of surrounding)
        numOfIndices = length(LaserScan_msg.ranges);

        # Number of points that passed threshold
        self.close_points = [];

        # Range to check for objects
        searchRange = 100; # +/- searchRange 
        
        # Index of point directly in front of laser scanner
        middleIndex = numOfLasterPoints / 2;

        # For every point within the searchRange
        for i in range(middleIndex - searchRange, middleIndex + searchRange):
            distance = LaserScan_msg.ranges[i];
            angle = (i * LaserScan_msg.angle_increment) + LaserScane_msg.angle_min;

            # Too close 
            # if distance < DISTANCE_THRESHOLD / math.cos(angle): # test with negative radians
            if distance < DISTANCE_THRESHOLD:
                # Save point
                self.close_points.append(LaserScan_msg.range[i]);

        # Tell if robot should emergency stop
        if length(self.close_points) > MAX_CLOSEPOINTS:
            # STOP RACECAR
            self.pubHandle.publish(True);
        else:
            self.pubHandle.publish(False);

        print LaserScan_msg.angle_min, LaserScan_msg.angle_increment, LaserScan_msg.ranges[500];

if __name__ == "__main__":
    rospy.init_node("safety_controller")
    node = SafetyControllerNode()
    rospy.spin()
