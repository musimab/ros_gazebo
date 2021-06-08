#!/usr/bin/env python

import rospy
import math
import time

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np
from numpy.random import randn
import matplotlib.pyplot as plt 


class myTurtleBot():
    def __init__(self):
        
        #Initialize odometry and imu callback with Odometry and Imu messageses
        self.odom_sb = rospy.Subscriber('/odom', Odometry, self.odom_Callback)
        self.imu_sb = rospy.Subscriber("/imu", Imu, self.imu_Callback)
        
        #self.laserScan_sb = rospy.Subscriber('/scan',LaserScan, self.laser_Callback)
        self.laserScan_sb = rospy.Subscriber('/scan',LaserScan, self.findDistanceBearing)

        self.velocity_pb = rospy.Publisher('/cmd_vel', Twist,queue_size=10)

        self.turtlebot_odom_pose = Odometry()
        self.range_distance_0 = 0
        self.range_value = 0
        self.range_index = 0

        self.zs =[] # Measurement
        self.pr = [] #Prior
        self.filter = [] # filtered list
        self.true_measurement =[]



    def odom_Callback(self,pose_message):

        # geometry_msgs/Pose pose
        self.turtlebot_odom_pose.pose.pose.position.x = pose_message.pose.pose.position.x
        self.turtlebot_odom_pose.pose.pose.position.y = pose_message.pose.pose.position.y
        self.turtlebot_odom_pose.pose.pose.position.z  = pose_message.pose.pose.position.z

        self.add_gaussian_noise(self.turtlebot_odom_pose.pose.pose.position.x, 2)

        self.turtlebot_odom_pose.pose.pose.orientation.w=pose_message.pose.pose.orientation.w
        self.turtlebot_odom_pose.pose.pose.orientation.x=pose_message.pose.pose.orientation.x
        self.turtlebot_odom_pose.pose.pose.orientation.y=pose_message.pose.pose.orientation.y
        self.turtlebot_odom_pose.pose.pose.orientation.z=pose_message.pose.pose.orientation.z
        """
        rospy.loginfo(" x:" + str(self.turtlebot_odom_pose.pose.pose.position.x ) + " y:" +
                      str(self.turtlebot_odom_pose.pose.pose.position.y) +"z:"+
                       str(self.turtlebot_odom_pose.pose.pose.position.z))
        rospy.loginfo("theta : " + str (self.turtlebot_odom_pose.pose.pose.orientation.z))
        """

    def imu_Callback(self,imu_message):
        #print(imu_message)
        #geometry_msgs/Quaternion 
        orientation_x = imu_message.orientation.x
        orientation_y = imu_message.orientation.y
        orientation_z = imu_message.orientation.z


    def findDistanceBearing(self,scan_data):
        self.range_value, self.range_index = self.min_range_index(scan_data.ranges)
        #print("distance {:.3f}   index  {:.3f}".format(self.range_value, self.range_index))
        

    def laser_Callback(self,msg):
        
        self.range_distance_0 = msg.ranges[0]

    def min_range_index(self,ranges):
        ranges = [x for x in ranges if not math.isnan(x)]
        if (len(ranges)!=0):
            return (min(ranges), ranges.index(min(ranges)) )
        else:
            return 0.1

    #find the max range 
    def max_range_index(self,ranges):
        ranges = [x for x in ranges if not math.isnan(x)]
        if (len(ranges)!=0):
            return (max(ranges), ranges.index(max(ranges)))
        else:
            return 4.0
    
    def rotate (self,angular_speed_degree, target_angle_degree, clockwise):

        velocity_message = Twist()
        # Resrt all linear and angular velocities
        velocity_message.linear.x=0
        velocity_message.linear.y=0
        velocity_message.linear.z=0
        velocity_message.angular.x=0
        velocity_message.angular.y=0
        velocity_message.angular.z=0

        #Get the initial orientation of the robot
        theta0 = self.turtlebot_odom_pose.pose.pose.orientation.z

        #Convert the degree to radian
        angular_speed = math.radians(abs(angular_speed_degree))

        if (clockwise):
            velocity_message.angular.z =-abs(angular_speed)
        else:
            velocity_message.angular.z =abs(angular_speed)

        angle_moved = 0.0
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    

        t0 = rospy.Time.now().to_sec()

        while True :
            rospy.loginfo("Turtlesim rotates")
            self.velocity_pb.publish(velocity_message)

            t1 = rospy.Time.now().to_sec()
            current_angle_degree = (t1-t0)*angular_speed
            loop_rate.sleep()
                        
            if  (current_angle_degree > target_angle_degree):
                rospy.loginfo("reached")
                break

        #finally, stop the robot when the distance is moved
        velocity_message.angular.z =0
        self.velocity_pb.publish(velocity_message)

    def go_to_goal(self,x_goal, y_goal):

        velocity_message = Twist()

        while not rospy.is_shutdown():
            K_linear = 0.08
            distance = abs(math.sqrt(((x_goal-self.turtlebot_odom_pose.pose.pose.position.x) ** 2) + ((y_goal-self.turtlebot_odom_pose.pose.pose.position.y) ** 2)))

            linear_speed = distance * K_linear
            
            if linear_speed>0.3:
                linear_speed=0.3
            if linear_speed <-0.3:
                linear_speed = -0.3
            

            K_angular = 0.2
            desired_angle_goal = math.atan2(y_goal-self.turtlebot_odom_pose.pose.pose.position.y, x_goal-self.turtlebot_odom_pose.pose.pose.position.x)
            angular_speed = (desired_angle_goal-self.turtlebot_odom_pose.pose.pose.orientation.z)*K_angular

            if angular_speed > 0.125:
                angular_speed = 0.125
            if angular_speed <-0.125:
                angular_speed = -0.125
            
            #rospy.loginfo('linear_speed:' + str(linear_speed) + 'angular_speed:' + str(angular_speed))
            velocity_message.linear.x = linear_speed
            velocity_message.angular.z = angular_speed

            self.velocity_pb.publish(velocity_message)

            if (distance <0.01):
                break
        velocity_message.linear.x = 0
        velocity_message.angular.z = 0
        self.velocity_pb.publish(velocity_message)

    def move_circular(self):
        
        vel_msg = Twist()
        wk = -0.2
        rk = 0
    
        while(not rospy.is_shutdown()):
            rk=rk+0.001
            vel_msg.linear.x =rk
            vel_msg.linear.y =0
            vel_msg.linear.z =0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z =wk
            if rk >0.2:
                rk =0.2

            print("obstacle distance:", self.range_distance_0)
            if self.range_distance_0 < 0.8:
                vel_msg.linear.x = 0
                vel_msg.angular.z = -0.3
        
            self.velocity_pb.publish(vel_msg)

    
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_pb.publish(vel_msg)

    def move(self,speed,distance,is_forward):
       
        velocity_message = Twist()
        
        # Reset all linear and angular velocities 
        
        velocity_message.linear.y = 0
        velocity_message.linear.z = 0

        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z = 0

        # Initial position of the robot with standart deviation : 1
        x0 =  self.turtlebot_odom_pose.pose.pose.position.x
        x0 =  self.add_gaussian_noise(x0, 1)
        y0 =  self.turtlebot_odom_pose.pose.pose.position.y
        y0 =  self.add_gaussian_noise(y0, 1)

        #  Adding Gaussian noise to speed std_002
        speed = self.add_gaussian_noise(speed, 0.02)

        if(is_forward):
            velocity_message.linear.x = abs(speed)
        else:
            velocity_message.linear.x = -abs(speed)
        
        measured_distance  =0.0
        loop_rate = rospy.Rate(40)

        # Initial position of the robot
        pos =self.predict(pos=x0, vel=2)
        
        while True:
           
            self.velocity_pb.publish(velocity_message)
            loop_rate.sleep()

            # Adding Gaussian Noise to Measurement
            measured_distance = abs(self.turtlebot_odom_pose.pose.pose.position.x-x0)
            measured_distance  = self.add_gaussian_noise(measured_distance , 5) #5
            
            # Find the prior belief
            x_prior = self.predict(pos,0.2)

            # Update the new pos based on prior belief
            pos = self.update(x_prior,measured_distance)
            
            # Append the results for plotting
            self.pr.append(x_prior)
            self.zs.append(measured_distance)
            self.filter.append(pos)
            self.true_measurement.append(self.turtlebot_odom_pose.pose.pose.position.x)
           
            print("Actual Position: {:.3f}  Measurement : {:.3f} KalmanFilter: {:.3f} Prediction: {:.3f}".
                       format(self.turtlebot_odom_pose.pose.pose.position.x,measured_distance,pos,x_prior))
                            
            if not (x_prior < distance):
                rospy.loginfo("reached")
                break
        
        velocity_message.linear.x = 0
        self.velocity_pb.publish(velocity_message)
        
        print("Measurement_variance : {:.3f} Filter_variance: {:.3f}".format(np.var(self.zs), np.var(self.filter)))

        # Plot the position with measurement and filtered results
        plt.figure()
        plt.plot(self.zs, c='k', marker='o', linestyle='', label='measurement')
        plt.plot(self.filter, c='#004080', alpha=0.9, label='Kalman_filter')
        plt.plot(self.true_measurement, c='r', alpha=0.7, label='Actual Position')
        plt.legend(loc=4)
        plt.show()

    
    def add_gaussian_noise(self, data, std_dev):

        # Add gaussian noise with std_deviation
        return randn()*std_dev + data
    
    def predict(self, pos, vel):
        dt = 1 # Time scale for the movement
        return pos + vel*dt # Prior + initial position


    def update(self, x, z):
        K = .3062  # Kalman_Gain = Prior_var/(Prior_var + Measurement_var)
        y = z-x    # Residual
        x = x+ K*y # Posterior
        return x 

        

if __name__ == '__main__':
        
    try:
        # init a new node and give it a name
        rospy.init_node('crazy_turtle', anonymous=True) 
        robot1 = myTurtleBot()
        time.sleep(1)
        robot1.move(speed=0.2,distance=20.,is_forward=True)
        rospy.spin()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
    

    
