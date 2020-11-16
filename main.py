#!/usr/bin/env python

import rospy
import math
import time

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class myTurtleBot():
    def __init__(self):
        
        #Initialize odometry and imu callback with Odometry and Imu messageses
        self.odom_sb = rospy.Subscriber('/odom', Odometry, self.odom_Callback)
        self.imu_sb = rospy.Subscriber("/imu", Imu, self.imu_Callback)
        
        self.laserScan_sb = rospy.Subscriber('/scan',LaserScan, self.laser_Callback)


        self.velocity_pb = rospy.Publisher('/cmd_vel', Twist,queue_size=10)

        self.turtlebot_odom_pose = Odometry()
        self.range_distance_0 = 0
        



    def odom_Callback(self,pose_message):

        # geometry_msgs/Pose pose
        self.turtlebot_odom_pose.pose.pose.position.x = pose_message.pose.pose.position.x
        self.turtlebot_odom_pose.pose.pose.position.y = pose_message.pose.pose.position.y
        self.turtlebot_odom_pose.pose.pose.position.z  = pose_message.pose.pose.position.z

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
        

    def laser_Callback(self,msg):
        
        self.range_distance_0 = msg.ranges[0]
        #self.range_distance_0 , min_index = self.min_range_index(msg.ranges)

    def min_range_index(self,ranges):
        ranges = [x for x in ranges if not math.isnan(x)]
        if (len(ranges)!=0):
            return (min(ranges), ranges.index(min(ranges)) )
        else:
            return 0.1


    def move(self,speed,distance,is_forward):
       
        velocity_message = Twist()
        
        # Reset all linear and angular velocities 
        
        velocity_message.linear.y = 0
        velocity_message.linear.z = 0

        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z = 0

        # Initial position of the robot
        x0 =  self.turtlebot_odom_pose.pose.pose.position.x
        y0 =  self.turtlebot_odom_pose.pose.pose.position.y

        if(is_forward):
            velocity_message.linear.x = abs(speed)
        else:
            velocity_message.linear.x = -abs(speed)
        
        distance_moved =0.0
        loop_rate = rospy.Rate(10)

        while True:
            rospy.loginfo("Turtlesim moves forwards")
            self.velocity_pb.publish(velocity_message)
            #loop_rate.sleep()
            distance_moved = abs(math.sqrt(((self.turtlebot_odom_pose.pose.pose.position.x-x0)** 2) + ((self.turtlebot_odom_pose.pose.pose.position.y-y0)**2)))

            if not (distance_moved< distance):
                rospy.loginfo("reached")
                break
        
        velocity_message.linear.x = 0
        self.velocity_pb.publish(velocity_message)
    
    def rotate (self,angular_speed_degree, target_angle_degree, clockwise):

        velocity_message = Twist()

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

            if (distance <0.1):
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


if __name__ == '__main__':
        # init a new node and give it a name
    try:
        rospy.init_node('crazy_turtle', anonymous=True) 
        robot1 = myTurtleBot()
        time.sleep(1)
        #robot1.move(speed=0.5,distance=1.5,is_forward=True)

        #robot1.rotate(10,90,True)
        time.sleep(2)
        #robot1.go_to_goal(4,0)
        robot1.move_circular()
        #rospy.spin()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
    

    
