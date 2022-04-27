#!/usr/bin/env python

# Subscribe to the odom topic to get the turtlebot odometry readings  
import rospy
from nav_msgs.msg import Odometry
    
def callback(data):
    print("Hello")
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
    try:
        # Initialise a new node
        rospy.init_node('odom_node')

        # Change the spin rate to 1 Hz which is ~1 second
        rate = rospy.Rate(1)

        # Continuous loop while ROS is running
        while not rospy.is_shutdown():

            # Subscribe to published images and camera intrinsic parameters
            rospy.Subscriber("/odom", Odometry, callback)
            #rospy.Subscriber("/raspicam_node/camera_info", CameraInfo, cam_param_callback)
            
            #print("\n\n HERE IN MAIN\n\n")

            # Sleep until next spin sduycbs
            rate.sleep()

    except rospy.ROSInterruptException:
        # exit if there is any interruption from ROS
        return
    except KeyboardInterrupt:
        # exit if there is any interrupt from the keyboard (e.g. Ctrl+c)
        return
   
if __name__ == '__main__':
    main()