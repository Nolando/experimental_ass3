 #!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray

# Run this node when your map is complete
# it will generate a text file with the landmark poses

def callback(data):
    #open a new file where the map is going to be stored
    f = open("map_slam.txt", "w")

    #reads all the landmarks in the map
    for marker in data.markers: 
        line = 'POINT2D %s %.3f %.3f \n' % (marker.id, marker.pose.position.x, marker.pose.position.y)
        f.write(line)
        
    #closes the text file and exits
    f.close()
    rospy.signal_shutdown('Map written in map_slam.txt')

def writer():

    rospy.init_node('map_writer') 
    rospy.Subscriber('/landmarks_state', MarkerArray, callback)
    rospy.spin()

if __name__ == '__main__':
    writer()