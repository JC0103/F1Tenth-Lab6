from visualization_msgs.msg import Marker
import rospy
from geometry_msgs.msg import Pose, Vector3, Point
from std_msgs.msg import ColorRGBA

def plot_marker(publisher, goal_x, goal_y):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time()
    marker.type = 0 #shape
    marker.action = 0

    pos = Pose()
    pos.position.x  = goal_x
    pos.position.y = goal_y
    pos.position.z = 0
    pos.orientation.x = 0
    pos.orientation.y = 0
    pos.orientation.z = 0
    pos.orientation.w = 1
    marker.pose = pos

    s = Vector3() #scale
    s.x = 0.4
    s.y = 0.4
    s.z = 0.4
    marker.scale = s

    colour = ColorRGBA() #color
    colour.r = 1
    colour.g = 0
    colour.b = 0
    colour.a = 1
    marker.color = colour

    marker.lifetime.secs = 0
    marker.frame_locked = True
    publisher.publish(marker)
    
def plot_track(publisher, data):
		line = Marker()
		line.header.frame_id = "map"
		line.header.stamp = rospy.Time()
		line.type = 4 #shape
		line.action = 0

		s = Vector3() #scale
		s.x = 0.1
		line.scale = s

		line.lifetime.secs = 0
		line.frame_locked = True

		start = data[0].copy() # connect back to start point
		for point in data:
				pos = Point()
				pos.x = point[0]
				pos.y = point[1]
				pos.z = 0
				line.points.append(pos)
		pos = Point()
		pos.x = start[0]
		pos.y = start[1]
		pos.z = 0
		line.points.append(pos)
		line.pose.orientation.w = 1

		for i in range(len(line.points)):
			colour = ColorRGBA() #color
			colour.r = 0
			colour.g = 0
			colour.b = 1
			colour.a = 1
			line.colors.append(colour)
		  
		publisher.publish(line)




