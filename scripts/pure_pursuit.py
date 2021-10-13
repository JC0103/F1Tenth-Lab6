#!/usr/bin/env python3
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Vector3
import numpy as np
import tf
from ackermann_msgs.msg import AckermannDriveStamped
from tf import transformations

rospack = rospkg.RosPack()
package = rospack.get_path('pure_pursuit')

class PurePursuit(object):
	"""
	The class that handles pure pursuit.
	"""
	def __init__(self):
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback, queue_size = 1)
		self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size = 1)
		self.marker_pub = rospy.Publisher('/mark', Marker, queue_size = 1)
		self.waypoints = np.genfromtxt(package+'/logs/wp-2021-10-13-11-18-23.csv', delimiter=',')[:, :2]
		self.l = 1
		self.n = 100
		self.listener = tf.TransformListener()

	def pose_callback(self, data):
		
		(trans,rot) = self.listener.lookupTransform('/base_link', '/map', data.header.stamp)
		rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
		matrix = np.zeros((4,4)) #4x4
		matrix[:3,:3] = rot
		matrix[:3, 3] = trans
		matrix[-1, -1] = 1 
		n = len(self.waypoints)
		ipt = np.zeros((4, n))
		ipt[:2, :] = self.waypoints.T
		ipt[3, :] = 1

		opt = matrix.dot(ipt)
		xy = opt[:2, :].T #transformed
		xy[xy[:,0]<0] = 10 #filter behind car

		distance = np.sum(xy**2, axis=1)
		idx = np.argmin(np.absolute(distance-self.l**2))
		goal_x, goal_y = xy[idx]
		print(goal_y)
		self.marker(goal_x, goal_y)

		steering_angle = 2 * goal_y / self.l ** 2
		if -np.pi/18 < steering_angle < np.pi/18:
			velocity = 3
		elif -np.pi/9 < steering_angle <= -np.pi/18 or np.pi/18 <= steering_angle < np.pi/9:
			velocity = 2
		else:
			velocity = 1
		drive_msg = AckermannDriveStamped()
		drive_msg.header.stamp = rospy.Time.now()
		drive_msg.header.frame_id = "laser"
		drive_msg.drive.steering_angle = steering_angle
		drive_msg.drive.speed = velocity
		self.drive_pub.publish(drive_msg)					
	
	def marker(self, goal_x, goal_y):
		marker = Marker()
		marker.header.frame_id = "base_link"
		marker.header.stamp = rospy.Time()
		marker.type = 2
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
		s = Vector3()
		s.x = 0.5
		s.y = 0.5
		s.z = 0.5
		marker.scale = s
		colour = ColorRGBA()
		colour.r = 255
		colour.g = 0
		colour.b = 0
		colour.a = 1
		marker.color = colour
		marker.lifetime.secs = 0
		marker.frame_locked = True
		self.marker_pub.publish(marker)

def main():
	rospy.init_node('pure_pursuit_node')
	pp = PurePursuit()
	rospy.spin()
if __name__ == '__main__':
	main()
