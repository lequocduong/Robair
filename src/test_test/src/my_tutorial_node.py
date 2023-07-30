import rospy
from std_msgs.msg import String

def faxer():
	rospy.init_node('faxer',anonymous= True)
	pub = rospy.Publisher('fax_line',String,queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		message = "hello_world"
		pub.publish(message)
		rate.sleep()
	
if __name__ == "__main__":
	try:
		faxer()
	except rospy.ROSInterruptException:
		pass
		
		
