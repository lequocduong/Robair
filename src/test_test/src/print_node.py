import rospy
from std_msgs.msg import String

def fax_handler(data):
	rospy.loginfo(data.data)


def printer():
	rospy.init_node('printer',anonymous=True)
	rospy.Subcriber('fax_line',String, fax_handler)	
	rospy.spin()
	

if __name__ =="__main__":
	printer()
	
		
