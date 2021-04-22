import sys
#############
# old
# import rospy
#############
# new
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
#############
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState

class Plan(Node):
    def __init__(self):
        #################################################
        # old
        # roscpp_initialize(sys.argv)
        # rospy.init_node('moveit_py_demo', anonymous=True)
        #################################################
        # new
        rclpy.init()
        super().__init__('plan')
        self.qos_profile = QoSProfile(depth=10)
        #################################################
            
        robot = RobotCommander()

        #################################################
        # old
        # rospy.sleep(1)
        #################################################
        # new https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
        rate = self.create_rate(100) # 100Hz = 1s
        rate.sleep()
        #################################################
        
        print ("Current state:")
        print (robot.get_current_state())
        
        # plan to a random location 
        a = robot.right_arm
        a.set_start_state(RobotState())
        r = a.get_random_joint_values()
        print ("Planning to random joint position: ")
        print (r)
        p = a.plan(r)
        print ("Solution:")
        print (p)
    
        #################################################
        # old
        # roscpp_shutdown()
        #################################################
        # new
        rclpy.shutdown()
        #################################################


if __name__=='__main__':
    main()

def main():
    node = Plan()
