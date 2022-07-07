import rospy
import math
import tf
from roscco.msg import CanFrame
from derived_object_msgs.msg import ObjectWithCovarianceArray, ObjectWithCovariance
from visualization_msgs.msg import Marker, MarkerArray
import cantools
from std_msgs.msg import Float32

class CAN_SPEED():

    def __init__(self):
        self.vehicle_speed = 0

        self.db = cantools.db.load_file('chassis_kia_niro.dbc')
        self.db.messages

        rospy.init_node('can_frame_speed')
        rospy.Subscriber('/can_frame', CanFrame, self.canframe_callback)
        self.speed_pub = rospy.Publisher('/can_frame_speed', Float32, queue_size=10)
        self.angle_pub = rospy.Publisher('/can_frame_angle', Float32, queue_size=10)

        rospy.spin()

    def canframe_callback(self, data):

        if data.frame.can_id == 1200:
            wheel = self.db.decode_message(data.frame.can_id, data.frame.data)
            #print(wheel)
            lr = wheel['LEFT_REAR']
            rr = wheel['RIGHT_REAR']
            lf = wheel['LEFT_FRONT']
            rf = wheel['RIGHT_FRONT']
            speed = (lf + rf)/(2*3.6)
            self.speed_pub.publish(speed)

        elif data.frame.can_id == 688:
            angle = self.db.decode_message(data.frame.can_id, data.frame.data)
            currAngle = math.radians(angle['STEERING_WHEEL_ANGLE']/13.9)
            self.angle_pub.publish(currAngle)

if __name__ == "__main__":
    try:
        CAN_SPEED()
    except rospy.ROSInterruptException:
        pass
