#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
#http://wiki.ros.org/tf/Overview/Transformations
#http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms
#to create an offset we can use the following(sec 8.1, 8.2):
from sensor_msgs.msg import Image 
from ar_track_alvar_msgs.msg import AlvarMarker
#from sensor_msgs.msg import JointState 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header



def xdisplay_pub(data):
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=100)
    #pub = rospy.Publisher('/robot_image', Image, latch=True, queue_size=10)
    pub.publish(data)
    
def select_tag(data):
    pub = rospy.Publisher('new_pose', Pose, latch=True, queue_size=10)
    in_pose = Pose()
    pub.publish(data)

    
def get_tag_pose():

    #subrate = rospy.Rate(1) # 10hz
    listener = tf.TransformListener()
    #initialize lists
    trans = []
    rot = []

    print "made it"
    #using time http://wiki.ros.org/rospy/Overview/Time
    now_time =rospy.Time.now()
    #create a time of 10 seconds = time_10 = rospy.Time(10)
    
    dt = 0
    try:
        #(trans[0],rot[0]) = listener.lookupTransform('right_hand_camera', '/ar_marker_1', rospy.Time(0))
        (trans[0],rot[0]) = listener.lookupTransform('/head_camera', '/ar_marker_1', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    '''
    try:
        (trans[1],rot[1]) = listener.lookupTransform('right_hand_camera', '/ar_marker_2', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    try:
        (trans[2],rot[2]) = listener.lookupTransform('right_hand_camera', '/ar_marker_3', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    try:
        (trans[3],rot[3]) = listener.lookupTransform('right_hand_camera', '/ar_marker_4', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    try:
        (trans[4],rot[4]) = listener.lookupTransform('right_hand_camera', '/ar_marker_5', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    try:
        (trans[5],rot[5]) = listener.lookupTransform('right_hand_camera', '/ar_marker_6', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    '''
        
    #for i in trans,rot:
        #run until both lists are full? or until some length paramet is satisfied?
    print trans
    #call select tag function
    #select_tag(trans,rot)
    #subrate.sleep()

    
def get_image():
    #img = Image()
    rospy.init_node('image_data', anonymous=False)
    rate = rospy.Rate(20) # 10hz
    #rostopic pub /robot/head/command_head_pan baxter_core_msgs/HeadPanCommand -- 0 6
    #time_now =rospy.Time.now()
    count = 0
    while not rospy.is_shutdown():
        #rospy.Subscriber("/usb_cam/image_raw", Image, xdisplay_pub)
        #rospy.subscriber("/cameras/right_hand_camera/image",Image, get_tag_pose)
        #if count%10 == 0:
            #rospy.Subscriber("/cameras/right_hand_camera/image", Image, xdisplay_pub)
        #rospy.Subscriber("/usb_cam/image_raw", Image, xdisplay_pub)
        count = count + 1
        #if count%2 == 0
        #    rospy.Subscriber("/cameras/right_hand_camera/image", Image, get_tag_pose)
        rospy.Subscriber("/usb_cam/image_raw", Image, get_tag_pose)
        rate.sleep()


def main_func():
    get_image()



# this shows you how to subscripe to images in ROS
#http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber

#open the head camera using the 
#rosrun baxter_tools camera_control.py --open head_camera --resolution 1280x800
#look at the baxter python API to get connection to camera
#http://sdk.rethinkrobotics.com/wiki/Hello_Baxter#Step_4:_Run_an_Example_Program

 
# he messages forma
#http://docs.ros.org/api/sensor_msgs/html/msg/CompressedImage.html
#rospy.Subscriber('/camerasleft_hand_camera/image', CompressedImage, ?,?)
#note that classes support two kinds of operations, attribute references and instantiation
#Alos classes can be assigned to variables
#how? if JointStates is a class (coming from sensor messages)then js = JointStates() assings this in the form 
#of a function
#js.header.stamp = rospy.Time.now() assigns the time given by rospy to the stamp attribute
# of js.


if __name__ == '__main__':
    
    main_func()
