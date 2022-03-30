#!/usr/bin/env python
from multiprocessing.connection import Listener
import rospy
import tf2_ros
import tf
import numpy as np
import geometry_msgs.msg


# Initialize the node
rospy.init_node("base_link_tf_pub")
br = tf2_ros.TransformBroadcaster()
r = rospy.Rate(10)
tfBuffer = tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)


def extract_rotation(block):
    x,y,z,w=tf.transformations.quaternion_from_matrix(block)
    return x,y,z,w

def extract_translate(block):
    x=block[0,3]
    y=block[1,3]
    z=block[2,3]
    return x,y,z


def base_link_pub():
    while not rospy.is_shutdown():

        #<--------------Find the transform-------------->
        # try:
        #     transform_msg = tfBuffer.lookup_transform("world","left_cam", rospy.Time())
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     r.sleep()
        #     continue
        # rotation=transform_msg.transform.rotation #this is a quaternion 
        # rotation_as_array= np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        # translation = transform_msg.transform.translation
        # translation_as_array = np.array([[translation.x, translation.y, translation.z]]).T
        # l_wrt_w = tf.transformations.quaternion_matrix(rotation_as_array)
        # l_wrt_w[0:3,3:4]=translation_as_array


        #<--------------Define gtbase2----------->
        base2=geometry_msgs.msg.TransformStamped()
        base2.header.stamp = rospy.Time.now()
        base2.header.frame_id = "base_link"
        base2.child_frame_id ="laser"
        #<--------------Define our matricies----------->   
        # l_wrt_b=np.array([[1.0,0.0,0.0,-0.05],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])
        
        # l_wrt_w=np.dot(np.linalg.inv(l_wrt_w), l_wrt_b)
        # l_wrt_w=np.linalg.inv(l_wrt_w)

         #<--------------Set gtbase2 cam----------->
        
        # x,y,z=extract_translate(l_wrt_w)
        base2.transform.translation.x = 0
        base2.transform.translation.y = 0
        base2.transform.translation.z = 0
        # x,y,z,w=extract_rotation(l_wrt_w)
        base2.transform.rotation.x = 0
        base2.transform.rotation.y = 0
        base2.transform.rotation.z = 1
        base2.transform.rotation.w = 0

        br.sendTransform(base2)
        r.sleep()


if __name__ == '__main__':
    try:
        base_link_pub()
    except rospy.ROSInterruptException:
        pass