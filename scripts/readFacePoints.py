#!/usr/bin/env python
import rospy
import rosbag
import math
import datetime

global frame
frame=0
bag = rosbag.Bag('/media/scosar/Windows/Users/scosar/Documents/DATASETS/facial_landmark_transformation_2_points.bag')
text_file = open("/media/scosar/Windows/Users/scosar/Documents/DATASETS/facial_landmark_transformation_2_points_thermal.txt", "w")
#text_file_2 = open("/media/scosar/Windows/Users/scosar/Documents/DATASETS/facial_landmark_transformation_2_points_thermal.txt", "w")
for topic, msg, t in bag.read_messages(topics=['/face_position_3d_thermal']):
    #print type(t)
    #print type(msg)
    if len(msg.data)>0:
        
        #print (datetime.datetime.fromtimestamp(t.secs).strftime("%Y-%m-%d %H:%M:%S"),t.nsecs,speed,status)
        print (msg.data)
        text_file.write("%d.%d,%f,%f,%f,%f,%f,%f\n" % (t.secs,t.nsecs,msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5]))        
bag.close()
text_file.close()
