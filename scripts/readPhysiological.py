#!/usr/bin/env python
import rospy
import rosbag
import math
import datetime

global frame
frame=0
bag = rosbag.Bag('/media/scosar/Windows/Users/scosar/Documents/DATASETS/20161122_thermal_dataset/2016-11-22-14-39-46-Jaime_onlyphysiological.bag')
text_file = open("/media/scosar/Windows/Users/scosar/Documents/DATASETS/20161122_thermal_dataset/2016-11-22-14-39-46-Jaime_physiological_estimated.txt", "w")
for topic, msg, t in bag.read_messages(topics=['/physiologicalData']):
    #print (t.secs,len(msg.data))
    #print type(msg)
    if len(msg.data)>0:
        
        #print (datetime.datetime.fromtimestamp(t.secs).strftime("%Y-%m-%d %H:%M:%S"),t.nsecs,speed,status)
        #print (msg.data)
        text_file.write("%d,%d,%f,%f,%f" % (t.secs,t.nsecs,msg.data[0],msg.data[1],msg.data[2]))
        #print noofPeople

        text_file.write("\n")

bag.close()
text_file.close()
