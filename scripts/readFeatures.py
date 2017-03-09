#!/usr/bin/env python
import rospy
import rosbag
import math
import datetime

global frame
frame=0
bag = rosbag.Bag('/media/scosar/Windows/Users/scosar/Documents/DATASETS/fdg_15.bag')
text_file = open("/media/scosar/Windows/Users/scosar/Documents/DATASETS/fdg_15_features.txt", "w")
for topic, msg, t in bag.read_messages(topics=['/reidentifier/features']):
    #print (t.secs,len(msg.data))
    #print type(msg)
    if len(msg.data)>0:
        
        #print (datetime.datetime.fromtimestamp(t.secs).strftime("%Y-%m-%d %H:%M:%S"),t.nsecs,speed,status)
        #print (msg.data)
        noofPeople = len(msg.data) / 7
        text_file.write("%d.%d;" % (t.secs,t.nsecs))
        #print noofPeople
        for pNo in range(0,noofPeople):
        	#print pNo
        	text_file.write("height:(%f):shoulder:(%f):face:(%f):volFace:(%f):volBreast:(%f):volBelly:(%f);dist:(%f);" % (msg.data[(pNo)*7+0],msg.data[(pNo)*7+1],msg.data[(pNo)*7+2],msg.data[(pNo)*7+4],msg.data[(pNo)*7+5],msg.data[(pNo)*7+6], msg.data[(pNo)*7+3]))        

        text_file.write("\n")

bag.close()
text_file.close()
