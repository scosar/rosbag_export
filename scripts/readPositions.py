#!/usr/bin/env python
import rospy
import rosbag
import math
import datetime

prev_x=0
prev_y=0
start=0
status="stopped"
bag = rosbag.Bag('/media/scosar/Windows/Users/scosar/Documents/DATASETS/lcas_kitchen_anomaly_positions.bag')
for topic, msg, t in bag.read_messages(topics=['/people_tracker/positions']):
    #print type(t)
    if len(msg.poses)>0:
        if start==0:
            prev_x = msg.poses[0].position.x
            prev_y = msg.poses[0].position.y
            speed = 0
            start=1
        else:
            dif_x = abs(prev_x - msg.poses[0].position.x)
            dif_y = abs(prev_y - msg.poses[0].position.y)
            speed = math.sqrt((dif_x*dif_x) + (dif_y*dif_y))
            prev_x = msg.poses[0].position.x
            prev_y = msg.poses[0].position.y

        if (speed<0.2):
            status = "stopped"
        elif (speed>0.2) & (speed<1.4 ):
            status = "walking"
        elif (speed>1.4):
            status = "running"
        else:
            status = "NaN"
        print (datetime.datetime.fromtimestamp(t.secs).strftime("%Y-%m-%d %H:%M:%S"),t.nsecs,speed,status)
bag.close()
