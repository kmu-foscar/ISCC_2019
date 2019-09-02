#!/usr/bin/env python

import rospy
import gmplot
from nav_msgs.msg import Odometry
from os.path import expanduser
import utm

file_directory = expanduser("~/ISCC_2019/src/race/src/path/final_path2.txt")

lat = []
lon = []

gmap3 = gmplot.GoogleMapPlotter(37.23937716270539, 126.77325026006068, 19)

with open(file_directory, 'r') as f:
	lines = f.readlines()
	index = 0
	for line in lines :
		index += 1
		utm_pose = line.split(' ')
		latlon = utm.to_latlon(float(utm_pose[0]), float(utm_pose[1]), 52, 'S')
		# if index == 188 :
		# 	break
		lat.append(latlon[0])
		lon.append(latlon[1])
		print("index :", index, "latlon :", latlon[0], latlon[1])


gmap3.plot(lat, lon, 'cornflowerblue', edge_width = 3.0)
gmap3.draw('contest1.html')