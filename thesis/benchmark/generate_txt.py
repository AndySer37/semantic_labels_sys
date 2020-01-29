#!/usr/bin/python
# -*- coding: utf-8 -*-
from os import walk
from os.path import join
import shutil 
import csv
mypath = "./"

# f1 = open('train.txt','a')
# f2 = open('val.txt','a')

import random

_f = []
count = 0
for root, dirs, files in walk(mypath):
	
	for f in files:
		count += 1
		fullpath = join(root, f)
		#print fullpath

		if "mask" in fullpath and ".png" == fullpath[-4:]:#and "000014" in fullpath:
			print fullpath
			_t = []
			_t.append(fullpath.replace("mask","images"))
			_t.append(fullpath[:])
			# _t = []
			# _t.append('/home/andyser/data/subt_real' + fullpath[1:])
			_f.append(_t)


with open('val.csv', mode='w') as csv_file:
	writer = csv.writer(csv_file)
	writer.writerows(_f)
	print(len(_f))
