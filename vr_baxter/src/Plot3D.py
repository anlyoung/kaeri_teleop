import csv

import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x = []
y = []
z = []

with open ('circle_data.csv') as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	line_count = 0
	for row in csv_reader:
		if line_count==0:
			print ('Head')
			line_count+=1
		else:
			x.append(float(row[0]))
			y.append(float(row[1]))
			z.append(float(row[2]))
			line_count+=1

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.scatter(x,y,z)
ax.legend()

ax.xaxis.set_ticks(np.arange(.35,.55, .10))
ax.yaxis.set_ticks(np.arange(-.05,.15, .10))
ax.zaxis.set_ticks(np.arange(0,.3, .10))


plt.xlim(.35,.65)
plt.ylim(-.15,.15)

plt.rcParams.update({'font.size': 30})

ax.set_zlim(0,.3)

ax.set_xlabel('X Position (m)',labelpad=40)
ax.set_ylabel('Y Position (m)',labelpad=40)
ax.set_zlabel('Z Position (m)',labelpad=40)

plt.show()


