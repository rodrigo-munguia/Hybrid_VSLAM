import numpy as np
import matplotlib.pyplot as plt


#---------------------------------------------------------------
# read data from file
data = np.loadtxt('log_ekf_time_per_frame', delimiter=',')
# extract x and y values
x = data[:, 0]
y = data[:, 1]
# plot data
fig1 = plt.figure(figsize=(8, 2))
plt.plot(x, y, '-', color='red', linewidth=.5)
plt.xlabel('Execution time (s)')
plt.ylabel('Computational time (s)')
plt.title('Local SLAM: computational time per frame')
plt.ylim(0, 0.05)
plt.xlim(0, 278)
# -------------------------------------------------------------
data = np.loadtxt('log_gmap_time_per_step', delimiter=',')
# extract x and y values
x = data[:, 0]
y = data[:, 1]
# plot data
fig2 = plt.figure(figsize=(8, 2))
plt.plot(x, y, '-', color='red', linewidth=1)
plt.xlabel('Steps')
plt.ylabel('Computational time (s)')
plt.title('Global map: computational time per step')
#plt.ylim(0, 0.06)
plt.xlim(0, 194)
# -------------------------------------------------------------
data = np.loadtxt('log_cloop_time_per_step', delimiter=',')
# extract x and y values
x = data[:, 0]
y = data[:, 1]
# plot data
fig3 = plt.figure(figsize=(8, 2))
plt.plot(x, y, '-', color='red', linewidth=1)
plt.xlabel('Steps')
plt.ylabel('Computational time (s)')
plt.title('Close loop: computational search time per step')
#plt.ylim(0, 0.06)
plt.xlim(0, 2660)
#---------------------------------------------------------------
data = np.loadtxt('log_ekf_feats_per_frame', delimiter=',')
# extract x and y values
x = data[:, 0]
y = data[:, 1]
# plot data
fig4 = plt.figure(figsize=(8, 2))
plt.plot(x, y, '-', color='blue', linewidth=1)
plt.xlabel('Execution time (s)')
plt.ylabel('Number of map features')
plt.title('Local SLAM: number of map features per frame')
#plt.ylim(0, 0.06)
plt.xlim(0, 278)
#-------------------------------------------------------------
data = np.loadtxt('log_gmap_anchors_per_step', delimiter=',')
# extract x and y values
x = data[:, 0]
y = data[:, 1]
# plot data
fig5 = plt.figure(figsize=(8, 2))
plt.plot(x, y, '-', color='blue', linewidth=1)
plt.xlabel('Steps')
plt.ylabel('Number of anchors')
plt.title('Global map: number of anchors per step')
#plt.ylim(0, 0.06)
plt.xlim(0, 194)


plt.show()