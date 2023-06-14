import numpy as np
import matplotlib.pyplot as plt

e2_1=np.genfromtxt("test_iris_1.txt",dtype=float)
e2_2=np.genfromtxt("test_iris_2.txt",dtype=float)
e2_3=np.genfromtxt("test_iris_3.txt",dtype=float)
e2_4=np.genfromtxt("test_iris_4.txt",dtype=float)
e2_5=np.genfromtxt("test_iris_5.txt",dtype=float)

p_1=np.genfromtxt("test_position_1.txt",dtype=float)
p_2=np.genfromtxt("test_position_2.txt",dtype=float)
p_3=np.genfromtxt("test_position_3.txt",dtype=float)
p_4=np.genfromtxt("test_position_4.txt",dtype=float)
p_5=np.genfromtxt("test_position_5.txt",dtype=float)

x_1 = np.arange(len(e2_1))
x_2 = np.arange(len(e2_2))
x_3 = np.arange(len(e2_3))
x_4 = np.arange(len(e2_4))
x_5 = np.arange(len(e2_5))

y_1 = np.arange(len(p_1))
y_2 = np.arange(len(p_2))
y_3 = np.arange(len(p_3))
y_4 = np.arange(len(p_4))
y_5 = np.arange(len(p_5))

plt.figure(figsize=(10, 6))
plt.subplot(6,1,1)
plt.plot(x_1, e2_1[:,0], color='#FF0000', label='ex1', linewidth=3.0)
plt.plot(x_2, e2_2[:,0], color='#FFA500', label='ex2', linewidth=3.0)
plt.plot(x_3, e2_3[:,0], color='#7CFC00', label='ex3', linewidth=3.0)
plt.plot(x_4, e2_4[:,0], color='#1E90FF', label='ex4', linewidth=3.0)
plt.plot(x_5, e2_5[:,0], color='#FF00FF', label='ex5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'ex', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(6,1,2)
plt.plot(x_1, e2_1[:,1], color='#FF0000', label='ey1', linewidth=3.0)
plt.plot(x_2, e2_2[:,1], color='#FFA500', label='ey2', linewidth=3.0)
plt.plot(x_3, e2_3[:,1], color='#7CFC00', label='ey3', linewidth=3.0)
plt.plot(x_4, e2_4[:,1], color='#1E90FF', label='ey4', linewidth=3.0)
plt.plot(x_5, e2_5[:,1], color='#FF00FF', label='ey5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'ey', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(6,1,3)
plt.plot(x_1, e2_1[:,2], color='#FF0000', label='etheta1', linewidth=3.0)
plt.plot(x_2, e2_2[:,2], color='#FFA500', label='etheta2', linewidth=3.0)
plt.plot(x_3, e2_3[:,2], color='#7CFC00', label='etheta3', linewidth=3.0)
plt.plot(x_4, e2_4[:,2], color='#1E90FF', label='etheta4', linewidth=3.0)
plt.plot(x_5, e2_5[:,2], color='#FF00FF', label='etheta5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'etheta', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(6,1,4)
plt.plot(x_1, e2_1[:,3], color='#FF0000', label='ev1', linewidth=3.0)
plt.plot(x_2, e2_2[:,3], color='#FFA500', label='ev2', linewidth=3.0)
plt.plot(x_3, e2_3[:,3], color='#7CFC00', label='ev3', linewidth=3.0)
plt.plot(x_4, e2_4[:,3], color='#1E90FF', label='ev4', linewidth=3.0)
plt.plot(x_5, e2_5[:,3], color='#FF00FF', label='ev5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'ev', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(6,1,5)
plt.plot(x_1, e2_1[:,4], color='#FF0000', label='ew1', linewidth=3.0)
plt.plot(x_2, e2_2[:,4], color='#FFA500', label='ew2', linewidth=3.0)
plt.plot(x_3, e2_3[:,4], color='#7CFC00', label='ew3', linewidth=3.0)
plt.plot(x_4, e2_4[:,4], color='#1E90FF', label='ew4', linewidth=3.0)
plt.plot(x_5, e2_5[:,4], color='#FF00FF', label='ew5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'ew', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(6,1,6)
plt.plot(x_1, e2_1[:,5], color='#FF0000', label='v1', linewidth=3.0)
plt.plot(x_2, e2_2[:,5], color='#FFA500', label='v2', linewidth=3.0)
plt.plot(x_3, e2_3[:,5], color='#7CFC00', label='v3', linewidth=3.0)
plt.plot(x_4, e2_4[:,5], color='#1E90FF', label='v4', linewidth=3.0)
plt.plot(x_5, e2_5[:,5], color='#FF00FF', label='v5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'velocity', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.figure(figsize=(10, 6))
plt.subplot(7,1,1)
plt.plot(y_1, p_1[:,0], color='#FF0000', label='x1', linewidth=3.0)
plt.plot(y_2, p_2[:,0], color='#FFA500', label='x2', linewidth=3.0)
plt.plot(y_3, p_3[:,0], color='#7CFC00', label='x3', linewidth=3.0)
plt.plot(y_4, p_4[:,0], color='#1E90FF', label='x4', linewidth=3.0)
plt.plot(y_5, p_5[:,0], color='#FF00FF', label='x5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'x', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(7,1,2)
plt.plot(y_1, p_1[:,1], color='#FF0000', label='y1', linewidth=3.0)
plt.plot(y_2, p_2[:,1], color='#FFA500', label='y2', linewidth=3.0)
plt.plot(y_3, p_3[:,1], color='#7CFC00', label='y3', linewidth=3.0)
plt.plot(y_4, p_4[:,1], color='#1E90FF', label='y4', linewidth=3.0)
plt.plot(y_5, p_5[:,1], color='#FF00FF', label='y5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'y', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(7,1,3)
plt.plot(y_1, p_1[:,2], color='#FF0000', label='z1', linewidth=3.0)
plt.plot(y_2, p_2[:,2], color='#FFA500', label='z2', linewidth=3.0)
plt.plot(y_3, p_3[:,2], color='#7CFC00', label='z3', linewidth=3.0)
plt.plot(y_4, p_4[:,2], color='#1E90FF', label='z4', linewidth=3.0)
plt.plot(y_5, p_5[:,2], color='#FF00FF', label='z5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'z', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(7,1,4)
plt.plot(y_1, p_1[:,3], color='#FF0000', label='theta1', linewidth=3.0)
plt.plot(y_2, p_2[:,3], color='#FFA500', label='theta2', linewidth=3.0)
plt.plot(y_3, p_3[:,3], color='#7CFC00', label='theta3', linewidth=3.0)
plt.plot(y_4, p_4[:,3], color='#1E90FF', label='theta4', linewidth=3.0)
plt.plot(y_5, p_5[:,3], color='#FF00FF', label='theta5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'theta', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(7,1,5)
plt.plot(y_1, p_1[:,4], color='#FF0000', label='omega1', linewidth=3.0)
plt.plot(y_2, p_2[:,4], color='#FFA500', label='omega2', linewidth=3.0)
plt.plot(y_3, p_3[:,4], color='#7CFC00', label='omega3', linewidth=3.0)
plt.plot(y_4, p_4[:,4], color='#1E90FF', label='omega4', linewidth=3.0)
plt.plot(y_5, p_5[:,4], color='#FF00FF', label='omega5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'omega', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(7,1,6)
plt.plot(y_1, p_1[:,5], color='#FF0000', label='theta_l1', linewidth=3.0)
plt.plot(y_2, p_2[:,5], color='#FFA500', label='theta_l2', linewidth=3.0)
plt.plot(y_3, p_3[:,5], color='#7CFC00', label='theta_l3', linewidth=3.0)
plt.plot(y_4, p_4[:,5], color='#1E90FF', label='theta_l4', linewidth=3.0)
plt.plot(y_5, p_5[:,5], color='#FF00FF', label='theta_l5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u't', fontsize=18)
plt.ylabel(u'theta_l', fontsize=18)
plt.legend(fontsize=18,loc=1)

plt.subplot(7,1,7)
plt.plot(p_1[:,0], p_1[:,1], color='#FF0000', label='iris1', linewidth=3.0)
plt.plot(p_2[:,0], p_2[:,1], color='#FFA500', label='iris2', linewidth=3.0)
plt.plot(p_3[:,0], p_3[:,1], color='#7CFC00', label='iris3', linewidth=3.0)
plt.plot(p_4[:,0], p_4[:,1], color='#1E90FF', label='iris4', linewidth=3.0)
plt.plot(p_5[:,0], p_5[:,1], color='#FF00FF', label='iris5', linewidth=3.0)

plt.xticks(fontsize=18)
plt.yticks(fontsize=18)

plt.xlabel(u'x', fontsize=18)
plt.ylabel(u'y', fontsize=18)
plt.legend(fontsize=18,loc=1)
plt.show()