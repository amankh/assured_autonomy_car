import numpy as np
import matplotlib.pyplot as plt
data = np.load('counter_eg1.npz')
all_data = np.load('all_data_counter_example.npz')
print all_data.keys()
x_all = all_data['x_all']
y_all = all_data['y_all']
ind_all = all_data['ind_all']
flow_x_all = all_data['flow_x_all']
flow_y_all = all_data['flow_y_all']
counter_eg_x = data['counter_eg_x']
counter_eg_y = data['counter_eg_y']
counter_flow_x = data['counter_flow_x']
counter_flow_y = data['counter_flow_y']
#print counter_flow_x[0]
#print counter_eg_x, counter_eg_y
mode = "rounded_square"
#for i in xrange(len(counter_flow_x[0])):
#    plt.plot(counter_flow_x[0][i], counter_flow_y[0][i], 'g')
if mode == "rounded_square":
    #plt.plot(counter_eg_x[0], counter_eg_x[0], 'r*', markersize=12)#current position
    
    #plt.plot(np.arange(0, 1.1, 0.1), 0*np.arange(0, 1.1, 0.1), 'b', linewidth = 3.0 )#1st
    #th = np.arange(1.5*np.pi, 2*np.pi+np.pi/10, np.pi/10)
    
    #plt.plot(1 * np.cos(th)+1, 1 * np.sin(th)+1, 'b', linewidth = 3.0)#2nd
    plt.plot(2+0*np.arange(1, 2.1, 0.1), np.arange(1, 2.1, 0.1), 'b', linewidth = 3.0, label = 'Planned Path')#3rd
    th = np.arange(0*np.pi, 0.5*np.pi+np.pi/10, np.pi/10)
    plt.plot(1 * np.cos(th)+1, 1 * np.sin(th)+2, 'b', linewidth = 3.0)#4th

    plt.plot(np.arange(0, 1.1, 0.1), 3+0*np.arange(0, 1.1, 0.1), 'b', linewidth = 3.0)#5th                       

    #th = np.arange(0.5*np.pi, 1.0*np.pi+np.pi/10, np.pi/10)
    #plt.plot(1 * np.cos(th)+0, 1 * np.sin(th)+2, 'b', linewidth = 3.0)#6th

    #plt.plot(-1+0*np.arange(0, 1.1, 0.1), np.arange(1, 2.1, 0.1), 'b', linewidth = 3.0)#7th

    #th = np.arange(np.pi, 1.5*np.pi+np.pi/10, np.pi/10)
    #plt.plot(1 * np.cos(th)+0, 1 * np.sin(th)+1, 'b', linewidth = 3.0)#8th
    #print flow_x_all[488]
    plt.plot([x_all[622], x_all[659]], [y_all[622], y_all[659]], 'r*', markersize=12, label = 'True Position')
    #plt.plot(x_all[659], y_all[659], 'r*', markersize=12, label = '')
    for item in [622, 659]:
        
        for j in xrange(len(flow_x_all[item])):
            plt.plot(flow_x_all[item][j], flow_y_all[item][j], 'g', label = 'Reachable States' if item == 622 and j == 0 else '')
            plt.plot(x_all[item-20:item+20], y_all[item-20:item+20], 'magenta', label = 'True Trajectory' if item == 622 and j == 0 else '', linewidth = 2.0)
    plt.xlim(-1, 3)
    plt.ylim(-0.5, 4.5)
    plt.xlabel('x position (m)', fontsize = 20)
    plt.ylabel('y position (m)', fontsize = 20)
    plt.legend()
    plt.show()
#print len(x_all), len(y_all), len(ind_all)
for i in xrange(len(x_all)):
    if ind_all[i] == False:
        print i
#print x_all[622], y_all[622], flow_x_all[622], flow_y_all[622]
print x_all[659], y_all[659], flow_x_all[659], flow_y_all[659]
