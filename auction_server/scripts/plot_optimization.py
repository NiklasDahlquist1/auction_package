#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray




import math
import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.lines import Line2D
from matplotlib.patches import Patch




curves_optimization_time_fixed_agents = [[],[],[], [],[],[]]
curves_agents_fixed_agents = [[],[],[], [],[],[]]
curves_tasks_fixed_agents = [[],[],[], [],[],[]]
curves_connections_fixed_agents = [[],[],[], [],[],[]]

curves_optimization_time_fixed_tasks = [[],[],[], [],[],[]]
curves_agents_fixed_tasks = [[],[],[], [],[],[]]
curves_tasks_fixed_tasks = [[],[],[], [],[],[]]
curves_connections_fixed_tasks = [[],[],[], [],[],[]]



def curve_fixed_agents_0_CB(data : Float64MultiArray):
    curves_optimization_time_fixed_agents[0].append(data.data[0])
    curves_agents_fixed_agents[0].append(data.data[1])
    curves_tasks_fixed_agents[0].append(data.data[2])
    curves_connections_fixed_agents[0].append(data.data[3])
    #print(data.data[0])

def curve_fixed_agents_1_CB(data):
    curves_optimization_time_fixed_agents[1].append(data.data[0])
    curves_agents_fixed_agents[1].append(data.data[1])
    curves_tasks_fixed_agents[1].append(data.data[2])
    curves_connections_fixed_agents[1].append(data.data[3])
    #print(data.data[0])


def curve_fixed_agents_2_CB(data):
    curves_optimization_time_fixed_agents[2].append(data.data[0])
    curves_agents_fixed_agents[2].append(data.data[1])
    curves_tasks_fixed_agents[2].append(data.data[2])
    curves_connections_fixed_agents[2].append(data.data[3])
    #print(data.data[0])



def curve_fixed_tasks_0_CB(data : Float64MultiArray):
    curves_optimization_time_fixed_tasks[0].append(data.data[0])
    curves_agents_fixed_tasks[0].append(data.data[1])
    curves_tasks_fixed_tasks[0].append(data.data[2])
    curves_connections_fixed_tasks[0].append(data.data[3])
    #print(data.data[0])

def curve_fixed_tasks_1_CB(data):
    curves_optimization_time_fixed_tasks[1].append(data.data[0])
    curves_agents_fixed_tasks[1].append(data.data[1])
    curves_tasks_fixed_tasks[1].append(data.data[2])
    curves_connections_fixed_tasks[1].append(data.data[3])
    #print(data.data[0])


def curve_fixed_tasks_2_CB(data):
    curves_optimization_time_fixed_tasks[2].append(data.data[0])
    curves_agents_fixed_tasks[2].append(data.data[1])
    curves_tasks_fixed_tasks[2].append(data.data[2])
    curves_connections_fixed_tasks[2].append(data.data[3])
    #print(data.data[0])

def callbacks():
    # setup subscribers
    rospy.Subscriber("optimization_tester_result_fixed_agents_0", Float64MultiArray, curve_fixed_agents_0_CB)
    rospy.Subscriber("optimization_tester_result_fixed_agents_1", Float64MultiArray, curve_fixed_agents_1_CB)
    rospy.Subscriber("optimization_tester_result_fixed_agents_2", Float64MultiArray, curve_fixed_agents_2_CB)

    rospy.Subscriber("optimization_tester_result_fixed_tasks_0", Float64MultiArray, curve_fixed_tasks_0_CB)
    rospy.Subscriber("optimization_tester_result_fixed_tasks_1", Float64MultiArray, curve_fixed_tasks_1_CB)
    rospy.Subscriber("optimization_tester_result_fixed_tasks_2", Float64MultiArray, curve_fixed_tasks_2_CB)

    # collect data until enter is pressed

    #rate = rospy.Rate(1.0 / 2)
    #rate.sleep()
    input("Press Enter to continue...") # requires python3... 
    #raw_input("Press Enter to continue...") # python2
 

def plot2():


    """figure.titlesize
    axes.labelsize
    axes.titlesize
    font.size"""

    # plot


    #plt.rc('text', usetex=True) # gotta install latex for this
    plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Computer Modern Roman"],
    #"ps.usedistiller" : "xpdf",
    "font.size" : 15,
    "axes.titlesize" : 19, #16
})


    edgecolor = [0.4, 0.4, 0.4]
    mpl.rcParams['axes.edgecolor'] = edgecolor



    mpl.rcParams['legend.fontsize'] = 11
    #fig, ax1 = plt.plot#subplots(3, sharex=True)#, figsize=[4, 3])
    fig_1 = plt.figure()
    ax_1 = fig_1.gca()
    ax_1.margins(x=0)

    fig_2 = plt.figure()
    ax_2 = fig_2.gca()
    ax_2.margins(x=0)


    plot_color = [1, 1, 1]
    #ax.w_xaxis.set_pane_color(plot_color)
    #ax.w_yaxis.set_pane_color([c * 1 for c in plot_color])
    #ax.w_zaxis.set_pane_color([c * 1 for c in plot_color])

    #ax.zaxis.pane.set_edgecolor(edgecolor)
    #ax.xaxis.pane.set_edgecolor(edgecolor)
    #ax.yaxis.pane.set_edgecolor(edgecolor)
    
    #ax.grid(False)
    #ax.xaxis.pane.fill = False
    #ax.yaxis.pane.fill = False
    #ax.zaxis.pane.fill = False
    #ax.tick_params(axis='x', colors='red')

    
    #ax.view_init(elev=30, azim=135)
    #ax.view_init(elev=25, azim=135)
    #ax.yaxis.pane.set_edgecolor([0,0,0])
    #ax.zaxis.pane.set_edgecolor([0,0,0])

    
    #ax.set_facecolor('xkcd:salmon')


    linewidth_plot = 1.1
    linewidth_marker = 0.8


    # plot all curves
    for i in range(0, len(curves_connections_fixed_agents)):

        #x = np.array(curves_connections[i])
        x = np.array(curves_tasks_fixed_agents[i])
        y = np.array(curves_optimization_time_fixed_agents[i])
        #z = np.array(curves_z[i])
        
        #t = np.array([tt.to_sec() - curves_t[i][0].to_sec() for tt in curves_t[i]])
        

        l_w = 1.1
        if(len(x) > 0):
            ax_1.plot(x, y, label='SOLVER', linewidth=l_w)

    for i in range(0, len(curves_connections_fixed_tasks)):

        #x = np.array(curves_connections[i])
        x = np.array(curves_agents_fixed_tasks[i])
        y = np.array(curves_optimization_time_fixed_tasks[i])
        #z = np.array(curves_z[i])
        
        #t = np.array([tt.to_sec() - curves_t[i][0].to_sec() for tt in curves_t[i]])
        

        l_w = 1.1
        if(len(x) > 0):
            ax_2.plot(x, y, label='SOLVER', linewidth=l_w)


    ax_1.set_title('Optimization analysis, 50 agents')
    ax_1.set_ylabel('Optimization time [ms]')

    ax_1.set_xlabel('Tasks [n]')


    ax_2.set_title('Optimization analysis, 50 tasks')
    ax_2.set_ylabel('Optimization time [ms]')

    ax_2.set_xlabel('Agents [n]')


    # custom legends
    """legend_elements = [Line2D([0], [0], color='b', lw=linewidth_plot, label='UAV Paths'),
                       Line2D([0], [0], marker='^', color='w', label='Failed tasks', markeredgewidth=linewidth_marker, markeredgecolor="k", markerfacecolor='r', markersize=10),
                       Line2D([0], [0], marker='^', color='w', label='Reached tasks', markeredgewidth=linewidth_marker, markeredgecolor="k", markerfacecolor='g', markersize=10),
                       
                       ]
                       #,
                       #Patch(facecolor='orange', edgecolor='r',
                       #  label='Color Patch')]
"""

    ax_1.legend(['30\% task participation', '60\% task participation', '90\% task participation'], loc='best')
    ax_2.legend(['30\% task participation', '60\% task participation', '90\% task participation'], loc='best')

    #plt.savefig('optimization.eps') 
    #plt.savefig('optimization.png', dpi=600) 
    fig_1.savefig('optimization_fixed_agents.eps') 
    fig_2.savefig('optimization_fixed_tasks.eps') 
    plt.show()
















if __name__ == '__main__':
    try:
        rospy.init_node('plotter', anonymous=True)
        callbacks()
        #plot3()
        plot2()

    except rospy.ROSInterruptException:
        pass





