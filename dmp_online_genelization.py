#!/usr/bin/env python
import roslib

roslib.load_manifest('dmp')
import rospy
import numpy as np
from dmp.msg import *
from dmp.srv import *
import matplotlib.pyplot as plt

# Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain,
                   D_gain, num_bases):
    demotraj = DMPTraj()

    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt * i)

    k_gains = [K_gain] * dims
    d_gains = [D_gain] * dims

    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    print "LfD done"

    return resp;


# Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    print "DMP planning done"

    return resp;

if __name__ == '__main__':
    import pickle
    resp = pickle.load(open("resp.pkl", "rb"))  # read data from resp.pkl
    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)


    #Now, generate a plan
    x_0 = [0.0,0.0]          #Plan starting at a different point than demo
    x_dot_0 = [0.0,0.0]
    t_0 = 6
    goal = [8.0,7.0]         #Plan to a different goal than demo
    goal_thresh = [0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                           seg_length, tau, dt, integrate_iter)

    print plan

    #plan.plan.times
    #plan.plan.points[0]
    #plan.plan.points[1]
    Column0_plan = [0.0] * len(plan.plan.times)
    Column1_plan = [0.0] * len(plan.plan.times)
    for i in range(len(plan.plan.times)):
        Column0_plan[i] = plan.plan.points[i].positions[0]
        Column1_plan[i] = plan.plan.points[i].positions[1]
    f1, axarr1 = plt.subplots(2, sharex=True)
    axarr1[0].plot(plan.plan.times, Column0_plan)
    axarr1[0].set_title('right_arm_joint_space1')
    axarr1[1].plot(plan.plan.times, Column1_plan)
    plt.show()

