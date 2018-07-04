import multiprocessing
import baxter_interface
import time
import dmp_gen as dmp_traj_gen
import rospy
import ipdb
from std_srvs.srv import Trigger,TriggerResponse
from multiprocessing import Queue

limb_interface =None
queue = None

def callback(req):
    traj_start_time = time.time()
    resp = TriggerResponse()
    resp.success = True
    
    starting_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    ending_angles = [ 0.8502088516854905, -0.9564370212465555,0.12310195822780445, 0.9618059540041545, 0.014956312681882784, 1.6689710972193301, 0.06979612584878632]
    latest_traj = dmp_traj_gen.main( starting_angles,  ending_angles)
    traj_to_ret = latest_traj
    global queue
    queue = Queue()
    for line in traj_to_ret:
        queue.put(line)
    print "traj done in callback %s" %(time.time()-traj_start_time)
    # print "get into callback"
    return resp

class TarjGen(multiprocessing.Process):
    def __init__(
        self,
        com_queue,
    
        ):
        multiprocessing.Process.__init__(self)
        self.com_queue = com_queue
        # self.traget_changed_Flag = traget_changed_Flag
    def run(self):
        rospy.init_node("TarjGen")
        rospy.loginfo("TarjGen run")    
        limb = 'right'
        global limb_interface
        limb_interface = baxter_interface.limb.Limb(limb)
        limb_interface.move_to_neutral()
        rospy.loginfo("move to neutral")
        joint_cmd_names = [
            'right_s0',
            'right_s1',
            'right_e0',
            'right_e1',
            'right_w0',
            'right_w1',
            'right_w2',
        ]
        move_to_start_position=[0.00841376457006,-0.550830582514,-0.00341249959604,0.757110843807,-0.00145716173142,1.25659545195,-0.00507347181644]
        limb_interface.move_to_joint_positions(dict(zip(joint_cmd_names, move_to_start_position)))
        rospy.loginfo("move to starting position")


        starting_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        ending_angles = [0.603621440033, -0.680703974624, -0.0709466114397,
                         1.06650014278, -0.00920388472731, 1.1780972451, -0.39883500485]

        traj_start_time = time.time()
        traj_to_ret = dmp_traj_gen.main( starting_angles,
                          ending_angles)
        global queue
        queue = Queue()
        for line in traj_to_ret:
            queue.put(line)
        rospy.loginfo("traj service done at %s"%(time.time()-traj_start_time,))

        # add traj_start_time to every timestamp in latest generated traj
        # b.c. timestamps in traj start from 0
        # for idx in range(len(traj_to_ret)):
        #     traj_to_ret[idx][0] += traj_start_time
        trigger = rospy.Service('/task_change_flag', Trigger, callback)
        while not rospy.is_shutdown():
                try: 
                    var = queue.get(timeout=0)
                except:
                    continue
                rospy.sleep(0.05)
                self.com_queue.put(var)
                # print var
                
                
            #     time.sleep(0.1)
            #     print idx
            #     if idx == 150:
            #         print idx
            #         break
            # starting_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
            # ending_angles = [ 0.7424467013365402, -0.8743690490946858,0.17487380981893716, 0.7121505807758033,
            #  0.06481068828815872, 1.7092380929013222, -0.21015536794030168]
            # latest_traj = dmp_traj_gen.main( starting_angles,  ending_angles)
            # traj_to_ret = latest_traj
            # for idx, lines in enumerate(traj_to_ret):
            #     self.com_queue.put(lines)
            # time.sleep(30)
        
