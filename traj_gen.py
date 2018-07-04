import multiprocessing
import baxter_interface
import time
import dmp_gen as dmp_traj_gen
import rospy
import ipdb
from std_srvs.srv import Trigger

target_changed = False
target_pose = None

def callback(msg):
    target_changed = msg.flag
    target_pose = msg.pose
    
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

        trigger = rospy.ServiceProxy('/task_change_flag', Trigger,callback)
        resp = trigger()

        limb = 'right'
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
       
        rospy.loginfo("traj service done at %s"%(time.time()-traj_start_time,))

        # add traj_start_time to every timestamp in latest generated traj
        # b.c. timestamps in traj start from 0
        for idx in range(len(traj_to_ret)):
            traj_to_ret[idx][0] += traj_start_time

        while not rospy.is_shutdown():
            if target_changed == False:
                for idx, lines in enumerate(traj_to_ret):
                    self.com_queue.put(lines)
                    time.sleep(0.1)
            else:
                starting_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
                ending_angles = [ 0.4398689909261424, -1.1060001480653834, 0.23584954613738238, 1.2605487124448387,-0.2389175077131532, 1.3656263964149895, -0.10776215034895031,]
                latest_traj = dmp_traj_gen.main( starting_angles,  ending_angles)
                print "get lasted trajectory"
                for lines in latest_traj:
                    self.com_queue.put(lines)
                target_changed = False


        rospy.loginfo("traj gen done at %s"%(time.time()-traj_start_time,))
        target_changed = False
        while True: 
            # time.sleep(5)
            # global target_changed 
           
            if target_changed:
                # get latest starting_angles
                # get latest ending_angles
                # traj_start_time = time.now()
                print "target changed"
                starting_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
                ending_angles = [ 0.4398689909261424, -1.1060001480653834, 0.23584954613738238, 1.2605487124448387,-0.2389175077131532, 1.3656263964149895, -0.10776215034895031,]
                latest_traj = dmp_traj_gen.main( starting_angles,  ending_angles)
                
                # time.sleep(5)
                # add traj_start_time to every timestamp in latest generated traj
                # b.c. timestamps in traj start from 0
                for lines in latest_traj:
                    self.com_queue.put(lines)
                target_changed = False
            else:
                pass
            time.sleep(1)