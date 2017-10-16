import multiprocessing
import baxter_interface
import time
import dmp_gen as dmp_traj_gen
import rospy

class TarjGen(multiprocessing.Process):
    def __init__(
        self,
        com_queue
    ):
        multiprocessing.Process.__init__(self)
        self.com_queue = com_queue

    def run(self):
        rospy.init_node("TarjGen")

        rospy.loginfo("TarjGen run")
        filename = "../pick_place_data/go_to_hover_position_dmp.txt"
        # dmp_run.map_file(filename, 1)

        record_trajectory_path = "go_to_pick_position.txt"
        generalized_dmp_trajectory_path = "go_to_pick_position_dmp.txt"
        limb = 'right'
        limb_interface = baxter_interface.limb.Limb(limb)
        starting_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        ending_angles = [0.603621440033, -0.680703974624, -0.0709466114397,
                         1.06650014278, -0.00920388472731, 1.1780972451, -0.39883500485]

        traj_start_time = time.time()
        traj_to_ret = dmp_traj_gen.main(record_trajectory_path, generalized_dmp_trajectory_path, starting_angles,
                          ending_angles)
        rospy.loginfo("traj service done at %s"%(time.time()-traj_start_time,))

        # add traj_start_time to every timestamp in latest generated traj
        # b.c. timestamps in traj start from 0
        for idx in range(len(traj_to_ret)):
            traj_to_ret[idx][0] += traj_start_time
        self.com_queue.put(traj_to_ret)
        rospy.loginfo("traj gen done at %s"%(time.time()-traj_start_time,))
        while True:
            target_changed = False
            if target_changed:
                # get latest starting_angles
                # get latest ending_angles
                traj_start_time = time.now()
                dmp_traj_gen.main(record_trajectory_path, generalized_dmp_trajectory_path, starting_angles,
                                  ending_angles)
                # add traj_start_time to every timestamp in latest generated traj
                # b.c. timestamps in traj start from 0

                # self.com_queue.put(latest_traj)
            else:
                pass
            time.sleep(1)