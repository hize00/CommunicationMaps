 Generic Robot
 
 def __init__(self, seed, robot_id, is_leader, sim, comm_range, map_filename, polling_signal_period, duration, 
                 log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist, strategy, resize_factor, errors_filename, client_topic='move_base'):
       
       
class Leader(GenericRobot):
    def __init__(self, seed, robot_id, sim, comm_range, map_filename, polling_signal_period, 
                 duration, disc_method, disc, log_filename, teammates_id, n_robots, ref_dist, 
                 env_filename, comm_dataset_filename, strategy, strategyParams, resize_factor, tiling, errors_filename,
                 communication_model):       

super(Leader, self).__init__(seed, robot_id, True, sim, comm_range, map_filename, polling_signal_period, 
                                     duration, log_filename, comm_dataset_filename, teammates_id, n_robots, 
                                     ref_dist, strategy, resize_factor, errors_filename)
                                     
                                     
class Follower(GenericRobot):
    def __init__(self, seed, robot_id, sim, comm_range, map_filename, polling_signal_period, duration, 
                 log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist, env_filename, strategy, resize_factor, errors_filename):

super(Follower, self).__init__(seed, robot_id, False, sim, comm_range, map_filename, polling_signal_period, 
                                       duration, log_filename,  comm_dataset_filename, teammates_id, n_robots, ref_dist, strategy, resize_factor, errors_filename)


if is_leader:
    l = Leader(seed, robot_id, sim, comm_range, map_filename, polling_signal_period, duration, 
               disc_method, disc, log_filename, teammates_id, n_robots, ref_dist, env_filename, 
               comm_dataset_filename, strategy, strategyParams, resize_factor, tiling, errors_filename,
               communication_model)
    l.explore_comm_maps()
else:
    f = Follower(seed, robot_id, sim, comm_range, map_filename, polling_signal_period, duration, 
                 log_filename, comm_dataset_filename, teammates_id, n_robots, ref_dist, env_filename,
                 strategy, resize_factor, errors_filename)
    rospy.spin()