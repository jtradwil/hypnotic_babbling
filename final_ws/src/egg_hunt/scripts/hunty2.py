#!/usr/bin/env python
import rospy
import smach
import smach_ros
import numpy as np
import time
import threading
import Queue

import explore4
import alvar_tracker

class myThread (threading.Thread):
    def __init__(self, name, function):
        threading.Thread.__init__(self)
        self.name = name
        self.function = function
        
    def run(self):
        print "Starting " + self.name
        self.function()
        print "Exiting " + self.name

class init_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_pass', 'init_fail'])

    def execute(self, userdata):
        #Start Jackal Description launchfile
        #Start Jackal Nodes launchfile
        rospy.loginfo('Entering init State')
        return 'init_pass'

class map_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['map_found_all', 'map_found_possible', 'map_found_none'])
        
    def cb(self):
        rospy.loginfo('Explorer returned')
        
    def execute(self, userdata):
        rospy.loginfo('Entering map State')
    
        alvar_queue = Queue.Queue()
        map_queue = Queue.Queue()
        
        # Setup the wall following thread
        explorer = explore4.explorer(map_queue)        
        explorer_thread = myThread("Mapping Thread", explorer._run_wall_follow)
        explorer_thread.start()
        
        tag_tracker = alvar_tracker.alvar_tracker(alvar_queue)
        tag_tracker_thread = myThread("Alvar Thread", tag_tracker._run)
        tag_tracker_thread.start()
        
        explorer_finished = 0
        while (explorer_thread.isAlive() and not(explorer_finished)) :
            time.sleep(10)
            
            # Alvar maker found
            if(not(alvar_queue.empty())):
                # Kill the map thread if an alvar tag is found
                map_queue.put("KILL")
                
                # Wait until the thread dies
                while (explorer_thread.isAlive()):
                    pass
                    
                    
                # Restart the map thread
                if(not(explorer_finished)):
                    explorer_thread = myThread("Explorer Thread", explorer._run_wall_follow)
                    explorer_thread.start()
                
                
            # clear the map queue, escape if mapping is done
            while(not(map_queue.empty())):
                cmd = map_queue.get()
                
                rospy.loginfo("COMMAND: %s", cmd)
                
                if(cmd == "DONE"):
                    rospy.loginfo("ESCAPE")
                    explorer_finished = 1
            

            
        explorer._save_map()
        explorer._open_cv_map()
            
        while(not(map_queue.empty())):
            bunny = map_queue.get()
            rospy.loginfo('\t Possible Bunny at X: %d, Y: %d, R: %d', bunny[0], bunny[1], bunny[2])
            
        return 'map_found_all'

class go_home_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['home_pass', 'home_fail'])

    def execute(self, userdata):
        rospy.loginfo('Entering home State')
        return 'home_pass'

class interrogate_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['interrogate_found_all', 'interrogate_found_none'])

    def execute(self, userdata):
        rospy.loginfo('Entering interrogate State')
        return 'interrogate_found_all'
        
class explore_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['explore_pass', 'explore_fail'])

    def execute(self, userdata):
        rospy.loginfo('Entering explore State')
        return 'explore_pass'
                     
class wait_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait_target_acquired', 'wait_for_target'])

    def execute(self, userdata):
        rospy.loginfo('Entering wait State')
        return 'wait_target_acquired'
        
class nav_to_target_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['nav_pass', 'nav_fail'])
        

    def execute(self, userdata):
        rospy.loginfo('Entering nav State')
        return 'nav_pass'
        
class count_eggs_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['count_pass', 'count_fail', 'count_error'])

    def execute(self, userdata):
        rospy.loginfo('Entering count State')
        while not(rospy.is_shutdown()):
            pass
        return 'count_error'
                        
# Error
class error_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_done'])

    def execute(self, userdata):
        while not(rospy.is_shutdown()):
            pass


def main():
    rospy.init_node("jackal_explore")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['impossibru_outcome'])

    # Open the container
    with sm:
        # Add states to the container
        
        smach.StateMachine.add('init_state', init_state(), 
            transitions={'init_pass':'map_state', 
                         'init_fail':'error_state'})
                     
        smach.StateMachine.add('map_state', map_state(), 
            transitions={'map_found_all':'go_home_state', 
                         'map_found_possible':'interrogate_state',
                         'map_found_none':'explore_state'})
                     
        smach.StateMachine.add('go_home_state', go_home_state(), 
            transitions={'home_pass':'wait_state', 
                         'home_fail':'go_home_state'})
                     
        smach.StateMachine.add('interrogate_state', interrogate_state(), 
            transitions={'interrogate_found_all':'go_home_state', 
                         'interrogate_found_none':'explore_state'})
                     
        smach.StateMachine.add('explore_state', explore_state(), 
            transitions={'explore_pass':'go_home_state', 
                         'explore_fail':'explore_state'})
                     
        smach.StateMachine.add('wait_state', wait_state(), 
            transitions={'wait_target_acquired':'nav_to_target_state', 
                         'wait_for_target':'wait_state'})         
                           
        smach.StateMachine.add('nav_to_target_state', nav_to_target_state(), 
            transitions={'nav_pass':'count_eggs_state', 
                         'nav_fail':'nav_to_target_state'})
                           
        smach.StateMachine.add('count_eggs_state', count_eggs_state(), 
            transitions={'count_pass':'go_home_state', 
                         'count_fail':'count_eggs_state',
                         'count_error':'error_state'})
                     
        smach.StateMachine.add('error_state', error_state(), 
            transitions={'error_done':'error_state'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()





