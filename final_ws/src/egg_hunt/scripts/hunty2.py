#!/usr/bin/env python
import rospy
import smach
import smach_ros
import numpy as np
import time
import threading
import Queue
import tf
from geometry_msgs.msg import PoseStamped
import cv_bridge
import cv2
from sensor_msgs.msg import Image

import eggs
import explore4
import alvar_tracker
import move_to
import explore_random

expected_bunnies = 3
possible_bunnies = []
found_markers = []
target_index = 0


class myThread (threading.Thread):
    def __init__(self, name, function):
        threading.Thread.__init__(self)
        self.name = name
        self.function = function
        
    def run(self):
        print "Starting " + self.name
        self.function()
        print "Exiting " + self.name
        
def kill_thread(my_thread, my_queue):
    my_queue.put("KILL")
    while (my_thread.isAlive()):
        pass
        
def clear_queue(my_queue):
    ret_val = 0
    time.sleep(1)   
    # clear the map queue, escape if mapping is done
    
    clearing = 1
    while(not(my_queue.empty()) and (clearing == 1)):
        cmd = my_queue.get()
        
        if(cmd == "DONE"):
            ret_val = 1
            
    return ret_val
        
def restart_thread():
    # Alvar maker found
    if(not(alvar_queue.empty())):
        kill_thread(explorer_thread, map_queue)                    
            
        # Restart the map thread
        if(not(explorer_finished)):
            explorer_thread = myThread("Explorer Thread", explorer._run_wall_follow)
            explorer_thread.start()

class init_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_pass', 'init_fail'])

    def execute(self, userdata):
        #Start Jackal Description launchfile
        #Start Jackal Nodes launchfile
        rospy.loginfo('Entering init State')
        rospy.set_param('rabbit_id', '0')
        rospy.set_param('colors', [0,0,0,0,0,0])
        return 'init_pass'

class map_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['map_found_all', 'map_found_possible', 'map_found_none'])
        
    def cb(self):
        rospy.loginfo('Explorer returned')
        
    def execute(self, userdata):
        global possible_bunnies, found_markers
    
        rospy.loginfo('Entering map State')
    
        alvar_queue = Queue.Queue()
        map_queue = Queue.Queue()
        
        # Setup the wall following thread
        explorer = explore4.explorer(map_queue)        
        explorer_thread = myThread("Mapping Thread", explorer._run_wall_follow)
        explorer_thread.start()
        
        tag_tracker = alvar_tracker.alvar_tracker(alvar_queue, found_markers, expected_bunnies)
        tag_tracker_thread = myThread("Alvar Thread", tag_tracker._run)
        tag_tracker_thread.start()
        
        explorer_finished = 0
        alvar_finished = 0
        
        while(explorer_thread.isAlive()) :
            if(tag_tracker_thread.isAlive()):
                pass
            else:
                alvar_finished = 1
                kill_thread(explorer_thread, map_queue)
                clear_queue(map_queue)
                    
        kill_thread(tag_tracker_thread, alvar_queue)       
        clear_queue(alvar_queue)
          
        # Alvar didn't find all nodes so look at map for possible locations  
        if(alvar_finished == 0):
            explorer._save_map()
            explorer._open_cv_map()
            
            while(not(map_queue.empty())):
                bunny = map_queue.get()
                possible_bunnies.append(bunny)
                
            # Get all markers
            tag_tracker._return_markers()
            
            while(not(alvar_queue.empty())):
                marker = alvar_queue.get()
                found_markers.append(marker)
                rospy.loginfo('Storing Marker %d at: X: %f, Y:, %f', marker[0], marker[1], marker[2])
            
            
            del alvar_queue
            del map_queue
            del explorer
            del explorer_thread
            del tag_tracker
            del tag_tracker_thread
            
            return 'map_found_none'
            
            if(len(possible_bunnies) > 0):
                return 'map_found_possible'
            else:
                return 'map_found_none'
                
        else: 
        
            tag_tracker._return_markers()
            
            while(not(alvar_queue.empty())):
                marker = alvar_queue.get()
                found_markers.append(marker)
                rospy.loginfo('Storing Marker %d at: X: %f, Y:, %f', marker[0], marker[1], marker[2])
        
            del alvar_queue
            del map_queue
            del explorer
            del explorer_thread
            del tag_tracker
            del tag_tracker_thread
        
            return 'map_found_all'

class go_home_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['home_pass', 'home_fail'])

    def execute(self, userdata):
        rospy.loginfo('Entering home State')
        home_mover = move_to.move_to()
        
        home_mover._move_to_goal(0,0,0)
        
        status = 0
        
        while(status == 0):
            status = home_mover._check_goal_status(30)
            pass
            
        del home_mover
            
        if(status == 1):
            return 'home_pass'
        else:
            return 'home_fail'

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
        global found_markers
        rospy.loginfo('Entering explore State')
        
        alvar2_queue = Queue.Queue()
        rand_queue = Queue.Queue()
        
        # Setup the wall following thread
        exp_rand = explore_random.explore_random(rand_queue)       
        exp_rand_thread = myThread("Random Explore Thread", exp_rand._run)
        exp_rand_thread.start()
        
        tag_tracker2 = alvar_tracker.alvar_tracker(alvar2_queue, found_markers, expected_bunnies)
        tag_tracker2_thread = myThread("Alvar Thread", tag_tracker2._run)
        tag_tracker2_thread.start()
        
        alvar2_finished = 0
        
        while(exp_rand_thread.isAlive()) :
            if(tag_tracker2_thread.isAlive()):
                pass
            else:
                alvar2_finished = 1
                kill_thread(exp_rand_thread, rand_queue)
                clear_queue(rand_queue)
                
                
        kill_thread(tag_tracker2_thread, alvar2_queue)       
        clear_queue(alvar2_queue)
        
        del found_markers[:] 
        
        if(alvar2_finished == 1):
        
            # Get all markers
            tag_tracker2._return_markers()
            
            while(not(alvar2_queue.empty())):
                marker = alvar2_queue.get()
                found_markers.append(marker)
                rospy.loginfo('Storing Marker %d at: X: %f, Y:, %f', marker[0], marker[1], marker[2])
        
            del alvar2_queue
            del rand_queue
            del exp_rand
            del exp_rand_thread
            del tag_tracker2
            del tag_tracker2_thread
        
            return 'explore_pass'
            
        else:
            # Get all markers
            tag_tracker2._return_markers()
            
            while(not(alvar2_queue.empty())):
                marker = alvar2_queue.get()
                found_markers.append(marker)
                rospy.loginfo('Storing Marker %d at: X: %f, Y:, %f', marker[0], marker[1], marker[2])
                
            del alvar2_queue
            del rand_queue
            del exp_rand
            del exp_rand_thread
            del tag_tracker2
            del tag_tracker2_thread
                
            return 'explore_fail'
                     
class wait_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait_target_acquired', 'wait_for_target'])

    def execute(self, userdata):
        global target_index
        rospy.loginfo('Entering wait State')
        
        time.sleep(5)       
        
        
        empty = []
        
        target_queue = Queue.Queue()
        target_tracker = alvar_tracker.alvar_tracker(target_queue, empty, 1)
        target_tracker_thread = myThread("Target Thread", target_tracker._run)
        target_tracker_thread.start()
        
        rospy.loginfo('Waiting for marker')
        
        while(target_tracker_thread.isAlive()) :
            pass
            
        kill_thread(target_tracker_thread, target_queue)       
        clear_queue(target_queue)
            
        rospy.loginfo('Found Marker')    
        target_tracker._return_last_marker()
        
        target_marker = target_queue.get()
        
        rospy.set_param('rabbit_id', target_marker[0])
        rospy.set_param('colors', [0,0,0,0,0,0])
        
        index = 0
        good_target = 0
        
        coord = [target_marker[1], target_marker[2], 0]
        
        distance = np.linalg.norm(coord)
        
        rospy.loginfo('Marker is %3.3f meters away', distance) 
        
        if(distance < 1.5):
        
            rospy.loginfo('Marker ID is %d', target_marker[0])    
            
            while((index < len(found_markers)) and not(good_target)):
                if(found_markers[index][0] == target_marker[0]):
                    good_target = 1
                    target_index = index
                index = index + 1
                    
            rospy.loginfo('Going to marker %d, index %d', target_marker[0], target_index)  
        else:
            rospy.loginfo('Target Marker %d is too far away', target_marker[0])    
                
        del target_queue
        del target_tracker
        del target_tracker_thread
                
        if(good_target == 1):
            return 'wait_target_acquired'
        else:
            return 'wait_for_target'
        
class nav_to_target_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['nav_pass', 'nav_fail'])
        self.start_bcst = tf.TransformBroadcaster()

    def execute(self, userdata):
        rospy.loginfo('Entering nav State')
        
        rate = rospy.Rate(10)
        time.sleep(2)  
        
        empty = []
        status = 0
        found_marker = 0
        offset = 0.875
        
        # Alvar Tracker for the target
        target2_queue = Queue.Queue()
        target_tracker2 = alvar_tracker.alvar_tracker(target2_queue, empty, -1)
        ar_frame = "ar_marker_" + str(found_markers[target_index][0])
        
        # Target pose and mover
        tar_x = found_markers[target_index][1]
        tar_y = found_markers[target_index][2]
        tar_rx = found_markers[target_index][4]
        tar_ry = found_markers[target_index][5]
        tar_rz = found_markers[target_index][6]
        tar_rw = found_markers[target_index][7]
        target_mover = move_to.move_to()
        target_mover._move_to_pose_backoff(tar_x, tar_y, tar_rx, tar_ry, tar_rz, tar_rw, offset)
        
        # Wait until we find the alvar marker again
        while((status < 1) and (found_marker == 0)):
            (success, trans, rot) = target_tracker2._get_transform("base_link", ar_frame, 2.0)
        
            if(success == 1):
                coord = [trans[0], trans[1], 0]
                
                distance = np.linalg.norm(coord)
                
                rospy.loginfo('Founds Marker %3.3f away', distance)
                
                if(distance < 1.5):
                    target_mover._cancel_goal()
                    
                    offset = 1.25
                    
                    time.sleep(3)  
                    
                    (success, trans, rot) = target_tracker2._get_transform("map", ar_frame, 2.0)
                    
                    if(success == 1):
                        dx = found_markers[target_index][1] - trans[0]
                        dy = found_markers[target_index][2] - trans[1]
                        
                        coord = [dx, dy, 0]
                        
                        distance = np.linalg.norm(coord)
                        
                        rospy.loginfo('Founds Marker %3.3f away', distance)
                        
                        if(distance < 0.5):
                            found_marker = 1
                        
                            target_mover._cancel_goal()
                            
                            tar_x = trans[0]
                            tar_y = trans[1]
                            tar_rx = rot[0]
                            tar_ry = rot[1]
                            tar_rz = rot[2]
                            tar_rw = rot[3]
                            
                            offset = 0.75
                        
                    target_mover._move_to_pose_backoff(tar_x, tar_y, tar_rx, tar_ry, tar_rz, tar_rw, offset)
                    time.sleep(1)
                
            status = target_mover._check_goal_status(30)
            
            if(status < 0):
                target_mover._move_to_pose_backoff(tar_x, tar_y, tar_rx, tar_ry, tar_rz, tar_rw, offset)
            
            rate.sleep()
            
        rospy.loginfo('Going to final marker')
            
        while(status < 1):
            status = target_mover._check_goal_status(30)
            if(status < 0):
                target_mover._move_to_pose_backoff(tar_x, tar_y, tar_rx, tar_ry, tar_rz, tar_rw, 0.75)
            rate.sleep()
               
        del target_mover
            
        if(status == 1):
            return 'nav_pass'
        else:
            return 'nav_fail'
            
            
    def _publish_tf(self, marker):
    
        ar_frame = "target_marker_" + str(marker[0])    
        
        self.start_bcst.sendTransform( (marker[1], marker[2], marker[3]),
                                       (marker[4], marker[5], marker[6], marker[7]),
                                       rospy.Time.now(), ar_frame, "map")
        
class count_eggs_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['count_pass', 'count_fail', 'count_error'])
        
    def image_callback(self, msg):
        global eggs_counted
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")	#convert image
        print eggs.count_eggs(image)	#count the eggs
        rospy.set_param('colors', eggs.count_eggs(image))
        self.eggs_counted=1	#set flag
        self.image_sb.unregister()	#unsubscribe
        
    def execute(self, userdata):
        rospy.loginfo('Entering count State')
        self.bridge = cv_bridge.CvBridge()
        self.eggs_counted = 0
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback) #subscribe to usb_cam topic
        while (self.eggs_counted==0):	#loop until eggs have been counted
            pass
        
        return 'count_pass'
                        
# Error
class error_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_done'])

    def execute(self, userdata):
        while not(rospy.is_shutdown()):
            pass


def main():
    global found_markers
    rospy.init_node("jackal_explore")
    
    del found_markers[:] 

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
        
    time.sleep(2)
    main()





