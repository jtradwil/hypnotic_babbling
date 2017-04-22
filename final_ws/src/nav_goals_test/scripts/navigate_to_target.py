

#define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
        time.sleep(1)

    goal = MoveBaseGoal()

    # set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards target position
    if(target = 1):
    	goal.target_pose.pose.position = Point(xpos,ypos,0)
    	goal.target_pose.pose.orientation.x = x_1
    	goal.target_pose.pose.orientation.y = y_1
    	goal.target_pose.pose.orientation.z = 0.0
    	goal.target_pose.pose.orientation.w = 1.0
    if(target =2):
    	goal.target_pose.pose.position = Point(xpos,ypos,0)
    	goal.target_pose.pose.orientation.x = x_2
    	goal.target_pose.pose.orientation.y = y_2
    	goal.target_pose.pose.orientation.z = 0.0
    	goal.target_pose.pose.orientation.w = 1.0
    if(target = 3):
    	goal.target_pose.pose.position = Point(xpos,ypos,0)
    	goal.target_pose.pose.orientation.x = x_3
    	goal.target_pose.pose.orientation.y = y_3
    	goal.target_pose.pose.orientation.z = 0.0
    	goal.target_pose.pose.orientation.w = 1.0
	
    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)

    #ac.wait_for_result(rospy.Duration(30))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True

    else:
        ac.cancel_goal() # Remove old goal that wasnt reached
        rospy.loginfo("The robot failed to reach the destination")
        return False


