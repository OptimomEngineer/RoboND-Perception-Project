#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle
import pcl

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32

from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml
from std_srvs.srv import Empty

#from arm mover to help with robot mapping collision and turning -pi/2 and pi/2
def at_goal(pos_j1, goal_j1):
    tolerance = .05
    result = abs(pos_j1 - goal_j1) <= abs(tolerance)
    return result

#from arm mover to help with robot mapping collision and turning -pi/2 and pi/2	
def move_pr2_robot(pos_j1):
    time_elapsed = rospy.Time.now()
    while True:
        joint_state = rospy.wait_for_message('/pr2/joint_states', JointState)
        if at_goal(joint_state.position[0], pos_j1):
	    pr2_base_mover_pub.publish(joint_state.position[0])
            time_elapsed = joint_state.header.stamp - time_elapsed
            break
    
    return joint_state.position[0]


    
# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
     
    global ros_cloud_table
    global detected_objects
    print("I am doing PCL callback code")
    # TODO: Convert ROS msg to PCL data, do this by using the pcl_helper functions:
    cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Voxel Grid Downsampling (http://pointclouds.org/documentation/tutorials/voxel_grid.php)
    #create a voxel grid filter object
    vox = cloud.make_voxel_grid_filter()
    #define leaf size and set the leave/voxel size
    LEAF_SIZE = .005
    vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE, LEAF_SIZE) #3Dimensional
    #call on filter and save pcl file.
    cloud_filtered = vox.filter()
     
    # TODO: Statistical Outlier Filtering (http://pointclouds.org/documentation/tutorials/statistical_outlier.php)
    #Create a filter object
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    #set neighboring point to analyze, here i used 50
    outlier_filter.set_mean_k(50)

    #set your multiplier: here if there is a mean distance larger than mean distance + x + std_dev we make it an outlier
    outlier_filter.set_std_dev_mul_thresh(.05)
    #now call the filter to your object
    cloud_filtered = outlier_filter.filter()
      
    # TODO: PassThrough Filter
    #create passthrough filter object for bins
    #assign axis and range to the passthrough filter object for y
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('y')
    passthrough.set_filter_limits(-.45, .45)
    #now appliy the filter and save the point cloud
    cloud_filtered = passthrough.filter()

    #extra x axis filter since we can't see eraser object
    #assign axis and range to the passthrough filter object for x
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('x')
    passthrough.set_filter_limits(.37, 4.7)
    #now appliy the filter and save the point cloud
    cloud_filtered = passthrough.filter()

    # Create a PassThrough filter object for z.
    # Assign axis and range to the passthrough filter object for z
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(.08, 2.5)
    #now appliy the filter and save the point cloud
    cloud_filtered = passthrough.filter()

    
    

    # TODO: RANSAC Plane Segmentation to remove the table
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit which is a plane mode in our case
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance .01 for a point to be considered fitting the model for plane (table segementation)
    # i tried different ones but ended up using .01
    seg.set_distance_threshold(.01)


    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()

    
    # Extract outliers 
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)

 

    # TODO: Euclidean Clustering
    #take cloud objects and change to xyz using the helper code
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    #make a kd tree
    tree = white_cloud.make_kdtree()
    #create an extraction cluster object
    ec_object = white_cloud.make_EuclideanClusterExtraction()
    #set tolerance and min and max parameters
    ec_object.set_ClusterTolerance(.02)
    ec_object.set_MinClusterSize(50)
    ec_object.set_MaxClusterSize(5000)
    #search tree for clusters
    ec_object.set_SearchMethod(tree)
    #extract the indices for each cluster
    cluster_indices = ec_object.Extract()
    

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
	for i, indice in enumerate(indices):
		color_cluster_point_list.append([white_cloud[indice][0], 
						white_cloud[indice][1],
						white_cloud[indice][2],
						rgb_to_float(cluster_color[j])])
    #create new cloud containing all clusters, each with a unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)


    # TODO: Convert PCL data to ROS messages
    #ros_collision_map = pcl_to_ros(collision_map) 
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    


    # Classify the clusters! (loop through each detected cluster one at a time
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices): 
        # Grab the points for the cluster
	pcl_cluster = cloud_objects.extract(pts_list)
	ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
         

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .2
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
    	do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    
    # Publish the list of detected objects before calling pr2_mover()
    if detected_objects:
        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
	detected_objects_pub.publish(detected_objects)
        try:
	    pr2_mover(detected_objects_labels)
    	except rospy.ROSInterruptException:
            pass
    else: 
        rospy.loginfo("No objects detected!") 


def pr2_mover(object_list): 

    # TODO: Initialize variables to use for sending message to pickplace service
    print("I am doing Mover code")
    test_scene_num = Int32()
    object_name    = String()
    arm_name       = String()    
    pick_pose      = Pose()
    place_pose     = Pose()
    yaml_dict_list = []

    # TODO: Get/Read parametersgedit 
    object_list_param = rospy.get_param('/object_list')
    box_param = rospy.get_param('/dropbox')
    if len(object_list_param) == 3:	
   	test_scene_num.data = 1
    elif len(object_list_param) == 5:
	test_scene_num.data = 2
    else:
	test_scene_num.data = 3

    rospy.loginfo('Object pick list, Dectected {} Objects, Setting scene number {}'.format(len(detected_objects), test_scene_num))


    # TODO: Parse parameters into individual variables
    red = box_param[0]['position']
    green = box_param[1]['position']        
    
#    # TODO: Rotate PR2 in place to capture side tables for the collision map, use simple mover - joint states topic.
#    rospy.loginfo('Rotating Robot to capture collision map')
#    right = 1.5 #around pi/2
#    left = -1.5 #around -pi/2

#    rospy.loginfo('Rotating pr2 to the right')
#    final_position_right = move_pr2_robot(right)
#    rospy.loginfo('Completed rotating pr2 to the right at {}, 1 cycle sleep'.format(final_position_right))
#    rate = rospy.Rate(10)
#    rate.sleep()

#   
#    rospy.loginfo('Rotating pr2 to the left')
#    final_position_left = move_pr2_robot(left)
#    rospy.loginfo('Completed rotating pr2 to the left at {}, 1 cycle sleep'.format(final_position_left))
#    rate = rospy.Rate(10)
#    rate.sleep()
#    
#    rospy.loginfo('Rotating pr2 to the center')
#    final_position_center = move_pr2_robot(0.0)
#    rospy.loginfo('Completed rotating pr2 to the center at {}, 1 cycle sleep'.format(final_position_center))
#    rate = rospy.Rate(10)
#    rate.sleep()

#    rospy.loginfo('Rotation for collision map is complete')


    # TODO: Get the PointCloud for a given object and obtain it's centroid, moved out of loop
    labels = []
    centroids = [] # to be list of tuples (x, y, z)np.asscalar(centroids[index][0])
    for obj in detected_objects:
	labels.append(obj.label)
    	points_arr = ros_to_pcl(obj.cloud).to_array()
    	centroids.append(np.mean(points_arr, axis=0)[:3])
	
    # TODO: Loop through the pick list
    yaml_dict_list = []
    for i in range(0, len(detected_objects)):
        
        # Read object name and group from object list.
        object_name.data = object_list_param[i]['name' ]
        object_group = object_list_param[i]['group']  
        rospy.loginfo('Plan to pick up {} and move to {} labeled box'.format(object_name.data, object_group))

	#create collision map without object (ie, remove object)
	#i could not figure out how to use the extact without getting indices, so i just decided to make 	 a new pointcloud and append everthing except our desired object.
	collision_map_objects = detected_objects  	
	collision_map = []
 
	for obj in collision_map_objects:
		if obj.label != object_name.data: 
			#if object is not our desired object, lets add to collision_map
			#need pcl helper code to append pcl cloud objects since cannot append directly.
			for data in pc2.read_points(obj.cloud, skip_nans=True):
        			collision_map.append([data[0], data[1], data[2], data[3]])
	#now i want to append a ros table cloud, so i made it a global variable.		
	for data in pc2.read_points(ros_cloud_table, skip_nans = True):
		collision_map.append([data[0], data[1], data[2], data[3]])
	collision_map_final = pcl.PointCloud_PointXYZRGB() #from pcl helper, turn into pointcloud
    	collision_map_final.from_list(collision_map) #from pcl helper, turn into pointcloud	
	#convert to ros map and publish	
	ros_collision_map = pcl_to_ros(collision_map_final)
	pr2_collision_pub.publish(ros_collision_map)
		
	rospy.loginfo('Published collision map to /pr2/3d_map/points') 
        
 
        # TODO: Create 'place_pose' for the object, this would be the x,y,z, from the centroids list.
	#http://docs.ros.org/api/geometry_msgs/html/msg/Point.html
	#pick the pose from the centroids of the object
	pick_pose.position.x = np.asscalar(centroids[i][0])
        pick_pose.position.y = np.asscalar(centroids[i][1])
        pick_pose.position.z = np.asscalar(centroids[i][2])

	#match the color and pick the place (red or green for corresponding left or right box)
        # TODO: Assign the arm to be used for pick_place and therfore also place position.
 	if object_group == box_param[0]['group']:
		arm_name.data = box_param[0]['name']
		place_pose.position.x = red[0]
		place_pose.position.y = red[1]
		place_pose.position.z = red[2]   	
	else: 
		arm_name.data = box_param[1]['name']
		place_pose.position.x = green[0]
		place_pose.position.y = green[1]
		place_pose.position.z = green[2]
       
	# TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
    	yaml_dict_list.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))
   	
        # TODO: Output your request parameters into output yaml file
        file_name = "output_{}.yaml".format(test_scene_num.data)
        send_to_yaml(file_name, yaml_dict_list)

        #Wait for 'pick_place_routine' service to come up
    	rospy.wait_for_service('pick_place_routine')
    	try:
		pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
        	# TODO: Insert your message variables to be sent as a service request
		
        	resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
		        	
		print ("Response: ",resp.success)
		
		
    	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e
	
	#here I found this book to help me clear the collision map for next object
	#Programming Robots with ROS: A Practical Introduction to the Robot Operating System, pg 248	
	rospy.wait_for_service("/clear_octomap")
	clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
	clear_octomap()
	rospy.loginfo('Cleared collision map to /pr2/3d_map/points')


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous = True)
    
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size =1)
    #joint_state = rospy.Subscriber("/pr2/joint_states", JointState, move_pr2_robot, queue_size =1)
    
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size =1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size =1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size =1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size =1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size =1)
    pr2_base_mover_pub = rospy.Publisher("/pr2/world_joint_controller/command", Float64, queue_size=1)
    pr2_collision_pub = rospy.Publisher("/pr2/3d_map/points",PointCloud2, queue_size =1)

    # Initialize color_list
    get_color_list.color_list = []

    #Load the SVM Model from disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
	rospy.spin()
