#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
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
    # CONST
    VOXEL_LEAF_SIZE                 = 0.005
    NOISE_THRESHOLD_SCALE_FACTOR    = 0.05
    NOISE_MEAN_K                    = 3
    PASS_THROUGH_AXIS_MIN_Z         = 0.5
    PASS_THROUGH_AXIS_MAX_Z         = 0.8
    PASS_THROUGH_AXIS_MIN_X         = 0.4
    PASS_THROUGH_AXIS_MAX_X         = 2.0
    RANSAC_MAX_DISTANCE             = 0.01
    CLUSTER_TOLERANCE               = 0.05
    MIN_CLUSTER_SIZE                = 50
    MAX_CLUSTER_SIZE                = 10000

    # Convert ROS msg to PCL data
    pclMsg = ros_to_pcl(pcl_msg)

    # Voxel Grid Downsampling
    vox = pclMsg.make_voxel_grid_filter()
    vox.set_leaf_size(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE)
    cloud_filtered = vox.filter()

    # Apply statistical outlier filter to get rid of noise
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(NOISE_MEAN_K)
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(NOISE_THRESHOLD_SCALE_FACTOR)
    cloud_filtered = outlier_filter.filter()

    # PassThrough Filter (horizontal)
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(PASS_THROUGH_AXIS_MIN_Z, PASS_THROUGH_AXIS_MAX_Z)
    cloud_filtered = passthrough.filter()

    # PassThrough Filter (vertical)
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(PASS_THROUGH_AXIS_MIN_X, PASS_THROUGH_AXIS_MAX_X)
    cloud_filtered = passthrough.filter()

    # RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(RANSAC_MAX_DISTANCE)
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    pcl_cloud_table   = cloud_filtered.extract(inliers, negative=False)
    pcl_cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(pcl_cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(CLUSTER_TOLERANCE)
    ec.set_MinClusterSize(MIN_CLUSTER_SIZE)
    ec.set_MaxClusterSize(MAX_CLUSTER_SIZE)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(pcl_cloud_objects)
    ros_cloud_table = pcl_to_ros(pcl_cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)


    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects_list   = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = pcl_cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects_list.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects_list)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    from std_msgs.msg import Int32
    from std_msgs.msg import String
    from geometry_msgs.msg import Pose

    test_scene_num = Int32()

    object_name = String()
    arm_name    = String()

    pick_pose  = Pose()
    place_pose = Pose()

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param     = rospy.get_param('/dropbox')
    test_scene_param  = rospy.get_param('/test_scene')

    # Parse parameters into individual variables
    test_scene_num.data = test_scene_param['num']
    object_names  = []
    object_groups = []
    for object_param in object_list_param:
        object_names.append(object_param['name'])
        object_groups.append(object_param['group'])

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Loop through the pick list
    dict_list = []
    for object in object_list:
        # Get the PointCloud for a given object and obtain it's centroid
        if object.label in object_names:
            object_name.data = str(object.label)

            points_arr = ros_to_pcl(object.cloud).to_array()
            centroid = np.mean(points_arr, axis=0)[:3]

            #Create 'place_pose' for the object
            pick_pose.position.x = np.asscalar(centroid[0])
            pick_pose.position.y = np.asscalar(centroid[1])
            pick_pose.position.z = np.asscalar(centroid[2])

            # Assign the arm to be used for pick_place
            object_idx = object_names.index(object.label)
            if object_groups[object_idx] == 'green':
                arm_name.data = 'right'
                position = dropbox_param[1]['position']
            else:
                arm_name.data = 'left'
                position = dropbox_param[0]['position']
            place_pose.position.x = position[0]
            place_pose.position.y = position[1]
            place_pose.position.z = position[2]

            # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
            dict_list.append(yaml_dict)

#            # Wait for 'pick_place_routine' service to come up
#            rospy.wait_for_service('pick_place_routine')
#
#            try:
#                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
#
#                # Insert message variables to be sent as a service request
#                resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
#
#                print ("Response: ",resp.success)
#
#            except rospy.ServiceException, e:
#                print "Service call failed: %s"%e
        else:
            print "Object is not recognized"

    # Output request parameters into output yaml file
    send_to_yaml("output_" + str(test_scene_num.data) + ".yaml", dict_list)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    object_markers_pub   = rospy.Publisher("/object_markers",   Marker,                 queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray,   queue_size=1)
    pcl_objects_pub      = rospy.Publisher("/pcl_objects",      PointCloud2,            queue_size=1)
    pcl_table_pub        = rospy.Publisher("/pcl_table",        PointCloud2,            queue_size=1)
    pcl_cluster_pub      = rospy.Publisher("/pcl_cluster",      PointCloud2,            queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_= model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
