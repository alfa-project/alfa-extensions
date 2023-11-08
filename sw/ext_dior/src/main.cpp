/*
 * Copyright 2023 ALFA Project. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <time.h>

#include <cmath>
#include <vector>

#include "alfa_node/alfa_node.hpp"
#include "rclcpp/rclcpp.hpp"

// Node parameters
#define NODE_NAME "dior"
#define NODE_ID 0
#define POINTCLOUD_ID 0
#define DEFAULT_TOPIC "/velodyne_points"

// Dior definitions
#define INTENSITY_PARAM_ID 0 //intensity thr
#define DISTANCE_TH_PARAM_ID 1 // radius multiplier
#define SEARCH_RADIUS_MIN_PARAM_ID 2 //min search radius
#define NUMBER_OF_NEIGH_TH_PARAM_ID 3 //neighbou thr
#define ANGLE_RES_PARAM_ID 4 // angular res
#define NUMBERTHREADS_ID 5

/**
 * This function performs a dynamic radius search for a given point using a
 * KdTree search algorithm.
 *
 * @param point The point for which the search is performed
 * @param kdtree The KdTree data structure that is used for the search
 * @param angle_resolution The angle resolution used for the search
 * @param search_radius_min The minimum search radius used for the search
 * @param distance_th The distance threshold used for the search
 * @param number_of_neigh_th The number of neighbors threshold used for the
 * search
 *
 * @return Returns true if the number of neighbors found is greater than or
 * equal to the specified threshold, false otherwise
 */

bool alfa_dynamicRadiusSearch(PointXYZIL point,
    pcl::KdTreeFLANN<PointXYZIL> kdtree,
    float angle_resolution,
    float search_radius_min,
    float distance_th,
    int number_of_neigh_th)
{

    float distance = sqrt(pow(point.x, 2) + pow(point.y, 2));
    float search_radius;

    if (distance < distance_th) {
        search_radius = search_radius_min;
    } else {
        search_radius = search_radius_min * (distance * angle_resolution);
    }

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int neighbors = kdtree.radiusSearch(
        point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    if (neighbors >= number_of_neigh_th) {
        return true;
    }
    return false;
}

/**
 * @brief Performs processing on a sequence of points for a given node.
 *
 * This function takes in a pointer to a node and performs processing on a
 * sequence of points using the node's stored parameters. The function uses the
 * node's Kdtree to search for neighbors and applies a dynamic radius search
 * based on the distance to the point and the node's angle resolution, search
 * radius minimum, and distance threshold parameters. Points that meet the
 * intensity threshold or dynamic radius search criteria are set for the node.
 *
 * @param node Pointer to the node object to perform processing on.
 */
void worker(AlfaNode* node)
{
    pcl::KdTreeFLANN<PointXYZIL> kdtree;
    node->get_kdtree_input_cloud(kdtree);

    float angle_resolution, search_radius_min, distance_th, intensity_th;
    int number_of_neigh_th;
 
    intensity_th = node->get_extension_parameter("intensity_thr");
    angle_resolution = node->get_extension_parameter("angular_res");
    search_radius_min = node->get_extension_parameter("min_search_radius");
    distance_th = node->get_extension_parameter("distance_thr");
    number_of_neigh_th = node->get_extension_parameter("num_neighbors_thr");

    while (!node->is_last_input_cloud_point()) {
        PointXYZIL point;
        node->get_point_sequentially_input_cloud(point);

        if (point.intensity > intensity_th) {
            node->push_point_output_cloud(point); 
        } else if (alfa_dynamicRadiusSearch(point,
                       kdtree,
                       angle_resolution,
                       search_radius_min,
                       distance_th,
                       number_of_neigh_th)) {
            node->push_point_output_cloud(point);
        }
    }
}

/**
 * @brief Sets the number of threads and assigns processing to each thread.
 *
 * This function takes in a pointer to a node, sets the number of threads to use
 * for processing, and assigns the `worker` function to each thread. The
 * `worker` function is called with the given node and runs in parallel across
 * the specified number of threads.
 *
 * @param node Pointer to the node object to perform processing on.
 */
void handler(AlfaNode* node)
{
    std::cout << "Point cloud received" << std::endl;
    node->set_multi_thread(
        (int)node->get_extension_parameter("num_threads"), &worker, node);
}

/**
 * @brief Post-processing function that publishes output point cloud and
 * metrics.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void post_processing(AlfaNode* node)
{
    node->publish_output_cloud();

    alfa_msg::msg::AlfaMetrics output_metrics;

    output_metrics.metrics.push_back(node->get_handler_time());
    output_metrics.metrics.push_back(node->get_full_processing_time());

    node->publish_metrics(output_metrics);
}

/**
 * @brief Entry point of the program.
 *
 * @param argc The number of arguments.
 * @param argv The array of arguments.
 * @return int The exit code of the program.
 */
int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Prepare the DIOR default configuration
    AlfaExtensionParameter parameters[10];

    parameters[INTENSITY_PARAM_ID].parameter_value = 4.0;
    parameters[INTENSITY_PARAM_ID].parameter_name = "intensity_thr";

    parameters[DISTANCE_TH_PARAM_ID].parameter_value = 0.9;
    parameters[DISTANCE_TH_PARAM_ID].parameter_name = "distance_thr";

    parameters[SEARCH_RADIUS_MIN_PARAM_ID].parameter_value = 0.1;
    parameters[SEARCH_RADIUS_MIN_PARAM_ID].parameter_name = "min_search_radius";

    parameters[NUMBER_OF_NEIGH_TH_PARAM_ID].parameter_value = 30.0;
    parameters[NUMBER_OF_NEIGH_TH_PARAM_ID].parameter_name = "num_neighbors_thr";

    parameters[ANGLE_RES_PARAM_ID].parameter_value = 0.3;
    parameters[ANGLE_RES_PARAM_ID].parameter_name = "angular_res";

    parameters[NUMBERTHREADS_ID].parameter_value = 2;
    parameters[NUMBERTHREADS_ID].parameter_name = "num_threads";

    std::string subscriber_topic = DEFAULT_TOPIC;
    if (argc > 1) {
        subscriber_topic = argv[1];
    }

    // Print the characteristics of the node
    std::cout << "Starting ALFA node with the following settings" << std::endl;
    std::cout << "Subscriber topic: " << subscriber_topic << std::endl;
    std::cout << "Name of the node: " << NODE_NAME << std::endl;
    std::cout << "Parameters: parameter list" << std::endl;
    std::cout << "Extension ID: " << NODE_ID << std::endl;
    std::cout << "Pointcloud ID: " << POINTCLOUD_ID << std::endl;
    std::cout << "Hardware Driver (SIU): false" << std::endl;
    std::cout << "Hardware Extension: false" << std::endl;
    std::cout << "Handler Function: handler" << std::endl;
    std::cout << "Post Processing Function: post_processing" << std::endl;

    // Create an instance of AlfaNode and spin it
    rclcpp::spin(std::make_shared<AlfaNode>(subscriber_topic,
        NODE_NAME,
        parameters,
        NODE_ID,
        POINTCLOUD_ID,
        AlfaHardwareSupport { false, false },
        &handler,
        &post_processing)); 

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
