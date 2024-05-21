
#include <vector>
#include <cmath>

#include "alfa_defines.hpp"
#include "alfa_node.hpp"
#include "alfa_structs.hpp"

#include "rclcpp/rclcpp.hpp"

#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "metrics.hpp"

#define NODE_NAME "ext_anand"
#define DEFAULT_TOPIC "/velodyne_points"

#define NODE_ID 0
#define POINTCLOUD_ID 0
#define METRICS ALL_METRICS // FULL_PROCESSING, HANDLER_PROCESSING 



using PointType = AlfaPoint;

boost::shared_ptr<Metrics<PointType>> AnandMetrics;

int ground_points = 0;
int outside_points = 0;
int frames = 0;

void callback_shutdown()
{
    AnandMetrics->callback_shutdown();
}

int estimate_ground(AlfaNode * node, pcl::PointCloud<PointType> input_cloud, 
                    float zeta, float epsilon, float delta)
{
    double x_min = 0;
    double x_max = 0;

    double y_min = 0;
    double y_max = 0;

    double z_min = 0;
    double z_max = 0;

    int number_ground_points = 0;

    /* BUILD THE GRID */
    for (const auto &point : input_cloud.points)
    {
        if (point.x < x_min)
            x_min = point.x;
            
        if (point.x > x_max)
            x_max = point.x;

        if (point.y < y_min)
            y_min = point.y;
            
        if (point.y > y_max)
            y_max = point.y;         
    }

    float csize = 0.5;

    int num_cells_x = std::ceil((x_max - x_min) / csize);
    int num_cells_y = std::ceil((y_max - y_min) / csize);

    std::vector<std::vector<std::vector<int>>> grid (num_cells_x, std::vector<std::vector<int>>(num_cells_y));

    pcl::PointCloud<PointType> output_cloud;
    output_cloud.resize(input_cloud.size());
    output_cloud = input_cloud;

    for (int i = 0; i < input_cloud.size(); i++)
    {
        int cell_x = std::floor((input_cloud.points[i].x - x_min) / csize);
        int cell_y = std::floor((input_cloud.points[i].y - y_min) / csize);

        if (cell_x == num_cells_x)
            cell_x -= 1;

        if (cell_y == num_cells_y)
            cell_y -= 1;

        grid[cell_x][cell_y].push_back(i);
    }

    /* GRID ITERATION */
    for (int j = 0; j < num_cells_y; j++)
    {
        for (int i = 0; i < num_cells_x; i++)
        {
            if (!grid[i][j].empty())
            {
                z_min = 10;
                z_max = 0;
                for (const auto index : grid[i][j]) // FIND CELL MIN AND MAX Z
                {
                    if (input_cloud.points[index].z < z_min)
                        z_min = input_cloud.points[index].z;
                    if (input_cloud.points[index].z > z_max)
                        z_max = input_cloud.points[index].z;
                }

                /* CLASSIFICATION */

                double z_sub = z_max - z_min;
                double z_th = z_sub / 5; // THRESHOLD

                if (z_min < zeta) // Cell contains ground points
                {
                    if (z_sub >= epsilon) // Cell contains ground points, all of which are below z_min + delta
                    {
                        for (const auto index : grid[i][j])
                        {
                            output_cloud.points[index].custom_field = 0;

                            if (input_cloud.points[index].z < (z_min + delta)) // GROUND
                            {
                                number_ground_points++;
                                output_cloud.points[index].custom_field = 1;
                            } 
                        }
                    }

                    else // Cell contains a small object, with ground points being those below z_min + z_th
                    {
                        for (const auto index : grid[i][j])
                        {
                            output_cloud[index].custom_field = 0;

                            if (input_cloud.points[index].z < (z_min + z_th)) // GROUND
                            {
                                number_ground_points++;
                                output_cloud.points[index].custom_field = 1;
                            }
                        }
                    }
                }
                else // Cell does not contain ground points
                {
                    for (const auto index : grid[i][j])
                    {
                        output_cloud[index].custom_field = 0;
                    }
                }
            }
        }
    }

    for (const auto &point : output_cloud.points)
    {
        node->push_point_output_pointcloud(point);
    }

    return number_ground_points;
}

/**
 * @brief Handler function that applies a distance filter to the received point cloud.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void handler (AlfaNode * node)
{
    #ifdef EXT_HARDWARE
         node->store_pointcloud(LOAD_STORE_CARTESIAN,node->get_input_pointcloud());
    #else
        // Simple example code aplying a distance filter to the pointcloud
        // Get PCL pointcloud

        float zeta = node->get_extension_parameter("zeta");
        float epsilon = node->get_extension_parameter("epsilon");
        float delta = node->get_extension_parameter("delta");

        ground_points = estimate_ground(node, *(node->get_input_pointcloud()), zeta, epsilon, delta);
    #endif
}


/**
 * @brief Post-processing function that publishes output point cloud and metrics.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void post_processing (AlfaNode * node)
{
    outside_points = 0;
    #ifdef EXT_HARDWARE
        node->load_pointcloud(LOAD_STORE_CARTESIAN,node->get_output_pointcloud());
        ground_points = node->get_debug_point(0);
        outside_points = node->get_debug_point(1);
    #endif
    
    AnandMetrics->post_processing(node->get_metric_message(HANDLER_TIME), node->get_metric_message(FULL_PROCESSING_TIME));
//    AnandMetrics->calculate_metrics(*(node->get_input_pointcloud()), *(node->get_output_pointcloud()), (float)ground_points, (float)outside_points);

    node->publish_pointcloud();
}

/**
 * @brief Entry point of the program.
 * 
 * @param argc The number of arguments.
 * @param argv The array of arguments.
 * @return int The exit code of the program.
 */
int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Get subscriber topic from command line arguments
    std::string subscriber_topic = DEFAULT_TOPIC;
    if (argc > 1) {
        subscriber_topic = argv[1];
    }

    #ifdef EXT_HARDWARE
        AlfaHardwareSupport hardware_support{false, true};
    #else
        AlfaHardwareSupport hardware_support{false, false};
    #endif
	
    AlfaConfiguration conf;
    AlfaExtensionParameter parameters[30];

    conf.subscriber_topic = subscriber_topic;
    conf.node_name = NODE_NAME;
    conf.pointcloud_id = POINTCLOUD_ID;
    conf.extension_id = NODE_ID;
    conf.hardware_support = hardware_support;
    conf.latency = 0;
    conf.number_of_debug_points = 2;
    conf.metrics_publishing_type = METRICS;
    conf.custom_field_conversion_type = CUSTOM_FIELD_LABEL;

    parameters[0].parameter_name = "zeta"; // threshold
    parameters[0].parameter_value = -1.0;   //[m]

    parameters[1].parameter_name = "epsilon"; // threshold
    parameters[1].parameter_value = 0.5;      //[m] 0.5

    parameters[2].parameter_name = "delta"; // threshold
    parameters[2].parameter_value = 0.15;   //[m] 0.12

    AnandMetrics.reset(new Metrics<PointType>(2,"Removed points","Points outside of grid"));

    rclcpp::on_shutdown(&callback_shutdown);
	
    // Create an instance of AlfaNode and spin it
    rclcpp::spin(std::make_shared<AlfaNode>(conf, 
                                    parameters,
                                    &handler, 
                                    &post_processing));

    // Shutdown ROS 2
    rclcpp::shutdown();
    
    return 0;
}