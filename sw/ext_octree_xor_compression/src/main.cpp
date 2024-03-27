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

#include <cmath>
#include <vector>

#include "alfa_node.hpp"
#include "alib_octree.hpp"
#include "rclcpp/rclcpp.hpp"

#define NODE_NAME "ext_octree_xor_compression"
#define DEFAULT_TOPIC "/velodyne_points"

#define NODE_ID 0
#define POINTCLOUD_ID 0

AlfaBB bb(-100, -100, -100, 100, 100, 100);

/**
 * @brief Handler function that copy input pointcloud into output pointcloud
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void handler(AlfaNode *node) {
#ifdef EXT_HARDWARE
  node->store_pointcloud(LOAD_STORE_CARTESIAN);
#else

  AlfaOctree octree1(bb, 12, 3, false);
  AlfaOctree octree2(bb, 13, 3, false);
  AlfaPoint point;

  octree1.insert_pointcloud(node->get_input_pointcloud());
  vector<char> code = octree1.get_occupation_code_DFS();

  // vector<char> code;

  // octree2.init_octree_from_occupation_code_DFS(occupation_code, bb);
  // code = octree2.get_occupation_code_DFS();

  size_t code_index = 0;

  point.x = bb.min_x;
  point.y = bb.min_y;
  point.z = bb.min_z;
  point.custom_field = code.size();
  node->push_point_output_pointcloud(point);

  point.x = bb.max_x;
  point.y = bb.max_y;
  point.z = bb.max_z;
  point.custom_field = code.size();
  node->push_point_output_pointcloud(point);

  while (code_index + sizeof(AlfaPoint) < code.size()) {
    std::memcpy(&point, &code[code_index], sizeof(AlfaPoint));
    node->push_point_output_pointcloud(point);
    code_index += sizeof(AlfaPoint);
  }

  if (code_index < code.size()) {
    std::memcpy(&point, &code[code_index], sizeof(code.size() - code_index));
    node->push_point_output_pointcloud(point);
  }

#endif
}

/**
 * @brief Post-processing function that publishes output point cloud and
 * metrics.
 *
 * @param node Pointer to the AlfaNode object handling the incoming point cloud.
 */
void post_processing(AlfaNode *node) {
#ifdef EXT_HARDWARE
  node->load_pointcloud(LOAD_STORE_CARTESIAN);
#endif

  node->publish_pointcloud();
}

/**
 * @brief Entry point of the program.
 *
 * @param argc The number of arguments.
 * @param argv The array of arguments.
 * @return int The exit code of the program.
 */
int main(int argc, char **argv) {
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
  AlfaExtensionParameter parameters[10];

  conf.subscriber_topic = subscriber_topic;
  conf.node_name = NODE_NAME;
  conf.pointcloud_id = POINTCLOUD_ID;
  conf.extension_id = NODE_ID;
  conf.hardware_support = hardware_support;
  conf.latency = 0;
  conf.number_of_debug_points = 1;
  conf.metrics_publishing_type = ALL_METRICS;
  conf.custom_field_conversion_type = CUSTOM_FIELD_INTENSITY;

  parameters[0].parameter_value = 5;
  parameters[0].parameter_name = "octree_depth";
  parameters[1].parameter_value = 20000;
  parameters[1].parameter_name = "max_bounding_box_x";
  parameters[2].parameter_value = 20000;
  parameters[2].parameter_name = "max_bounding_box_y";
  parameters[3].parameter_value = 20000;
  parameters[3].parameter_name = "max_bounding_box_z";

  // Create an instance of AlfaNode and spin it
  rclcpp::spin(
      std::make_shared<AlfaNode>(conf, parameters, &handler, &post_processing));

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
