/**
 * This file is part of stairs_detection.
 *
 * Copyright (C) 2019 Alejandro Pérez Yus <alperez at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/aperezyus/stairs_detection>
 *
 * stairs_detection is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * stairs_detection is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with stairs_detection. If not, see <http://www.gnu.org/licenses/>.
 */

#include <dirent.h>  // To read directory
#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/PCLHeader.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/PointCloud2.h>
#include <signal.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

#include "stair/current_scene_stair.h"
#include "stair/global_scene_stair.h"
#include "stair/stair_classes.h"
#include "stair/visualizer_stair.h"

static int capture_mode = 0;  // Capure mode can be 0 (reading clouds from ROS topic), 1 (reading from .pcd file), 2 (reading all *.pcd from directory)

void sayHelp() {
    std::cout << "-- Arguments to pass:" << std::endl;
    std::cout << "<no args>               - If no arguments ('$ rosrun stairs_detection stairs'), by default the algorithm proceed reading the pcd ROS topic /camera/depth/color/points" << std::endl;
    std::cout << "pcd <path to file>      - To run a PCD example (e.g. from Tang dataset), it should be '$ rosrun stairs_detection stairs pcd /path/to.pcd'" << std::endl;
    std::cout << "dir <path to directory> - To run all PCDs in a dataset, you can point at the folder, e.g. '$ rosrun stairs_detection stairs dir /path/to/pcds/" << std::endl;
}

// To run the program, create an object mainLoop
class mainLoop {
   public:
    mainLoop() : viewer(), gscene() {
        color_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        color_cloud_show.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    ~mainLoop() {}

    void cloudCallback(const sensor_msgs::PointCloud2 &cloud_msg) {
        pcl::fromROSMsg(cloud_msg, *color_cloud);
    }

    // This functions configures and executes the regular loop reading from a ROS topic (capture_mode = 0)
    void startMainLoop(int argc, char *argv[]) {
        // ROS subscribing
        ros::init(argc, argv, "d435i_pcd_node");
        ros::NodeHandle nh;

        ros::Subscriber cloud_sub = nh.subscribe("/camera/depth/color/points", 1, &mainLoop::cloudCallback, this);

        int tries = 0;
        while (cloud_sub.getNumPublishers() == 0) {
            ROS_INFO("Waiting for subscibers");
            sleep(1);
            tries++;
            if (tries > 5) {
                sayHelp();
                return;
            }
        }

        ros::Rate r(100);

        capture_.reset(new ros::AsyncSpinner(0));
        capture_->start();

        while (nh.ok() && !viewer.cloud_viewer_.wasStopped()) {
            if (color_cloud->points.size() > 0) {
                pcl::copyPointCloud(*color_cloud, *cloud);

                this->execute();
            }
            r.sleep();
        }
        capture_->stop();
    }

    // Main loop to run the stairs detection and modelling algorithm for all nodes.
    void execute() {
        // Prepare viewer for this iteration
        pcl::copyPointCloud(*color_cloud, *color_cloud_show);
        viewer.cloud_viewer_.removeAllPointClouds();
        viewer.cloud_viewer_.removeAllShapes();
        viewer.createAxis();

        // Process cloud from current view
        CurrentSceneStair scene;
        scene.applyVoxelFilter(0.01f, cloud);  // Typically 0.04m voxels works fine for this method, however, bigger number (for more efficiency) or smaller (for more accuracy) can be used

        // The method first attempts to find the floor automatically. The floor position allows to orient the scene to reason about planar surfaces (including stairs)
        if (!gscene.initial_floor_) {
            gscene.findFloor(scene.fcloud);
            gscene.computeCamera2FloorMatrix(gscene.floor_normal_);
            viewer.drawAxis(gscene.f2c);
            viewer.drawColorCloud(color_cloud_show, 1);
            viewer.cloud_viewer_.spinOnce();
        } else {
            // Compute the normals
            scene.getNormalsNeighbors(8);  // 8 Neighbours provides better accuracy, ideal for close distances (like the rosbag provided)
            // scene.getNormalsNeighbors(16);   // 16 works better for more irregular pointclouds, like those from far distances (e.g. Tang's dataset)
            // scene.getNormalsRadius(0.01f);   // Alternatively, radius may be used instead of number of neighbors

            // Segment the scene in planes and clusters
            scene.regionGrowing();
            scene.extractClusters(scene.remaining_points);

            // Find and update the floor position
            gscene.findFloorFast(scene.vPlanes);
            if (gscene.new_floor_)
                gscene.computeCamera2FloorMatrix(gscene.floor_normal_);

            // Get centroids, contours and plane coefficients to floor reference
            scene.getCentroids();
            scene.getContours();
            scene.getPlaneCoeffs2Floor(gscene.c2f);
            scene.getCentroids2Floor(gscene.c2f);

            // Classify the planes in the scene
            scene.classifyPlanes();

            // Get Manhattan directions to rotate floor reference to also be aligned with vertical planes (OPTIONAL)
            gscene.getManhattanDirections(scene);

            // Some drawing functions for the PCL to see how the method is doing untill now
            // viewer.drawNormals (scene.normals, scene.fcloud);
            // viewer.drawPlaneTypesContour(scene.vPlanes);
            // viewer.drawCloudsRandom(scene.vObstacles);
            // viewer.drawAxis(gscene.f2c);

            // STAIR DETECTION AND MODELING
            if (scene.detectStairs()) {  // First a quick check if horizontal planes may constitute staircases
                // Ascending staircase
                if (scene.getLevelsFromCandidates(scene.upstair, gscene.c2f)) {            // Sort planes in levels
                    scene.upstair.modelStaircase(gscene.main_dir, gscene.has_manhattan_);  // Perform the modeling
                    if (scene.upstair.validateStaircase()) {                               // Validate measurements
                        std::cout << "---------- [ASCENDING STAIRCASE] ----------\n"
                                  << "[Steps] " << scene.upstair.vLevels.size() - 1 << "\n\n"
                                  << "[Measurements]\nWidth\t\tHeight\t\tLength\n"
                                  << "" << scene.upstair.step_width << "m\t" << scene.upstair.step_height << "m\t" << scene.upstair.step_length << "m\n\n"
                                  << "[Error]\nWidth error\tHeight error\tLength error\n"
                                  << (scene.upstair.step_width - 0.40) / 0.40 * 100 << "%\t" << (scene.upstair.step_height - 0.06) / 0.06 * 100 << "%\t" << (scene.upstair.step_length - 0.17) / 0.17 * 100 << "%\n\n"
                                  << "[Pose (stair axis w.r.t. camera)]\n"
                                  << scene.upstair.s2i.matrix() << "\n\n";

                        // Draw staircase
                        viewer.addStairsText(scene.upstair.i2s, gscene.f2c, scene.upstair.type);
                        viewer.drawFullAscendingStairUntil(scene.upstair, int(scene.upstair.vLevels.size()), scene.upstair.s2i);
                        viewer.drawStairAxis(scene.upstair, scene.upstair.type);
                    }
                }

                // Descending staircase
                if (scene.getLevelsFromCandidates(scene.downstair, gscene.c2f)) {            // Sort planes in levels
                    scene.downstair.modelStaircase(gscene.main_dir, gscene.has_manhattan_);  // Perform the modeling
                    if (scene.downstair.validateStaircase()) {                               // Validate measurements
                        std::cout << "---------- [DESCENDING STAIRCASE] ----------\n"
                                  << "[Steps] " << scene.downstair.vLevels.size() - 1 << "\n\n"
                                  << "[Measurements]\nWidth\t\tHeight\t\tLength\n"
                                  << "" << scene.downstair.step_width << "m\t" << scene.downstair.step_height << "m\t" << scene.downstair.step_length << "m\n\n"
                                  << "[Error]\nWidth error\tHeight error\tLength error\n"
                                  << (scene.downstair.step_width - 0.40) / 0.40 * 100 << "%\t" << (scene.downstair.step_height - 0.06) / 0.06 * 100 << "%\t" << (scene.downstair.step_length - 0.17) / 0.17 * 100 << "%\n\n"
                                  << "[Pose (stair axis w.r.t. camera)]\n"
                                  << scene.downstair.s2i.matrix() << "\n\n";

                        // Draw staircase
                        viewer.addStairsText(scene.downstair.i2s, gscene.f2c, scene.downstair.type);
                        viewer.drawFullDescendingStairUntil(scene.downstair, int(scene.downstair.vLevels.size()), scene.downstair.s2i);
                        viewer.drawStairAxis(scene.downstair, scene.downstair.type);
                    }
                }
            }

            // Draw color cloud and update viewer
            viewer.drawColorCloud(color_cloud_show, 1);
            viewer.cloud_viewer_.spinOnce();
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_show;  // just for visualization of each iteration, since color_cloud keeps being updated in the callback
    boost::shared_ptr<ros::AsyncSpinner> capture_;
    ViewerStair viewer;       // Visualization object
    GlobalSceneStair gscene;  // Global scene (i.e. functions and variables that should be kept through iterations)
};

void parseArguments(int argc, char **argv, int &capture_mode) {
    // Capture mode goes as follows:
    // 0 (default) - Read clouds from ROS topic '/camera/depth/color/points/' [USE YOUR ROS PCD TOPIC!]
    // Removed all other modes <- this code is ONLY for ROS nodes
    capture_mode = 0;
}

int main(int argc, char *argv[]) {
    mainLoop app;
    parseArguments(argc, argv, capture_mode);

    app.startMainLoop(argc, argv);

    return 0;
}