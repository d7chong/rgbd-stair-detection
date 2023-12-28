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

#include "stair/visualizer_stair.h"

#include <string>
#include <vector>

void ViewerStair::drawStairAxis(Stair stair, std::string stair_type) {
    const char* axis_names[] = {
        (stair_type + "x").c_str(),
        (stair_type + "y").c_str(),
        (stair_type + "z").c_str()};

    for (int i = 0; i < 3; i++) {
        pcl::PointXYZ end_point(
            stair.initial_point.x + stair.stair_dir.col(i)(0) * 1,
            stair.initial_point.y + stair.stair_dir.col(i)(1) * 1,
            stair.initial_point.z + stair.stair_dir.col(i)(2) * 1);

        cloud_viewer_.removeShape(axis_names[i]);
        cloud_viewer_.addArrow(
            end_point, stair.initial_point,
            (i == 0) ? 1.0f : 0.0f,
            (i == 1) ? 1.0f : 0.0f,
            (i == 2) ? 1.0f : 0.0f,
            false,
            axis_names[i]);
    }
}

void ViewerStair::drawStairAxis(Stair stair, std::string stair_type, Eigen::Affine3d pose) {
    Eigen::Vector3f initial_point;
    pcl::transformPoint(stair.initial_point.getVector3fMap(), initial_point, pose.cast<float>());

    const float arrow_length = 1.0f;

    const char* axis_names[] = {(stair_type + "x").c_str(),
                                (stair_type + "y").c_str(),
                                (stair_type + "z").c_str()};

    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3f arrow_start(stair.initial_point.x + stair.stair_dir.col(i)(0) * arrow_length,
                                    stair.initial_point.y + stair.stair_dir.col(i)(1) * arrow_length,
                                    stair.initial_point.z + stair.stair_dir.col(i)(2) * arrow_length);

        cloud_viewer_.removeShape(axis_names[i]);
        cloud_viewer_.addArrow(pcl::PointXYZ(arrow_start(0), arrow_start(1), arrow_start(2)),
                               pcl::PointXYZ(initial_point(0), initial_point(1), initial_point(2)),
                               1.0f, 0.0f, 0.0f, false, axis_names[i]);
    }
}

void ViewerStair::drawFullStairUntil(Stair stair, int level, Eigen::Affine3d s2p) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr riser_vertices(new pcl::PointCloud<pcl::PointXYZ>);

    double w = stair.step_width;
    double h = stair.step_height;
    double l = stair.step_length;

    // Preallocate memory for PointClouds
    vertices->reserve(5 * (level - 1));
    riser_vertices->reserve(5 * (level - 1));

    std::string name, rname, text, stepstring;

    for (int Q = 1; Q < level; Q++) {
        vertices->clear();
        riser_vertices->clear();

        pcl::PointXYZ stairPoints[] = {
            pcl::PointXYZ(-w / 2,       h * (Q - 1),            l * (Q)),
            pcl::PointXYZ( w / 2,       h * (Q - 1),            l * (Q)),
            pcl::PointXYZ( w / 2,       h * (Q - 1),            l * (Q - 1)),
            pcl::PointXYZ(-w / 2,       h * (Q - 1),            l * (Q - 1)),
            pcl::PointXYZ(0,            h * (Q - 1),            l * (4*Q - 2) / 4)};

        pcl::PointXYZ riserPoints[] = {
            pcl::PointXYZ(-w / 2,       h * (Q - 1),            l * (Q - 1)),
            pcl::PointXYZ( w / 2,       h * (Q - 1),            l * (Q - 1)),
            pcl::PointXYZ( w / 2,       h * (Q - 2),            l * (Q - 1)),
            pcl::PointXYZ(-w / 2,       h * (Q - 2),            l * (Q - 1)),
            pcl::PointXYZ(0,            h * (4*Q - 6) / 4,      l * (Q - 1))};

        // Populate PointClouds
        vertices->insert(vertices->end(), std::begin(stairPoints), std::end(stairPoints));
        riser_vertices->insert(riser_vertices->end(), std::begin(riserPoints), std::end(riserPoints));

        name = "step_" + std::to_string(Q);
        rname = "riser_" + std::to_string(Q);
        text = "step_textup_" + std::to_string(Q);
        stepstring = "Step " + std::to_string(Q);

        Eigen::Vector3f point_to_draw2s(vertices->points[4].x, vertices->points[4].y + 0.20f, vertices->points[4].z);
        Eigen::Vector3f point_to_draw2c;
        pcl::transformPoint(point_to_draw2s, point_to_draw2c, s2p.cast<float>());
        //    cloud_viewer_.addText3D (stepstring, getPointXYZ(point_to_draw2c), 0.05, 1.0, 1.0, 1.0, text);
        cloud_viewer_.removeText3D(text);
        cloud_viewer_.addText3D(stepstring, pcl::PointXYZ(point_to_draw2c(0), point_to_draw2c(1), point_to_draw2c(2)), 0.05, 1.0, 1.0, 1.0, text);

        pcl::transformPointCloud(*vertices, *vertices, s2p);
        pcl::transformPointCloud(*riser_vertices, *riser_vertices, s2p);

        this->drawRectangle(vertices, 0, 0, 1, name);
        this->drawRectangle(riser_vertices, 1, 1, 0, rname);
    }
}

void ViewerStair::drawFullAscendingStairUntil(Stair stair, int level, Eigen::Affine3d s2p) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr riser_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_vertices(new pcl::PointCloud<pcl::PointXYZ>);

    double w = stair.step_width;
    double h = stair.step_height;
    double l = stair.step_length;

    // Preallocate memory for PointClouds
    vertices->reserve(5 * (level - 1));
    riser_vertices->reserve(5 * (level - 1));
    left_vertices->reserve(5 * (level - 1));
    right_vertices->reserve(5 * (level - 1));

    std::string name, rname, lpname, rpname, text, stepstring;

    for (int Q = 1; Q < level; Q++) {
        vertices->clear();
        riser_vertices->clear();
        left_vertices->clear();
        right_vertices->clear();

        pcl::PointXYZ stairPoints[] = {
            pcl::PointXYZ(-w / 2,       h * (Q - 1),            l * (Q)),
            pcl::PointXYZ( w / 2,       h * (Q - 1),            l * (Q)),
            pcl::PointXYZ( w / 2,       h * (Q - 1),            l * (Q - 1)),
            pcl::PointXYZ(-w / 2,       h * (Q - 1),            l * (Q - 1)),
            pcl::PointXYZ(0,            h * (Q - 1),            l * (4*Q - 2) / 4)};

        pcl::PointXYZ riserPoints[] = {
            pcl::PointXYZ(-w / 2,       h * (Q - 1),            l * (Q - 1)),
            pcl::PointXYZ( w / 2,       h * (Q - 1),            l * (Q - 1)),
            pcl::PointXYZ( w / 2,       h * (Q - 2),            l * (Q - 1)),
            pcl::PointXYZ(-w / 2,       h * (Q - 2),            l * (Q - 1)),
            pcl::PointXYZ(0,            h * (4*Q - 6) / 4,      l * (Q - 1))};

        pcl::PointXYZ leftPoints[] = {
            pcl::PointXYZ( w / 2,       h * (Q - 1),            l * (Q)),
            pcl::PointXYZ( w / 2,       h * (Q - 1),            l * (Q - 1)),
            pcl::PointXYZ( w / 2,       h * (-1),               l * (Q - 1)),
            pcl::PointXYZ( w / 2,       h * (-1),               l * (Q)),
            pcl::PointXYZ( w / 2,       h * (2*Q - 4) / 4,      l * (4*Q - 2) / 4)};

        pcl::PointXYZ rightPoints[] = {
            pcl::PointXYZ(-w / 2,       h * (Q - 1),            l * (Q)),
            pcl::PointXYZ(-w / 2,       h * (Q - 1),            l * (Q - 1)),
            pcl::PointXYZ(-w / 2,       h * (-1),               l * (Q - 1)),
            pcl::PointXYZ(-w / 2,       h * (-1),               l * (Q)),
            pcl::PointXYZ(-w / 2,       h * (2*Q - 4) / 4,      l * (4*Q - 2) / 4)};

        // Populate PointClouds
        vertices->insert(vertices->end(), std::begin(stairPoints), std::end(stairPoints));
        riser_vertices->insert(riser_vertices->end(), std::begin(riserPoints), std::end(riserPoints));
        left_vertices->insert(left_vertices->end(), std::begin(leftPoints), std::end(leftPoints));
        right_vertices->insert(right_vertices->end(), std::begin(rightPoints), std::end(rightPoints));

        name =          "stepup_"       + std::to_string(Q);
        rname =         "riserup_"      + std::to_string(Q);
        lpname =        "leftup_"       + std::to_string(Q);
        rpname =        "rightup_"      + std::to_string(Q);
        text =          "step_textup_"  + std::to_string(Q);
        stepstring =    "Step "         + std::to_string(Q);

        Eigen::Vector3f point_to_draw2s(vertices->points[4].x, vertices->points[4].y + 0.10f, vertices->points[4].z);
        Eigen::Vector3f point_to_draw2c;
        pcl::transformPoint(point_to_draw2s, point_to_draw2c, s2p.cast<float>());
        cloud_viewer_.removeText3D(text);
        cloud_viewer_.addText3D(stepstring, pcl::PointXYZ(point_to_draw2c(0), point_to_draw2c(1), point_to_draw2c(2)), 0.05, 1.0, 1.0, 1.0, text);

        pcl::transformPointCloud(*vertices,         *vertices,          s2p);
        pcl::transformPointCloud(*riser_vertices,   *riser_vertices,    s2p);
        pcl::transformPointCloud(*left_vertices,    *left_vertices,     s2p);
        pcl::transformPointCloud(*right_vertices,   *right_vertices,    s2p);

        this->drawRectangle(vertices,       0, 0, 1, name);
        this->drawRectangle(riser_vertices, 1, 1, 0, rname);
        this->drawRectangle(left_vertices,  0, 1, 0, lpname);
        this->drawRectangle(right_vertices, 0, 1, 0, rpname);
    }
}

void ViewerStair::drawFullDescendingStairUntil(Stair stair, int level, Eigen::Affine3d s2p) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr riser_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_vertices(new pcl::PointCloud<pcl::PointXYZ>);

    double w = stair.step_width;
    double h = stair.step_height;
    double l = stair.step_length;

    // Preallocate memory for PointClouds
    vertices->reserve(5 * (level - 1));
    riser_vertices->reserve(5 * (level - 1));
    left_vertices->reserve(5 * (level - 1));
    right_vertices->reserve(5 * (level - 1));

    std::string name, rname, lpname, rpname, text, stepstring;

    for (int Q = 1; Q < level; Q++) {
        vertices->clear();
        riser_vertices->clear();
        left_vertices->clear();
        right_vertices->clear();

        pcl::PointXYZ stairPoints[] = { // points for the stair plane horizontal to the floor
            pcl::PointXYZ(-w / 2,       -h * (Q),                       l * (Q)),
            pcl::PointXYZ( w / 2,       -h * (Q),                       l * (Q)),
            pcl::PointXYZ( w / 2,       -h * (Q),                       l * (Q - 1)),
            pcl::PointXYZ(-w / 2,       -h * (Q),                       l * (Q - 1)),
            pcl::PointXYZ(0,            -h * (Q),                       (l * (4*Q - 2)) / 4)};

        pcl::PointXYZ riserPoints[] = { // points for the stair plane perpendicular to the floor
            pcl::PointXYZ(-w / 2,       -h * (Q - 1),                   l * (Q)),
            pcl::PointXYZ( w / 2,       -h * (Q - 1),                   l * (Q)),
            pcl::PointXYZ( w / 2,       -h * (Q),                       l * (Q)),
            pcl::PointXYZ(-w / 2,       -h * (Q),                       l * (Q)),
            pcl::PointXYZ(0,            -h * (4*Q - 2) / 4,             l * (Q))};

        pcl::PointXYZ leftPoints[] = { // points on the left side of the staircase, at the current level
            pcl::PointXYZ( w / 2,       -h * (Q - 1),                   l * (Q)),
            pcl::PointXYZ( w / 2,       -h * (Q - 1),                   l * (Q - 1)),
            pcl::PointXYZ( w / 2,       -h * (level - 1),               l * (Q - 1)),
            pcl::PointXYZ( w / 2,       -h * (level - 1),               l * (Q)),
            pcl::PointXYZ( w / 2,       -h * (2*Q + 2*level - 4) / 4,   l * (4*Q - 2) / 4)};

        pcl::PointXYZ rightPoints[] = { // points on the right side of the staircase, at the current level
            pcl::PointXYZ(-w / 2,       -h * (Q - 1),                   l * (Q)),
            pcl::PointXYZ(-w / 2,       -h * (Q - 1),                   l * (Q - 1)),
            pcl::PointXYZ(-w / 2,       -h * (level - 1),               l * (Q - 1)),
            pcl::PointXYZ(-w / 2,       -h * (level - 1),               l * (Q)),
            pcl::PointXYZ(-w / 2,       -h * (2*Q + 2*level - 4) / 4,   l * (4*Q - 2) / 4)};

        // Populate PointClouds
        vertices->insert(vertices->end(), std::begin(stairPoints), std::end(stairPoints));
        riser_vertices->insert(riser_vertices->end(), std::begin(riserPoints), std::end(riserPoints));
        left_vertices->insert(left_vertices->end(), std::begin(leftPoints), std::end(leftPoints));
        right_vertices->insert(right_vertices->end(), std::begin(rightPoints), std::end(rightPoints));

        name =          "stepdown_"         + std::to_string(Q);
        rname =         "riserdown_"        + std::to_string(Q);
        lpname =        "leftdown_"         + std::to_string(Q);
        rpname =        "rightdown_"        + std::to_string(Q);
        text =          "step_textdown_"    + std::to_string(Q);
        stepstring =    "Step -"            + std::to_string(Q);

        // vertices->points[4] is the mid point of the flat stair at the current level
        Eigen::Vector3f point_to_draw2s(vertices->points[4].x, vertices->points[4].y + 0.20f, vertices->points[4].z);
        Eigen::Vector3f point_to_draw2c;
        pcl::transformPoint(point_to_draw2s, point_to_draw2c, s2p.cast<float>());
        cloud_viewer_.removeText3D(text);
        cloud_viewer_.addText3D(stepstring, pcl::PointXYZ(point_to_draw2c(0), point_to_draw2c(1), point_to_draw2c(2)), 0.05, 1.0, 1.0, 1.0, text);

        pcl::transformPointCloud(*vertices,         *vertices,          s2p);
        pcl::transformPointCloud(*riser_vertices,   *riser_vertices,    s2p);
        pcl::transformPointCloud(*left_vertices,    *left_vertices,     s2p);
        pcl::transformPointCloud(*right_vertices,   *right_vertices,    s2p);

        this->drawRectangle(vertices,       0, 0, 1, name);
        this->drawRectangle(riser_vertices, 1, 1, 0, rname);
        this->drawRectangle(left_vertices,  0, 1, 0, lpname);
        this->drawRectangle(right_vertices, 0, 1, 0, rpname);
    }
}

void ViewerStair::addStairsText(Eigen::Affine3d i2s, Eigen::Affine3d f2c, std::string type) {
    Eigen::Affine3d f2s = i2s * f2c;
    double x = f2s.translation()[0];
    double z = f2s.translation()[2];

    std::stringstream ss;
    if (type == "up") {
        ss << "Ascending stairs at " << sqrt(x * x + z * z) << "m";
        cloud_viewer_.addText(ss.str(), 50, 50, 20, 1.0, 1.0, 1.0, "uptext");
    } else {
        ss << "Descending stairs at " << sqrt(x * x + z * z) << "m";
        cloud_viewer_.addText(ss.str(), 50, 100, 20, 1.0, 1.0, 1.0, "downtext");
    }
}