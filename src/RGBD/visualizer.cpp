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

#include "RGBD/visualizer.h"

void Viewer::drawCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double r, double g, double b, int index) {
    const std::string cloud_name = "cloud_" + std::to_string(index);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, r, g, b);
    bool exists = cloud_viewer_.updatePointCloud<pcl::PointXYZ>(cloud, cloud_color, cloud_name);
    if (!exists)
        cloud_viewer_.addPointCloud<pcl::PointXYZ>(cloud, cloud_color, cloud_name);
    cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 16, cloud_name);
}

void Viewer::drawCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Affine3d pose, double r, double g, double b, int index) {
    const std::string cloud_name = "cloud_" + std::to_string(index);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_draw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloud_to_draw, pose);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud_to_draw, r, g, b);
    bool exists = cloud_viewer_.updatePointCloud<pcl::PointXYZ>(cloud_to_draw, cloud_color, cloud_name);
    if (!exists)
        cloud_viewer_.addPointCloud<pcl::PointXYZ>(cloud_to_draw, cloud_color, cloud_name);
        
    cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 16, cloud_name);
}

void Viewer::drawColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, int index) {
    const std::string cloud_name = "cloud_" + std::to_string(index);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

    bool exists = cloud_viewer_.updatePointCloud(cloud, rgb, cloud_name);
    if (!exists)
        cloud_viewer_.addPointCloud(cloud, rgb, cloud_name);

    cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
}

void Viewer::drawColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Eigen::Affine3d pose, int index) {
    const std::string cloud_name = "color_cloud_" + std::to_string(index);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_draw(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *cloud_to_draw, pose);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_to_draw);

    bool exists = cloud_viewer_.updatePointCloud(cloud_to_draw, rgb, cloud_name);
    if (!exists)
        cloud_viewer_.addPointCloud(cloud_to_draw, rgb, cloud_name);
    cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
}

void Viewer::drawNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    cloud_viewer_.removeShape("normals");
    cloud_viewer_.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.05f, "normals");
}

void Viewer::drawRectangle(pcl::PointCloud<pcl::PointXYZ>::Ptr vertices, double r, double g, double b, std::string string) {
    const std::string name = "rectangle_" + string;

    pcl::PolygonMesh polygon;

    pcl::Vertices vertex;
    for (size_t P = 0; P < vertices->points.size(); P++)
        vertex.vertices.push_back(P);

    std::vector<pcl::Vertices> vvertex;
    vvertex.push_back(vertex);

    sensor_msgs::PointCloud2 msg;
    pcl::PCLPointCloud2 msg_aux;
    pcl::toROSMsg(*vertices, msg);
    pcl_conversions::toPCL(msg, msg_aux);

    polygon.cloud = msg_aux;
    polygon.polygons = vvertex;

    cloud_viewer_.addPolygonMesh(polygon, name);
    cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name);
    cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7f, name);
}

void Viewer::drawCloudsRandom(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vClouds) {
    int random_r, random_g, random_b;

    for (size_t Q = 0; Q < vClouds.size(); Q++) {
        random_r = rand() % 255;
        random_g = rand() % 255;
        random_b = rand() % 255;

        this->drawCloud(vClouds[Q], double(random_r), double(random_g), double(random_b), int(Q + 100));
    }
}

void Viewer::drawCloudsRandom(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vClouds, Eigen::Affine3d pose) {
    int random_r, random_g, random_b;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_draw(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t Q = 0; Q < vClouds.size(); Q++) {
        pcl::transformPointCloud(*vClouds[Q], *cloud_to_draw, pose);

        random_r = rand() % 255;
        random_g = rand() % 255;
        random_b = rand() % 255;

        this->drawCloud(cloud_to_draw, double(random_r), double(random_g), double(random_b), int(Q + 100));
    }
}

void Viewer::getColorType(double &r, double &g, double &b, int type) {
    switch (type) {
        case 0: // Sides -> Green 
            r = 0.0;
            g = 255.0;
            b = 0.0;
            break;
        case 1: // Horizontal plane -> Blue
            r = 0.0;
            g = 0.0;
            b = 255.0;
            break;
        case 2:
        case 3:
        case 4: // Vertical plane -> Yellow
            r = 255.0;
            g = 255.0;
            b = 0.0;
            break;
        case 5: // Red
            r = 255.0;
            g = 0.0;
            b = 0.0;
            break;
    }
}

void Viewer::drawPlaneTypes(std::vector<Plane> vPlanes) {
    double r, g, b;

    for (size_t Q = 0; Q < vPlanes.size(); Q++) {
        getColorType(r, g, b, vPlanes[Q].type);
        this->drawCloud(vPlanes[Q].cloud, r, g, b, int(Q));
    }
}

void Viewer::drawPlaneTypesContour(std::vector<Plane> vPlanes) {
    double r, g, b;

    for (size_t Q = 0; Q < vPlanes.size(); Q++) {
        if (vPlanes[Q].contour == nullptr)
            vPlanes[Q].getContour();

        getColorType(r, g, b, vPlanes[Q].type);
        this->drawCloud(vPlanes[Q].contour, r, g, b, int(Q));
        this->drawPlaneNormal(vPlanes[Q], r / 255, g / 255, b / 255, int(Q));
    }
}

void Viewer::drawPlaneTypesContour(std::vector<Plane> vPlanes, Eigen::Affine3d pose) {
    double r, g, b;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_draw(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t Q = 0; Q < vPlanes.size(); Q++) {
        if (vPlanes[Q].contour == nullptr)
            vPlanes[Q].getContour();

        pcl::transformPointCloud(*vPlanes[Q].contour, *cloud_to_draw, pose);
        getColorType(r, g, b, vPlanes[Q].type);
        this->drawCloud(cloud_to_draw, r, g, b, int(Q));
    }
}

void Viewer::drawPlaneTypes(std::vector<Plane> vPlanes, Eigen::Affine3d pose) {
    double r, g, b;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_draw(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t Q = 0; Q < vPlanes.size(); Q++) {
        getColorType(r, g, b, vPlanes[Q].type);
        pcl::transformPointCloud(*vPlanes[Q].cloud, *cloud_to_draw, pose);
        this->drawCloud(cloud_to_draw, r, g, b, int(Q));
    }
}

void Viewer::drawBox(pcl::PointXYZ centroid, double z_dir, double y_dir, double x_dir, Eigen::Matrix3f dir, int Q) {
    const Eigen::Quaternionf qfinal(dir);
    const Eigen::Vector3f tfinal = Eigen::Vector3f(centroid.x, centroid.y, centroid.z);
    const std::string cube_name = "Cube_" + std::to_string(Q);

    cloud_viewer_.addCube(tfinal, qfinal, z_dir, y_dir, x_dir, cube_name);
}

void Viewer::drawAxis(Eigen::Affine3d &pose) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr axis_points_aux(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*axis_points, *axis_points_aux, pose);

    *axis_points = *axis_points_aux;

    const std::string x_arrow = "x_arrow_" + std::to_string(iteration);
    const std::string y_arrow = "y_arrow_" + std::to_string(iteration);
    const std::string z_arrow = "z_arrow_" + std::to_string(iteration);

    if (iteration == 0) {
        cloud_viewer_.removeShape("x_arrow_0");
        cloud_viewer_.removeShape("y_arrow_0");
        cloud_viewer_.removeShape("z_arrow_0");
    }

    if ((iteration == 0) or (iteration % 2 == 0)) {
        cloud_viewer_.addArrow(axis_points->points[1], axis_points->points[0], 1.0, 0.0, 0.0, false, x_arrow);
        cloud_viewer_.addArrow(axis_points->points[2], axis_points->points[0], 0.0, 1.0, 0.0, false, y_arrow);
        cloud_viewer_.addArrow(axis_points->points[3], axis_points->points[0], 0.0, 0.0, 1.0, false, z_arrow);
    }
}

void Viewer::drawAxis(Eigen::Matrix3f pose) {
    Eigen::Affine3d pose_affine = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0)) * Eigen::AngleAxisd(pose.cast<double>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr axis_points_aux(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*axis_points, *axis_points_aux, pose_affine);

    *axis_points = *axis_points_aux;

    const std::string x_arrow = "x_arrow_" + std::to_string(iteration);
    const std::string y_arrow = "y_arrow_" + std::to_string(iteration);
    const std::string z_arrow = "z_arrow_" + std::to_string(iteration);

    if (iteration == 0) {
        cloud_viewer_.removeShape("x_arrow_0");
        cloud_viewer_.removeShape("y_arrow_0");
        cloud_viewer_.removeShape("z_arrow_0");
    }

    if ((iteration == 0) or (iteration % 2 == 0)) {
        cloud_viewer_.addArrow(axis_points->points[1], axis_points->points[0], 1.0, 0.0, 0.0, false, x_arrow);
        cloud_viewer_.addArrow(axis_points->points[2], axis_points->points[0], 0.0, 1.0, 0.0, false, y_arrow);
        cloud_viewer_.addArrow(axis_points->points[3], axis_points->points[0], 0.0, 0.0, 1.0, false, z_arrow);
    }
}

void Viewer::drawPlaneAxis(Plane plane, int index) {
    if (plane.main_dir.isZero())
        plane.getMeasurements();

    Eigen::Vector3f vec_x = plane.center.getVector3fMap() + plane.main_dir.col(0) * 0.5;
    pcl::PointXYZ point_x(vec_x(0), vec_x(1), vec_x(2));
    Eigen::Vector3f vec_y = plane.center.getVector3fMap() + plane.main_dir.col(1) * 0.5;
    pcl::PointXYZ point_y(vec_y(0), vec_y(1), vec_y(2));
    Eigen::Vector3f vec_z = plane.center.getVector3fMap() + plane.main_dir.col(2) * 0.5;
    pcl::PointXYZ point_z(vec_z(0), vec_z(1), vec_z(2));

    const std::string x_arrow = "x_arrow_" + std::to_string(iteration);
    const std::string y_arrow = "y_arrow_" + std::to_string(iteration);
    const std::string z_arrow = "z_arrow_" + std::to_string(iteration);

    cloud_viewer_.addArrow(point_x, plane.center, 0.5, 0.0, 0.0, false, x_arrow);
    cloud_viewer_.addArrow(point_y, plane.center, 0.0, 0.5, 0.0, false, y_arrow);
    cloud_viewer_.addArrow(point_z, plane.center, 0.0, 0.0, 0.5, false, z_arrow);
}

void Viewer::drawPlaneNormal(Plane plane, double r, double g, double b, int index) {
    Eigen::Vector3f vec_x = plane.centroid.getVector3fMap() + plane.coeffs.head<3>() * 0.5;
    pcl::PointXYZ point_x(vec_x(0), vec_x(1), vec_x(2));

    const std::string x_arrow = "normal_arrow_" + std::to_string(index);

    cloud_viewer_.addArrow(point_x, plane.centroid, r, g, b, false, x_arrow);
}

void Viewer::createAxis() {
    pcl::PointXYZ pto(0, 0, 0);
    pcl::PointXYZ ptx(1, 0, 0);
    pcl::PointXYZ pty(0, 1, 0);
    pcl::PointXYZ ptz(0, 0, 1);
    axis_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
    axis_points->points.push_back(pto);
    axis_points->points.push_back(ptx);
    axis_points->points.push_back(pty);
    axis_points->points.push_back(ptz);
}