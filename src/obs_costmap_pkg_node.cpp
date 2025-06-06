#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>

class LaserObstacleAvoidance
{
    public:
        LaserObstacleAvoidance() :
            tf_buffer_(),
            tf_listener_(tf_buffer_),

            costmap_ros_("custom_costmap" , tf_buffer_),

            obstacle_range_(2.5),
            max_obstacle_height_(2.0),
            raytrace_range_(3.0),

            fx_(525.0), fy_(525.0), cx_(319.5), cy_(239.5),
            have_camera_info_(true),
            max_depth_(4.0)
        {
            //创建私有节点句柄
            ros::NodeHandle nh_("~");

            //从参数服务器读取参数，覆盖默认值
            nh_.param("obstacle_range", obstacle_range_, 2.5);
            nh_.param("max_obstacle_height", max_obstacle_height_, 0.6);
            nh_.param("raytrace_range", raytrace_range_, 3.0);

            //相机内参
            nh_.param("fx", fx_, 525.0);
            nh_.param("fy", fy_, 525.0);
            nh_.param("cx", cx_, 319.5);
            nh_.param("cy", cy_, 239.5);

            nh_.param("max_depth", max_depth_, 4.0);                   //深度图最大有效距离

            //初始化订阅和地图
            laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &LaserObstacleAvoidance::laserCallback, this);
            rgbd_sub_ = nh_.subscribe<sensor_msgs::Image>("/camera/depth/points", 1, &LaserObstacleAvoidance::rgbdCallback, this);
            
            costmap_ = costmap_ros_.getCostmap();
            costmap_->setDefaultValue(costmap_2d::FREE_SPACE);
        }

    private:
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            ROS_INFO("Get laser msgs");
            // 1. 清除旧障碍物信息
            clearOldObstacles(scan_msg->header.stamp);

            // 2. 处理激光点
            for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
            {
                float range = scan_msg->ranges[i];
                if (range < scan_msg->range_min || range > scan_msg->range_max)
                {
                    continue;
                }

                // 计算激光点角度
                double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                
                // 转换为笛卡尔坐标
                double x = range * cos(angle);
                double y = range * sin(angle);

                // 转换到地图坐标系
                geometry_msgs::PointStamped laser_point, map_point;
                laser_point.header = scan_msg->header;
                laser_point.point.x = x;
                laser_point.point.y = y;
                laser_point.point.z = 0;

                try
                {
                    // 使用tf2进行转换
                    map_point = tf_buffer_.transform
                    (
                        laser_point, 
                        costmap_ros_.getGlobalFrameID(), 
                        ros::Duration(1.0)
                    );
                } 
                catch (tf2::TransformException& ex)
                {
                    ROS_WARN("TF转换失败: %s", ex.what());
                    continue;
                }

                // 3. 更新代价值
                unsigned int mx, my;
                if (costmap_->worldToMap(map_point.point.x, map_point.point.y, mx, my))
                {
                    // 检查高度是否在有效范围内
                    if (fabs(map_point.point.z) <= max_obstacle_height_)
                    { 
                        costmap_->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
                    }
                }
            }
            costmap_ros_.updateMap();
            ROS_INFO("Laser costmap updated");
        }

        void rgbdCallback(const sensor_msgs::Image::ConstPtr& image_msg)
        {
            ROS_INFO ("Get rgbd msgs");
            if (!have_camera_info_)
            {
                ROS_WARN("Cant get depthcamera info");
                return;
            }

            clearOldObstacles(image_msg->header.stamp);

            for (size_t v = 0; v < image_msg->height; ++v)
            {
                for (size_t u = 0; u < image_msg->width; ++u)
                {
                    float depth = 0.0;

                    if (image_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
                    {
                        const uint16_t* row = reinterpret_cast<const uint16_t*>(&image_msg->data[0] + v * image_msg->step);
                        depth = static_cast<float>(row[u]) / 1000.0; // mm -> m
                    }
                    else if (image_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
                    {
                        const float* row = reinterpret_cast<const float*>(&image_msg->data[0] + v * image_msg->step);
                        depth = row[u];
                    }
                    else
                    {
                        ROS_WARN("Unsupported depth encoding: %s", image_msg->encoding.c_str());
                        return;
                    }

                    if (depth <= 0.0 || depth > max_depth_)
                        continue;

                    double x = (u - cx_) * depth / fx_;
                    double y = (v - cy_) * depth / fy_;
                    double z = depth;

                    geometry_msgs::PointStamped camera_point, map_point;
                    camera_point.header = image_msg->header;
                    camera_point.point.x = x;
                    camera_point.point.y = y;
                    camera_point.point.z = z;

                    try
                    {
                        map_point = tf_buffer_.transform(camera_point, costmap_ros_.getGlobalFrameID(), ros::Duration(0.1));
                    }
                    catch (tf2::TransformException& ex)
                    {
                        ROS_WARN("Transform failed: %s", ex.what());
                        continue;
                    }

                    if (fabs(map_point.point.z) > max_obstacle_height_)
                        continue;

                    if (sqrt(x*x + y*y) > obstacle_range_)
                        continue;

                    unsigned int mx, my;
                    if (costmap_->worldToMap(map_point.point.x, map_point.point.y, mx, my))
                    {
                        costmap_->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
                    }
                }
            }
            costmap_ros_.updateMap();
            ROS_INFO("RGBD costmap updated");
        }

        void clearOldObstacles(const ros::Time& stamp)
        {
            geometry_msgs::TransformStamped robot_transform;
            try
            {
                robot_transform = tf_buffer_.lookupTransform(
                    costmap_ros_.getGlobalFrameID(),
                    costmap_ros_.getBaseFrameID(),
                    stamp,
                    ros::Duration(1.0)
                );
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN("Cant get robot pos: %s", ex.what());
                return;
            }

            double robot_x = robot_transform.transform.translation.x;
            double robot_y = robot_transform.transform.translation.y;

            double origin_x = costmap_->getOriginX();
            double origin_y = costmap_->getOriginY();
            double resolution = costmap_->getResolution();
            
            double robot_mx = (robot_x - origin_x) / resolution;
            double robot_my = (robot_y - origin_y) / resolution;

            double clear_radius_cells = raytrace_range_ / resolution;
            double clear_radius_sq = clear_radius_cells * clear_radius_cells;

            // 计算边界
            int start_x = std::max(0, static_cast<int>(robot_mx - clear_radius_cells));
            int start_y = std::max(0, static_cast<int>(robot_my - clear_radius_cells));
            int end_x = std::min(static_cast<int>(costmap_->getSizeInCellsX()), 
                            static_cast<int>(robot_mx + clear_radius_cells) + 1);
            int end_y = std::min(static_cast<int>(costmap_->getSizeInCellsY()), 
                            static_cast<int>(robot_my + clear_radius_cells) + 1);

            for (int y = start_y; y < end_y; ++y)
            {
                for (int x = start_x; x < end_x; ++x)
                {
                    // 使用浮点数计算精确距离
                    double dx = (x + 0.5) - robot_mx; // 栅格中心坐标
                    double dy = (y + 0.5) - robot_my;
                    double distance_sq = dx * dx + dy * dy;
                    
                    if (distance_sq <= clear_radius_sq)
                    {
                        // 仅清除障碍物层
                        unsigned char current_cost = costmap_->getCost(x, y);
                        if (current_cost == costmap_2d::LETHAL_OBSTACLE) {
                            costmap_->setCost(x, y, costmap_2d::FREE_SPACE);
                        }
                    }
                }
            }
        }

        ros::NodeHandle nh_;
        ros::Subscriber laser_sub_;
        ros::Subscriber rgbd_sub_;

        tf2_ros::Buffer             tf_buffer_;
        tf2_ros::TransformListener  tf_listener_;
        costmap_2d::Costmap2DROS    costmap_ros_;
        costmap_2d::Costmap2D*      costmap_;

        double obstacle_range_;
        double max_obstacle_height_;
        double raytrace_range_;

        double fx_, fy_, cx_, cy_;
        bool have_camera_info_;
        double max_depth_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_obstacle_avoidance");
    LaserObstacleAvoidance processor;
    ros::spin();
    return 0;
}