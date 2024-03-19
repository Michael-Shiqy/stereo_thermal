# include <memory>
# include <rclcpp/rclcpp.hpp>
# include <sensor_msgs/msg/point_cloud2.hpp>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl/point_cloud.h>
# include <pcl/point_types.h>
# include <math.h>

using std::placeholders::_1;

struct PointXYZT 
{
  PCL_ADD_POINT4D;  // This macro adds the members x,y,z which can also be accessed using the data[4] array.
  float temp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Make sure our new point type is aligned
} EIGEN_ALIGN16;  // Ensure SSE alignment for the new point type

// Register the new point type
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, temp, temp))

pcl::PointCloud<pcl::PointXYZRGB> pc2_to_pcl(const sensor_msgs::msg::PointCloud2& input) 
{
    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(input, cloud);

    // pcl::PointCloud<PointXYZT> thermCloud;
    for (auto& point : cloud)
    {
        point.r = 255;
        point.g = 255;
        point.b = 255;
    } 
        

    // Now you can access points directly, for example, access the first point
    // int i = 0; 
    // while(true)
    // {
    //     const auto& point = (cloud)[i];
    //     if (point.x == point.x && point.y == point.y && point.z == point.z)
    //     {
    //         std::cout << "First point: x=" << point.x << ", y=" << point.y << ", z=" << point.z << ", t=" << point.temp << std::endl;
    //         break;
    //     }
    //     else
    //         i++;
    // }
    return cloud;
}

// mapping from temp to color
uint32_t temp_to_rgb(double temp) 
{
    uint8_t r = std::min(255.0, std::max(0.0, (temp - 10) * 12)); 
    uint8_t g = 0;
    uint8_t b = 255 - r;
    return ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
}

class Pc_Sub : public rclcpp::Node
{
  /*
  Node Subsribing the output data from zed_node
  Type: sensor_msgs::msg::PointCloud2
  Topic: /zed/zed_node/point_cloud/cloud_registered
  */

public:
  Pc_Sub() : Node("pc_sub")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed/zed_node/point_cloud/cloud_registered", 10, std::bind(&Pc_Sub::pointcloud_callback, this, _1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_point_cloud", 10);
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud");
    RCLCPP_INFO(this->get_logger(), "is_dense: %d", msg->is_dense);
    RCLCPP_INFO(this->get_logger(), "height: %d", msg->height);
    RCLCPP_INFO(this->get_logger(), "width: %d", msg->width);

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Main part to modify
    for (auto& point : cloud)
    {
        point.r = 255;
        point.g = 0;
        point.b = 255;
    }
    // Main part to modify

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.stamp = this->get_clock()->now();
    output.header.frame_id = msg->header.frame_id; // Use the same frame_id as the input cloud

    // Publish the modified point cloud
    publisher_->publish(output); 

  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pc_Sub>());
  rclcpp::shutdown();
  return 0;
}
