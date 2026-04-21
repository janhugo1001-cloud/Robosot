#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cstdint>
#include <rclcpp/qos.hpp> // 加上這個 include

class MapPublisher : public rclcpp::Node
{
public:
    MapPublisher() : Node("map_publisher")
    {	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.transient_local();  
        // 創建發布者
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapadd", qos);

        // 聲明參數
        this->declare_parameter("map_file", "");
        this->declare_parameter("yaml_file", "");

        // 獲取參數
        std::string map_file = this->get_parameter("map_file").as_string();
        std::string yaml_file = this->get_parameter("yaml_file").as_string();

        if (map_file.empty() || yaml_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please provide map_file and yaml_file parameters");
            return;
        }

        // 讀取YAML文件
        try {
            YAML::Node config = YAML::LoadFile(yaml_file);
            
            // 創建地圖消息
            auto map_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
            
            // 設置地圖元數據
            map_msg->header.frame_id = "map";
            map_msg->info.resolution = config["resolution"].as<double>();
            
            // 讀取PGM文件
            std::ifstream map_file_stream(map_file, std::ios::binary);
            if (!map_file_stream) {
                RCLCPP_ERROR(this->get_logger(), "Could not open map file: %s", map_file.c_str());
                return;
            }

            // 讀取PGM頭部
            std::string line;
            std::getline(map_file_stream, line); // P5
            std::getline(map_file_stream, line); // 註釋
            std::getline(map_file_stream, line); // 尺寸
            std::stringstream ss(line);
            ss >> map_msg->info.width >> map_msg->info.height;
            std::getline(map_file_stream, line); // 最大值

            // 設置原點
            map_msg->info.origin.position.x = config["origin"][0].as<double>();
            map_msg->info.origin.position.y = config["origin"][1].as<double>();
            map_msg->info.origin.position.z = 0.0;
            map_msg->info.origin.orientation.w = 1.0;

            // 讀取PGM數據 (使用 uint8_t 讀取以正確處理 0-255 的數值)
            std::vector<uint8_t> pgm_data(map_msg->info.width * map_msg->info.height);
            map_file_stream.read(reinterpret_cast<char*>(pgm_data.data()), pgm_data.size());
            
            // 準備地圖消息數據
            map_msg->data.resize(map_msg->info.width * map_msg->info.height);

            // 轉換數據格式並進行 Y 軸翻轉 (解決鏡像問題)
            // ROS 地圖原點在左下 (y=0 在下)，PGM 原點在左上 (y=0 在上)
            for (size_t y = 0; y < map_msg->info.height; y++) {
                for (size_t x = 0; x < map_msg->info.width; x++) {
                    // 讀取 PGM 圖像的倒數第 y 行 (即將圖像上下翻轉投影到地圖)
                    size_t pgm_index = (map_msg->info.height - 1 - y) * map_msg->info.width + x;
                    
                    // 寫入 Map 的第 y 行
                    size_t map_index = y * map_msg->info.width + x;

                    uint8_t pixel_val = pgm_data[pgm_index];
                    int8_t map_val;

                    if (pixel_val == 0) {
                        map_val = 100; // 佔用 (黑色)
                    } else if (pixel_val == 254) {
                        map_val = 0;   // 空閒 (白色)
                    } else {
                        map_val = -1;  // 未知 (灰色)
                    }
                    
                    map_msg->data[map_index] = map_val;
                }
            }

            // 發布地圖
            publisher_->publish(std::move(map_msg));
            RCLCPP_INFO(this->get_logger(), "Map published successfully");

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file: %s", e.what());
        }
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
