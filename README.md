#include <ros/ros.h>
#include <ros/master.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

// 订阅和发布器管理类
class DynamicSubPub {
public:
    DynamicSubPub(ros::NodeHandle& nh) : nh_(nh) {
        setupDynamicSubPub();
    }

    // 运行主循环
    void spin() {
        ROS_INFO("Dynamic subscription and publishing setup complete.");
        ros::spin();
    }

private:
    ros::NodeHandle& nh_;
    std::map<std::string, ros::Subscriber> subscribers_;
    std::map<std::string, ros::Publisher> publishers_;

    // 设置动态订阅和发布
    void setupDynamicSubPub() {
        // 获取所有已注册的话题
        std::vector<ros::master::TopicInfo> topic_list;
        ros::master::getTopics(topic_list);

        for (const auto& topic : topic_list) {
            const std::string& topic_name = topic.name;
            const std::string& topic_type = topic.datatype;

            ROS_INFO("Found topic: %s, type: %s", topic_name.c_str(), topic_type.c_str());

            try {
                // 根据消息类型创建订阅者和发布者
                if (topic_type == "std_msgs/String") {
                    createSubPub<std_msgs::String>(topic_name);
                } else if (topic_type == "sensor_msgs/LaserScan") {
                    createSubPub<sensor_msgs::LaserScan>(topic_name);
                } else {
                    ROS_WARN("Unsupported message type: %s for topic: %s", topic_type.c_str(), topic_name.c_str());
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Failed to create subscriber/publisher for topic %s: %s", topic_name.c_str(), e.what());
            }
        }
    }

    // 模板方法：创建订阅者和发布者
    template <typename T>
    void createSubPub(const std::string& topic_name) {
        // 发布到新话题 (原始话题 + "_copy")
        std::string pub_topic_name = topic_name + "_copy";
        publishers_[topic_name] = nh_.advertise<T>(pub_topic_name, 10);

        // 订阅原始话题并绑定回调函数
        subscribers_[topic_name] = nh_.subscribe<T>(
            topic_name, 10,
            boost::bind(&DynamicSubPub::messageCallback<T>, this, _1, topic_name));
        ROS_INFO("Created subscriber for topic: %s and publisher for topic: %s", topic_name.c_str(), pub_topic_name.c_str());
    }

    // 模板方法：订阅回调函数
    template <typename T>
    void messageCallback(const boost::shared_ptr<T const>& msg, const std::string& topic_name) {
        // 发布接收到的消息到对应的发布器
        if (publishers_.find(topic_name) != publishers_.end()) {
            ROS_INFO("Forwarding message on topic: %s", topic_name.c_str());
            publishers_[topic_name].publish(*msg);
        }
    }
};

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "dynamic_sub_pub");
    ros::NodeHandle nh;

    // 实例化动态订阅和发布器类
    DynamicSubPub dsp(nh);

    // 运行节点
    dsp.spin();

    return 0;
}





