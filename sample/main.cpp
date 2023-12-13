#include <ros/ros.h>
#include <sglog/sglog.h>
#include <template/decision_interface.h>

std::shared_ptr<jarvis::decision_lib::DecisionInterface> interface_ptr;

void timer_handler(const ros::TimerEvent &) {
    static double times = 0;
    times++;

    auto result = interface_ptr->execute();
    SG_INFO("timer_handler execute = %s", result.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "template_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    auto template_handle =
        jarvis::decision_lib::DecisionInterface::create_instance("test");
    if (template_handle == nullptr) {
        SG_ERROR("create instance failed!");
        return -2;
    }
    interface_ptr.reset(template_handle);

    SG_INFO("version: %s", interface_ptr->get_version().c_str());

    if (!interface_ptr->init()) {
        SG_ERROR("init failed");
        return -1;
    }

    auto timer = nh.createTimer(ros::Duration(1), timer_handler);

    ros::spin();

    return 0;
}