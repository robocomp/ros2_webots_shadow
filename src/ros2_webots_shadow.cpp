// Incluir encabezados necesarios
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <webots/Supervisor.hpp>

#define TIME_STEP 33

class WebotsShadowNode : public rclcpp::Node {
public:
  WebotsShadowNode() : Node("webots_shadow_node"), robot(nullptr) {

    robot = new webots::Supervisor();

    // Calcular la frecuencia del temporizador en Hz a partir de TIME_STEP
    double timer_frequency = 1000.0 / static_cast<double>(TIME_STEP);

    publisher_ = create_publisher<std_msgs::msg::String>("robot_data", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(TIME_STEP),
                               std::bind(&WebotsShadowNode::publish_robot_data, this));

    RCLCPP_INFO(get_logger(), "Timer frequency set to %.2f Hz", timer_frequency);
  }

  ~WebotsShadowNode() {
    delete robot;
  }

  void webots_step(){
    robot->step(TIME_STEP);
  }

private:
  void publish_robot_data() {
    // Obtener valores del robot desde Webots
    std_msgs::msg::String msg;
    msg.data = "Robot data from Webots Shadow";

    // Publicar datos en el topic
    RCLCPP_INFO(get_logger(), "Publishing robot data...");
    publisher_->publish(msg);

    webots_step();
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  webots::Supervisor* robot;
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebotsShadowNode>());
    rclcpp::shutdown();
    return 0;
}
