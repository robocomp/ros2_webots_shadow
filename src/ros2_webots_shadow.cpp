// Incluir encabezados necesarios
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <webots/Supervisor.hpp>
#include <webots/Lidar.hpp>

#define TIME_STEP 33

class WebotsShadowNode : public rclcpp::Node {
public:
  WebotsShadowNode() : Node("webots_shadow_node"), robot(nullptr) {

    robot = new webots::Supervisor();
    lidar_helios = robot->getLidar("helios"); 

    if(lidar_helios) lidar_helios->enable(TIME_STEP);

    
    timer_ = create_wall_timer(std::chrono::milliseconds(TIME_STEP),
                               std::bind(&WebotsShadowNode::publish_robot_data, this));


    // Calcular la frecuencia del temporizador en Hz a partir de TIME_STEP
    double timer_frequency = 1000.0 / static_cast<double>(TIME_STEP);
    RCLCPP_INFO(get_logger(), "Timer frequency set to %.2f Hz", timer_frequency);

    // Creación de los publishers
    lidar_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("shadow/lidar_helios", 10);
  }

  ~WebotsShadowNode() {
    if(robot)
      delete robot;

    if(lidar_helios)
      delete lidar_helios;
  }

  void webots_step(){
    robot->step(TIME_STEP);
  }

  void publish_lidar_helios_data(){

    sensor_msgs::msg::LaserScan lidar_msg; 
    lidar_msg.header.stamp = this->get_clock()->now();
    lidar_msg.header.frame_id = "lidar_helios_frame";

    // TODO: Rellenar algunos valores
    // Configurar parámetros del mensaje según lidar en Webots
    //lidar_msg.angle_min = 
    //lidar_msg.angle_max = 
    lidar_msg.angle_increment = lidar_helios->getHorizontalResolution();
    //lidar_msg.time_increment = 
    //lidar_msg.scan_time = 
    lidar_msg.range_min = lidar_helios->getMinRange();
    lidar_msg.range_max = lidar_helios->getMaxRange();

    // Obtener datos de escaneo
    const float *ranges_float = lidar_helios->getRangeImage();

    // Copiar los datos de escaneo al mensaje
    if (ranges_float != nullptr){
      lidar_msg.ranges.resize(lidar_helios->getHorizontalResolution());
      for (int i = 0; i < lidar_helios->getHorizontalResolution(); ++i) {
        lidar_msg.ranges[i] = static_cast<double>(ranges_float[i]);
      }
      lidar_publisher_->publish(lidar_msg);
    }
  }

private:
  void publish_robot_data() {

    publish_lidar_helios_data();

    webots_step();
  }


  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  webots::Supervisor* robot;
  webots::Lidar* lidar_helios;

};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebotsShadowNode>());
    rclcpp::shutdown();
    return 0;
}
