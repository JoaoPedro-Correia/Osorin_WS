#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "serial_publisher");
  ros::NodeHandle nh;

  // Substitua "/dev/ttyACM1" pela porta serial desejada
  std::string serial_port = "/dev/ttyACM0";
  int baudrate = 9600;

  // Inicializa o objeto da classe Serial com a porta e a taxa de baudrate
  serial::Serial ser;
  ser.setPort(serial_port);
  ser.setBaudrate(baudrate);

  try {
    ser.open();
  } catch (serial::IOException& e) {
    ROS_ERROR("Falha ao abrir a porta serial. Certifique-se de que a porta serial esteja disponível e configurada corretamente.");
    return 1;
  }

  // Cria um publisher para publicar mensagens na porta serial
  ros::Publisher serial_pub = nh.advertise<std_msgs::String>("serial_data", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    std_msgs::String msg;
    msg.data = "Hello, Arduino!";

    // Publica a mensagem na porta serial
    ser.write(msg.data);

    // Publica a mensagem no tópico "serial_data"
    serial_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Fecha a porta serial
  ser.close();

  return 0;
}