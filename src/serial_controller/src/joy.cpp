#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <iostream>
/*
void callback(const sensor_msgs::Joy::ConstPtr& msg) {
  
  //printf("Velocidade x: %.2f\n", msg->axes[1]);
  //printf("Velocidade y: %.2f\n", msg->axes[0]);


  if(msg->axes[1] == 1){
    printf("W\n");
  }else if(msg->axes[1] == -1){
    printf("S\n");
  }else if(msg->axes[0] == 1){
    printf("A\n");
  }else if(msg->axes[0] == -1){
    printf("D\n");
  }
}

void callback(const sensor_msgs::Joy::ConstPtr& msg) {
  serial::Serial arduino;
  arduino.setPort("/dev/ttyACM0");  
  arduino.setBaudrate(9600);

  if (!arduino.isOpen()) {
    try {
      arduino.open();
    } catch (serial::IOException& e) {
      ROS_ERROR("Falha ao abrir a porta serial. Certifique-se de que o Arduino esteja conectado e configurado corretamente.");
      return;
    }
  }

  uint8_t command;

  if (msg->axes[1] == 1) {
    command = 11;
  } else if (msg->axes[1] == -1) {
    command = 12;  
  } else if (msg->axes[0] == 1) {
    command = 13;  
  } else if (msg->axes[0] == -1) {
    command = 14;  
  }

  arduino.write(&command, 1);
  
  arduino.close();
}

int main(int argc, char** argv) {
 
  ros::init(argc, argv, "joystick_control");

  
  ros::NodeHandle nh;


  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, callback);

  ros::spin();

  return 0;
}



serial::Serial arduino;

void callback(const sensor_msgs::Joy::ConstPtr& msg) {
  uint8_t command;

  if (msg->axes[1] == 1) {
    command = 'W';  // Envie 'W' para o Arduino
  } else if (msg->axes[1] == -1) {
    command = 'S';  // Envie 'S' para o Arduino
  } else if (msg->axes[0] == 1) {
    command = 'A';  // Envie 'A' para o Arduino
  } else if (msg->axes[0] == -1) {
    command = 'D';  // Envie 'D' para o Arduino
  } else {
    // Não envie nenhum comando se os eixos não corresponderem a nenhum dos casos acima
    return;
  }

  // Abra a comunicação serial somente quando necessário
  if (!arduino.isOpen()) {
    try {
      arduino.open();
    } catch (serial::IOException& e) {
      ROS_ERROR("Falha ao abrir a porta serial. Certifique-se de que o Arduino esteja conectado e configurado corretamente.");
      return;
    }
  }

  // Envie o comando para o Arduino via comunicação serial
  arduino.write(&command, 1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subiscriber");
  ros::NodeHandle nh;

  arduino.setPort("/dev/ttyACM0");  // Substitua pelo caminho da porta serial do seu Arduino
  arduino.setBaudrate(9600);
  // Subscreva ao tópico de controle de joystick
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, callback);

  // Inicialize o objeto serial para comunicação com o Arduino
  
  arduino.close();
  ros::spin();

  // Feche a comunicação serial ao encerrar o nó
  

  return 0;
}

*/


serial::Serial arduino;

void callback(const sensor_msgs::Joy::ConstPtr& msg) {
  uint8_t command;
  int d;
  if (msg->axes[1] == 1) {
    command = 'W';
  } else if (msg->axes[1] == -1) {
    command = 'S';  
  } else if (msg->axes[0] == 1) {
    command = 'A';  
  } else if (msg->axes[0] == -1) {
    command = 'D';  
  }

  //scanf("%d",&d);

  printf("%d\n",command);
  try{
    arduino.write(&command, 1);
  }catch(serial::PortNotOpenedException &e){
    //ROS_ERROR(" aaa %s\n",e.what());
  }catch (const serial::IOException& e) {
    
    //ROS_ERROR(" ee %s\n",e.what());
    // Lidar com o erro com base no código retornado (error_code)
  }
  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joystick_control");
  ros::NodeHandle nh;

  arduino.setPort("/dev/ttyACM0");
  arduino.setBaudrate(9600);

  try {
    arduino.open();
  } catch (serial::IOException& e) {
    ROS_ERROR("%s\n",e.what());
    ROS_ERROR("Falha ao abrir a porta serial. Certifique-se de que o Arduino esteja conectado e configurado corretamente.");
    return 1;
  }

  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, callback);

  ros::spin();

  arduino.close();  

  return 0;
}