#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
/*
  rosparam set joy_node/dev "/dev/input/js0"
  rosrun joy joy_node
  source osorim_ws/devel/setup.bash

  Ao abrir o terminal primeira coisa deve se colocar o parametro de imput para setar o Joystick
  depois dar um source.... 
  igual os codigos acima
*/

//variaveis Statica
serial::Serial arduino;
int serial_port;


void callback(const sensor_msgs::Joy::ConstPtr& msg) {
  /*
    nessa parte do codigo, setamos uma estrutura terminos, tty
    e damos um serial_port = open, para abrir a comunicação com a porta serial
  */
  struct termios tty;
  serial_port = open("/dev/ttyUSB0", O_RDWR);
  if (serial_port < 0) {
    perror("Erro ao abrir a porta serial");
    return ;
  }

  /*
    Aqui setamos os parametros da porta serial
  */
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_port, &tty) != 0) {
    perror("Erro ao obter os atributos da porta serial");
    return ;
  }

  /*
    8 bits por caractere, habilita leitura, ignora controle de modem
  */
  tty.c_cflag = CS8 | CREAD | CLOCAL; 

  /*
    Ignora erros de paridade
  */
  tty.c_iflag = IGNPAR; 

  /*
    Modo não canônico
  */
  tty.c_oflag = 0;
  tty.c_lflag = 0; 

  /*
    Define a velocidade de transmissão (9600 bps)
  */
  cfsetospeed(&tty, B9600); 

  /*
    Aplica as configurações à porta serial
  */
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    perror("Erro ao aplicar os atributos da porta serial");
    return ;
  }

  /*
    Ao passar por todas essas verificaçoes a porta serial esta aberta e configurada
    corretamente para ser utilizada.
    String a ser enviada
  */  
  char message;

  /*
    Leio atraves da "msg", q basicamente é uma classe do tipo Joy
    passada por parametro na função callback
    leio o vetor axes, que tem duas posiçoes [0,1]
  */
  if (msg->axes[1] == 1) {
    message = 'W';
  } else if (msg->axes[1] == -1) {
    message = 'S';
  } else if (msg->axes[0] == 1) {
    message = 'A';
  } else if (msg->axes[0] == -1) {
    message = 'D';
  }

  // Envia a string para a porta serial 
  printf("%c\n",message);
  write(serial_port, &message, strlen(&message));
    
  // Fecha a porta serial
  close(serial_port);

}

int main(int argc, char** argv) { 
  ros::init(argc, argv, "joystick_control");

  ros::NodeHandle nh;

  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, callback);

  ros::spin();

  return 0;
}



// Codigos Comentados

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

  arduino.setPort("/dev/ttyUSB0");
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

Código comentado pq estava testando, foram tentativas falhas de comunicação com a porta serial
*/