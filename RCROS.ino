/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>

//Parametros de pines/movimiento de ruedas
int R_ADELANTE = 10;
int R_ATRAS = 9;
int L_ADELANTE = 6;
int L_ATRAS = 11;

ros::NodeHandle  nh;

//Mover servo
void servo_cb( const geometry_msgs::Twist& cmd_msg){
  if (cmd_msg.angular.z == 0 && cmd_msg.linear.x == 0 ){
      // parar
  } else{
  
  if (cmd_msg.angular.z < 0){
  int a=cmd_msg.angular.z * 255;//Velocidad angular = 255
   adelante(a); //ir a la derecha con velocidad 255
  delay(1000);

 } else if(cmd_msg.angular.z > 0){
   izquierda(255);//Izquierda a 255
  delay(1000);
 izquierda(280);
adelante(a);

// Hace Girar
  }else if(cmd_msg.linear.x < 0){
    // atras
  }else if(cmd_msg.linear.x > 0){
    // adelante
  }
  }
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", servo_cb);


void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  // Prueba para definir sentido de giro
    //digitalWrite(6, HIGH);
    //digitalWrite(9, LOW);
    //digitalWrite(10, LOW);
    //digitalWrite(11, LOW);

  nh.spinOnce();
  delay(100);
}

//Funcion para ir adelante
void adelante(int velocidad){ // Funcion recibe como parametro la velocidad
analogWrite(R_ADELANTE, velocidad); //Rueda de la derecha va hacia adelante
digitalWrite(R_ATRAS, LOW); //Rueda de la derecha apagado ir atras
analogWrite(L_ADELANTE, velocidad); //Rueda de la izquierda va hacia adelante
digitalWrite(L_ATRAS, LOW);//Rueda de la izquierda apagado ir atras
}

void atras(int velocidad){//Invierte la funcion ADELANTE
digitalWrite(R_ADELANTE, LOW);
analogWrite(R_ATRAS, velocidad);
digitalWrite(L_ADELANTE, LOW);
analogWrite(L_ATRAS, velocidad);
}

void izquierda(int velocidad){
analogWrite(R_ADELANTE, velocidad); //Rueda derecha va hacia adelante
digitalWrite(R_ATRAS, LOW);//Atras apagado
digitalWrite(L_ADELANTE, LOW);//Adelante apagado
analogWrite(L_ATRAS, velocidad);//Rueda izquierda va hacia atras
}// Hace girar en circulos a la izquierda

void derecha(int velocidad){//Invierte la funcion IZQUIERDA
digitalWrite(R_ADELANTE, LOW);
analogWrite(R_ATRAS, velocidad);
analogWrite(L_ADELANTE, velocidad);
digitalWrite(L_ATRAS, LOW);
}

void parar(){//Apaga los motores
digitalWrite(R_ADELANTE, LOW);
digitalWrite(R_ATRAS, LOW);
digitalWrite(L_ADELANTE, LOW);
digitalWrite(L_ATRAS, LOW);
}

