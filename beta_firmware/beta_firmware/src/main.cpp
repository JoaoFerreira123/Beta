#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <Adafruit_NeoPixel.h> // Biblioteca para controlar a fita de LED

rcl_subscription_t subscriber1;
rcl_subscription_t subscriber2;
rcl_subscription_t subscriber3;
rcl_subscription_t subscriber4;

std_msgs__msg__Int32 msg1;
std_msgs__msg__Int32 msg2;
std_msgs__msg__Int32 msg3;
std_msgs__msg__Int32 msg4;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2
#define FITA_LED_PIN 18
#define NUM_LEDS 30

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, FITA_LED_PIN, NEO_GRB + NEO_KHZ800);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback_motor1(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if(msg->data == 1){
    strip.fill(strip.Color(0, 0, 255), 0, NUM_LEDS); // Azul
  }
  
  strip.show();
}

void subscription_callback_motor2(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if(msg->data == 1){
    strip.fill(strip.Color(255, 0, 0), 0, NUM_LEDS); // Vermelho
  }

  strip.show();
}

void subscription_callback_motor3(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if(msg->data == 1){
    strip.fill(strip.Color(255, 255, 0), 0, NUM_LEDS); // amarelo
  }
  
  strip.show();
}


void subscription_callback_motor4(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if(msg->data == 1){
    strip.fill(strip.Color(0, 255, 0), 0, NUM_LEDS); // verde
  }
  
  strip.show();
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(FITA_LED_PIN, OUTPUT);

  strip.begin();
  strip.show(); // Inicializa todos os LEDs como desligados
  // Pisca a fita de LED para verificar o hardware
  for (int i = 0; i < 3; i++) {
    strip.fill(strip.Color(255, 0, 0), 0, NUM_LEDS); // Vermelho
    strip.show();
    delay(500);
    strip.fill(strip.Color(0, 0, 0), 0, NUM_LEDS); // Desliga
    strip.show();
    delay(500);
  }

  delay(3000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "hardware_controller", "", &support));

  // create subscriber for motor1
  RCCHECK(rclc_subscription_init_default(
    &subscriber1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motorR"));

  // create subscriber for motor2
  RCCHECK(rclc_subscription_init_default(
    &subscriber2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motorA"));

  // create subscriber for motor3
  RCCHECK(rclc_subscription_init_default(
    &subscriber3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motorB"));

  // create subscriber for motor4
  RCCHECK(rclc_subscription_init_default(
    &subscriber4,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motorC"));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &msg1, &subscription_callback_motor1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg2, &subscription_callback_motor2, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber3, &msg3, &subscription_callback_motor3, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber4, &msg4, &subscription_callback_motor4, ON_NEW_DATA));
}

void loop() {
  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}