#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

static const int motorR = 25;
static const int motorA = 32;
static const int motorB = 33;
static const int motorC = 26;
static const int motorD = 27;
static const int motorG = 13;

Servo baseGir, bracoA, bracoB, bracoC, bracoD, garra;

rcl_subscription_t subscriber1;
rcl_subscription_t subscriber2;
rcl_subscription_t subscriber3;
rcl_subscription_t subscriber4;
rcl_subscription_t subscriber5;

std_msgs__msg__Int32 msg1;
std_msgs__msg__Int32 msg2;
std_msgs__msg__Int32 msg3;
std_msgs__msg__Int32 msg4;
std_msgs__msg__Int32 msg5;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2
#define FITA_LED_PIN 18
#define NUM_LEDS 30

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, FITA_LED_PIN, NEO_GRB + NEO_KHZ800);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){handle_error(temp_rc, __LINE__);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

int error_count = 0;
const int max_error_count = 5; // Número máximo de tentativas antes de reinicializar
unsigned long last_error_time = 0;
const unsigned long error_reset_time = 10000; // Resetar contador de erros após 10 segundos de funcionamento estável

void handle_error(rcl_ret_t error, int line) {
  Serial.printf("Erro na linha %d: %d\n", line, error);
  error_count++;
  last_error_time = millis();

  if (error_count >= max_error_count) {
    // Se o número máximo de tentativas for atingido, reinicialize o ESP32
    Serial.println("Reinicializando ESP32...");
    ESP.restart();
  } else {
    // Pisca o LED para indicar erro
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
}

void subscription_callback_motor1(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 1) {
    strip.fill(strip.Color(0, 0, 255), 0, NUM_LEDS); // Azul
  }
  strip.show();
}

void subscription_callback_motor2(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 1) {
    strip.fill(strip.Color(255, 0, 0), 0, NUM_LEDS); // Vermelho
  }
  strip.show();
}

void subscription_callback_motor3(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 1) {
    strip.fill(strip.Color(255, 255, 0), 0, NUM_LEDS); // Amarelo
  }
  strip.show();
}

void subscription_callback_motor4(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 1) {
    strip.fill(strip.Color(0, 255, 0), 0, NUM_LEDS); // Verde
  }
  strip.show();
}

void subscription_callback_motor5(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 1) {
    strip.fill(strip.Color(255, 0, 255), 0, NUM_LEDS); // xxx
  }
  int id_motor = (msg->data / 1000);
  int pos_bracoD = (msg->data % 1000); //resto da divisao
  strip.fill(strip.Color(255, 0, 255), 0, NUM_LEDS); 
  bracoD.write(pos_bracoD);
  strip.fill(strip.Color(0, 255, 0), 0, NUM_LEDS); 


  strip.show();
}

void setup() {
  Serial.begin(115200);
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

  baseGir.attach(motorR);
  bracoA.attach(motorA);
  bracoB.attach(motorB);
  bracoC.attach(motorC);
  bracoD.attach(motorD);
  garra.attach(motorG);

  allocator = rcl_get_default_allocator();

  // Inicializa o suporte do micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Cria o nó
  RCCHECK(rclc_node_init_default(&node, "hardware_controller", "", &support));

  // Cria os subscribers
  RCCHECK(rclc_subscription_init_default(&subscriber1, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motorR"));
  RCCHECK(rclc_subscription_init_default(&subscriber2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motorA"));
  RCCHECK(rclc_subscription_init_default(&subscriber3, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motorB"));
  RCCHECK(rclc_subscription_init_default(&subscriber4, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motorC"));
  RCCHECK(rclc_subscription_init_default(&subscriber5, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor_position"));

  // Cria o executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &msg1, &subscription_callback_motor1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg2, &subscription_callback_motor2, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber3, &msg3, &subscription_callback_motor3, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber4, &msg4, &subscription_callback_motor4, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber5, &msg5, &subscription_callback_motor5, ON_NEW_DATA));
}

void loop() {
  // Verifica se o sistema está estável há mais de 10 segundos
  if (millis() - last_error_time > error_reset_time) {
    error_count = 0; // Resetar contador de erros
  }

  // Executa o executor com um timeout de 100 ms (convertido para nanossegundos)
  RCCHECK(rclc_executor_spin_some(&executor, 100 * 1000000LL));
}