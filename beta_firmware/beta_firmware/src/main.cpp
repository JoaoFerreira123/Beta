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

static const int motorR = 13;
static const int motorA = 32;
static const int motorB = 33;
static const int motorC = 27;
static const int motorD = 25;
static const int motorG = 26;

Servo baseGir, bracoA, bracoB, bracoC, bracoD, garra;

rcl_subscription_t subscriber_state;
rcl_subscription_t subscriber2;
rcl_subscription_t subscriber3;
rcl_subscription_t subscriber4;
rcl_subscription_t subscriber_motor_position;

std_msgs__msg__Int32 msg_state;
std_msgs__msg__Int32 msg2;
std_msgs__msg__Int32 msg3;
std_msgs__msg__Int32 msg4;
std_msgs__msg__Int32 msg_motor_position;

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

void subscription_callback_state(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 0) {
    strip.fill(strip.Color(255, 255, 0), 0, NUM_LEDS); // Amarelo
  }else if (msg->data == 1){
    strip.fill(strip.Color(255, 255, 0), 0, NUM_LEDS); // Amarelo piscando

  }else if (msg->data == 2){
    strip.fill(strip.Color(0, 0, 255), 0, NUM_LEDS); // azul p/ teste, idealmente vermelho fade
  }else if (msg->data == 3){
    strip.fill(strip.Color(255, 0, 0), 0, NUM_LEDS); // Vermelho
  }else if (msg->data == 4){
    strip.fill(strip.Color(0, 255, 0), 0, NUM_LEDS); // Verde p/ teste
  }

  strip.show();
}

void subscription_callback_motor_position(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  strip.fill(strip.Color(255, 0, 255), 0, NUM_LEDS); 
  int id_motor = (msg->data / 1000);

  if (id_motor == 1){
    int pos_bracoR = (msg->data % 1000); //resto da divisao
    baseGir.write(pos_bracoR);
  }

  if (id_motor == 2){
    int pos_bracoA = (msg->data % 1000); //resto da divisao
    bracoA.write(pos_bracoA);
  }

  if (id_motor == 3){
    int pos_bracoB = (msg->data % 1000); //resto da divisao
    bracoB.write(pos_bracoB);
  }

  if (id_motor == 4){
    int pos_bracoC = (msg->data % 1000); //resto da divisao
    bracoC.write(pos_bracoC);
  }

  if (id_motor == 5){
    int pos_bracoD = (msg->data % 1000); //resto da divisao
    bracoD.write(pos_bracoD);
  }

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

  RCCHECK(rclc_subscription_init_default(&subscriber2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motorA"));
  RCCHECK(rclc_subscription_init_default(&subscriber3, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motorB"));
  RCCHECK(rclc_subscription_init_default(&subscriber_state, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "beta_state"));
  RCCHECK(rclc_subscription_init_default(&subscriber_motor_position, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor_position"));

  // Cria o executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg2, &subscription_callback_motor2, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber3, &msg3, &subscription_callback_motor3, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_state, &msg_state, &subscription_callback_state, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_motor_position, &msg_motor_position, &subscription_callback_motor_position, ON_NEW_DATA));
}

void loop() {
  // Verifica se o sistema está estável há mais de 10 segundos
  if (millis() - last_error_time > error_reset_time) {
    error_count = 0; // Resetar contador de erros
  }

  // Executa o executor com um timeout de 100 ms (convertido para nanossegundos)
  RCCHECK(rclc_executor_spin_some(&executor, 100 * 1000000LL));
}