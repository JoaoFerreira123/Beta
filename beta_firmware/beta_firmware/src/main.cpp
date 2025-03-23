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
rcl_subscription_t subscriber_motor_position;

std_msgs__msg__Int32 msg_state;
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

void blinkLEDStrip(uint32_t color, unsigned long blinkInterval) {
  static unsigned long previousMillis = 0;  // Armazena o último tempo que o LED foi atualizado
  static bool ledState = false;            // Estado do LED (ligado/desligado)

  unsigned long currentMillis = millis();  // Obtém o tempo atual

  // Verifica se o intervalo de tempo passou
  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;  // Salva o tempo atual

    // Alterna o estado do LED
    if (ledState == false) {
      strip.fill(color, 0, NUM_LEDS);  // Liga o LED com a cor especificada
      ledState = true;
    } else {
      strip.fill(strip.Color(0, 0, 0), 0, NUM_LEDS);  // Desliga o LED (Preto)
      ledState = false;
    }

    strip.show();  // Atualiza a fita de LED com o novo estado
  }
}

void fadeRedEffect() {
  static unsigned long previousMillis = 0;  // Armazena o último tempo que o LED foi alterado
  static int brightness = 0;               // Brilho atual do LED
  static bool increasing = true;           // Direção do fade (aumentando ou diminuindo)
  const long interval = 20;                // Intervalo de tempo para atualizar o fade (em milissegundos)

  unsigned long currentMillis = millis();  // Obtém o tempo atual

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Atualiza o brilho
    if (increasing) {
      brightness += 5; // Aumenta o brilho
      if (brightness >= 255) {
        brightness = 255;
        increasing = false; // Inverte a direção (começa a diminuir)
      }
    } else {
      brightness -= 5; // Diminui o brilho
      if (brightness <= 0) {
        brightness = 0;
        increasing = true; // Inverte a direção (começa a aumentar)
      }
    }

    // Aplica o brilho à cor vermelha
    uint32_t fadedColor = strip.Color(brightness, 0, 0); // Vermelho com brilho variável
    strip.fill(fadedColor, 0, NUM_LEDS); // Aplica a cor a todos os LEDs
    strip.show(); // Atualiza a fita de LED
  }
}


void subscription_callback_state(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  switch (msg->data) {
    case 0: // Amarelo estático
      strip.fill(strip.Color(255, 255, 0), 0, NUM_LEDS); // Amarelo
      break;

    case 1: // Amarelo piscando
      blinkLEDStrip(strip.Color(255, 255, 0), 500); // Amarelo piscando
      break;

    case 2: // Vermelho fade in and out
      fadeRedEffect(); // Vermelho fade in/out
      break;

    case 3: // Vermelho estático
      strip.fill(strip.Color(255, 0, 0), 0, NUM_LEDS); // Vermelho
      break;

    case 4: // Verde estático (para teste)
      strip.fill(strip.Color(0, 255, 0), 0, NUM_LEDS); // Verde
      break;

    default:
      // Caso padrão: desliga os LEDs
      strip.fill(0, 0, NUM_LEDS); // Desliga todos os LEDs
      break;
  }

  strip.show(); // Atualiza a fita de LED
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
  RCCHECK(rclc_subscription_init_default(&subscriber_state, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "beta_state"));
  RCCHECK(rclc_subscription_init_default(&subscriber_motor_position, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor_position"));

  // Cria o executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_state, &msg_state, &subscription_callback_state, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_motor_position, &msg_motor_position, &subscription_callback_motor_position, ON_NEW_DATA));
}

void loop() {
  unsigned long currentMillis = millis();  // Obtém o tempo atual
  // Verifica se o sistema está estável há mais de 10 segundos
  if (millis() - last_error_time > error_reset_time) {
    error_count = 0; // Resetar contador de erros
  }

  // Executa o executor com um timeout de 100 ms (convertido para nanossegundos)
  RCCHECK(rclc_executor_spin_some(&executor, 100 * 1000000LL));
}