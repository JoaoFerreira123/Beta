import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import json
import time

# Inicializando o dicionário global para armazenar os comandos recebidos
gui_commands = {}

# Estado do robo 0
state = -1  # Estado inicial, sem receber info. da GUI

class beta_motion_planner(Node):
    def __init__(self):
        super().__init__('beta_motion_planner')  # Nome do nó

        # Criando o publisher para o tópico /beta_state
        self.beta_state_publisher = self.create_publisher(Int32, '/beta_state', 10)

        # Publisher para as posições dos motores
        self.motor_position_publisher = self.create_publisher(Int32, '/motor_position', 10)

        # Inicializa um timer para publicar a mensagem a cada 1 segundo
        self.timer = self.create_timer(0.01, self.check_and_publish)

        # Criando o subscriber para o tópico /gui_commands
        self.subscription = self.create_subscription(String, '/gui_commands', self.listener_callback, 10)

        self.command_string = ""  # String para armazenar a mensagem concatenada
        self.state = "E0"
        self.timer_start = None
        self.timer_Reproduzir = None
        self.recorded_positions = []  # Matriz para armazenar posições gravadas
        self.last_slider_values = {}  # Armazena os últimos valores dos sliders
        self.inactivity_timer = None  # Temporizador para detectar inatividade nos sliders
        self.slider_active = False  # Flag para indicar se os sliders estão ativos
        self.reproducao_index = 0  # Índice para controlar a reprodução das posições
        self.reproducao_start_time = None  # Tempo de início da reprodução
        self.last_gravar_posicao_state = False  # Estado anterior do botão gravar_posicao
        self.slider_changed = False  # Flag para indicar se algum slider foi movido

    def listener_callback(self, msg):
        global gui_commands  # Usando a variável global gui_commands

        # Converte a string recebida no formato JSON para dicionário
        gui_commands = string_to_dict(msg.data)
        
        if gui_commands is not None:
            # Passa o dicionário para o método que irá verificar e publicar a mensagem
            self.state_machine()

    def state_machine(self):
        print(f"Estado atual: {self.state}")  # Log para depuração

        if 'stop' in gui_commands and gui_commands['stop'] == True:
            self.state = "E0"
            self.E0()
            self.timer_start = None
            self.inactivity_timer = None
            self.slider_active = False
            self.reproducao_index = 0

        elif self.state == "E0" and 'start' in gui_commands and gui_commands['start'] == True:
            self.state = "E1"
            self.E1()
            self.timer_start = time.time()

        elif self.state == "E1" and self.timer_start is not None:
            elapsed_time = time.time() - self.timer_start
            print(f"Tempo decorrido em E1: {elapsed_time}s")
            if elapsed_time >= 5:
                self.state = "E2"
                self.E2()
                self.timer_start = None

        elif self.state == "E2":
            current_slider_values = {key: gui_commands[key] for key in gui_commands if key.endswith("_slider_pos")}
            
            # Verifica se os valores dos sliders mudaram
            if current_slider_values != self.last_slider_values:
                print(f"Sliders movidos! Valores atuais: {current_slider_values}")  # Log para depuração
                self.last_slider_values = current_slider_values  # Atualiza os últimos valores
                self.state = "E3"
                self.E3()
                self.inactivity_timer = time.time()  # Reinicia o temporizador de inatividade
                print("Transição para E3: Controle manual com easing")  # Log para depuração

                # Define a flag para indicar que os sliders foram movidos
                self.slider_changed = True

            elif 'reproduzir_pos' in gui_commands and gui_commands['reproduzir_pos'] == True:
                self.state = "E4"
                self.E4()
                self.reproducao_start_time = time.time()
                self.reproducao_index = 0

            elif 'gravar_posicao' in gui_commands:
                if gui_commands['gravar_posicao'] == True and self.last_gravar_posicao_state == False:
                    slider_positions = [
                        gui_commands["base_gir_slider_pos"],
                        gui_commands["base_bracoA_slider_pos"],
                        gui_commands["base_bracoB_slider_pos"],
                        gui_commands["base_bracoC_slider_pos"],
                        gui_commands["base_bracoD_slider_pos"],
                        gui_commands["abrir_garra"]
                    ]
                    self.recorded_positions.append(slider_positions)
                    print(f"Posição gravada no estado E2: {slider_positions}")
                    print(f"Total de posições gravadas: {len(self.recorded_positions)}")
                
                self.last_gravar_posicao_state = gui_commands['gravar_posicao']

            if 'apagar_tudo' in gui_commands and gui_commands['apagar_tudo'] == True:
                self.recorded_positions = []
                print("Todas as posições gravadas foram apagadas.")

        elif self.state == "E3":
            current_slider_values = {key: gui_commands[key] for key in gui_commands if key.endswith("_slider_pos")}
            
            # Verifica se os sliders pararam de ser movidos por 1 segundo
            if current_slider_values != self.last_slider_values:
                self.last_slider_values = current_slider_values
                self.inactivity_timer = time.time()  # Reinicia o temporizador de inatividade
                print("Sliders em movimento...")

                # Define a flag para indicar que os sliders foram movidos
                self.slider_changed = True
            elif time.time() - self.inactivity_timer >= 1:
                self.state = "E2"
                self.E2()
                print("Sliders parados por 1 segundo. Voltando para E2.")

        elif self.state == "E4":
            if 'reproduzir_pos' in gui_commands and gui_commands['reproduzir_pos'] == False:
                self.state = "E2"
                self.E2()
                self.reproducao_index = 0
            else:
                if self.reproducao_index < len(self.recorded_positions):
                    pos = self.recorded_positions[self.reproducao_index]
                    print(f"Reproduzindo posição {self.reproducao_index + 1}: {pos}")
                    self.reproducao_index += 1
                else:
                    if time.time() - self.reproducao_start_time >= 5:
                        self.state = "E2"
                        self.E2()
                        self.reproducao_index = 0
                        print("Reprodução concluída. Voltando para E2.")

                        
    def E0(self):
        self.command_string += "E0"
        print("Executando E0: Parado e bloqueado")

    def E1(self):
        self.command_string += "E1"
        print("Executando E1: Inicializando, indo para home")

    def E2(self):
        self.command_string += "E2"
        print("Executando E2: Aguardando Instrucoes")

    def E3(self):
        self.command_string += "E3"
        print("Executando E3: Controle manual com easing")

    def E4(self):
        self.command_string += "E4"
        print("Executando E4: Repetindo posições gravadas")
        # Adiciona a impressão da matriz de posições gravadas
        print("Matriz de posições gravadas:")
        for i, pos in enumerate(self.recorded_positions):
            print(f"Posição {i + 1}: {pos}")
                
    def check_and_publish(self):
        # Publica o estado no tópico /beta_state como um Int32
        if self.state:
            state_number = int(self.state[1:])  # Extrai o número do estado (E0 -> 0, E1 -> 1, etc.)
            msg = Int32()
            msg.data = state_number
            self.beta_state_publisher.publish(msg)
            #print(f"Publicado no /beta_state: {msg.data}")

        # Publica as posições dos motores no tópico /motor_position apenas se algum slider foi movido
        if self.slider_changed:
            for motor_name, angle in self.last_slider_values.items():
                # Mapeia diretamente os nomes dos sliders para os IDs dos motores
                if motor_name == 'base_gir_slider_pos':
                    motor_id = 1  # baseGir
                elif motor_name == 'base_bracoA_slider_pos':
                    motor_id = 2  # bracoA
                elif motor_name == 'base_bracoB_slider_pos':
                    motor_id = 3  # bracoB
                elif motor_name == 'base_bracoC_slider_pos':
                    motor_id = 4  # bracoC
                elif motor_name == 'base_bracoD_slider_pos':
                    motor_id = 5  # bracoD
                else:
                    motor_id = None  # Ignora sliders não mapeados

                # Publica a mensagem do motor se o ID for válido
                if motor_id is not None:
                    motor_message = self.format_motor_message(motor_id, angle)
                    self.publish_motor_position(motor_message)
            
            # Reseta a flag após publicar
            self.slider_changed = False

    def publish_motor_position(self, motor_message):
        msg = Int32()
        msg.data = motor_message
        self.motor_position_publisher.publish(msg)
        print(f"Publicado no /motor_position: {msg.data}")  # Log para depuração

    def format_motor_message(self, motor_id, angle):
        """
        Formata a mensagem do motor.
        :param motor_id: ID do motor (1 a 6).
        :param angle: Ângulo do motor (0 a 180).
        :return: Inteiro de 4 dígitos (ID + ângulo).
        """
        if motor_id < 1 or motor_id > 9:
            raise ValueError("ID do motor deve estar entre 1 e 6.")
        if angle < 0 or angle > 180:
            raise ValueError("Ângulo deve estar entre 0 e 180.")
        
        # Formata o ângulo para 3 dígitos, preenchendo com zeros à esquerda
        angle_str = f"{angle:03d}"
        
        # Combina o ID do motor e o ângulo em um único número
        return int(f"{motor_id}{angle_str}")


# Função para converter a string JSON para um dicionário
def string_to_dict(string):
    try:
        dicionario = json.loads(string)
        return dicionario
    except json.JSONDecodeError:
        print("Erro ao converter a string para um dicionário.")
        return None


def main(args=None):
    rclpy.init(args=args)
    node = beta_motion_planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()