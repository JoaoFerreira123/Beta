import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

# Inicializando o dicionário global para armazenar os comandos recebidos
gui_commands = {}

# Estado do robo 0
state = -1  # Estado inicial, sem receber info. da GUI

class beta_motion_planner(Node):
    def __init__(self):
        super().__init__('beta_motion_planner')  # Nome do nó

        # Criando o publisher para o tópico /beta_commands
        self.publisher_ = self.create_publisher(String, '/beta_commands', 10)
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

    def listener_callback(self, msg):
        global gui_commands  # Usando a variável global gui_commands

        # Converte a string recebida no formato JSON para dicionário
        gui_commands = string_to_dict(msg.data)
        
        if gui_commands is not None:
            # Passa o dicionário para o método que irá verificar e publicar a mensagem
            self.state_machine()

    def state_machine(self):
        #print(f"Estado atual: {self.state}")
        
        if 'stop' in gui_commands and gui_commands['stop'] == True:
            self.state = "E0"
            self.E0()
            self.timer_start = None
            self.inactivity_timer = None  # Reseta o temporizador de inatividade
            self.slider_active = False  # Reseta a flag de sliders ativos
            self.reproducao_index = 0  # Reseta o índice de reprodução

        elif self.state == "E0" and 'start' in gui_commands and gui_commands['start'] == True:
            self.state = "E1"
            self.E1()
            self.timer_start = time.time()  # Inicia o timer

        elif self.state == "E1" and self.timer_start is not None:
            elapsed_time = time.time() - self.timer_start
            print(f"Tempo decorrido em E1: {elapsed_time}s")
            if elapsed_time >= 5:
                self.state = "E2"
                self.E2()
                self.timer_start = None

        elif self.state == "E2":
            current_slider_values = {key: gui_commands[key] for key in gui_commands if key.endswith("_slider_pos")}
            
            # Verifica se os valores dos sliders mudaram em relação aos últimos valores salvos
            if current_slider_values != self.last_slider_values:
                self.slider_active = True  # Sliders estão sendo movimentados
                self.last_slider_values = current_slider_values  # Atualiza os últimos valores
                self.inactivity_timer = time.time()  # Reinicia o temporizador de inatividade
                print("Sliders em movimento...")
            
            # Se os sliders estiverem ativos e pararem de ser movimentados por 1 segundos, volta para E2
            if self.slider_active and time.time() - self.inactivity_timer >= 1:
                self.slider_active = False  # Reseta a flag de sliders ativos
                self.state = "E2"
                self.E2()
                print("Sliders parados por 1 segundos. Voltando para E2.")

            # Transita para E3 apenas se os sliders estiverem ativos
            if self.slider_active:
                self.state = "E3"
                self.E3()

            elif 'reproduzir_pos' in gui_commands and gui_commands['reproduzir_pos'] == True:
                self.state = "E4"
                self.E4()
                self.reproducao_start_time = time.time()  # Inicia o timer de reprodução
                self.reproducao_index = 0  # Reseta o índice de reprodução

            elif 'gravar_posicao' in gui_commands:
                # Verifica se houve uma transição de False para True
                if gui_commands['gravar_posicao'] == True and self.last_gravar_posicao_state == False:
                    # Grava as posições dos sliders como uma linha da matriz
                    slider_positions = [
                        gui_commands["base_gir_slider_pos"],
                        gui_commands["base_bracoA_slider_pos"],
                        gui_commands["base_bracoB_slider_pos"],
                        gui_commands["base_bracoC_slider_pos"],
                        gui_commands["base_bracoD_slider_pos"],
                        gui_commands["abrir_garra"]
                    ]
                    self.recorded_positions.append(slider_positions)  # Adiciona a linha à matriz
                    print(f"Posição gravada no estado E2: {slider_positions}")
                    print(f"Total de posições gravadas: {len(self.recorded_positions)}")
                
                # Atualiza o estado anterior do botão gravar_posicao
                self.last_gravar_posicao_state = gui_commands['gravar_posicao']

            # Verifica se a chave "apagar_tudo" é True e apaga todas as posições gravadas
            if 'apagar_tudo' in gui_commands and gui_commands['apagar_tudo'] == True:
                self.recorded_positions = []  # Limpa a lista de posições gravadas
                print("Todas as posições gravadas foram apagadas.")

        elif self.state == "E3":
            current_slider_values = {key: gui_commands[key] for key in gui_commands if key.endswith("_slider_pos")}
            
            # Verifica se os valores dos sliders mudaram
            if current_slider_values != self.last_slider_values:
                self.slider_active = True  # Sliders estão sendo movimentados
                self.last_slider_values = current_slider_values  # Atualiza os últimos valores
                self.inactivity_timer = time.time()  # Reinicia o temporizador de inatividade
                print("Sliders em movimento...")
            else:
                # Se os sliders não mudaram por 1 segundos, volta para E2
                if time.time() - self.inactivity_timer >= 1:
                    self.slider_active = False  # Reseta a flag de sliders ativos
                    self.state = "E2"
                    self.E2()
                    print("Sliders parados por 1 segundos. Voltando para E2.")

        elif self.state == "E4":
            if 'reproduzir_pos' in gui_commands and gui_commands['reproduzir_pos'] == False:
                self.state = "E2"
                self.E2()
                self.reproducao_index = 0  # Reseta o índice de reprodução
            else:
                # Reproduzir as posições gravadas
                if self.reproducao_index < len(self.recorded_positions):
                    pos = self.recorded_positions[self.reproducao_index]
                    print(f"Reproduzindo posição {self.reproducao_index + 1}: {pos}")
                    # Aqui você pode adicionar a lógica para mover o robô para a posição gravada
                    self.reproducao_index += 1  # Avança para a próxima posição
                else:
                    # Todas as posições foram reproduzidas
                    if time.time() - self.reproducao_start_time >= 5:
                        self.state = "E2"
                        self.E2()
                        self.reproducao_index = 0  # Reseta o índice de reprodução
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
        if self.command_string != "":
            msg = String()
            msg.data = self.command_string
            self.publisher_.publish(msg)
            self.command_string = ""  # Limpa a string após publicar

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