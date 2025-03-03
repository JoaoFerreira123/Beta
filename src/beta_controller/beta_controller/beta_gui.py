#!/usr/bin/env python3
import sys, os
import threading
import json

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import frontEnd

    #TODO - PARA A GUI, ADD O TEXTO DO VALOR DOS SLIDERS NA CAIXA DE TEXTO PARA AS POSICOES
    #TODO - ADICIONAR SWITCH PARA HABILITAR OU DESABILITAR O SERVO EASING

class beta_gui(Node):
    def __init__(self):
        super().__init__('beta_gui')
        self.publisher_ = self.create_publisher(String, 'gui_commands', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  #10ms
        self.get_logger().info('Beta GUI Iniciado! Publicando a cada 10ms segundo.')

        # Definindo um dicionário inicial
        self.guiCommands = {}

    def timer_callback(self):
        # Converte o dicionário guiCommands para uma string JSON
        msg = String()
        msg.data = dict_to_string(self.guiCommands)
        
        # Publica a mensagem no tópico gui_commands
        self.publisher_.publish(msg)
        self.get_logger().info('Publicando: "%s"' % msg.data)

    def update_gui_commands(self, key, value):
        # Atualiza o dicionário guiCommands
        self.guiCommands[key] = value


def dict_to_string(dicionario):
    # Converte o dicionário para uma string JSON
    return json.dumps(dicionario)

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = beta_gui()

    # Criando a thread para o spin do ROS
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,))
    ros_thread.start()

    # Passando a instância do node para a GUI
    frontEnd.RobotControlApp(node).run()

    # Aguarda o término da thread ROS antes de encerrar
    ros_thread.join()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
