from kivy.app import App
from kivy.uix.slider import Slider
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.core.window import Window
from kivy.uix.image import Image
from kivy.graphics import Color, RoundedRectangle
from kivy.uix.spinner import Spinner
from kivy.clock import Clock

import json

guiCommands = {}

class RobotControlApp(App):
    def __init__(self, node, **kwargs):
        super().__init__(**kwargs)
        self.node = node  # Instância do nó ROS passada na inicialização
        self.positions_matrix = []  # Matriz para armazenar as posições gravadas
        self.texto_posicoes = None  # Referência para a caixa de texto

    def build(self):
        Window.size = (1280, 720)
        Window.clearcolor = (0.2, 0.2, 0.2, 1)
        # Layout principal usando FloatLayout
        root = FloatLayout()

        # Imagem / Logo Beta
        logo = Image(source='/home/joaoferreira/beta/src/beta_controller/beta_controller/beta_logo.png', size_hint=(None, None), size=(300, 300), pos=(30, 470))

        # Add the image to the layout
        root.add_widget(logo)
    
        self.guiCommands = {} #criacao do dicionario

        # -------------- BOTOES ---------------------
        self.start_button = Button(text="Iniciar", size_hint=(None, None), size=(150, 50))
        self.start_button.pos = (10, 450)  # Posição (x, y)
        self.start_button.background_color = (0.9,0.9,0.9,1)  # Cor original cinza
        self.start_button.bind(on_press=self.start_robot)  # Conecta o evento de clique

        self.stop_button = Button(text="Parar", size_hint=(None, None), size=(150, 50))
        self.stop_button.pos = (170, 450)  # Posição (x, y)
        self.stop_button.background_color = (0.9,0.9,0.9,1)  # Cor original cinza
        self.stop_button.bind(on_press=self.stop_robot)  # Conecta o evento de clique

        self.home_button = Button(text="Home", size_hint=(None, None), size=(310, 50))
        self.home_button.pos = (10, 380)  # Posição (x, y)
        self.home_button.background_color = (0.9,0.9,0.9,1)  # Cor original cinza
        self.home_button.bind(on_press=self.home_robot)  # Conecta o evento de clique
        self.home_button.bind(on_release=lambda instance: self.reset_button_color(instance))  # Conecta o evento de soltar o botão

        self.gravar_pos = Button(text=" Gravar Posicao", size_hint=(None, None), size=(200,50))
        self.gravar_pos.pos = (380, 350)  # Posição (x, y)
        self.gravar_pos.background_color = (0.9,0.9,0.9,1)  # Cor original cinza
        self.gravar_pos.bind(on_press=self.gravar_posicao)  # Conecta o evento de clique
        self.gravar_pos.bind(on_release=lambda instance: self.reset_button_color(instance))  # Conecta o evento de soltar o botão

        self.reproduzir_pos = Button(text=" Reproduzir Posicoes", size_hint=(None, None), size=(200,50))
        self.reproduzir_pos.pos = (380, 280)  # Posição (x, y)
        self.reproduzir_pos.background_color = (0.9,0.9,0.9,1)  # Cor original cinza
        self.reproduzir_pos.bind(on_press=self.reproduzir_posicoes)  # Conecta o evento de clique
        self.reproduzir_pos.bind(on_release=lambda instance: self.reset_button_color(instance))  # Conecta o evento de soltar o botão

        self.ir_para_pos = Button(text="Ir para Posicao", size_hint=(None, None), size=(150,50))
        self.ir_para_pos.pos = (380, 210)  # Posição (x, y)
        self.ir_para_pos.background_color = (0.9,0.9,0.9,1)  # Cor original cinza
        self.ir_para_pos.bind(on_press=self.go_to_pos)  # Conecta o evento de clique
        self.ir_para_pos.bind(on_release=lambda instance: self.reset_button_color(instance))  # Conecta o evento de soltar o botão

        self.apagar_pos = Button(text="Apagar Posicao", size_hint=(None, None), size=(150,50))
        self.apagar_pos.pos = (380, 140)  # Posição (x, y)
        self.apagar_pos.background_color = (0.9,0.9,0.9,1)  # Cor original cinza
        self.apagar_pos.bind(on_press=self.apagar_posicao)  # Conecta o evento de clique
        self.apagar_pos.bind(on_release=lambda instance: self.reset_button_color(instance))  # Conecta o evento de soltar o botão

        self.apagar_todas_pos = Button(text="Apagar Todas as Posicoes", size_hint=(None, None), size=(200,50))
        self.apagar_todas_pos.pos = (380, 70)  # Posição (x, y)
        self.apagar_todas_pos.background_color = (0.9,0.9,0.9,1)  # Cor original cinza
        self.apagar_todas_pos.bind(on_press=self.apagar_todas_posicoes)  # Conecta o evento de clique
        self.apagar_todas_pos.bind(on_release=lambda instance: self.reset_button_color(instance))  # Conecta o evento de soltar o botão

        self.abrir_garra = Button(text="Abrir Garra", size_hint=(None, None), size=(200,50))
        self.abrir_garra.pos = (900, 600)  # Posição (x, y)
        self.abrir_garra.background_color = (0.9,0.9,0.9,1)  # Cor original cinza
        self.abrir_garra.bind(on_press=self.abrir_garra_f)  # Conecta o evento de clique
        self.abrir_garra.bind(on_release=lambda instance: self.reset_button_color(instance))  # Conecta o evento de soltar o botão

        self.fechar_garra = Button(text="Fechar Garra", size_hint=(None, None), size=(200,50))
        self.fechar_garra.pos = (900, 500)  # Posição (x, y)
        self.fechar_garra.background_color = (0.9,0.9,0.9,1)  # Cor original cinza
        self.fechar_garra.bind(on_press=self.fechar_garra_f)  # Conecta o evento de clique
        self.fechar_garra.bind(on_release=lambda instance: self.reset_button_color(instance))  # Conecta o evento de soltar o botão

        # Adicionando botões ao layout principal
        root.add_widget(self.start_button)
        root.add_widget(self.stop_button)
        root.add_widget(self.home_button)
        root.add_widget(self.gravar_pos)
        root.add_widget(self.reproduzir_pos)
        root.add_widget(self.ir_para_pos)
        root.add_widget(self.apagar_pos)
        root.add_widget(self.apagar_todas_pos)
        root.add_widget(self.abrir_garra)
        root.add_widget(self.fechar_garra)

        #--------------- SLIDERS --------------------------------------

        #Base Giratoria
        label = Label(text=f"Base Giratoria", size_hint=(None, None), size=(50, 30))
        label.pos = (400, 470)  # Posição do rótulo (um pouco abaixo do slider)
        root.add_widget(label)
        slider_base_gir = Slider(orientation='vertical', min=0, max=180, value=90, size_hint=(None, None), size=(50, 200))  
        slider_base_gir.pos = (400, 500)
        slider_base_gir.thumb_color = (1, 0, 0, 1)
        slider_base_gir.bind(value=self.change_base_gir_position)
        root.add_widget(slider_base_gir)

        #Braco A
        label = Label(text="Braco A", size_hint=(None, None), size=(50, 30))
        label.pos = (500, 470)  # Posição do rótulo (um pouco abaixo do slider)
        root.add_widget(label)
        slider_bracoA = Slider(orientation='vertical', min=0, max=180, value=90, size_hint=(None, None), size=(50, 200))  
        slider_bracoA.pos = (500, 500)
        slider_bracoA.thumb_color = (1, 0, 0, 1)
        slider_bracoA.bind(value=self.change_bracoA_position)
        root.add_widget(slider_bracoA)

        #Braco B
        label = Label(text="Braco B", size_hint=(None, None), size=(50, 30))
        label.pos = (600, 470)  # Posição do rótulo (um pouco abaixo do slider)
        root.add_widget(label)
        slider_bracoB = Slider(orientation='vertical', min=0, max=180, value=90, size_hint=(None, None), size=(50, 200))  
        slider_bracoB.pos = (600, 500)
        slider_bracoB.thumb_color = (1, 0, 0, 1)
        slider_bracoB.bind(value=self.change_bracoB_position)
        root.add_widget(slider_bracoB)

        #Braco C
        label = Label(text="Braco C", size_hint=(None, None), size=(50, 30))
        label.pos = (700, 470)  # Posição do rótulo (um pouco abaixo do slider)
        root.add_widget(label)
        slider_bracoC = Slider(orientation='vertical', min=0, max=180, value=90, size_hint=(None, None), size=(50, 200))  
        slider_bracoC.pos = (700, 500)
        slider_bracoC.thumb_color = (1, 0, 0, 1)
        slider_bracoC.bind(value=self.change_bracoC_position)
        root.add_widget(slider_bracoC)

        #Braco D
        label = Label(text="Braco D", size_hint=(None, None), size=(50, 30))
        label.pos = (800, 470)  # Posição do rótulo (um pouco abaixo do slider)
        root.add_widget(label)
        slider_bracoD = Slider(orientation='vertical', min=0, max=180, value=90, size_hint=(None, None), size=(50, 200))  
        slider_bracoD.pos = (800, 500)
        slider_bracoD.thumb_color = (1, 0, 0, 1)
        slider_bracoD.bind(value=self.change_bracoD_position)
        root.add_widget(slider_bracoD)

        # ------------------ TEXTO POSICOES GRAVADAS -------------------
        #Adicionando Caixa de Texto 
        self.texto_posicoes = Label(text="Posições Gravadas:\n", font_size=20, color=(1,0,0,1), size_hint=(None, None), size=(400,330))
        self.texto_posicoes.pos = (600, 70)
        with self.texto_posicoes.canvas.before:
            Color(0, 0, 0, 1)
            self.rect = RoundedRectangle(size=self.texto_posicoes.size, pos=self.texto_posicoes.pos, radius=[(0, 0, 0, 0)])  # Raio de 20 para todos os cantos

        # Método correto de vinculação
        self.texto_posicoes.bind(size=self.update_rect, pos=self.update_rect)

        root.add_widget(self.texto_posicoes)

        #------------------- DROP-DOWNS ----------------------------

        # Drop-Down Menu p/ IR PARA POSICAO
        pos_go_selector = Spinner(
            text='nº', 
            values=('1', '2', '3', '4', '5', '6', '7', '8', '9', '10'),
            size_hint=(None, None), 
            size=(50, 50),
        
        )
        pos_go_selector.pos = (530, 210)
        pos_go_selector.bind(text=self.pos_go_selector_f)
        root.add_widget(pos_go_selector)
 
        # Drop-Down Menu p/ APAGAR POSICAO
        pos_delete_selector = Spinner(
            text='nº', 
            values=('1', '2', '3', '4', '5', '6', '7', '8', '9', '10'),
            size_hint=(None, None), 
            size=(50, 50),
        
        )
        pos_delete_selector.pos = (530, 140)
        pos_delete_selector.bind(text=self.pos_delete_selector_f)
        root.add_widget(pos_delete_selector)

        # Drop-Down Menu p/ DEFINIR VELOCIDADE
        seletor_velocidade = Spinner(
            text='Velocidade', 
            values=('Velocidade Baixa', 'Velocidade Media', 'Velocidade Alta'),
            size_hint=(None, None), 
            size=(310, 50),
        
        )
        seletor_velocidade.pos = (10, 310)
        seletor_velocidade.bind(text=self.seletor_velocidade_f)
        root.add_widget(seletor_velocidade)

        return root

    #-###################  FUNCOES da GUI ############################

    def update_gui_commands(self, key, value):
        """Atualiza o dicionário guiCommands"""
        self.node.update_gui_commands(key, value)

    def update_rect(self, instance, value):
        # Atualizando o retângulo de fundo para refletir mudanças no tamanho ou posição
        self.rect.pos = instance.pos
        self.rect.size = instance.size

    def change_base_gir_position(self, instance, value):
        self.guiCommands['base_gir_slider_pos'] = int(value)  # Atualiza o dicionário localmente
        self.update_gui_commands('base_gir_slider_pos', int(value))

    def change_bracoA_position(self, instance, value):
        self.guiCommands['base_bracoA_slider_pos'] = int(value)  # Atualiza o dicionário localmente
        self.update_gui_commands('base_bracoA_slider_pos', int(value))

    def change_bracoB_position(self, instance, value):
        self.guiCommands['base_bracoB_slider_pos'] = int(value)  # Atualiza o dicionário localmente
        self.update_gui_commands('base_bracoB_slider_pos', int(value))

    def change_bracoC_position(self, instance, value):
        self.guiCommands['base_bracoC_slider_pos'] = int(value)  # Atualiza o dicionário localmente
        self.update_gui_commands('base_bracoC_slider_pos', int(value))

    def change_bracoD_position(self, instance, value):
        self.guiCommands['base_bracoD_slider_pos'] = int(value)  # Atualiza o dicionário localmente
        self.update_gui_commands('base_bracoD_slider_pos', int(value))


    def start_robot(self, instance):
        self.start_button.background_color = (1, 0, 0, 1)
        self.update_gui_commands('start', True)
        self.update_gui_commands('stop', False)
        self.stop_button.background_color = (0.9, 0.9, 0.9, 1)

    def stop_robot(self, instance):
        self.stop_button.background_color = (1, 0, 0, 1)
        self.update_gui_commands('stop', True)
        self.update_gui_commands('start', False)
        self.start_button.background_color = (0.9, 0.9, 0.9, 1)

    def home_robot(self, instance):
        self.home_button.background_color = (1, 0, 0, 1)
        self.update_gui_commands('home', True)
        Clock.schedule_once(lambda dt: self.update_gui_commands('home', False), 0.1)

    def gravar_posicao(self, instance):
        self.gravar_pos.background_color = (1, 0, 0, 1)
        self.update_gui_commands('gravar_posicao', True)
        Clock.schedule_once(lambda dt: self.update_gui_commands('gravar_posicao', False), 0.1)

        # Gravar as posições atuais dos sliders na matriz
        current_positions = [
            self.guiCommands.get('base_gir_slider_pos'),
            self.guiCommands.get('base_bracoA_slider_pos'),
            self.guiCommands.get('base_bracoB_slider_pos'),
            self.guiCommands.get('base_bracoC_slider_pos'),
            self.guiCommands.get('base_bracoD_slider_pos'),
            self.guiCommands.get('abrir_garra')      
        ]
        self.positions_matrix.append(current_positions)
        print(self.positions_matrix)

        # Atualizar a caixa de texto com as posições gravadas
        self.update_texto_posicoes()

    def update_texto_posicoes(self):
        # Atualiza o texto na caixa de texto com as posições gravadas
        texto = "Posições Gravadas:\n"
        for i, pos in enumerate(self.positions_matrix):
            texto += f"Posição {i+1}: {pos}\n"
        self.texto_posicoes.text = texto

    def reproduzir_posicoes(self, instance):
        self.reproduzir_pos.background_color = (1, 0, 0, 1)
        self.update_gui_commands('reproduzir_pos', True)     
        Clock.schedule_once(lambda dt: self.update_gui_commands('reproduzir_pos', False), 0.1)

    def reset_button_color(self, instance):
        instance.background_color = (0.9, 0.9, 0.9, 1)

    def pos_go_selector_f(self, instance, value):
        self.update_gui_commands('posicao_selecionada_go', value)      

    def pos_delete_selector_f(self, instance, value):
        self.update_gui_commands('posicao_selecionada_delete', value)      

    def go_to_pos(self,instance):
        self.ir_para_pos.background_color = (1, 0, 0, 1)
        self.update_gui_commands('ir_posicao_selecionada', True)      

    def apagar_posicao(self, instance):
        self.apagar_pos.background_color = (1, 0, 0, 1)
        self.update_gui_commands('apagar_posicao_selecionada', True) 

    def apagar_todas_posicoes(self, instance):
        instance.background_color = (1, 0, 0, 1)
        self.update_gui_commands('apagar_tudo', True) 
        self.positions_matrix = []  # Limpa a matriz de posições
        self.update_texto_posicoes()  # Atualiza a caixa de texto
        Clock.schedule_once(lambda dt: self.update_gui_commands('apagar_tudo', False), 0.1)

    def seletor_velocidade_f (self, instance, value):
        instance.background_color = (0.9, 0.9, 0.9, 1)
        self.update_gui_commands('velocidade', value) 

    def abrir_garra_f(self, instance):
        instance.background_color = (1, 0, 0, 1)
        self.update_gui_commands('abrir_garra', True) 
        self.update_gui_commands('fechar_garra', False)
        self.guiCommands['abrir_garra'] = True  # Atualiza o dicionário localmente

    def fechar_garra_f(self, instance):
        instance.background_color = (1, 0, 0, 1)
        self.update_gui_commands('fechar_garra', True) 
        self.update_gui_commands('abrir_garra', False) 
        self.guiCommands['abrir_garra'] = False  # Atualiza o dicionário localmente

def dict_to_string(dicionario):
    dicionario = guiCommands
    print(dicionario)
    return json.loads(dicionario)