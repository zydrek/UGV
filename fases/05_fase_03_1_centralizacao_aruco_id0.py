#fase 3.1  - centralização do robo pelo id==0 (centralização por 1 só aruco no 
chão) 
#importação de bibliotecas para utilização de arucos, display no ecra, time e 
mexer nos motores 
import cv2 
from IPython.display import display,Image 
import time 
from base_ctrl import BaseController 
 
#importar threading e widgets para botões 
import threading 
import ipywidgets as widgets 
#atraso entre amostragem de imagem e leitura de frames  
#(frames com step 0.1s e leitura de imagem a 1s de momento.ver time.sleep) 
#handler_counter = 0 
 
# Detecta modelo 
 
def is_raspberry_pi5(): 
    with open('/proc/cpuinfo', 'r') as file: 
        for line in file: 
            if 'Model' in line: 
                return 'Raspberry Pi 5' in line 
    return False 
 
# Inicializa base motores  
if is_raspberry_pi5(): 
    base = BaseController('/dev/ttyAMA0', 115200) 
else: 
    base = BaseController('/dev/serial0', 115200) 
 
#criar interface visual (botoes) 
stopButton = widgets.ToggleButton( 
    value=False, 
    description='Stop', 
    disabled=False, 
    button_style='danger', 
    tooltip='Stop', 
    icon='square' 
) 
#marcadores aruco 
 
dicionario_aruco=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250) 
parametros_aruco=cv2.aruco.DetectorParameters() 
 
#criar objeto detetor 
detetor=cv2.aruco.ArucoDetector(dicionario_aruco,parametros_aruco) 
 
#ligar visao do robo 
def iniciar_visao(): 
    # global handler_counter 
    #estabilizar camara do robo 
    gimbal_x = 0 
    gimbal_y = 0 
    gimbal_speed = 0 
    gimbal_acc = 0 
    base.gimbal_ctrl(gimbal_x, gimbal_y, gimbal_speed, gimbal_acc) 
 
    #captura da camara do robo 
    camera = cv2.VideoCapture(-1) 
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
    #preparar ecrã jupyter 
    display_handle=display(None,display_id=True) 
    #ciclo infinito de leitura visual 
    while True: 
        sucesso,frame=camera.read() 
 
        if not sucesso: 
            print("erro") 
            break 
        #converter palete para cinzento (otimizar aruco. visão maquina) 
        frame_cinza=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
        #caçar aruco com o detetor 
        cantos,ids,rejeitados=detetor.detectMarkers(frame_cinza) 
 
         
        #desenhar quadrado verde a cores (visao humana) 
        if ids is not None: 
            #print(ids) #deteção dos ids dos aruco 
            
cv2.aruco.drawDetectedMarkers(frame,cantos,ids,borderColor=(0,255,0)) 
             
        #mostrar imagem 
        _,jpeg=cv2.imencode('.jpeg', frame) 
        display_handle.update(Image(data=jpeg.tobytes())) 
#codigo para fazer screen refresh a cada segundo (cada +1 equivale 0.1s) 
       # handler_counter += 1 
        #if handler_counter==10: 
       #    handler_counter=0 
#------------------------------------------------------------- 
        # INICIO FASE3.1 - CENTRALIZAÇÃO ROBO NA DETEÇÃO DE ARUCO 
DE MOVIMENTO (ID==0) 
        if ids is not None and 0 in ids: 
                 
            # 1.extrair os cantos do marcador id 0 
            #procuramos o índice (posição) do id 0 na lista 
            indice_do_zero = list(ids.squeeze()).index(0) if ids.size > 1 else 0 
            pontos_do_marcador = cantos[indice_do_zero][0] 
             
            #2.calcular o centro 
            centro_x, centro_y = pontos_do_marcador.mean(axis=0) 
            centro_x = int(centro_x) # Converter para inteiro (não existem meios-
píxeis) 
             
            # 3.calcular o desvio em relação ao centro do robô 
            erro_x = centro_x - 320 
             
            # 4.cinemática (ação dos motores com zona morta de 50 píxeis) 
            if erro_x < -50: 
                print(f"marcador à esquerda (Erro: {erro_x}). A centralizar") 
                # T:13 é a ordem de movimento. X é frente/trás. Z é a rotação. 
                base.base_json_ctrl({"T":13, "X":0.0, "Z": 0.3})  
                 
            elif erro_x > 50: 
                print(f"marcador à direita (erro: {erro_x}). A centralizar") 
                base.base_json_ctrl({"T":13, "X":0.0, "Z": -0.3})  
                 
            else: 
                print(f"centrado (erro: {erro_x}). A avançar para o alvo.") 
                base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
             
            #pequena pausa para os motores reagirem 
            time.sleep(0.1)  
            base.base_speed_ctrl(0, 0) 
   
        #botao grafico stop 
        if stopButton.value==True: 
            camera.release() 
            display_handle.update(None) 
            print("Câmara e recursos desligados com segurança") 
            break 
 
#lançar codigo 2º plano 
display(stopButton) 
thread = threading.Thread(target=iniciar_visao) 
thread.start()
