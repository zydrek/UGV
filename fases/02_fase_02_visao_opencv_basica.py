#codigo teste fase 2: Visão computacional básica com openCV 
#captação de video real e mostrar no ecrã ; converter video para escala de 
cinzentos. 
 
import cv2 
from IPython.display import display,Image 
import time 
 
#importar bibliotecas graficas para visualização da câmara 
import threading 
import ipywidgets as widgets 
 
#criar interface visual (botoes) 
stopButton = widgets.ToggleButton( 
    value=False, 
    description='Stop', 
    disabled=False, 
    button_style='danger', 
    tooltip='Stop', 
    icon='square' 
) 
#ligar olhos do robo 
def iniciar_visao(): 
    camera = cv2.VideoCapture(-1) 
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
    #preparar ecrã jupyter 
    display_handle=display(None,display_id=True) 
        #ciclo infinito de visao 
    while True: 
        sucesso,frame=camera.read() 
 
        if sucesso: 
 
            #converter palete para cinzento (otimizar aruco) 
            frame_cinza=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
 
            #mostrar imagem 
            _,jpeg=cv2.imencode('.jpeg', frame_cinza) 
            display_handle.update(Image(data=jpeg.tobytes())) 
 
        #pausa para estabilizaçao 
        time.sleep(0.05) 
   
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
 
Código 3
