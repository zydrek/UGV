#fase 3.3.1 
 
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
    global handler_counter 
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
        handler_counter += 1 
        if handler_counter==10: 
           handler_counter=0 
        #inicio da fase 3.1.1 - orientação e centralizaçao por bandeiras 
        if ids is not None and 0 in ids: 
             
            # armazém temporário para guardar os dados dos ArUcos id== 0 
            marcadores_validos = [] 
             
            # varrer todos os marcadores que a câmara está a ver 
            for i in range(len(ids)): 
                if ids[i][0] == 0:  # só nos interessam os arucos de navegação 
                    pontos = cantos[i][0] 
                     
                    #calcular a área e o centro x 
                    area = cv2.contourArea(pontos) 
                    centro_x = int(pontos.mean(axis=0)[0]) 
                     
                    # guardar no armazém 
                    marcadores_validos.append({"area": area, "x": centro_x}) 
             
            #filtrar mais próximos: ordenar a lista de maior para menor 
            marcadores_validos = sorted(marcadores_validos, key=lambda d: 
d['area'], reverse=True) 
             
            #isolar apenas os dois maiores (top2) 
            alvos = marcadores_validos[:2] 
            #daqui acontecem 2 cenarios: cenario A vê 2 paredes perfeitamente, 
cenario B só vê uma parede.  
            #cenario A 
            if len(alvos) == 2: 
                # Ordenar os dois alvos pelo eixo X para sabermos fisicamente qual 
é o da esquerda e direita 
                alvos = sorted(alvos, key=lambda d: d['x']) 
                marcador_esq = alvos[0] 
                marcador_dir = alvos[1] 
                 
                #calculo do erro entre arucos laterais: subtrair A_dir com A_esq 
                diferenca_area = marcador_esq['area'] - marcador_dir['area'] 
                 
                # tolerância de pixeis (Afinar no laboratório! Define o quão "perfeito" o 
centro tem de ser) 
                tolerancia = 4000  
                 
                if diferenca_area > tolerancia: 
                    print(f"Balança pende à ESQUERDA (Dif: {diferenca_area}). A 
corrigir para a Direita.") 
                    base.base_json_ctrl({"T":13, "X":0.0, "Z": -0.2})  
                     
                elif diferenca_area < -tolerancia: 
                    print(f"Balança pende à DIREITA (Dif: {diferenca_area}). A corrigir 
para a Esquerda.") 
                    base.base_json_ctrl({"T":13, "X":0.0, "Z": 0.2})  
                     
                else: 
                    print("Corredor estabilizado. A avançar.") 
                    base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
 
            #cenario b (perda de fov - erro formatado para um intervalo de 320 
pixeis) 
            elif len(alvos) == 1: 
                unico_marcador = alvos[0] 
                 
                # Se o marcador estiver na metade esquerda do ecrã, a parede 
esquerda está muito perto 
                if unico_marcador['x'] < 320: 
                    print("PERIGO FOV: Colisão iminente à Esquerda! A afastar para a 
Direita.") 
                    base.base_json_ctrl({"T":13, "X":0.0, "Z": -0.4}) # Rotação de fuga 
agressiva 
                     
                # Se estiver na metade direita do ecrã 
                else: 
                    print("PERIGO FOV: Colisão iminente à Direita! A afastar para a 
Esquerda.") 
                    base.base_json_ctrl({"T":13, "X":0.0, "Z": 0.4}) # Rotação de fuga 
agressiva 
        #-------------------------------------------------------------------------Fim da fase 3.1.1-
---------------------# 
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
