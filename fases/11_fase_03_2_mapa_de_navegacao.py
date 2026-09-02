#fase 3.2 - implementação de mapa de navegação 
 
import cv2 
from IPython.display import display, Image 
import time 
from base_ctrl import BaseController 
import threading 
import ipywidgets as widgets 
 
 
#inercia na amostragem de imagem (delay) 
handler_counter = 0 
#variavel de inercia quando o array de ponteiros dá conjunto vazio 
c1 = 0  
#declarar variaveis de controlo pid (I e D) 
erro_i = 0 
erro_anterior = 0 
#iniciar os motores do robo 
def is_raspberry_pi5(): 
    with open('/proc/cpuinfo', 'r') as file: 
        for line in file: 
            if 'Model' in line: 
                return 'Raspberry Pi 5' in line 
    return False 
 
if is_raspberry_pi5(): 
    base = BaseController('/dev/ttyAMA0', 115200) 
else: 
    base = BaseController('/dev/serial0', 115200) 
#declaração de cruzamentos e caminhos que podem ser feitos  
mapa_de_navegacao = { 
    10: { 
        1:  { 21: "esquerda", 22: "direita" }, 
        21: { 1: "direita",   22: "frente" },  # Lógica de regresso do 21 para o 1 
        22: { 1: "esquerda",  21: "frente" }   # Lógica de regresso do 22 para o 1 
    } 
} 
#criaçao de widget botão stop  
stopButton = widgets.ToggleButton( 
    value=False, description='Stop', disabled=False, 
    button_style='danger', tooltip='Stop', icon='square' 
) 
 
dicionario_aruco = 
cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250) 
parametros_aruco = cv2.aruco.DetectorParameters() 
detetor = cv2.aruco.ArucoDetector(dicionario_aruco, parametros_aruco) 
 
def iniciar_visao(): 
    global handler_counter, c1, erro_i, erro_anterior 
     
    gimbal_x, gimbal_y, gimbal_speed, gimbal_acc = 0, 0, 0, 0 
    base.gimbal_ctrl(gimbal_x, gimbal_y, gimbal_speed, gimbal_acc) 
 
    camera = cv2.VideoCapture(-1) 
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
    display_handle = display(None, display_id=True) 
     
    print("A iniciar Navegação P e Matchmaking...") 
    origem_atual = 1        # Ponto de partida 
    print("atual detetado") 
    destino_final = 21      # Destino escolhido para a viagem de ida 
    print("final detetado") 
    bloqueio_cruzamento = 0 
    print("bloqueio detetado") 
    while True: 
        sucesso, frame = camera.read() 
        if not sucesso: 
            print("erro") 
            break 
             
        frame_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        cantos, ids, rejeitados = detetor.detectMarkers(frame_cinza) 
 
        if ids is not None: 
            cv2.aruco.drawDetectedMarkers(frame, cantos, ids, 
borderColor=(0,255,0)) 
             
        _, jpeg = cv2.imencode('.jpeg', frame) 
        display_handle.update(Image(data=jpeg.tobytes())) 
         
        handler_counter += 1 
        if handler_counter == 10: 
            handler_counter = 0 
 
        #------------------------------------------------------------- 
        # INICIO FASE 3.1.3 - NAVEGAÇÃO GEOMÉTRICA (FILTRO DE PARES) 
        #------------------------------------------------------------- 
        if ids is not None and 0 in ids: 
             
            esq_lista = [] 
            dir_lista = [] 
             
            for i in range(len(ids)): 
                if ids[i][0] == 0:   
                    pontos = cantos[i][0] 
                    area = cv2.contourArea(pontos) 
                    centro_x = int(pontos.mean(axis=0)[0]) 
                    marcador = {"area": area, "x": centro_x} 
                     
                    if centro_x < 320: 
                        esq_lista.append(marcador) 
                    else: 
                        dir_lista.append(marcador) 
             
            esq_lista = sorted(esq_lista, key=lambda d: d['area'], reverse=True) 
            dir_lista = sorted(dir_lista, key=lambda d: d['area'], reverse=True) 
             
            # 
======================================================== 
            #constante limiar de perigo (quando está numa zona de possivel 
embate) 
            LIMIAR_PERIGO = 25000  
            #constantes PID 
            kp = 0.01 
            ki = 0.0001 
            kd = 0.05 
            #tolerancia de par de arucos mediante os mais proximos e os que vêm 
a seguir 
            TOLERANCIA_PAR = 0.4  
            # 
======================================================== 
             
            par_valido = None 
             
            for e in esq_lista: 
                for d in dir_lista: 
                    racio_area = min(e['area'], d['area']) / max(e['area'], d['area']) 
                    if racio_area > TOLERANCIA_PAR: 
                        par_valido = (e, d) 
                        break  
                if par_valido:  
                    break 
 
            # --- CENÁRIO A: Par Válido --- 
            if par_valido is not None: 
                esq = par_valido[0] 
                dir = par_valido[1] 
 
                #calculo da media entre arucos e do erro instantaneo (P) 
                ponto_medio = (esq['x'] + dir['x']) / 2 
                erro_x = ponto_medio - 320 
                #somatorio de erros do passado (Integral) 
                erro_i += erro_x  
                #somatorio de erros do futuro (Derivado) 
                erro_d = erro_x - erro_anterior 
                # Proteção Anti-Windup (Muito importante em robótica!) 
                # Impede que a memória do erro cresça infinitamente se o robô ficar 
preso. 
                if erro_i > 1000: erro_i = 1000 
                elif erro_i < -1000: erro_i = -1000 
                 
                #controlo pid somado 
                correcao_z =  -(kp * erro_x) - (ki * erro_i) - (kd * erro_d) 
                print(f"Erro: {erro_x} | P: {-kp*erro_x:.3f} | I: {-ki*erro_i:.3f} | D: {-
kd*erro_d:.3f}") 
                base.base_json_ctrl({"T":13, "X":0.25, "Z": correcao_z}) 
                 
                #atualizar a variavel do erro D para o próximo ciclo 
                erro_anterior = erro_x 
                # ZERAR INÉRCIA: Vemos o caminho, o pânico acaba!  
                c1 = 0  
 
            # --- CENÁRIO B: Falha no Par (Só vemos órfão) --- 
            else: 
                todos = esq_lista + dir_lista 
                if len(todos) > 0: 
                    unico = max(todos, key=lambda d: d['area']) 
                    print(f"[Sem Par Válido] A usar maior visível - Área: 
{int(unico['area'])}") 
                     
                    if unico['area'] > LIMIAR_PERIGO: 
                        if unico['x'] < 320: 
                            print("!! PERIGO: Fuga p/ Direita !!") 
                            base.base_json_ctrl({"T":13, "X":0.0, "Z": -0.4}) 
                        else: 
                            print("!! PERIGO: Fuga p/ Esquerda !!") 
                            base.base_json_ctrl({"T":13, "X":0.0, "Z": 0.4}) 
                    else: 
                        if unico['x'] > 320: 
                            base.base_json_ctrl({"T":13, "X":0.25, "Z": -0.15}) 
                        else: 
                            base.base_json_ctrl({"T":13, "X":0.25, "Z": 0.15}) 
                             
                    # ZERAR INÉRCIA: Apesar de ser só 1 marcador, não estamos 
totalmente cegos. 
                    c1 = 0 
 
        # --- CENÁRIO C (A TUA INÉRCIA): Cegueira Total (Nenhum ArUco 
visível) --- 
        else: 
            if c1 < 10: # Cerca de 1 segundo a 10 frames/s 
                print(f"Inércia [{c1}/10]: Perda Visual. A manter rota a direito...") 
                base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
                c1 += 1 
            else: 
                print("Cegueira Confirmada. Motores Parados.") 
                base.base_speed_ctrl(0, 0) 
         
        #------------------------------------------------------------------------- 
         
        #------------------------------------------------------------- 
        # INICIO FASE 3.2 - GESTÃO DE CRUZAMENTOS E REGRESSO 
AUTÓNOMO 
        #------------------------------------------------------------- 
        if ids is not None: 
             
            # 1. VERIFICAR SE CHEGÁMOS AO DESTINO (O Fim da Linha) 
            if destino_final in ids: 
                indice = list(ids).index(destino_final) 
                area_destino = cv2.contourArea(cantos[indice][0]) 
                 
                LIMIAR_DESTINO = 30000 # Área para considerar que estamos "em 
cima" da meta 
                 
                if area_destino > LIMIAR_DESTINO: 
                    print(f"🏁 DESTINO {destino_final} ALCANÇADO! A travar...") 
                    base.base_speed_ctrl(0, 0) 
                    time.sleep(1.0) 
                     
                    # --- A TUA LÓGICA DE REGRESSO AUTÓNOMO --- 
                    if destino_final != 1:  
                        print("🔄 A inverter marcha para regressar à Base (1)...") 
                         
                        # Executa o Pião de 180º 
                        base.base_json_ctrl({"T":13, "X":0.0, "Z": 0.8}) 
                        time.sleep(2.5) # <- TEMPO PARA 180º 
                        base.base_speed_ctrl(0, 0) 
                         
                        # A Máquina de Estados inverte-se! 
                        origem_atual = destino_final 
                        destino_final = 1 
                        bloqueio_cruzamento = 30 # Dá tempo ao robô para sair do foco 
do destino 
                         
                        print(f"-> Nova Missão: Ir de {origem_atual} para {destino_final}") 
                     
                    else: 
                        print("🏠 REGRESSO À BASE CONCLUÍDO. Fim do 
programa.") 
                        break # Se o destino era 1 e já lá chegou, desliga tudo. 
 
            # 2. VERIFICAR CRUZAMENTOS (ID 10) 
            if 10 in ids and bloqueio_cruzamento == 0: 
                indice = list(ids).index(10) 
                area_cruzamento = cv2.contourArea(cantos[indice][0]) 
                #---------------------------------- 
                LIMIAR_CHAO = 5340 
                #--------------------------------- 
                print(f"area de aruco {area_cruzamento}") 
                  
                if area_cruzamento > LIMIAR_CHAO: 
                    print(f"🛑 CRUZAMENTO 10 ATINGIDO! A consultar o mapa...") 
                    base.base_speed_ctrl(0, 0) 
                    time.sleep(0.5) 
                     
                    try: 
                        # Extrai a instrução do teu dicionário (Esquerda, Direita ou 
Frente) 
                        acao = mapa_de_navegacao[10][origem_atual][destino_final] 
                        print(f"-> Rota: Origem {origem_atual} | Destino {destino_final} 
=> Ação: {acao.upper()}") 
                         
                        # TRADUÇÃO DAS STRINGS PARA COMANDOS MOTORES 
                        if acao == "esquerda": 
                            base.base_json_ctrl({"T":13, "X":0.0, "Z": 0.5}) 
                            time.sleep(1.2) # Tempo para curvar 90º à Esquerda 
                             
                        elif acao == "direita": 
                            base.base_json_ctrl({"T":13, "X":0.0, "Z": -0.5}) 
                            time.sleep(1.2) # Tempo para curvar 90º à Direita 
                             
                        elif acao == "frente": 
                            base.base_json_ctrl({"T":13, "X":0.20, "Z": 0.0}) 
                            time.sleep(1.5) # Tempo para atravessar o cruzamento a 
direito 
                             
                        # Atualiza a memória de onde o robô veio para a próxima 
decisão 
                        origem_atual = 10 
                        bloqueio_cruzamento = 20  
                         
                    except KeyError: 
                        print(f"ERRO DE MAPA: Rota de {origem_atual} para 
{destino_final} não existe!") 
                        base.base_speed_ctrl(0, 0) 
                        break 
 
        # Reduzir o bloqueio gradualmente a cada frame 
        if bloqueio_cruzamento > 0: 
            bloqueio_cruzamento -= 1 
 
         
         
        time.sleep(0.1)  
   
        if stopButton.value == True: 
            base.base_speed_ctrl(0, 0) 
            camera.release() 
            display_handle.update(None) 
            print("Câmara e recursos desligados com segurança") 
            break 
 
display(stopButton) 
thread = threading.Thread(target=iniciar_visao) 
thread.start()
