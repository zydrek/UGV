#fase 4.1 - adição de cruzamento ao match/case - teste com id10 
import cv2 
import time 
from base_ctrl import BaseController 
import threading 
import ipywidgets as widgets 
from IPython.display import display # Apenas usado para mostrar o Dashboard 
final 
 
# 
============================================================
======== 
# 1. VARIÁVEIS GLOBAIS E INICIALIZAÇÃO 
# 
============================================================
======== 
base = None 
detetor = None 
handler_counter = 0 
c1 = 0 #contador handler_counter 
erro_i = 0  
erro_anterior = 0 
robo_ativo = False # O robô arranca parado à espera do botão Start 
bloqueio_cruzamento = 0 #Impede leituras repetidas do ID 10 
 
# NOVAS VARIÁVEIS DE FOCO 
cruzamento_focado = False  
ultimo_marcador_10 = None 
inercia_cruz = 0 
 
def is_raspberry_pi5(): 
    with open('/proc/cpuinfo', 'r') as file: 
        for line in file: 
            if 'Model' in line: 
                return 'Raspberry Pi 5' in line 
    return False 
 
def inicializar_hardware(): 
    global base, detetor 
 
    output_str = "" 
    output_str += "[SETUP] A iniciar componentes de hardware...\n" 
     
    if is_raspberry_pi5(): 
        base = BaseController('/dev/ttyAMA0', 115200) 
    else: 
        base = BaseController('/dev/serial0', 115200) 
         
    output_str += "[SETUP] Comunicação Serial com os motores 
estabelecida.\n" 
     
    dicionario_aruco = 
cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250) 
    parametros_aruco = cv2.aruco.DetectorParameters() 
    detetor = cv2.aruco.ArucoDetector(dicionario_aruco, parametros_aruco) 
     
    output_str += "[SETUP] Dicionário ArUco e motor de visão carregados.\n" 
    output_str += "[SETUP] Robô pronto a operar!\n" 
 
    return output_str 
 
# 
============================================================
======== 
# 2. FUNÇÕES DO CÉREBRO E CONDUÇÃO 
# 
============================================================
======== 
 
def classificar_estado_visao(ids, cantos): 
    global bloqueio_cruzamento, cruzamento_focado, ultimo_marcador_10, 
inercia_cruz 
     
    # 
============================================================
===== 
    # ETAPA 1: GESTÃO DE DESFOQUE E MEMÓRIA DE CURTO PRAZO 
    # Se a câmara não encontrar nenhum Aruco na frame atual 
    # 
============================================================
===== 
    if ids is None: 
        # Verifica se o robô estava a meio de uma aproximação a um cruzamento 
        if cruzamento_focado: 
            inercia_cruz += 1 # Incrementa o contador de "cegueira temporária" 
             
            # Tolerância de ~1.5 segundos para o Gimbal focar ou recuperar o alvo 
            if inercia_cruz < 15:  
                # Devolve o estado de cruzamento usando os últimos dados 
conhecidos (Memória) 
                return "CRUZAMENTO", [], [], ultimo_marcador_10 
            else: 
                # O alvo foi perdido definitivamente (ex: removido à força). Aborta a 
missão. 
                cruzamento_focado = False  
         
        # Se não estava focado em nada, ou se a inércia expirou, declara 
cegueira total. 
        return "CEGUEIRA", [], [], None 
 
    # 
============================================================
===== 
    # ETAPA 2: DETEÇÃO ATIVA E "LOCK-ON" (TRANCAR ALVO) 
    # Se o ID 10 for detetado e não estivermos em período de "cooldown" 
    # 
============================================================
===== 
    if 10 in ids and bloqueio_cruzamento == 0: 
        cruzamento_focado = True # Ativa o estado de prioridade máxima 
        inercia_cruz = 0         # Zera a inércia pois a visão do alvo é clara e atual 
         
        # Extração de dados geométricos do ArUco 10 
        indice = list(ids).index(10) 
        pontos = cantos[indice][0] 
        area = cv2.contourArea(pontos) 
        centro_x = int(pontos.mean(axis=0)[0]) 
         
        # Atualização constante da memória com os dados mais recentes 
        ultimo_marcador_10 = {"area": area, "x": centro_x}  
         
        return "CRUZAMENTO", [], [], ultimo_marcador_10 
         
    # 
============================================================
===== 
    # ETAPA 3: FILTRO DE PRIORIDADE (IGNORAR RUÍDO) 
    # Se a câmara detetar as paredes (ID 0) enquanto desce o Gimbal,  
    # deve ignorá-las para não interromper a aproximação ao cruzamento. 
    # 
============================================================
===== 
    if cruzamento_focado: 
        inercia_cruz += 1 
        if inercia_cruz < 15: 
            return "CRUZAMENTO", [], [], ultimo_marcador_10 
        else: 
            cruzamento_focado = False 
 
    # 
============================================================
===== 
    # ETAPA 4: NAVEGAÇÃO NORMAL DE CORREDOR (ID 0) 
    # Executada apenas se nenhum cruzamento estiver ativado ou focado. 
    # 
============================================================
===== 
    if 0 not in ids: 
        return "CEGUEIRA", [], [], None 
 
    esq_lista, dir_lista = [], [] 
     
    # (O resto do teu código de separar esquerdas e direitas continua aqui 
normalmente...) 
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
     
    if len(esq_lista) > 0 and len(dir_lista) > 0: 
        TOLERANCIA_PAR = 0.4  
        for e in esq_lista: 
            for d in dir_lista: 
                racio_area = min(e['area'], d['area']) / max(e['area'], d['area']) 
                if racio_area > TOLERANCIA_PAR: 
                    return "PARES", [e], [d], None  
                     
    return "ORFAO", esq_lista, dir_lista, None 
def testar_cruzamento_generico(marcador_10): 
    global bloqueio_cruzamento, cruzamento_focado  
     
    LIMIAR_CHAO = 5550  
    area = marcador_10['area'] 
    erro_x = marcador_10['x'] - 320 
     
    # ETAPA 1: Longe do Cruzamento -> Aproximação Guiada 
    if area < LIMIAR_CHAO: 
        base.gimbal_ctrl(0, -45, 0, 0)  
         
        kp_chao = 0.005 
        Z = -kp_chao * erro_x 
         
        base.base_json_ctrl({"T":13, "X":0.10, "Z": Z}) 
        return f"[CRUZAMENTO] A aproximar... Área: {int(area)} | Z: {Z:.3f}" 
         
    # ETAPA 2: Em cima do Cruzamento -> Manobra Centrada 
    else: 
        # 1. Avançar cego para centrar as rodas do robô em cima do ArUco 10 
        base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
        time.sleep(1.5) 
         
        # 2. Trava o robô 
        base.base_speed_ctrl(0, 0) 
        time.sleep(0.5) 
         
        # 3. Levanta a cabeça para o estado de repouso 
        base.gimbal_ctrl(0, 0, 0, 0) 
        time.sleep(0.5)  
         
        # 4. Rotação a 90º (Fixo para testes) 
        acao_teste = "direita" 
         
        if acao_teste == "esquerda": 
            base.base_json_ctrl({"T":13, "X":0.0, "Z": 2}) 
            time.sleep(1) 
        elif acao_teste == "direita": 
            base.base_json_ctrl({"T":13, "X":0.0, "Z": -2}) 
            time.sleep(1) 
        elif acao_teste == "frente": 
            base.base_json_ctrl({"T":13, "X":0.20, "Z": 0.0}) 
            time.sleep(1) 
             
        # 5. Parar após a curva 
        base.base_speed_ctrl(0, 0) 
         
        # 6. ATIVAR O BLOQUEIO E LIBERTAR FOCO 
        bloqueio_cruzamento = 30 # Aumentei um pouco para dar margem de 
descompressão 
        cruzamento_focado = False # Missão cumprida, volta ao normal! 
         
        return f"[CRUZAMENTO] Manobra concluída: {acao_teste.upper()}!" 
 
def navegar_pares(esq, dir): 
    global erro_i, erro_anterior, c1  
     
    kp, ki, kd = 0.005, 0.0001, 0.05 
     
    ponto_medio = (esq['x'] + dir['x']) / 2 
    erro_x = ponto_medio - 320 
     
    erro_i += erro_x  
    erro_d = erro_x - erro_anterior 
     
    if erro_i > 1000: erro_i = 1000 
    elif erro_i < -1000: erro_i = -1000 
     
    correcao_z = -(kp * erro_x) - (ki * erro_i) - (kd * erro_d) 
     
    
    base.base_json_ctrl({"T":13, "X":0.15, "Z": correcao_z}) 
     
    erro_anterior = erro_x 
    c1 = 0 
    # Reset do Gimbal para a posição de repouso (olhar em frente) 
    base.gimbal_ctrl(0, 0, 0, 0) 
     
    return f"[PARES] Erro: {erro_x} | Z: {correcao_z:.3f}" 
     
def navegar_orfao(todos_marcadores): 
    global c1 
    base.base_json_ctrl({"T":13, "X":0.10, "Z": 0}) 
    print("[ORFÃO] gimbal e inercia para zero") 
    # Reset do Gimbal para a posição de repouso (olhar em frente) 
    base.gimbal_ctrl(0, 0, 0, 0) 
    c1 = 0 
    return "[ORFÃO] Seguindo em frente até encontrar par." 
     
def navegar_cegueira(): 
    global c1 
    if c1 < 10: 
         
        base.base_json_ctrl({"T":13, "X":0.10, "Z": 0.0}) 
        c1+=1 
        return f"[CEGUEIRA] Inércia {c1}/10. A manter frente..." 
    else: 
         
        base.base_speed_ctrl(0, 0) 
         
        # Reset do Gimbal para a posição de repouso (olhar em frente) 
        base.gimbal_ctrl(0, 0, 0, 0) 
        print("gimbal 0 cegueira") 
        return "[CEGUEIRA] Paragem de segurança." 
# 
============================================================
======== 
# 3. CICLO PRINCIPAL (A THREAD DE VISÃO) 
# 
============================================================
======== 
 
def iniciar_visao(widget_imagem): 
    global handler_counter, robo_ativo 
 
    gimbal_x, gimbal_y, gimbal_speed, gimbal_acc = 0, 0, 0, 0 
    base.gimbal_ctrl(gimbal_x, gimbal_y, gimbal_speed, gimbal_acc) 
     
    camera = cv2.VideoCapture(-1) 
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
     
    print("Câmara iniciada. À espera que cliques em START...") 
     
    while True: 
        sucesso, frame = camera.read() 
        if not sucesso:  
            print("Erro fatal: Não foi possível ler a câmara.") 
            break 
             
        frame_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        cantos, ids, rejeitados = detetor.detectMarkers(frame_cinza) 
         
        if ids is not None: 
            cv2.aruco.drawDetectedMarkers(frame, cantos, ids, 
borderColor=(0,255,0)) 
             
        # Atualização LIMPA da imagem no Dashboard 
        _, jpeg = cv2.imencode('.jpeg', frame) 
        widget_imagem.value = jpeg.tobytes() 
 
        # --------------------------------------------------------- 
        # TOMADA DE DECISÃO (SÓ FUNCIONA SE START FOR 
PRESSIONADO) 
        # --------------------------------------------------------- 
        if robo_ativo: 
            # Atenção: Adicionamos a recolha do 4º parâmetro (marcador_10) 
            estado_atual, lista_e, lista_d, marcador_10 = 
classificar_estado_visao(ids, cantos) 
             
            match estado_atual: 
                case "CRUZAMENTO": 
                    print(testar_cruzamento_generico(marcador_10)) 
                case "PARES": 
                    print(navegar_pares(lista_e[0], lista_d[0])) 
                case "ORFAO": 
                    todos = lista_e + lista_d 
                    print(navegar_orfao(todos)) 
                case "CEGUEIRA": 
                    print(navegar_cegueira()) 
        else: 
            base.base_speed_ctrl(0, 0) # Mantém-se quieto se não houver Start 
                 
        time.sleep(0.1) 
 
        # --------------------------------------------------------- 
        # SEGURANÇA (STOP) 
        # --------------------------------------------------------- 
        if stopButton.value == True: 
            camera.release() 
            print("Câmara desligada. Ciclo terminado.") 
            base.base_speed_ctrl(0, 0) 
            break 
 
# 
============================================================
======== 
# 4. EXECUÇÃO PRINCIPAL (DASHBOARD REDUZIDO) 
# 
============================================================
======== 
if __name__ == "__main__": 
     
    # 1. Criação dos Botões 
    startButton = widgets.ToggleButton( 
        value=False, description='Start', button_style='success', icon='play' 
    ) 
    stopButton = widgets.ToggleButton( 
        value=False, description='Stop', button_style='danger', icon='square' 
    ) 
     
    # Esta função deteta quando clicas no botão verde 
    def ao_clicar_start(change): 
        global robo_ativo 
        robo_ativo = change['new'] 
    startButton.observe(ao_clicar_start, names='value') 
     
    # 2. Criação do Ecrã 
    ecra_cam = widgets.Image(format='jpeg', width=640, height=480) 
     
    # 3. Montagem do Layout Simplificado (Só Botões + Câmara) 
    botoes = widgets.HBox([startButton, stopButton]) 
    dashboard = widgets.VBox([botoes, ecra_cam]) 
     
    display(dashboard) 
     
    # 4. Arranque do Sistema 
    inicializar_hardware() 
    # passagem widget imagem 
    thread = threading.Thread(target=iniciar_visao, args=(ecra_cam,)) 
    thread.start()
