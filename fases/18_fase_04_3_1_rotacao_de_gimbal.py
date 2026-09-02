#fase 4.3.1 - Melhoria de cruzamento na tomada de decisão: rotação de gimbal 
mediante esq ou dir 
 
import cv2 
import time 
from base_ctrl import BaseController 
import threading 
import ipywidgets as widgets 
from IPython.display import display 
 
# 
============================================================
======== 
# 1. VARIÁVEIS GLOBAIS E INICIALIZAÇÃO 
# 
============================================================
======== 
base = None 
detetor = None 
robo_ativo = False 
 
# Variáveis de Controlo e Memória 
c1 = 0                  # Contador de inércia para a cegueira 
erro_i = 0              # Erro integral do PID 
erro_anterior = 0       # Erro derivativo do PID 
 
# Variáveis de Cruzamento 
bloqueio_cruzamento = 0 # Tempo de espera (cooldown) após curvar 
cruzamento_focado = False  
ultimo_marcador_cruzamento = None 
inercia_cruz = 0 
manobra_em_curso = False  
 
# Mapa Topológico 
destino_atual = 20 
mapa_navegacao = { 
    10: {22: "direita"}, 
    11: {20: "esquerda", 21: "frente"} 
} 
LIMIAR_CHAO = 6000      # Área a partir da qual o robô está prestes a perder 
visão do ArUco cruzamento (min 5340) 
LIMIAR_DESTINO = 8000 # Área maior que a do cruzamento, para ele parar 
bem perto da parede/sinal 
 
def is_raspberry_pi5(): 
    with open('/proc/cpuinfo', 'r') as file: 
        for line in file: 
            if 'Model' in line: 
                return 'Raspberry Pi 5' in line 
    return False 
 
def inicializar_hardware(): 
    global base, detetor 
    output_str = "[SETUP] A iniciar componentes de hardware...\n" 
     
    if is_raspberry_pi5(): 
        base = BaseController('/dev/ttyAMA0', 115200) 
    else: 
        base = BaseController('/dev/serial0', 115200) 
         
    output_str += "[SETUP] Comunicação Serial estabelecida.\n" 
    dicionario_aruco = 
cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250) 
    parametros_aruco = cv2.aruco.DetectorParameters() 
    detetor = cv2.aruco.ArucoDetector(dicionario_aruco, parametros_aruco) 
    output_str += "[SETUP] Robô pronto a operar!\n" 
    return output_str 
 
# 
============================================================
======== 
# 2. PERCEÇÃO: CLASSIFICADOR DE ESTADOS (O que o robô vê) 
# 
============================================================
======== 
def analisar_imagem(ids, cantos): 
    global bloqueio_cruzamento, cruzamento_focado, 
ultimo_marcador_cruzamento, inercia_cruz 
     
    if ids is None: 
        if cruzamento_focado: 
            inercia_cruz += 1  
            if inercia_cruz < 15:  
                if ultimo_marcador_cruzamento['area'] < LIMIAR_CHAO: 
                    return "APROXIMAR_CRUZAMENTO", 
ultimo_marcador_cruzamento 
                else: 
                    return "EXECUTAR_MANOBRA", ultimo_marcador_cruzamento 
            else: 
                cruzamento_focado = False  
        return "CEGUEIRA", None 
 
    if bloqueio_cruzamento == 0: 
        cruzamentos_na_vista = [id_lido[0] for id_lido in ids if id_lido[0] in 
mapa_navegacao] 
         
        if len(cruzamentos_na_vista) > 0: 
            maior_area = 0 
            marcador_mais_proximo = None 
             
            for id_cruz in cruzamentos_na_vista: 
                indice = list(ids).index(id_cruz) 
                pontos = cantos[indice][0] 
                area = cv2.contourArea(pontos) 
                centro_x = int(pontos.mean(axis=0)[0]) 
                 
                if area > maior_area: 
                    maior_area = area 
                    marcador_mais_proximo = {"id": id_cruz, "area": area, "x": 
centro_x} 
             
            if maior_area > 300: 
                cruzamento_focado = True  
                inercia_cruz = 0          
                ultimo_marcador_cruzamento = marcador_mais_proximo  
                 
                if maior_area < LIMIAR_CHAO: 
                    return "APROXIMAR_CRUZAMENTO", 
ultimo_marcador_cruzamento 
                else: 
                    return "EXECUTAR_MANOBRA", ultimo_marcador_cruzamento 
                     
    destinos_na_vista = [id_lido[0] for id_lido in ids if 20 <= id_lido[0] <= 29] 
     
    if len(destinos_na_vista) > 0: 
        maior_area_dest = 0 
        marcador_destino = None 
         
        for id_dest in destinos_na_vista: 
            indice = list(ids).index(id_dest) 
            pontos = cantos[indice][0] 
            area = cv2.contourArea(pontos) 
            centro_x = int(pontos.mean(axis=0)[0]) 
             
            if area > maior_area_dest: 
                maior_area_dest = area 
                marcador_destino = {"id": id_dest, "area": area, "x": centro_x} 
         
        if maior_area_dest > 300: 
            if maior_area_dest < LIMIAR_DESTINO: 
                return "APROXIMAR_DESTINO", marcador_destino 
            else: 
                return "CHEGOU_DESTINO", marcador_destino 
                 
    if cruzamento_focado: 
        inercia_cruz += 1 
        if inercia_cruz < 15: 
            if ultimo_marcador_cruzamento['area'] < LIMIAR_CHAO: 
                return "APROXIMAR_CRUZAMENTO", ultimo_marcador_cruzamento 
            else: 
                return "EXECUTAR_MANOBRA", ultimo_marcador_cruzamento 
        else: 
            cruzamento_focado = False 
 
    if 0 not in ids: 
        return "CEGUEIRA", None 
 
    marcadores_zero = [] 
    for i in range(len(ids)): 
        if ids[i][0] == 0:   
            pontos = cantos[i][0] 
            area = cv2.contourArea(pontos) 
            centro_x = int(pontos.mean(axis=0)[0]) 
            marcadores_zero.append({"area": area, "x": centro_x}) 
             
    marcadores_zero = sorted(marcadores_zero, key=lambda d: d['area'], 
reverse=True) 
     
    if len(marcadores_zero) >= 2: 
        m1 = marcadores_zero[0] 
        m2 = marcadores_zero[1] 
         
        TOLERANCIA_PAR = 0.30  
        racio_area = min(m1['area'], m2['area']) / max(m1['area'], m2['area']) 
         
        if racio_area > TOLERANCIA_PAR: 
            esq = m1 if m1['x'] < m2['x'] else m2 
            dir = m1 if m1['x'] > m2['x'] else m2 
            return "SEGUIR_CORREDOR", {"esq": esq, "dir": dir}  
                 
    return "PROCURAR_PAR", None 
 
# 
============================================================
======== 
# 3. AÇÃO: FUNÇÕES DE NAVEGAÇÃO E CONTROLO MOTRIZ 
# 
============================================================
======== 
 
def aproximar_cruzamento(marcador_cruz):  
    base.gimbal_ctrl(0, -45, 0, 0)  
     
    kp_chao = 0.005 
    erro_x = marcador_cruz['x'] - 320 
    Z = -kp_chao * erro_x 
     
    base.base_json_ctrl({"T":13, "X":0.10, "Z": Z}) 
    return f"[CRUZAMENTO {marcador_cruz['id']}] A aproximar... Área: 
{int(marcador_cruz['area'])} | Z: {Z:.3f}" 
 
def aproximar_destino(marcador_dest): 
    base.gimbal_ctrl(0, 0, 0, 0)  
     
    kp_dest = 0.005 
    erro_x = marcador_dest['x'] - 320 
    Z = -kp_dest * erro_x 
     
    LIMITE_Z = 0.5 
    if Z > LIMITE_Z: Z = LIMITE_Z 
    elif Z < -LIMITE_Z: Z = -LIMITE_Z 
     
    base.base_json_ctrl({"T":13, "X":0.10, "Z": Z}) 
    return f"[DESTINO {marcador_dest['id']}] A aproximar... Área: 
{int(marcador_dest['area'])} | Z: {Z:.3f}" 
 
def chegou_destino(marcador_dest): 
    base.base_speed_ctrl(0, 0) 
    base.gimbal_ctrl(0, 0, 0, 0) 
    return f"[SUCESSO] O robô chegou ao destino {marcador_dest['id']} e 
parou." 
 
def executar_manobra(marcador_cruz): 
    global bloqueio_cruzamento, cruzamento_focado 
    id_atual = marcador_cruz['id'] 
     
    base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
    time.sleep(1.5)  
     
    base.base_speed_ctrl(0, 0)  
    time.sleep(0.5) 
     
    acao_mapa = mapa_navegacao.get(id_atual, {}).get(destino_atual, "frente") 
     
    if acao_mapa == "esquerda": 
        ang_gimbal_inicial = -90    
        vel_rotacao_z = 1.0        
    elif acao_mapa == "direita": 
        ang_gimbal_inicial = 90   
        vel_rotacao_z = -1.0       
    else: 
        ang_gimbal_inicial = 0 
        vel_rotacao_z = 0 
 
    if acao_mapa in ["esquerda", "direita"]: 
        base.gimbal_ctrl(ang_gimbal_inicial, 0, 0, 0)  
        time.sleep(0.5)  
         
        passos = 10             
        tempo_por_passo = 0.2   
         
        for i in range(passos): 
            angulo_atual = ang_gimbal_inicial - (ang_gimbal_inicial / passos) * (i + 
1) 
            base.gimbal_ctrl(int(angulo_atual), 0, 0, 0) 
            base.base_json_ctrl({"T":13, "X":0.0, "Z": vel_rotacao_z}) 
             
            # --- O TEU PRINT DE DEBUG --- 
            print(f"   -> [CURVA] Passo {i+1}/{passos} | Motor Z: {vel_rotacao_z} | 
Gimbal: {int(angulo_atual)}º") 
             
            time.sleep(tempo_por_passo) 
             
    elif acao_mapa == "frente": 
        base.gimbal_ctrl(0, 0, 0, 0)  
        time.sleep(0.5) 
        base.base_json_ctrl({"T":13, "X":0.10, "Z": 0.0}) 
        time.sleep(1) 
         
    base.base_speed_ctrl(0, 0) 
    base.gimbal_ctrl(0, 0, 0, 0)  
     
    bloqueio_cruzamento = 30  
    cruzamento_focado = False  
     
    return f"[CRUZAMENTO {id_atual}] Manobra concluída: 
{acao_mapa.upper()}! (Destino:{destino_atual})" 
     
def arrancar_thread_manobra(marcador_cruz): 
    global manobra_em_curso 
    executar_manobra(marcador_cruz)  
    manobra_em_curso = False  
     
def seguir_corredor(dados_pares): 
    global erro_i, erro_anterior, c1  
     
    kp, ki, kd = 0.005, 0.0001, 0.05 
    esq, dir = dados_pares['esq'], dados_pares['dir'] 
     
    ponto_medio = (esq['x'] + dir['x']) / 2 
    erro_x = ponto_medio - 320 
     
    erro_i += erro_x  
    erro_d = erro_x - erro_anterior 
     
    if erro_i > 1000: erro_i = 1000 
    elif erro_i < -1000: erro_i = -1000 
     
    correcao_z = -(kp * erro_x) - (ki * erro_i) - (kd * erro_d) 
     
    LIMITE_Z = 0.8 
    if correcao_z > LIMITE_Z: 
        correcao_z = LIMITE_Z 
    elif correcao_z < -LIMITE_Z: 
        correcao_z = -LIMITE_Z 
     
    base.base_json_ctrl({"T":13, "X":0.15, "Z": correcao_z}) 
     
    erro_anterior = erro_x 
    c1 = 0  
    base.gimbal_ctrl(0, 0, 0, 0) 
     
    return f"[PARES] Erro: {erro_x} | Z: {correcao_z:.3f}" 
 
def procurar_pares(): 
    global c1 
    base.base_json_ctrl({"T":13, "X":0.10, "Z": 0}) 
    base.gimbal_ctrl(0, 0, 0, 0) 
    c1 = 0 
    return "[ORFÃO] Seguindo em frente até encontrar par." 
 
def lidar_cegueira(): 
    global c1 
    if c1 < 10: 
        base.base_json_ctrl({"T":13, "X":0.10, "Z": 0.0}) 
        c1 += 1 
        return f"[CEGUEIRA] Inércia {c1}/10. A manter frente..." 
    else: 
        base.base_speed_ctrl(0, 0)  
        base.gimbal_ctrl(0, 0, 0, 0) 
        return "[PARAGEM] Segurança ativada por falta de visão." 
 
# 
============================================================
======== 
# 4. CICLO PRINCIPAL (A THREAD DE VISÃO E MÁQUINA DE ESTADOS) 
# 
============================================================
======== 
 
def iniciar_visao(widget_imagem): 
    global robo_ativo, bloqueio_cruzamento, manobra_em_curso 
 
    base.gimbal_ctrl(0, 0, 0, 0) 
    camera = cv2.VideoCapture(-1) 
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
     
    print("Câmara iniciada. À espera que cliques em START...") 
     
    while True: 
        sucesso, frame = camera.read() 
        if not sucesso:  
            print("Erro fatal: Não foi possível ler a câmara.") 
            break 
             
        if bloqueio_cruzamento > 0: 
            bloqueio_cruzamento -= 1 
             
        frame_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        cantos, ids, rejeitados = detetor.detectMarkers(frame_cinza) 
         
        if ids is not None: 
            cv2.aruco.drawDetectedMarkers(frame, cantos, ids, 
borderColor=(0,255,0)) 
             
        _, jpeg = cv2.imencode('.jpeg', frame) 
        widget_imagem.value = jpeg.tobytes() 
 
        # 
========================================================= 
        # O CÉREBRO: MÁQUINA DE ESTADOS 
        # 
========================================================= 
        if robo_ativo: 
            if not manobra_em_curso:  
                estado_atual, dados_estado = analisar_imagem(ids, cantos) 
                 
                match estado_atual: 
                    case "APROXIMAR_CRUZAMENTO": 
                        print(aproximar_cruzamento(dados_estado)) 
                         
                    case "EXECUTAR_MANOBRA": 
                        manobra_em_curso = True  
                        print(f"[A INICIAR THREAD] Curva no Cruzamento 
{dados_estado['id']}") 
                        threading.Thread(target=arrancar_thread_manobra, 
args=(dados_estado,)).start() 
                         
                    case "APROXIMAR_DESTINO": 
                        print(aproximar_destino(dados_estado)) 
                     
                    case "CHEGOU_DESTINO": 
                        print(chegou_destino(dados_estado)) 
                        robo_ativo = False  
                         
                    case "SEGUIR_CORREDOR": 
                        print(seguir_corredor(dados_estado)) 
                     
                    case "PROCURAR_PAR": 
                        print(procurar_pares()) 
                     
                    case "CEGUEIRA": 
                        print(lidar_cegueira()) 
            # O else que estava aqui foi eliminado. 
        else: 
            # Se o robo_ativo for falso (ex: clicaste Stop), desliga os motores. 
            base.base_speed_ctrl(0, 0) 
                 
        time.sleep(0.1) 
 
        if stopButton.value == True: 
            camera.release() 
            print("Câmara desligada. Ciclo terminado.") 
            base.base_speed_ctrl(0, 0) 
            break 
 
# 
============================================================
======== 
# 5. EXECUÇÃO DA INTERFACE DO JUPYTER (DASHBOARD) 
# 
============================================================
======== 
if __name__ == "__main__": 
    startButton = widgets.ToggleButton(value=False, description='Start', 
button_style='success', icon='play') 
    stopButton = widgets.ToggleButton(value=False, description='Stop', 
button_style='danger', icon='square') 
     
    def ao_clicar_start(change): 
        global robo_ativo 
        robo_ativo = change['new'] 
    startButton.observe(ao_clicar_start, names='value') 
     
    ecra_cam = widgets.Image(format='jpeg', width=640, height=480) 
    botoes = widgets.HBox([startButton, stopButton]) 
    dashboard = widgets.VBox([botoes, ecra_cam]) 
     
    display(dashboard) 
    print(inicializar_hardware()) 
     
    thread = threading.Thread(target=iniciar_visao, args=(ecra_cam,)) 
    thread.start()
