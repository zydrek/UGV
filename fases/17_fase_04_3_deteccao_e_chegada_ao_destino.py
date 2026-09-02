#fase 4.3 - Adição de funções de deteção destino, chegada destino. Paragem 
do robo quando chega ao destino 
# otimização da deteção de corredor 
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
manobra_em_curso = False #quando o robô decide que tem de virar, ele lança 
a manobra para uma linha de montagem secundária (uma thread)  
#e a câmara continua a ler frames no ecrã. 
 
# Mapa Topológico 
destino_atual = 22 
mapa_navegacao = { 
    10: {22: "direita"}, 
    11: {20: "esquerda", 21: "frente"} 
} 
LIMIAR_CHAO = 5550      # Área a partir da qual o robô está prestes a perder 
visão do ArUco cruzamento 
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
# Faz apenas a avaliação do ambiente. Conforme a área do ArUco de destino e 
as condições do corredor,  
# ela cospe uma String exata (ex: "APROXIMAR_CRUZAMENTO") e o 
dicionário de dados relevante (os cantos e áreas) necessário para essa 
manobra. 
# 
============================================================
======== 
def analisar_imagem(ids, cantos): 
    global bloqueio_cruzamento, cruzamento_focado, 
ultimo_marcador_cruzamento, inercia_cruz 
     
    # 1. Tratamento de Cegueira Imediata e Memória de Cruzamento 
    if ids is None: 
        if cruzamento_focado: 
            inercia_cruz += 1  
            if inercia_cruz < 15:  
                # Continua a reportar o cruzamento durante uns frames se o perder 
                if ultimo_marcador_cruzamento['area'] < LIMIAR_CHAO: 
                    return "APROXIMAR_CRUZAMENTO", 
ultimo_marcador_cruzamento 
                else: 
                    return "EXECUTAR_MANOBRA", ultimo_marcador_cruzamento 
            else: 
                cruzamento_focado = False  
        return "CEGUEIRA", None 
 
    # 2. Deteção de Cruzamentos 
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
                 
                # A grande diferença: A visão já diz à ação qual é a etapa do 
cruzamento 
                if maior_area < LIMIAR_CHAO: 
                    return "APROXIMAR_CRUZAMENTO", 
ultimo_marcador_cruzamento 
                else: 
                    return "EXECUTAR_MANOBRA", ultimo_marcador_cruzamento 
                     
    # 2.5 Deteção de Destino (IDs 20 a 29) 
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
         
        # Filtro de tamanho para evitar falsos positivos longe 
        if maior_area_dest > 300: 
            if maior_area_dest < LIMIAR_DESTINO: 
                return "APROXIMAR_DESTINO", marcador_destino 
            else: 
                return "CHEGOU_DESTINO", marcador_destino 
                 
    # Filtro de Inércia de Ruído no Cruzamento 
    if cruzamento_focado: 
        inercia_cruz += 1 
        if inercia_cruz < 15: 
            if ultimo_marcador_cruzamento['area'] < LIMIAR_CHAO: 
                return "APROXIMAR_CRUZAMENTO", ultimo_marcador_cruzamento 
            else: 
                return "EXECUTAR_MANOBRA", ultimo_marcador_cruzamento 
        else: 
            cruzamento_focado = False 
 
    # 3. Deteção de Corredor (Otimizado) 
    if 0 not in ids: 
        return "CEGUEIRA", None 
 
    marcadores_zero = [] 
    for i in range(len(ids)): 
        if ids[i][0] == 0:   
            pontos = cantos[i][0] 
            area = cv2.contourArea(pontos) 
            centro_x = int(pontos.mean(axis=0)[0]) 
            marcadores_zero.append({"area": area, "x": centro_x}) 
             
    # Ordenar por área para agarrar apenas nos dois MAIORES ArUcos ID 0 
(ignora ruído ao fundo) 
    marcadores_zero = sorted(marcadores_zero, key=lambda d: d['area'], 
reverse=True) 
     
    if len(marcadores_zero) >= 2: 
        m1 = marcadores_zero[0] 
        m2 = marcadores_zero[1] 
         
        # Tolerância de comparação de arucos 0 par (comparação do tamanho de 
um com o outro) 
        TOLERANCIA_PAR = 0.30  
        racio_area = min(m1['area'], m2['area']) / max(m1['area'], m2['area']) 
         
        if racio_area > TOLERANCIA_PAR: 
            # Em vez de olhar para o centro do ecrã, compara o X de ambos para 
saber quem é quem 
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
 
def aproximar_cruzamento(marcador_cruz): # ativa o gimbal_ctrl com pitch -45 
para olhar para o chão e  
    # manda um vetor Z suave) e executar_manobra(dados) (que faz os 
time.sleep e a rotação dura consoante o mapa_navegacao). 
     
    # base.gimbal_ctrl: (yaw, pitch, velocidade, aceleracao) - Pitch -45 baixa a 
câmara 
    base.gimbal_ctrl(0, -45, 0, 0)  
     
    kp_chao = 0.005 
    erro_x = marcador_cruz['x'] - 320 
    Z = -kp_chao * erro_x 
     
    # base.base_json_ctrl: T=13 (Comando de velocidade em malha fechada), 
X=Frente(m/s), Z=Rotação(rad/s) 
    base.base_json_ctrl({"T":13, "X":0.10, "Z": Z}) 
    return f"[CRUZAMENTO {marcador_cruz['id']}] A aproximar... Área: 
{int(marcador_cruz['area'])} | Z: {Z:.3f}" 
 
def aproximar_destino(marcador_dest): 
    # 1. Baixar a câmara para não perder o ArUco de vista ao chegar perto 
    base.gimbal_ctrl(0, 0, 0, 0)  
     
    # 2. Cálculo Proporcional para centrar 
    kp_dest = 0.005 
    erro_x = marcador_dest['x'] - 320 
    Z = -kp_dest * erro_x 
     
    # 3. Clamping de segurança para o eixo Z (evitar esticões) 
    LIMITE_Z = 0.5 
    if Z > LIMITE_Z: Z = LIMITE_Z 
    elif Z < -LIMITE_Z: Z = -LIMITE_Z 
     
    # 4. Envio de movimento (Avança devagar, X=0.10) 
    base.base_json_ctrl({"T":13, "X":0.10, "Z": Z}) 
     
    return f"[DESTINO {marcador_dest['id']}] A aproximar... Área: 
{int(marcador_dest['area'])} | Z: {Z:.3f}" 
 
def chegou_destino(marcador_dest): 
    # 1. Parar os motores imediatamente 
    base.base_speed_ctrl(0, 0) 
     
    # 2. Repor a câmara na horizontal 
    base.gimbal_ctrl(0, 0, 0, 0) 
     
    # código para emissao de som 
     
    return f"[SUCESSO] O robô chegou ao destino {marcador_dest['id']} e 
parou." 
 
def executar_manobra(marcador_cruz): 
    global bloqueio_cruzamento, cruzamento_focado 
    id_atual = marcador_cruz['id'] 
     
    base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
    time.sleep(1.5) # Tempo extra para o robô se posicionar bem no centro 
geométrico 
     
    base.base_speed_ctrl(0, 0) # T=1 (Paragem de motores) 
    time.sleep(0.5) 
     
    base.gimbal_ctrl(0, 0, 0, 0) # Levanta a câmara de volta ao nível dos olhos 
    time.sleep(0.5)  
     
    acao_mapa = mapa_navegacao.get(id_atual, {}).get(destino_atual, "frente") 
    # 
    if acao_mapa == "esquerda": 
        base.base_json_ctrl({"T":13, "X":0.0, "Z": 1}) 
        time.sleep(2) 
    elif acao_mapa == "direita": 
        base.base_json_ctrl({"T":13, "X":0.0, "Z": -1})  
        time.sleep(2) 
    elif acao_mapa == "frente": 
        base.base_json_ctrl({"T":13, "X":0.10, "Z": 0.0}) 
        time.sleep(1) 
         
    base.base_speed_ctrl(0, 0) 
     
    bloqueio_cruzamento = 30 # Impede que o robô leia o mesmo cruzamento 
nos próximos 30 frames 
    cruzamento_focado = False  
     
    return f"[CRUZAMENTO {id_atual}] Manobra concluída: 
{acao_mapa.upper()}! (Destino:{destino_atual})" 
     
def arrancar_thread_manobra(marcador_cruz):#serve apenas para ser o "motor 
de arranque" da thread. 
    global manobra_em_curso 
    #manobra_em_curso = True # Avisa o cérebro que está ocupado (já não é 
preciso) 
    executar_manobra(marcador_cruz) # Executa os time.sleep todos à vontade 
    manobra_em_curso = False # Quando terminar, avisa que já está livre 
     
def seguir_corredor(dados_pares): 
    #Se o erro derivativo disparar por causa de um mau frame da câmara, o 
comando cortará 
    #o pico e enviará apenas o limite estipulado no comando base_json_ctrl(..., 
"Z": correcao_z), evitando que o robô 
    #faça rotações bruscas 
    global erro_i, erro_anterior, c1  
     
    kp, ki, kd = 0.005, 0.0001, 0.05 
    esq, dir = dados_pares['esq'], dados_pares['dir'] 
     
    ponto_medio = (esq['x'] + dir['x']) / 2 
    erro_x = ponto_medio - 320 
     
    erro_i += erro_x  
    erro_d = erro_x - erro_anterior 
     
    # Anti-windup (Impede a saturação da memória integral) 
    if erro_i > 1000: erro_i = 1000 
    elif erro_i < -1000: erro_i = -1000 
     
    correcao_z = -(kp * erro_x) - (ki * erro_i) - (kd * erro_d) 
     
    # CLAMPING DO SINAL (Resolve o problema do "esticão" no motor) 
    LIMITE_Z = 0.8 
    if correcao_z > LIMITE_Z: 
        correcao_z = LIMITE_Z 
    elif correcao_z < -LIMITE_Z: 
        correcao_z = -LIMITE_Z 
     
    base.base_json_ctrl({"T":13, "X":0.15, "Z": correcao_z}) 
     
    erro_anterior = erro_x 
    c1 = 0 # Reinicia a inércia de cegueira 
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
        base.base_speed_ctrl(0, 0) # Corta energia aos motores 
instantaneamente 
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
            if not manobra_em_curso: # 
                estado_atual, dados_estado = analisar_imagem(ids, cantos) 
                 
                match estado_atual: 
                    case "APROXIMAR_CRUZAMENTO": 
                        print(aproximar_cruzamento(dados_estado)) 
                         
                    case "EXECUTAR_MANOBRA": 
                    # TRANCA A PORTA AQUI, antes sequer da thread nascer! 
                        manobra_em_curso = True  
                        print(f"[A INICIAR THREAD] Curva no Cruzamento 
{dados_estado['id']}") 
                        threading.Thread(target=arrancar_thread_manobra, 
args=(dados_estado,)).start() 
                         
                    case "APROXIMAR_DESTINO": 
                        print(aproximar_destino(dados_estado)) 
                     
                    case "CHEGOU_DESTINO": 
                        print(chegou_destino(dados_estado)) 
                        robo_ativo = False # Chega ao destino, robo pára 
                    case "SEGUIR_CORREDOR": 
                        print(seguir_corredor(dados_estado)) 
                     
                    case "PROCURAR_PAR": 
                        print(procurar_pares()) 
                     
                    case "CEGUEIRA": 
                        print(lidar_cegueira()) 
            else: 
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
