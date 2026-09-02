#fase 5 - Limpeza de código da fase 4.4 (Código final) 
#implementação de variaveis dinamicas (aruco inicial e destino).   
#Criação de um menu (escolha de vários destinos de uma vez só ou de cada 
destino a pedido do utilizador) 
#Remoção de strings de direção estaticas e troca para orientação por bussola 
#em mapa_navegação 
 
import cv2 
import time 
import threading 
import heapq 
import ipywidgets as widgets 
from IPython.display import display 
from base_ctrl import BaseController 
 
# 
============================================================
======== 
# 1. CONFIGURAÇÕES E VARIÁVEIS GLOBAIS 
# 
============================================================
======== 
base = None 
detetor = None 
robo_ativo = False 
manobra_em_curso = False  
 
LIMIAR_CHAO = 5000       
LIMIAR_DESTINO = 7000    
c1 = 0                   
erro_i = 0               
erro_anterior = 0        
 
bloqueio_cruzamento = 0 
cruzamento_focado = False  
ultimo_marcador_cruzamento = None 
inercia_cruz = 0 
 
# Variáveis do Menu e Bússola 
no_inicial = 1          
destino_final = 1       
rota_calculada = []      
indice_alvo = 1  
orientacao_atual = 0 # Arranca na Base (1) virado para o corredor a Norte (0º) 
 
# Mapa "A" com Ângulos Absolutos (0=Norte/Frente, 90=Este/Direita, 
180=Sul/Trás, 270=Oeste/Esquerda) 
mapa_navegacao = { 
    1: {10: (0, 1.0)},  
    10: {1: (180, 1.0), 11: (0, 2.0), 22: (90, 1.5)},  
    11: {10: (180, 2.0), 20: (270, 1.0), 21: (0, 3.0)}, 
    20: {11: (90, 1.0)}, 
    21: {11: (180, 3.0)},   
    22: {10: (270, 1.5)}     
} 
 
# 
============================================================
======== 
# 2. SETUP E HARDWARE 
# 
============================================================
======== 
def is_raspberry_pi5(): 
    with open('/proc/cpuinfo', 'r') as file: 
        for line in file: 
            if 'Model' in line: return 'Raspberry Pi 5' in line 
    return False 
 
def inicializar_hardware(): 
    global base, detetor 
    output = "[SETUP] A iniciar componentes de hardware...\n" 
    porta = '/dev/ttyAMA0' if is_raspberry_pi5() else '/dev/serial0' 
    base = BaseController(porta, 115200) 
    output += "[SETUP] Comunicação Serial estabelecida.\n" 
    detetor = cv2.aruco.ArucoDetector( 
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250),  
        cv2.aruco.DetectorParameters() 
    ) 
    output += "[SETUP] Robô pronto a operar!\n" 
    return output 
 
# 
============================================================
======== 
# 3. CÉREBRO E NAVEGAÇÃO (Dijkstra) 
# 
============================================================
======== 
def calcular_melhor_rota(mapa, inicio, fim): 
    print(f"\n[GPS] A iniciar Dijkstra: Nó {inicio} -> Nó {fim}") 
    distancias = {no: float('inf') for no in mapa} 
    distancias[inicio] = 0  
    caminhos = {no: None for no in mapa} 
    fila = [(0, inicio)] 
     
    while fila: 
        dist_atual, atual = heapq.heappop(fila) 
        if atual == fim: break 
        if dist_atual > distancias[atual]: continue 
             
        for vizinho, (direcao, custo) in mapa.get(atual, {}).items(): 
            nova_dist = dist_atual + custo 
            if nova_dist < distancias.get(vizinho, float('inf')): 
                distancias[vizinho] = nova_dist 
                caminhos[vizinho] = atual 
                heapq.heappush(fila, (nova_dist, vizinho)) 
                 
    rota = [] 
    passo = fim 
    while passo is not None: 
        rota.insert(0, passo) 
        passo = caminhos.get(passo) 
         
    print(f"[GPS] Rota Calculada: {rota} | Distância: {distancias[fim]}m\n")     
    return rota, distancias[fim] 
 
# 
============================================================
======== 
# 4. PERCEÇÃO: CLASSIFICADOR DE IMAGEM 
# 
============================================================
======== 
def analisar_imagem(ids, cantos): 
    global bloqueio_cruzamento, cruzamento_focado, 
ultimo_marcador_cruzamento, inercia_cruz 
    ids_lista = [id_lido[0] for id_lido in ids] if ids is not None else [] 
 
    if cruzamento_focado: 
        if ultimo_marcador_cruzamento['id'] not in ids_lista: 
            inercia_cruz += 1  
            if inercia_cruz < 15:  
                if ultimo_marcador_cruzamento['area'] < LIMIAR_CHAO: 
                    return "APROXIMAR_CRUZAMENTO", 
ultimo_marcador_cruzamento 
                else: return "EXECUTAR_MANOBRA", ultimo_marcador_cruzamento 
            else: cruzamento_focado = False  
        else: inercia_cruz = 0  
 
    if not ids_lista: return "CEGUEIRA", None 
         
    id_esperado = rota_calculada[indice_alvo] if indice_alvo < 
len(rota_calculada) else destino_final 
 
    # 1. Destinos (Prioridade Máxima) 
    destinos_vista = [id_lido for id_lido in ids_lista if id_lido == destino_final and 
id_lido == id_esperado] 
    if destinos_vista and bloqueio_cruzamento == 0: 
        maior_area = 0 
        marcador = None 
        for id_dest in destinos_vista: 
            idx = ids_lista.index(id_dest) 
            pts = cantos[idx][0] 
            area = cv2.contourArea(pts) 
            if area > maior_area: 
                maior_area = area 
                marcador = {"id": id_dest, "area": area, "x": int(pts.mean(axis=0)[0])} 
         
        if maior_area > 300: 
            if (marcador['id'] == 1 and maior_area >= 3000) or maior_area >= 
LIMIAR_DESTINO: 
                return "CHEGOU_DESTINO", marcador 
            return "APROXIMAR_DESTINO", marcador 
 
    # 2. Cruzamentos 
    cruzamentos_vista = [id_lido for id_lido in ids_lista if 10 <= id_lido <= 19 and 
id_lido == id_esperado] 
    if cruzamentos_vista and bloqueio_cruzamento == 0: 
        maior_area = 0 
        marcador = None 
        for id_cruz in cruzamentos_vista: 
            idx = ids_lista.index(id_cruz) 
            pts = cantos[idx][0] 
            area = cv2.contourArea(pts) 
            if area > maior_area: 
                maior_area = area 
                marcador = {"id": id_cruz, "area": area, "x": int(pts.mean(axis=0)[0])} 
         
        if maior_area > 300: 
            cruzamento_focado = True  
            inercia_cruz = 0          
            ultimo_marcador_cruzamento = marcador  
            if maior_area < LIMIAR_CHAO: return "APROXIMAR_CRUZAMENTO", 
marcador 
            else: return "EXECUTAR_MANOBRA", marcador 
 
    # 3. Corredor (IDs 0) 
    if 0 in ids_lista: 
        pares = [{"area": cv2.contourArea(cantos[i][0]), "x": 
int(cantos[i][0].mean(axis=0)[0])}  
                 for i in range(len(ids_lista)) if ids_lista[i] == 0] 
        pares = sorted(pares, key=lambda d: d['area'], reverse=True) 
        if len(pares) >= 2: 
            m1, m2 = pares[0], pares[1] 
            if min(m1['area'], m2['area']) / max(m1['area'], m2['area']) > 0.30: 
                esq = m1 if m1['x'] < m2['x'] else m2 
                dir = m1 if m1['x'] > m2['x'] else m2 
                return "SEGUIR_CORREDOR", {"esq": esq, "dir": dir}  
        return "PROCURAR_PAR", None 
 
    return "CEGUEIRA", None 
 
# 
============================================================
======== 
# 5. AÇÃO: MOVIMENTO E MANOBRAS (Com Bússola) 
# 
============================================================
======== 
def executar_manobra(marcador_cruz): 
    global bloqueio_cruzamento, cruzamento_focado, indice_alvo, 
orientacao_atual 
    id_atual = marcador_cruz['id'] 
    base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
    time.sleep(1.5)  
    base.base_speed_ctrl(0, 0)  
    time.sleep(0.5) 
     
    acao_mapa = "frente" 
    if id_atual in rota_calculada and indice_alvo < len(rota_calculada): 
        proximo_no = rota_calculada[indice_alvo + 1] if (indice_alvo + 1) < 
len(rota_calculada) else destino_final 
         
        # A MATEMÁTICA DA BÚSSOLA 
        angulo_destino = mapa_navegacao.get(id_atual, {}).get(proximo_no, 
(orientacao_atual, 0))[0] 
        curva_graus = (angulo_destino - orientacao_atual) % 360 
         
        if curva_graus == 0: acao_mapa = "frente" 
        elif curva_graus == 90: acao_mapa = "direita" 
        elif curva_graus == 270: acao_mapa = "esquerda" 
        else: acao_mapa = "frente" # Fallback 
             
        print(f"[BÚSSOLA] Atual: {orientacao_atual}º | Destino: {angulo_destino}º | 
Curva calculada: {acao_mapa.upper()}") 
         
        # O robô assume a nova orientação mal define a curva! 
        orientacao_atual = angulo_destino 
 
    # Gaze e Curva Motriz 
    ang_gimbal = -90 if acao_mapa == "esquerda" else 90 if acao_mapa == 
"direita" else 0 
    vel_z = 1.0 if acao_mapa == "esquerda" else -1.0 if acao_mapa == "direita" 
else 0 
 
    if acao_mapa in ["esquerda", "direita"]: 
        base.gimbal_ctrl(ang_gimbal, 0, 0, 0)  
        time.sleep(0.5)  
        for i in range(10): 
            angulo = ang_gimbal - (ang_gimbal / 10) * (i + 1) 
            base.gimbal_ctrl(int(angulo), 0, 0, 0) 
            base.base_json_ctrl({"T":13, "X":0.0, "Z": vel_z}) 
            time.sleep(0.2) 
    else: 
        base.gimbal_ctrl(0, 0, 0, 0)  
        time.sleep(0.5) 
        base.base_json_ctrl({"T":13, "X":0.10, "Z": 0.0}) 
        time.sleep(1) 
         
    base.base_speed_ctrl(0, 0) 
    bloqueio_cruzamento = 30  
    cruzamento_focado = False  
    indice_alvo += 1 
    return f"[MANOBRA] Concluída! (Próximo Alvo: {rota_calculada[indice_alvo] 
if indice_alvo < len(rota_calculada) else destino_final})" 
 
def inversao_marcha_ativa(): 
    global bloqueio_cruzamento, manobra_em_curso, orientacao_atual 
    bloqueio_cruzamento = 30  
    manobra_em_curso = True 
    for _ in range(2): 
        base.gimbal_ctrl(90, 0, 0, 0)  
        time.sleep(0.5) 
        for i in range(10): 
            base.gimbal_ctrl(int(90 - (90/10) * (i+1)), 0, 0, 0) 
            base.base_json_ctrl({"T":13, "X":0.0, "Z": -1.0}) 
            time.sleep(0.2) 
             
    # Atualiza a Bússola do robô em 180º! 
    orientacao_atual = (orientacao_atual + 180) % 360 
     
    base.gimbal_ctrl(0, 0, 0, 0) 
    print(f"[MISSÃO] A sair da sala... (Nova Orientação: {orientacao_atual}º)") 
    base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
    time.sleep(1.5)  
    base.base_speed_ctrl(0, 0) 
    time.sleep(0.5)  
    manobra_em_curso = False  
    bloqueio_cruzamento = 10  
 
def chegou_destino(marcador_dest): 
    global no_inicial, robo_ativo 
    id_atual = marcador_dest['id'] 
    base.base_speed_ctrl(0, 0) 
    base.gimbal_ctrl(0, 0, 0, 0) 
     
    print(f"\n[SUCESSO] Missão Concluída! O robô chegou à Sala {id_atual}.") 
    no_inicial = id_atual # A localização atual passa a ser este ID 
     
    # === A TUA SOLUÇÃO: O ROBÔ VIRA-SE SEMPRE 180º PARA O 
CORREDOR === 
    print("[SISTEMA] A executar inversão de preparação em 3 segundos...") 
    time.sleep(3)  
    inversao_marcha_ativa()  
         
    robo_ativo = False 
    print("[STAND-BY] À espera de nova ordem no Menu...\n") 
 
def seguir_corredor(dados): 
    global erro_i, erro_anterior, c1  
    esq, dir = dados['esq'], dados['dir'] 
    erro_x = ((esq['x'] + dir['x']) / 2) - 320 
    erro_i = max(min(erro_i + erro_x, 1000), -1000) 
    z = -(0.005 * erro_x) - (0.0001 * erro_i) - (0.05 * (erro_x - erro_anterior)) 
    z = max(min(z, 0.8), -0.8) 
    base.base_json_ctrl({"T":13, "X":0.15, "Z": z}) 
    erro_anterior = erro_x 
    c1 = 0  
    base.gimbal_ctrl(0, 0, 0, 0) 
    return f"[PARES] Erro: {erro_x:.1f} | Z: {z:.3f}" 
 
def lidar_cegueira(): 
    global c1 
    if c1 < 30: 
        base.base_json_ctrl({"T":13, "X":0.10, "Z": 0.0}) 
        c1 += 1 
        return f"[CEGUEIRA] Inércia {c1}/30..." 
    else: 
        base.base_speed_ctrl(0, 0)  
        return "[PARAGEM] Segurança ativada por falta de visão." 
 
# 
============================================================
======== 
# 6. THREADS DE MANOBRA E LOOP PRINCIPAL 
# 
============================================================
======== 
def arrancar_thread_manobra(marcador_cruz): 
    global manobra_em_curso 
    print(executar_manobra(marcador_cruz)) 
    manobra_em_curso = False  
 
def arrancar_thread_destino(marcador_dest): 
    global manobra_em_curso 
    chegou_destino(marcador_dest)  
    if robo_ativo: manobra_em_curso = False 
 
def iniciar_visao(widget_imagem): 
    global robo_ativo, bloqueio_cruzamento, manobra_em_curso, c1 
    base.gimbal_ctrl(0, 0, 0, 0) 
    camera = cv2.VideoCapture(-1) 
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
    print("Câmara iniciada. Escolhe um Destino e clica em 'Ir'...") 
     
    while True: 
        sucesso, frame = camera.read() 
        if not sucesso: break 
        if bloqueio_cruzamento > 0: bloqueio_cruzamento -= 1 
             
        frame_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        cantos, ids, _ = detetor.detectMarkers(frame_cinza) 
        if ids is not None: cv2.aruco.drawDetectedMarkers(frame, cantos, ids, 
(0,255,0)) 
        widget_imagem.value = cv2.imencode('.jpeg', frame)[1].tobytes() 
 
        if robo_ativo and not manobra_em_curso:  
            estado, dados = analisar_imagem(ids, cantos) 
             
            match estado: 
                case "APROXIMAR_CRUZAMENTO": 
                    z_calc = -0.005 * (dados['x'] - 320) 
                    base.gimbal_ctrl(0, -45, 0, 0)  
                    base.base_json_ctrl({"T":13, "X":0.10, "Z": z_calc}) 
                    print(f"[CRUZAMENTO {dados['id']}] A aproximar... Área: 
{int(dados['area'])} | Z: {z_calc:.3f}") 
                     
                case "EXECUTAR_MANOBRA": 
                    manobra_em_curso = True  
                    threading.Thread(target=arrancar_thread_manobra, 
args=(dados,)).start() 
                     
                case "APROXIMAR_DESTINO": 
                    z_calc = max(min(-0.005 * (dados['x'] - 320), 0.5), -0.5) 
                    base.gimbal_ctrl(0, 0, 0, 0)  
                    base.base_json_ctrl({"T":13, "X":0.10, "Z": z_calc}) 
                    print(f"[DESTINO {dados['id']}] A aproximar... Área: 
{int(dados['area'])} | Z: {z_calc:.3f}") 
                     
                case "CHEGOU_DESTINO": 
                    manobra_em_curso = True  
                    threading.Thread(target=arrancar_thread_destino, 
args=(dados,)).start() 
                     
                case "SEGUIR_CORREDOR": 
                    print(seguir_corredor(dados)) 
                     
                case "PROCURAR_PAR": 
                    c1 = 0 
                    base.base_json_ctrl({"T":13, "X":0.10, "Z": 0}) 
                     
                case "CEGUEIRA": 
                    print(lidar_cegueira()) 
                     
        elif not robo_ativo: 
            base.base_speed_ctrl(0, 0) 
             
        time.sleep(0.1) 
        if stopButton.value == True: 
            camera.release() 
            base.base_speed_ctrl(0, 0) 
            break 
 
# 
============================================================
======== 
# 7. DASHBOARD JUPYTER (MENU INTERATIVO) 
# 
============================================================
======== 
if __name__ == "__main__": 
    menu_destino = widgets.Dropdown(options=[1, 20, 21, 22], 
description='Destino:') 
    btn_ir = widgets.Button(description='Ir!', button_style='success', icon='play') 
    stopButton = widgets.ToggleButton(value=False, description='Emergência', 
button_style='danger', icon='square') 
     
    def ao_clicar_ir(b): 
        global destino_final, rota_calculada, indice_alvo, robo_ativo, 
orientacao_atual 
         
        destino_final = menu_destino.value 
        if no_inicial == destino_final: 
            print("[ERRO] O robô já está nessa sala!") 
            return 
             
        rota_calculada, custo = calcular_melhor_rota(mapa_navegacao, 
no_inicial, destino_final) 
        indice_alvo = 1 
         
        if no_inicial == 1 and not robo_ativo: 
            orientacao_atual = mapa_navegacao[1][rota_calculada[1]][0] 
             
        robo_ativo = True 
        print(f"[START] A iniciar marcha para o ID {destino_final}...") 
         
    btn_ir.on_click(ao_clicar_ir) 
     
    ecra_cam = widgets.Image(format='jpeg', width=640, height=480) 
    painel_controlo = widgets.HBox([menu_destino, btn_ir, stopButton]) 
    display(widgets.VBox([painel_controlo, ecra_cam])) 
     
    print(inicializar_hardware()) 
    threading.Thread(target=iniciar_visao, args=(ecra_cam,)).start()
