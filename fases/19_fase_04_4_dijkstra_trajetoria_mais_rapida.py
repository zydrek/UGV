#fase 4.4 - Integração de Algoritmo de Dijkstra para Cálculo de Trajetória Mais 
Rápida. Adição da possibilidade de fazer o caminho reverso 
# Mantém otimização de corredor e sistema de Active Gaze (Fase 4.4.1) 
# Melhoramento de seleção prioritária: cruzamento - destino 
import cv2 
import time 
from base_ctrl import BaseController 
import threading 
import ipywidgets as widgets 
from IPython.display import display 
import heapq # Biblioteca nativa Linux/Python para manipulação de Filas de 
Prioridade (Heaps) 
 
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
 
# 
============================================================
======== 
# MAPA TOPOLÓGICO COM PESOS (DISTÂNCIAS EM METROS) 
# Formato: { ID_Origem: { ID_Destino: ( "Direção_Física", Distância_Metros ) } } 
# 
============================================================
======== 
mapa_navegacao = { 
    1: {10: ("frente", 1.0)}, # A Base liga ao cruzamento 10 
    10: { 
        1: ("tras", 1.0),     # O cruzamento 10 volta para a Base 
        11: ("frente", 2.0), 
        22: ("direita", 1.5) 
    }, 
    11: { 
        10: ("tras", 2.0), 
        20: ("esquerda", 1.0), 
        21: ("frente", 3.0) 
    }, 
    # Nós de Destino (Fim de linha) adicionados ao grafo para cálculo 
bidirecional 
    20: {11: ("tras", 1.0)}, 
    21: {11: ("tras", 3.0)}, 
    22: {10: ("tras", 1.5)} 
} 
 
# Variáveis de Missão (GPS) 
no_inicial = 1         # Ponto de partida do robô (pode ser atualizado 
dinamicamente) 
destino_final = 20      # O objetivo do robô 
rota_calculada = []     # Lista que vai guardar o trajeto (ex: [10, 11, 20]) 
indice_alvo = 1         # NOVO: O "dedo no mapa" que aponta para o próximo 
passo obrigatório 
 
# Limiares de Visão 
LIMIAR_CHAO = 6000      # Área a partir da qual o robô está prestes a perder 
visão do ArUco cruzamento (min 5340) 
LIMIAR_DESTINO = 7000   # Área maior que a do cruzamento, para ele parar 
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
# CORAÇÃO DO GPS: ALGORITMO DE DIJKSTRA 
# 
============================================================
======== 
def calcular_melhor_rota(mapa, inicio, fim): 
    """ 
    Processa o grafo topológico e encontra o trajeto com a menor soma de 
distância física. 
    Inclui logs de terminal detalhados para visualização matemática na 
apresentação. 
    """ 
    
print(f"\n======================================================
=") 
    print(f"[GPS CÉREBRO] A iniciar Algoritmo Dijkstra...") 
    print(f"[GPS CÉREBRO] Missão: Origem (Nó {inicio}) -> Destino (Nó {fim})") 
    
print(f"======================================================="
) 
     
    # 1. Inicializa tabela de custos com infinito para todos os nós 
    distancias = {no: float('inf') for no in mapa} 
    distancias[inicio] = 0 # O custo para o ponto de partida é zero 
     
    # 2. Dicionário de rastreio para reconstruir o caminho ("migalhas de pão") 
    caminhos_anteriores = {no: None for no in mapa} 
     
    # 3. Cria a Fila de Prioridade (Heap estruturado em C de baixo nível pelo 
Python) 
    # Guarda tuplos no formato: (custo_acumulado, no_atual) 
    fila_prioridade = [(0, inicio)] 
     
    while fila_prioridade: 
        # Extrai o nó que tem a menor distância acumulada até ao momento 
        distancia_atual, no_atual = heapq.heappop(fila_prioridade) 
 
        print(f"[DIJKSTRA] A expandir Nó {no_atual} (Distância acumulada: 
{distancia_atual}m)") 
        # Otimização: Se chegámos ao destino, interrompe imediatamente a 
exploração 
        if no_atual == fim: 
            print(f"[DIJKSTRA] -> Destino {fim} alcançado na simulação 
matemática!") 
            break 
             
        # Se encontramos um caminho mais longo do que um já processado, 
ignora 
        if distancia_atual > distancias[no_atual]: 
            continue 
             
        # Explora os corredores vizinhos do nó atual 
        for vizinho, (direcao, custo) in mapa.get(no_atual, {}).items(): 
            distancia_calculada = distancia_atual + custo 
            print(f"   -> A testar corredor para Nó {vizinho} ({direcao}): +{custo}m 
(Total estimado: {distancia_calculada}m)") 
             
            # Se este novo corredor for mais rápido do que o registado 
anteriormente: 
            if distancia_calculada < distancias.get(vizinho, float('inf')): 
                print(f"      [!] NOVO CAMINHO MAIS RÁPIDO para Nó {vizinho} 
guardado na memória!") 
                distancias[vizinho] = distancia_calculada 
                caminhos_anteriores[vizinho] = no_atual 
                # Alimenta a fila com a nova rota promissora 
                heapq.heappush(fila_prioridade, (distancia_calculada, vizinho)) 
                 
    # 4. Reconstrução da rota de trás para a frente (do destino até ao início) 
    rota = [] 
    no_passo = fim 
    while no_passo is not None: 
        rota.insert(0, no_passo) # Insere sempre no início da lista para inverter a 
ordem 
        no_passo = caminhos_anteriores.get(no_passo) 
    
print(f"======================================================="
) 
    print(f"[GPS CÉREBRO] Cálculo Concluído!") 
    print(f"[GPS CÉREBRO] Rota a seguir: {rota}") 
    print(f"[GPS CÉREBRO] Distância Total: {distancias[fim]} metros") 
    
print(f"=======================================================\
n")     
    return rota, distancias[fim] 
 
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
        # Gestão de inércia para cruzamentos (mantém-se) 
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
    # Descobre qual o próximo passo obrigatório no mapa 
    if indice_alvo < len(rota_calculada): 
        id_esperado = rota_calculada[indice_alvo] 
    else: 
        id_esperado = destino_final 
 
     
 
    # 
============================================================
======== 
    # COMPORTAMENTO 1: DETEÇÃO DE DESTINOS (IDs 1, 20-29) - 
PRIORIDADE TOTAL 
    # 
============================================================
======== 
    # foca se no destino se ele for o passo exato em que estamos no mapa 
    destinos_na_vista = [id_lido[0] for id_lido in ids if (id_lido[0] == destino_final) 
and (id_lido[0] == id_esperado)] 
     
    if len(destinos_na_vista) > 0 and bloqueio_cruzamento == 0: 
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
            # Se for a Base (ID 1), usamos um limiar de área menor porque o robô 
já começa perto 
            if marcador_destino['id'] == 1 and maior_area_dest >= 3000: 
                return "CHEGOU_DESTINO", marcador_destino 
             
            # Se for uma sala (20-29), avalia se está longe ou se já chegou perto da 
parede 
            elif maior_area_dest < LIMIAR_DESTINO: 
                return "APROXIMAR_DESTINO", marcador_destino 
            else: 
                return "CHEGOU_DESTINO", marcador_destino 
 
    # 
============================================================
======== 
    # COMPORTAMENTO 2: DETEÇÃO DE CRUZAMENTOS (IDs 10 a 19) 
    # 
============================================================
======== 
    if bloqueio_cruzamento == 0: 
        #foca-se no cruzamento se ele for o passo exato em que estamos no 
mapa 
        cruzamentos_na_vista = [id_lido[0] for id_lido in ids if (10 <= id_lido[0] <= 
19) and (id_lido[0] == id_esperado)] 
         
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
 
    # 
============================================================
======== 
    # COMPORTAMENTO 3: SEGUIR CORREDOR (IDs 0) 
    # 
============================================================
======== 
    if 0 in [id_lido[0] for id_lido in ids]: 
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
 
    return "CEGUEIRA", None 
 
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
    global destino_final, rota_calculada, no_inicial, robo_ativo, indice_alvo 
    id_atual = marcador_dest['id'] 
     
    base.base_speed_ctrl(0, 0) 
    base.gimbal_ctrl(0, 0, 0, 0) 
     
    # CORREÇÃO AQUI: Só desliga na Base se o objetivo final for mesmo a 
Base (ID 1) 
    if id_atual == 1 and destino_final == 1: 
        # CONDIÇÃO 1: Chegou a Casa vindo do circuito 
        robo_ativo = False 
        return "[SUCESSO FINAL] Robô regressou à Base. Sistema Desligado." 
         
    elif id_atual == 1 and destino_final != 1: 
        # Ignora a paragem porque estamos apenas a arrancar da base 
        return "[GPS] A ignorar ID 1 de arranque. Destino final é outra sala!" 
         
    else: 
        # CONDIÇÃO 2: Chegou ao destino intermédio (Ex: 22, 20...) 
        print(f"[MISSÃO] Objetivo {id_atual} atingido. A iniciar regresso à Base em 
5 segundos...") 
        time.sleep(5)  
         
        # O destino final passa a ser a Casa (ID 1) 
        destino_final = 1 
        no_inicial = id_atual  
         
        # Recalcula a rota com o Dijkstra  
        rota_calculada, custo = calcular_melhor_rota(mapa_navegacao, 
no_inicial, destino_final) 
        # NOVO: Reseta o rastreador de passos para a nova viagem de regresso! 
        indice_alvo = 1 
        print(f"[GPS] Nova Rota de Regresso: {rota_calculada}") 
         
        # Faz a inversão ativa inteligente para dar meia volta e sair da sala 
        inversao_marcha_ativa()  
         
        return "[MISSÃO] A voltar para casa!" 
 
def executar_manobra(marcador_cruz): 
    global bloqueio_cruzamento, cruzamento_focado, rota_calculada, 
indice_alvo # <-- Adiciona o indice_alvo aqui 
    id_atual = marcador_cruz['id'] 
     
    base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
    time.sleep(1.5)  
     
    base.base_speed_ctrl(0, 0)  
    time.sleep(0.5) 
     
    # ---------------------------------------------------------------- 
    # NOVA TOMADA DE DECISÃO INTELIGENTE BASEADA NA ROTA DO 
DIJKSTRA 
    # ---------------------------------------------------------------- 
    acao_mapa = "frente" # Fallback padrão de segurança 
     
    if id_atual in rota_calculada: 
        indice_atual = rota_calculada.index(id_atual) 
        # Verifica se ainda existe um "próximo nó" na nossa lista de rota 
        if indice_atual + 1 < len(rota_calculada): 
            proximo_no = rota_calculada[indice_atual + 1] 
            # Extrai do mapa a direção correspondente para ir do nó atual para o 
próximo 
            dados_aresta = mapa_navegacao.get(id_atual, {}).get(proximo_no, 
("frente", 0)) 
            acao_mapa = dados_aresta[0] # Pega no elemento de texto 
("esquerda", "direita", etc) 
            print(f"[GPS] No cruzamento {id_atual}, rumo ao nó {proximo_no}. 
Comando: {acao_mapa.upper()}") 
     
    # Configuração de Gaze  
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
 
    # NOVO: Curva feita! Avança o dedo no mapa para o próximo objetivo. 
    indice_alvo += 1 
     
    return f"[CRUZAMENTO {id_atual}] Manobra concluída: 
{acao_mapa.upper()}! (Próximo Alvo: {rota_calculada[indice_alvo] if indice_alvo 
< len(rota_calculada) else destino_final})" 
 
def inversao_marcha_ativa(): 
    """ 
    Roda o robô 180º dividindo o movimento em duas fases de 90º com Active 
Gaze. 
    """ 
    global bloqueio_cruzamento, manobra_em_curso 
     
    print("[MISSÃO] Inversão de marcha ativa iniciada...") 
    # Tranca as leituras visuais durante a manobra 
    bloqueio_cruzamento = 30  
    manobra_em_curso = True 
 
    ang_gimbal = 90       # Câmara olha 90º à direita 
    vel_z = -1.0          # Base roda à direita 
    passos = 10 
    tempo_por_passo = 0.2 
 
    # Ciclo que corre duas vezes (Fase 1 e Fase 2) 
    for fase in [1, 2]: 
        base.gimbal_ctrl(ang_gimbal, 0, 0, 0)  
        time.sleep(0.5) 
         
        for i in range(passos): 
            angulo_atual = ang_gimbal - (ang_gimbal / passos) * (i + 1) 
            base.gimbal_ctrl(int(angulo_atual), 0, 0, 0) 
            base.base_json_ctrl({"T":13, "X":0.0, "Z": vel_z}) 
            time.sleep(tempo_por_passo) 
             
    # --- FIM DA CURVA --- 
    base.base_speed_ctrl(0, 0) 
    base.gimbal_ctrl(0, 0, 0, 0) 
    time.sleep(0.5) # Dá meio segundo para estabilizar a imagem física 
     
    # Liberta o robô para a marcha autónoma 
    manobra_em_curso = False  
    bloqueio_cruzamento = 10 # Dá 10 frames de tolerância para ele largar o ID 
1 
    print("[MISSÃO] Inversão concluída! Cérebro libertado para seguir corredor.") 
     
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
     
    # Prepara o GPS antes do arranque: calcula a rota ótima usando o Dijkstra 
    # Retorna uma lista de nós e o custo total em metros 
    rota_calculada, custo_total = calcular_melhor_rota(mapa_navegacao, 
no_inicial, destino_final) 
    print(f"[GPS] Rota Ótima Calculada pelo Dijkstra: {rota_calculada}") 
    print(f"[GPS] Distância Total estimada até ao destino: {custo_total} 
metros.\n") 
    def arrancar_base_thread(): 
        """Função auxiliar para correr a inversão numa Thread isolada.""" 
        print("[SISTEMA] A lançar Inversão de Marcha em Thread...") 
        inversao_marcha_ativa() 
 
    def ao_clicar_start(change): 
        global robo_ativo 
        if change['new'] == True: 
            # Põe o robô ativo (liga a câmara e o loop), mas a manobra corre em 
paralelo 
            robo_ativo = True 
            threading.Thread(target=arrancar_base_thread).start() 
        else: 
            robo_ativo = False 
         
    startButton.observe(ao_clicar_start, names='value') 
     
    ecra_cam = widgets.Image(format='jpeg', width=640, height=480) 
    botoes = widgets.HBox([startButton, stopButton]) 
    dashboard = widgets.VBox([botoes, ecra_cam]) 
     
    display(dashboard) 
    print(inicializar_hardware()) 
     
    thread = threading.Thread(target=iniciar_visao, args=(ecra_cam,)) 
    thread.start()
