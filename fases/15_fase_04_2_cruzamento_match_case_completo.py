#fase 4.2 - adição de cruzamento ao match/case - completo 
#adição de mapa teste de navegação 
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
bloqueio_cruzamento = 0 #Impede leituras repetidas do mesmo cruzamento 
 
# NOVAS VARIÁVEIS DE FOCO 
cruzamento_focado = False  
ultimo_marcador_cruzamento = None # para marcadores cruzamento id10 a 19 
(mais à frente um for dedicado só para isso) 
inercia_cruz = 0 #timer block caso verifique um id cruzamento 
 
# 
============================================================
======== 
# MAPA TOPOLÓGICO (O Cérebro de Navegação) 
# 
============================================================
======== 
# Define para onde o robô quer ir (Destinos possíveis: 20 a 29) 
destino_atual = 21 
 
# Dicionário de Rotas: 
mapa[cruzamento_que_estou_a_pisar][destino_que_quero_ir] = "ação" 
mapa_navegacao = { 
    10: {20: "direita"}, 
    11: {21: "esquerda", 22: "frente"} 
    # Podemos adicionar mais cruzamentosdestinos aqui depois 
} 
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
    global bloqueio_cruzamento, cruzamento_focado, 
ultimo_marcador_cruzamento, inercia_cruz 
     
    # 
============================================================
===== 
    # ETAPA 1: GESTÃO DE DESFOQUE E MEMÓRIA DE CURTO PRAZO 
    # 
============================================================
===== 
    if ids is None: 
        if cruzamento_focado: 
            inercia_cruz += 1  
            if inercia_cruz < 15:  
                return "CRUZAMENTO", [], [], ultimo_marcador_cruzamento 
            else: 
                cruzamento_focado = False  
         
        return "CEGUEIRA", [], [], None 
 
    # 
============================================================
===== 
    # ETAPA 2: DETEÇÃO ATIVA DE MÚLTIPLOS CRUZAMENTOS (A Tua 
Ideia!) 
    # 
============================================================
===== 
    if bloqueio_cruzamento == 0: 
        # Cria uma lista apenas com os IDs que existam nas chaves do teu 
mapa_navegacao! 
        cruzamentos_na_vista = [id_lido[0] for id_lido in ids if id_lido[0] in 
mapa_navegacao] 
         
        if len(cruzamentos_na_vista) > 0: 
            maior_area = 0 
            marcador_mais_proximo = None 
             
            # Vai avaliar todos os cruzamentos válidos para descobrir qual é o mais 
perto 
            for id_cruz in cruzamentos_na_vista: 
                indice = list(ids).index(id_cruz) 
                pontos = cantos[indice][0] 
                area = cv2.contourArea(pontos) 
                centro_x = int(pontos.mean(axis=0)[0]) 
                 
                if area > maior_area: 
                    maior_area = area 
                    marcador_mais_proximo = {"id": id_cruz, "area": area, "x": 
centro_x} 
             
            # Filtro de tamanho (Anti-Fantasmas físicos) 
            if maior_area > 300: 
                cruzamento_focado = True  
                inercia_cruz = 0          
                ultimo_marcador_cruzamento = marcador_mais_proximo  
                return "CRUZAMENTO", [], [], ultimo_marcador_cruzamento 
         
    # 
============================================================
===== 
    # ETAPA 3: FILTRO DE PRIORIDADE (IGNORAR RUÍDO) 
    # 
============================================================
===== 
    if cruzamento_focado: 
        inercia_cruz += 1 
        if inercia_cruz < 15: 
            return "CRUZAMENTO", [], [], ultimo_marcador_cruzamento 
        else: 
            cruzamento_focado = False 
 
    # 
============================================================
===== 
    # ETAPA 4: NAVEGAÇÃO NORMAL DE CORREDOR (ID 0) 
    # 
============================================================
===== 
    if 0 not in ids: 
        return "CEGUEIRA", [], [], None 
 
    esq_lista, dir_lista = [], [] 
     
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
 
 
def navegar_cruzamento(marcador_cruz): 
    global bloqueio_cruzamento, cruzamento_focado, destino_atual, 
mapa_navegacao 
     
    LIMIAR_CHAO = 5550  
    area = marcador_cruz['area'] 
    erro_x = marcador_cruz['x'] - 320 
    id_atual = marcador_cruz['id'] # Lê qual é o cruzamento onde estamos (ex: 
10 ou 11) 
     
    # ETAPA 1: Longe do Cruzamento -> Aproximação Guiada 
    if area < LIMIAR_CHAO: 
        base.gimbal_ctrl(0, -45, 0, 0)  
         
        kp_chao = 0.005 
        Z = -kp_chao * erro_x 
         
        base.base_json_ctrl({"T":13, "X":0.10, "Z": Z}) 
        return f"[CRUZAMENTO {id_atual}] A aproximar... Área: {int(area)} | Z: 
{Z:.3f}" 
         
    # ETAPA 2: Em cima do Cruzamento -> Manobra Centrada 
    else: 
        base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
        time.sleep(1.5) 
         
        base.base_speed_ctrl(0, 0) 
        time.sleep(0.5) 
         
        base.gimbal_ctrl(0, 0, 0, 0) 
        time.sleep(0.5)  
         
        # ------------------------------------------------------------- 
        # MAPA TOPOLÓGICO 
        # Consulta o dicionário: "Estando no ID atual, e querendo ir  
        # para o destino atual, qual é a ação?" 
        # O .get(..., "frente") significa que se houver um erro no mapa, ele vai em 
frente por segurança 
        # ------------------------------------------------------------- 
        acao_mapa = mapa_navegacao.get(id_atual, {}).get(destino_atual, 
"frente") 
         
        if acao_mapa == "esquerda": 
            base.base_json_ctrl({"T":13, "X":0.0, "Z": 2}) 
            time.sleep(2) 
        elif acao_mapa == "direita": 
            base.base_json_ctrl({"T":13, "X":0.0, "Z": -1}) 
            time.sleep(2) 
        elif acao_mapa == "frente": 
            base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
            time.sleep(1) 
             
        base.base_speed_ctrl(0, 0) 
         
        bloqueio_cruzamento = 30  
        cruzamento_focado = False  
         
        return f"[CRUZAMENTO {id_atual}] Manobra concluída: 
{acao_mapa.upper()}! (Destino:{destino_atual})" 
         
def navegar_pares(esq, dir): 
    global erro_i, erro_anterior, c1  
     
    # 1. Definição das constantes PID 
    kp, ki, kd = 0.005, 0.0001, 0.05 
     
    # 2. Cálculo do ponto médio entre os dois marcadores e o erro em relação 
ao centro (320) 
    ponto_medio = (esq['x'] + dir['x']) / 2 
    erro_x = ponto_medio - 320 
     
    # 3. Acumulação do erro integral e cálculo da variação do erro (derivada) 
    erro_i += erro_x  
    erro_d = erro_x - erro_anterior 
     
    # 4. Limitação da acumulação integral (Anti-windup já presente no teu 
código) 
    if erro_i > 1000: erro_i = 1000 
    elif erro_i < -1000: erro_i = -1000 
     
    # 5. Cálculo bruto da correção Z baseada na fórmula PID 
    correcao_z = -(kp * erro_x) - (ki * erro_i) - (kd * erro_d) 
     
    # --------------------------------------------------------- 
    # 6. Saturação (Clamping) do sinal de saída Z 
    # Impede que o robô faça curvas superiores a um limite seguro. 
    # Um valor de Z de 1.0 ou -1.0 já é uma curva bastante fechada. 
    # --------------------------------------------------------- 
    LIMITE_Z = 1.0  
    if correcao_z > LIMITE_Z: 
        correcao_z = LIMITE_Z 
    elif correcao_z < -LIMITE_Z: 
        correcao_z = -LIMITE_Z 
         
    # 7. Envio do comando de movimento para a base 
    base.base_json_ctrl({"T":13, "X":0.15, "Z": correcao_z}) 
     
    # 8. Atualização de variáveis para o próximo ciclo 
    erro_anterior = erro_x 
    c1 = 0 
    base.gimbal_ctrl(0, 0, 0, 0) 
    return f"[PARES] Erro: {erro_x} | Z: {correcao_z:.3f}" 
 
 
def navegar_orfao(todos_marcadores): 
    global c1 
    base.base_json_ctrl({"T":13, "X":0.10, "Z": 0}) 
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
             
        # --------------------------------------------------------- 
        # CRONÓMETRO DE DESBLOQUEIO 
        # --------------------------------------------------------- 
        global bloqueio_cruzamento 
        if bloqueio_cruzamento > 0: 
            bloqueio_cruzamento -= 1 
             
        frame_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        cantos, ids, rejeitados = detetor.detectMarkers(frame_cinza) 
         
        if ids is not None: 
            cv2.aruco.drawDetectedMarkers(frame, cantos, ids, 
borderColor=(0,255,0)) 
             
        # Atualização LIMPA da imagem no Dashboard 
        _, jpeg = cv2.imencode('.jpeg', frame) 
        widget_imagem.value = jpeg.tobytes() 
 
        if robo_ativo: 
            estado_atual, lista_e, lista_d, marcador_cruz = 
classificar_estado_visao(ids, cantos) 
             
            match estado_atual: 
                case "CRUZAMENTO": 
                    print(navegar_cruzamento(marcador_cruz)) 
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
