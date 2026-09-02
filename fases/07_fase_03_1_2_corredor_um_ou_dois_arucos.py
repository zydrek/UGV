#fase 3.1.2 
 
            marcadores_validos = sorted(marcadores_validos, key=lambda d: 
d['area'], reverse=True) 
            alvos = marcadores_validos[:2]  
             
            # 
======================================================== 
            LIMIAR_PERIGO = 25000  
            TOLERANCIA_X = 20      
            # 
======================================================== 
             
            # --- CENÁRIO A: Vemos dois marcadores próximos --- 
            if len(alvos) == 2: 
                alvos_x = sorted(alvos, key=lambda d: d['x']) 
                esq = alvos_x[0] 
                dir = alvos_x[1] 
                 
                # Proteção Crossover: Um de cada lado 
                if esq['x'] < 320 and dir['x'] >= 320: 
                     
                    ponto_medio = (esq['x'] + dir['x']) / 2 
                    erro_x = ponto_medio - 320 
                     
                    print(f"Centro: {ponto_medio} | Erro: {erro_x}") 
 
                     
                     
                    if erro_x < -TOLERANCIA_X: 
                        print("-> Corredor à Esquerda. A virar à Esquerda.") 
                        base.base_json_ctrl({"T":13, "X":0.0, "Z": 0.2}) # VOLANTE 
CORRIGIDO 
                    elif erro_x > TOLERANCIA_X: 
                        print("-> Corredor à Direita. A virar à Direita.") 
                        base.base_json_ctrl({"T":13, "X":0.0, "Z": -0.2}) # VOLANTE 
CORRIGIDO 
                    else: 
                        print("-> Centrado! A avançar a direito.") 
                        base.base_json_ctrl({"T":13, "X":0.15, "Z": 0.0}) 
                         
                # Crossover: Os dois marcadores estão do MESMO lado! 
                else: 
                    if esq['x'] > 320: 
                        print("-> AVISO: Perdemos parede Esquerda! A virar à Direita 
para procurar.") 
                        base.base_json_ctrl({"T":13, "X":0.0, "Z": -0.25}) 
                    else: 
                        print("-> AVISO: Perdemos parede Direita! A virar à Esquerda 
para procurar.") 
                        base.base_json_ctrl({"T":13, "X":0.0, "Z": 0.25}) 
 
            # --- CENÁRIO B: Só vemos UM marcador --- 
            elif len(alvos) == 1: 
                unico = alvos[0] 
                print(f"[Apenas 1 Visível] Área: {int(unico['area'])} | X: {unico['x']}") 
                 
                if unico['area'] > LIMIAR_PERIGO: 
                    if unico['x'] < 320: 
                        print("!! PERIGO: Fuga p/ Direita !!") 
                        base.base_json_ctrl({"T":13, "X":0.0, "Z": -0.4}) 
                    else: 
                        print("!! PERIGO: Fuga p/ Esquerda !!") 
                        base.base_json_ctrl({"T":13, "X":0.0, "Z": 0.4}) 
                else: 
                    # Se só vê o da direita, vira ligeiramente à direita para não bater na 
cadeira 
                    if unico['x'] > 320: 
                        print("-> A seguir parede Direita. A corrigir centro...") 
                        base.base_json_ctrl({"T":13, "X":0.10, "Z": -0.15}) 
                    # Se só vê o da esquerda, vira ligeiramente à esquerda 
                    else: 
                        print("-> A seguir parede Esquerda. A corrigir centro...") 
                        base.base_json_ctrl({"T":13, "X":0.10, "Z": 0.15}) 
        #-------------------------------------------------------------------------Fim da fase 3.1.2-
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
