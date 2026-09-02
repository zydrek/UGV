# UGV - Veículo Guiado Autonomamente

Projeto de desenvolvimento incremental de um veículo guiado autonomamente (UGV), com controlo de motores, visão computacional e navegação baseada em marcadores ArUco.

Cada ficheiro representa uma etapa concreta da evolução do sistema. As fases iniciais permitem validar componentes isolados e as últimas combinam esses componentes num fluxo de navegação mais completo.

## Estrutura do projeto

- fases/: implementações organizadas por fase e subfase.
- docs/roteiro-de-fases.md: visão geral do objetivo de cada implementação.
- requirements.txt: dependências Python usadas nos exemplos.

## Começar

Crie e ative um ambiente virtual e instale as dependências:

    python -m venv .venv
    .\.venv\Scripts\Activate.ps1
    pip install -r requirements.txt

Os programas que controlam o robô necessitam do módulo base_ctrl.py e de uma ligação configurada ao respetivo hardware. Confirme a porta série, o acesso à câmara e os limites de velocidade antes de executar qualquer fase num robô físico.

## Como explorar as fases

Comece em 01_fase_01_controle_motores.py para validar a comunicação e o controlo base. Em seguida, avance pelas fases de visão e deteção de ArUcos. As fases de navegação e máquina de estados pressupõem que os testes anteriores já funcionam no ambiente do robô.

Consulte o roteiro das fases em docs/roteiro-de-fases.md para escolher a implementação mais adequada ao componente que pretende testar.

## Segurança

Teste primeiro com as rodas suspensas ou em velocidade reduzida. Mantenha uma forma de interromper a alimentação do robô durante qualquer teste de movimento autónomo.
