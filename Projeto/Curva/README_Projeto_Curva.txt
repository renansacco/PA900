Projeto do Controlador de Curva — CME (Controle por Modos Escorregantes)
========================================================================

Resumo
------
O controlador de curva usa uma lei de controle nao-linear baseada em CME
(saturacao suave). Os parametros do CME sao derivados de ganhos lineares
[K_psi, K_r] obtidos por otimizacao numerica sobre a planta nao-linear.

O pipeline completo e:

  1. Projeto_Linear.m   ->  ganhos lineares [K_psi, K_r]
  2. Projeto_CME_Curva.m ->  parametros CME (eps, B, er, ur)
  3. Param_Controller.m  ->  carrega no Simulink (Controlador.Value.Curva.*)
  4. Controller ERT      ->  executa a lei CME no modelo embarcado


1. Projeto dos Ganhos Lineares (Projeto_Linear.m)
--------------------------------------------------

Planta:
  Modelo bicicleta nao-linear com atuador (7 estados), mesma dinamica de
  dinamica_veiculo.m. A velocidade longitudinal e fixa em vx = 2.0 m/s.
  Parametros do veiculo carregados de param_MF6713.mat.

Lei de controle linear:
  omega_m_ref = K_psi * (psi_ref - psi) + K_r * (0 - r)
  saturada em [-15, +15] rad/s.

Otimizacao:
  Usa fminsearch (Nelder-Mead) para minimizar o custo J sobre uma simulacao
  de step response (psi_ref = 10 deg, tsim = 10s):

    J = integral{ t * [ Q_psi*(psi_ref - psi)^2 + Q_r*r^2 ] } dt
      + integral{ R * omega_m_ref^2 } dt

  O fator 't' penaliza erros tardios (settling time).

  O peso R determina a agressividade:
    R = 1/10^2  ->  ganhos menores (mais conservador)
    R = 1/15^2  ->  intermediario
    R = 1/20^2  ->  intermediario
    R = 1/25^2  ->  ganhos maiores (mais agressivo)

  Resultado (7 pares de ganhos, variando R e B):
    Ks(1,:) = [43.10, 52.75]   R=1/10^2, B=0
    Ks(2,:) = [57.23, 61.84]   R=1/15^2, B=-1
    Ks(3,:) = [69.72, 69.26]   R=1/20^2, B=-1
    Ks(4,:) = [81.09, 75.70]   R=1/25^2, B=-1
    Ks(5,:) = [57.23, 61.84]   R=1/15^2, B=+1
    Ks(6,:) = [69.72, 69.26]   R=1/20^2, B=+1
    Ks(7,:) = [81.09, 75.70]   R=1/25^2, B=+1


2. Conversao para Parametros CME (Projeto_CME_Curva.m)
------------------------------------------------------

A lei CME para cada canal (psi e r) e:

  1. Normaliza o erro:    e_norm = (y_ref - y) / er,  clamp em [-1, +1]
  2. Aplica nao-linearidade: se |e_norm| == 1, ut = e_norm
                             senao, ss = e_norm - sign(e_norm)
                                    ut = sign(ss) * (|ss|^(2^-B) - 1)
  3. Mapeia para saida:   u = ((u_max - u_min)/2) * (ut - 1) + u_max

O parametro B controla a forma da curva de saturacao:
  B = 0   ->  saturacao linear (equivale ao ganho linear puro)
  B > 0   ->  curva mais agressiva perto do zero (reage rapido a erros pequenos)
  B < 0   ->  curva mais suave perto do zero

A conversao de ganho linear para parametro CME e:

  er = (ur / K_linear) * 2^(-B)

onde:
  ur  = limite de saturacao (15 rad/s para ambos os canais)
  K_linear = ganho linear otimizado (K_psi ou K_r)

Isso garante que, para erros pequenos (regiao linear da CME), o ganho
efetivo du/dy = ur/er * 2^B = K_linear, ou seja, a CME se comporta
como o controlador linear projetado na vizinhanca do equilibrio.

A saida de Projeto_CME_Curva.m e a matriz CME_Gains (7x8), onde cada
linha contem: [er_psi, B_psi, -ur_psi, ur_psi, er_r, B_r, -ur_r, ur_r]

Essa matriz e salva em Curva_Gains_Final.mat e carregada pelo Controller.


3. Uso no Simulink (Param_Controller.m)
----------------------------------------

Param_Controller.m carrega os parametros de duas formas:

a) Ganho fixo (usado diretamente no modo curva simples):
   Controlador.Value.Curva.eps  = 0.1817   (er de psi)
   Controlador.Value.Curva.Bps  = 1        (B de psi)
   Controlador.Value.Curva.urps = [-15, 15] (saturacao)
   Controlador.Value.Curva.er   = 0.1089   (er de r)
   Controlador.Value.Curva.Br   = 1        (B de r)
   Controlador.Value.Curva.urr  = [-15, 15] (saturacao)

   Estes correspondem a K_linear = [41.28, 68.9] com B=1.

b) Tabela de ganhos (controllerGainsCurve, 7x8):
   Indexada por nivel de agressividade (curveAggressiveness = 1..7).
   Cada linha da tabela e um conjunto CME completo para um par (R, B).


4. Estrutura de Arquivos
------------------------

Projeto_Linear.m      — Script principal: carrega planta, roda fminsearch
objetivo.m            — Funcao custo (simula step + integra custo LQR)
controle.m            — Lei de controle linear (K_psi, K_r com saturacao)
dinamica.m            — Dinamica nao-linear do veiculo (7 estados)
dinamica_MF.m         — Wrapper malha fechada (controle + dinamica)
Projeto_CME_Curva.m   — Converte ganhos lineares em parametros CME


5. Como reprojetar
------------------

Para obter novos ganhos (ex: outro veiculo ou outra agressividade):

  1. Editar Projeto_Linear.m: ajustar param_MF6713.mat e o peso R em objetivo.m
  2. Rodar Projeto_Linear.m — retorna [K_psi, K_r] otimizados
  3. Inserir os novos ganhos em Projeto_CME_Curva.m (matriz Ks)
  4. Rodar Projeto_CME_Curva.m — gera CME_Gains
  5. Salvar: save('Curva_Gains_Final.mat', 'CME_Gains')
  6. Param_Controller.m carrega automaticamente na proxima inicializacao
