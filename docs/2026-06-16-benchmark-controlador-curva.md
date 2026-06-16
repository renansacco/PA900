# Benchmark do Controlador de Curva

## Objetivo

Criar um framework de testes automatizado para avaliar o desempenho do controlador de curva (modo Curva) em cenarios realistas. O benchmark roda N simulacoes no modelo Simulink `modelClosedLoop.slx`, variando trajetoria, condicao inicial e velocidade, e produz metricas quantitativas e plots comparativos.

O objetivo imediato e estabelecer a **baseline** com os ganhos atuais (CME). Em seguida, o mesmo benchmark sera usado para avaliar novos controladores.

## Matriz de testes

### Trajetorias (3)

1. **Reta sintetica** — 200m em linha reta, waypoints espacados a cada 3m.
2. **Curva fechada sintetica** — arco de circulo com R=6m, 180 graus, waypoints a cada 3m. Simula cabeceira de taipa.
3. **Taipa real** — guia de `taipas_boeck.mat` que contenha transicao reta-curva-reta.

Trajetorias geradas por `gerar_trajetorias.m` e salvas em `benchmark_trajs.mat`.

### Condicoes iniciais (3)

1. **Alinhado** — veiculo posicionado no inicio da guia, heading alinhado.
2. **Offset lateral 0.5m** — deslocamento perpendicular de 0.5m em relacao ao inicio da guia.
3. **Heading +10 graus** — veiculo no inicio da guia com erro de heading de +10 graus.

### Velocidades (3)

- vx = 1.0 m/s (3.6 km/h)
- vx = 2.0 m/s (7.2 km/h)
- vx = 3.0 m/s (10.8 km/h)

### Total: 27 simulacoes por controlador

## Metricas

Calculadas por `calcular_metricas.m` a partir dos sinais de saida do Simulink (`out`). Um periodo de descarte inicial (ex: 2s) e aplicado para remover o transiente de partida quando a condicao inicial e alinhada.

### Erro lateral

| Metrica | Descricao |
|---------|-----------|
| `e_lat_mean` | Media de \|e_lat\| ao longo da trajetoria [m] |
| `e_lat_max` | Maximo de \|e_lat\| [m] |
| `e_lat_rms` | RMS do erro lateral [m] |
| `e_lat_hist` | Histograma de e_lat (bins de 5cm) |

### Sinal de controle

| Metrica | Descricao |
|---------|-----------|
| `ctrl_energy` | RMS de omega_m_ref [rad/s] |
| `ctrl_smooth` | RMS de d/dt(omega_m_ref) [rad/s^2] |

### Transiente

| Metrica | Descricao |
|---------|-----------|
| `overshoot` | Maximo de \|e_lat\| nos primeiros 5s [m] |
| `settling_time` | Tempo ate \|e_lat\| < 0.10m permanentemente [s] |

### Saturacao

| Metrica | Descricao |
|---------|-----------|
| `sat_omegam_pct` | % do tempo com \|omega_m\| >= omega_m_sat |
| `sat_delta_pct` | % do tempo com \|delta\| >= delta_max |

## Arquitetura

```
Simulacao/Benchmark/
  gerar_trajetorias.m     — gera reta e curva sinteticas, seleciona taipa real, salva benchmark_trajs.mat
  rodar_benchmark.m       — loop sobre cenarios: seta workspace, sim(), coleta metricas
  calcular_metricas.m     — function(out, Ts, t_descarte) -> struct metrics
  plotar_resultados.m     — tabela resumo, histogramas, plots comparativos
```

### Fluxo de execucao

1. Rodar `gerar_trajetorias.m` uma vez — gera `benchmark_trajs.mat`.
2. Rodar `rodar_benchmark.m` — carrega trajetorias, carrega modelo, itera:
   - Para cada combinacao (trajetoria, IC, vx):
     - Seta `wps`, `X0`, `vx`, `Tsim` no workspace
     - Roda `Param_Controller` (carrega buses e ganhos)
     - Executa `out = sim('modelClosedLoop')`
     - Chama `calcular_metricas(out, Ts, t_descarte)` e armazena resultado
   - Salva `benchmark_results.mat` com todos os cenarios e metricas.
3. Rodar `plotar_resultados.m` — le `benchmark_results.mat` e gera:
   - Tabela resumo (cenario x metricas)
   - Histogramas de erro lateral por cenario
   - Plots comparativos entre controladores (quando houver mais de um)

### Modelo Simulink

Usa o `modelClosedLoop.slx` existente. O modelo recebe do workspace:
- `wps` — struct com .x, .y (waypoints)
- `X0` — vetor 7x1 de condicoes iniciais
- `vx` — velocidade longitudinal [m/s]
- `Tsim` — tempo de simulacao [s]
- `params` — struct de param_MF6713.mat
- Variaveis do `Param_Controller` (buses, ganhos, etc.)

Saidas via "To Workspace": `out.e`, `out.omegam`, `out.omegam_ref`, `out.delta_deg`, `out.r_IC`, `out.curvature`, `out.psi_deg`, `out.psi_ref_deg`, `out.r`, `out.vx`.

### Sinais necessarios para saturacao

O modelo precisa exportar `omega_m` e `delta` em unidades absolutas para calcular as metricas de saturacao. Os sinais `out.omegam` e `out.delta_deg` ja existem nas saidas atuais.

Limites de saturacao:
- `omega_m_sat = 15 rad/s` (Controlador.Value.Curva.omegam_sat)
- `delta_max` — derivado dos parametros do veiculo (a ser verificado no modelo)

## Saida esperada

Apos rodar o benchmark com os ganhos atuais (CME), teremos:

1. `benchmark_results.mat` com 27 structs contendo cenario + metricas
2. Tabela resumo impressa no console
3. Figuras com histogramas e comparativos

Esses resultados servem como **baseline** para comparacao com futuros controladores.
