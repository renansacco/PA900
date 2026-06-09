# PA900 — Controle de Direcao para Piloto Automatico

Repositorio de desenvolvimento do controlador de direcao do PA900 (piloto automatico agricola).

## Estrutura

| Pasta | Descricao |
|---|---|
| `Controller/` | Modelo Simulink do controlador + codigo ERT gerado + buses + gains |
| `Planta/` | Modelo dinamico do veiculo (NL + S-Function + linearizacao) |
| `Guidance/` | Algoritmo de guiagem em MATLAB (reimplementacao simplificada) |
| `Simulacao/` | Harnesses de simulacao (cada subpasta eh autocontida) |
| `Projeto/` | Scripts de projeto de ganhos (LQR, CME) |
| `Analise/` | Scripts de analise de logs de campo |
| `docs/` | Documentacao |

## Integracao com Software-MG900

O Controller ERT (`Controller/Controller_ert_rtw/`) e o unico artefato que vai pro produto embarcado.

**Fluxo de deploy:**
1. Abrir `Controller/Controller.mdl` no Simulink
2. Gerar codigo (Ctrl+B)
3. Copiar os arquivos `.h` e `.cpp` de `Controller/Controller_ert_rtw/` para `Software-MG900/CarMoviment2DCpp/src/model/controller/` e `inc/model/controller/`

**Versao atual do Controller:** v1.82 (20/Out/2025)

**Arquivos copiados pro MG900:**
- `Controller.h`, `Controller.cpp`, `Controller_data.cpp`
- `Controller_private.h`, `Controller_types.h`
- `ert_main.cpp`, `rtwtypes.h`

## Como usar

1. Abrir `PA900.prj` no MATLAB (configura paths automaticamente)
2. Para simulacao: abrir `Simulacao/ClosedLoop/init_sim.m` ou `Simulacao/PlantaAberta/init_sim.m`
3. Para projeto de ganhos: scripts em `Projeto/`
4. Para analise de logs: scripts em `Analise/`
