# Integracao Controller ERT com Software-MG900

## Visao geral

O Controller ERT eh o unico artefato Simulink usado no Software-MG900 (embarcado).
Todo o resto (Guidance, Navigation) eh implementado em C++ nativo no MG900.

## Arquivos gerados

O Simulink Coder gera os seguintes arquivos em `Controller/Controller_ert_rtw/`:

| Arquivo | Descricao |
|---|---|
| `Controller.h` | Header principal: classe `ControladorModelClass`, structs ExtU/ExtY |
| `Controller.cpp` | Implementacao do step(), initialize(), terminate() |
| `Controller_data.cpp` | Tabelas de ganhos e constantes (hardcoded pelo Simulink) |
| `Controller_private.h` | Macros e defines internos |
| `Controller_types.h` | Definicoes de tipos (buses, enums) |
| `ert_main.cpp` | Main de exemplo (nao usado no MG900, que tem seu proprio wrapper) |
| `rtwtypes.h` | Tipos basicos do runtime Simulink |

## Destino no MG900

```
Software-MG900/CarMoviment2DCpp/
  inc/model/controller/    <- headers (.h)
  src/model/controller/    <- sources (.cpp)
```

## Interface

### Inputs (ExtU_Controller_T)
- `Measurements`: Psi (rad), r (rad/s), theta (rad), vx (m/s)
- `Guidance`: alpha (rad), e (m), curvature (1/m), psiError (rad)
- `Enable`: boolean
- `vehicleMode`: VehicleMode_t enum

### Outputs (ExtY_Controller_T)
- `Angular_Speed_Target`: comando pro servo (rad/s)
- `Flag_Enable_Servo`: habilita motor
- `Flag_Delta_Saturation`: saturacao da direcao
- `Operation_Mode`: IDLE/WAIT/KEEP/ENTRY/CURVE

### Wrapper no MG900
- `controller_handler.cpp` instancia `ControladorModelClass`
- Chama `step()` a cada 50ms via QTimer
- Conversao NED->ENU do heading feita no wrapper, nao no modelo

## Procedimento de deploy

1. Abrir `Controller/Controller.mdl` no Simulink
2. Ajustar ganhos se necessario (via `Param_Controller.m` e arquivos em `gains/`)
3. Gerar codigo: Ctrl+B (ou menu Code > C/C++ Code > Build Model)
4. Copiar de `Controller/Controller_ert_rtw/` para `Software-MG900`:
   - `Controller.h`, `Controller.cpp`, `Controller_data.cpp` -> `src/model/controller/`
   - `Controller_private.h`, `Controller_types.h`, `rtwtypes.h` -> `inc/model/controller/`
5. Compilar o MG900 e testar

## Guidance: diferenca entre Simulink e C++

O Guidance no Software-MG900 eh implementado em C++ nativo (`guidance_core.cpp`),
NAO usa o modelo Simulink de `ERT_Model/Guidance/`.

O algoritmo usa B-spline cubico uniforme com biseccao para projecao no caminho.
A reimplementacao MATLAB em `Guidance/guidance_step.m` replica este algoritmo
para permitir simulacoes closed-loop consistentes com o embarcado.
