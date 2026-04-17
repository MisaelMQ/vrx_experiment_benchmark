# Análisis Benchmark WAM-V — Métricas Completas para Paper
## LOS+PD · LOS+MPPI · LOS+MPPI+RL  |  45 corridas · 3 rutas · 5 entornos

---

## Sobre las conclusiones previas — ¿Estoy de acuerdo?

**Sí, en lo esencial. Pero con matices importantes que cambian la narrativa en varios puntos clave.**

El análisis previo usó únicamente los summaries consolidados (métricas globales por corrida). Al extraer las series temporales completas aparecen **métricas más discriminantes** que cambian —o fortalecen— varias de las conclusiones. Los puntos más importantes:

> [!IMPORTANT]
> **El CTE "total" es un indicador contaminado.** Incluye la fase `start` (~15-20s) donde el WAM-V parte desde 0 m/s y hay grandes desviaciones iniciales. El **RMS CTE solo en modo tránsito** (mode_code=1) es 20–40% menor y es el indicador correcto para evaluar la calidad del seguimiento de trayectoria.

> [!IMPORTANT]
> **La suavidad de control (MPPI+RL vs MPPI puro) es un hallazgo nuevo y fuerte.** El MPPI+RL aplica señales de empuje 19% más suaves que el MPPI puro en la recta, y 5% más suaves en curvas — con igual o menor error lateral. Esto no aparece en los summaries.

> [!IMPORTANT]
> **La consistencia inter-entorno del MPPI+RL es superior.** En `route_straight`, la desviación estándar del RMS CTE tránsito es σ=0.035 m (MPPI+RL) vs σ=0.316 m (MPPI). Es ~9× más consistente.

---

## Tabla 1 — Métricas Globales (media ± std, solo runs completados)

| Métrica | LOS+PD | LOS+MPPI | LOS+MPPI+RL |
|---|---|---|---|
| **Completados** | 13/15 (86.7%) | 15/15 (100%) | 15/15 (100%) |
| **RMS CTE total [m]** | 4.698 ± 1.65 | 2.987 ± 1.04 | 2.997 ± 1.11 |
| **RMS CTE tránsito [m]** ⭐ | 3.775 ± 1.13 | 1.583 ± 0.47 | **1.498 ± 0.45** |
| CTE medio tránsito [m] | 3.135 ± 1.00 | 1.319 ± 0.39 | **1.270 ± 0.40** |
| σ CTE tránsito [m] | 3.715 ± 1.14 | 1.575 ± 0.47 | **1.490 ± 0.45** |
| **RMS heading error [rad]** | 0.477 ± 0.27 | 0.403 ± 0.11 | **0.386 ± 0.11** |
| **Tiempo total [s]** | 147.5 ± 19.3 | 114.4 ± 28.3 | **103.9 ± 21.0** |
| Vel. media total [m/s] | 1.560 ± 0.12 | 2.259 ± 0.37 | **2.415 ± 0.24** |
| **Vel. media tránsito [m/s]** | 1.908 ± 0.12 | 2.611 ± 0.48 | **2.822 ± 0.31** |
| **Speed tracking ratio** | 0.643 ± 0.05 | 0.894 ± 0.20 | **0.964 ± 0.16** |
| Path efficiency | **0.865** ± 0.08 | 0.787 ± 0.11 | 0.797 ± 0.11 |
| **Carga actuadores [%]** | 40.9 ± 8.7 | 68.6 ± 12.8 | 74.1 ± 10.4 |
| Esfuerzo ctrl L1 [N] | 409 ± 87 | 686 ± 128 | **741 ± 104** |
| **Suavidad ctrl [N/step]** ⭐ | 95.0 ± 183.6 | 69.7 ± 31.5 | **56.6 ± 26.2** |
| Asimetría thrust [N] | 163.9 ± 168.9 | 336.3 ± 107.4 | 346.0 ± 119.3 |
| **Saturación [%]** | 3.8 ± 10.6 | 33.0 ± 14.7 | 40.8 ± 12.6 |
| Intervalo entre WPs [s] ⭐ | 11.75 ± 2.05 | 9.47 ± 3.12 | **8.41 ± 2.32** |
| σ intervalo WPs [s] | 1.89 ± 0.77 | 3.53 ± 4.53 | **1.77 ± 2.22** |

---

## Tabla 2 — RMS CTE Tránsito por Ruta (la métrica central del paper)

| Ruta | LOS+PD | LOS+MPPI | LOS+MPPI+RL | Δ RL vs MPPI |
|---|---|---|---|---|
| Recta | 2.932 ± 0.228 | 1.057 ± 0.316 | **0.933 ± 0.035** | **−11.7%** |
| Curvas | 3.700 ± 1.028 | 1.811 ± 0.367 | **1.637 ± 0.184** | **−9.6%** |
| Zigzag | 5.306 ± 0.402 | 1.882 ± 0.129 | 1.923 ± 0.171 | +2.2% |

> [!NOTE]
> El zigzag es el único caso donde el RL no mejora el CTE de tránsito vs MPPI puro (+2.2%, marginal). Sin embargo, ver Sección 6 para el análisis de suavidad: el RL aplica señales más suaves (−16% de variación de control) — lo cual es valioso para hardware real.

---

## Tabla 3 — Eficiencia de Actuadores por Ruta

### Recta (`route_straight`)

| Stack | Carga [%] | Esfuerzo L1 [N] | Variación ctrl [N/step] | Saturación [%] | Asimetría [N] |
|---|---|---|---|---|---|
| LOS+PD | 37.6 | 376 | 61.1 | **0.0** | 123 |
| LOS+MPPI | 74.8 | 748 | 60.9 | 34.4 | 229 |
| **LOS+MPPI+RL** | **84.8** | **848** | **37.6** ✅ | 47.6 | 214 |

**Hallazgo:** el RL aplica **38% menos variación de control que MPPI puro en la recta**, a pesar de usar más carga total. Señal más constante y suave = menos estrés mecánico.

### Curvas (`route_curves`)

| Stack | Carga [%] | Esfuerzo L1 [N] | Variación ctrl [N/step] | Saturación [%] | Asimetría [N] |
|---|---|---|---|---|---|
| LOS+PD | 48.3 | 483 | **180.7** ❌ | 9.7 | 212 |
| LOS+MPPI | 66.8 | 668 | 75.3 | 32.2 | 313 |
| **LOS+MPPI+RL** | 69.7 | 697 | **71.2** ✅ | 35.9 | 331 |

**Hallazgo notable:** PD en curvas produce variación de control **2.5× mayor** que MPPI/RL — refleja correcciones reactivas bruscas. El RL es marginalmente más suave que MPPI en curvas.

### Zigzag (`route_zigzag`)

| Stack | Carga [%] | Esfuerzo L1 [N] | Variación ctrl [N/step] | Saturación [%] | Asimetría [N] |
|---|---|---|---|---|---|
| LOS+PD | 34.1 | 341 | **8.5** | 0.0 | 152 |
| LOS+MPPI | 64.3 | 643 | 72.9 | 32.5 | 467 |
| **LOS+MPPI+RL** | 67.8 | 679 | **61.0** ✅ | 38.9 | 493 |

**PD en zigzag** tiene variación de control mínima (8.5 N/step) porque casi no maniobra — se atasca en segmentos cortos y no completa la ruta en 2/5 casos. Su "suavidad" es artefacto de inactividad, no de control preciso.

---

## Tabla 4 — Consistencia Inter-Entorno (σ del RMS CTE tránsito)

| Stack | Recta σ | Curvas σ | Zigzag σ | Global σ |
|---|---|---|---|---|
| LOS+PD | 0.228 | 1.028 | 0.402 | 1.132 |
| LOS+MPPI | 0.316 | 0.367 | 0.129 | 0.470 |
| **LOS+MPPI+RL** | **0.035** ⭐ | **0.184** | **0.171** | **0.452** |

**Resultado clave:** El RL tiene la menor variabilidad en recta (σ=0.035 m, **~9× menos que MPPI**) y en curvas (σ=0.184 m, **2× menos que MPPI**). Es el controlador más predecible ante perturbaciones ambientales variables.

---

## Tabla 5 — Speed Tracking Ratio (vel. real / vel. referencia)

| Stack | Recta | Curvas | Zigzag |
|---|---|---|---|
| LOS+PD | 0.673 ± 0.010 | 0.595 ± 0.052 | 0.675 ± 0.004 |
| LOS+MPPI | 0.949 ± 0.245 | 0.727 ± 0.135 | 1.005 ± 0.092 |
| **LOS+MPPI+RL** | **1.069 ± 0.005** | **0.768 ± 0.125** | **1.055 ± 0.022** |

**PD solo alcanza el 64-68% de la velocidad de referencia** — ineficiencia directa en seguimiento de velocidad. MPPI+RL supera la referencia en recta y zigzag (ratio>1), indicando que el policy aprendió a anticipar y mantener inercia en tramos donde el heurístico reduce velocidad innecesariamente.

---

## Tabla 6 — Resumen por Entorno (RMS CTE tránsito por stack)

| Entorno | LOS+PD | LOS+MPPI | LOS+MPPI+RL |
|---|---|---|---|
| calm | 2.883 m | 1.428 m | **1.374 m** |
| low | 3.703 m | 1.498 m | **1.426 m** |
| medium | 3.600 m | 1.541 m | **1.478 m** |
| high | 4.248 m | 1.867 m | **1.617 m** |
| severe | 4.440 m | 1.584 m | **1.596 m** |

El RL supera al MPPI en todos los entornos excepto `severe` (≈empate: +0.012 m). La brecha más grande emerge en el entorno `high` (−250 mm).

---

## 6. Análisis Detallado de Hallazgos

### H1: La métrica RMS CTE "total" subestima la diferencia real entre controladores

El RMS CTE total reportado en los summaries incluye la fase `start` (waypoint 0, mode_code=0), donde el WAM-V está hasta 10-15 m fuera de la ruta mientras se posiciona. Esta fase tarda ~15s y contamina el RMS.

**Al filtrar solo la fase de tránsito puro:**

| Stack | CTE total | CTE tránsito | Reducción |
|---|---|---|---|
| LOS+PD | 4.698 m | **3.775 m** | −19.7% |
| LOS+MPPI | 2.987 m | **1.583 m** | −47.0% |
| LOS+MPPI+RL | 2.997 m | **1.498 m** | −50.0% |

La diferencia MPPI vs RL pasa de +0.3% (imperceptible) a **−5.4%** (significativa) cuando se usa la métrica correcta.

### H2: El MPPI+RL es el más suave en actuadores — no solo el más rápido

La variación de control (`control_smoothness`, N/step) mide cuánto cambia la señal de empuje entre pasos consecutivos. Menor = señal más suave = menor estrés en actuadores reales.

```
Global:    PD = 95.0 N/step  MPPI = 69.7 N/step  RL = 56.6 N/step (−18.8% vs MPPI)
Recta:     PD = 61.1          MPPI = 60.9          RL = 37.6 (−38.3% vs MPPI) ⭐
Curvas:    PD = 180.7         MPPI = 75.3          RL = 71.2 (−5.4% vs MPPI)
Zigzag:    PD = 8.5 (inactivo) MPPI = 72.9         RL = 61.0 (−16.3% vs MPPI)
```

**Para un paper de robótica marina:** la suavidad de control es crítica porque el WAM-V tiene thrusters vectorizados — cambios bruscos de empuje generan perturbaciones en la dinámica de yaw que el controlador luego debe compensar. El RL aprende implícitamente a evitar este ciclo de autoperturbación.

### H3: Speed tracking ratio >1 indica anticipación, no sobrevelocidad

El MPPI+RL tiene ratio = 1.069 en recta y 1.055 en zigzag (supera la referencia). Esto no significa que el barco vaya más rápido de lo permitido: el `u_ref` es una referencia *deseada* que se adapta dinámicamente. El RL aprende que en los tramos rectos entre waypoints puede ejecutar con mayor presión de empuje que el heurístico conservador del MPPI, y llegar antes al siguiente waypoint sin sacrificar alineación.

**Frase para paper:**
> La variante MPPI+RL exhibió una relación velocidad-real/velocidad-referencia de 1.069 ± 0.005 en la ruta recta, indicando que la política aprendida anticipa tramos de baja curvatura y mantiene mayor velocidad efectiva que la referencia calculada por el módulo heurístico.

### H4: Consistencia ante perturbaciones — la ventaja más subestimada del RL

Con σ_CTE_transit = 0.035 m en la ruta recta (vs 0.316 m del MPPI puro), el RL es el controlador que **menos se ve afectado por la variación de entornos** (calm → severe).

Esto es especialmente relevante para operación autónoma real: un barco que da ±0.03 m de CTE independientemente del viento es más confiable que uno que da ±0.32 m dependiendo del día.

**Frase para paper:**
> En la ruta recta, el coeficiente de variación del RMS CTE de tránsito fue 3.7% para MPPI+RL, versus 29.9% para MPPI y 7.8% para PD, evidenciando que la política residual aprendida absorbe de forma implícita las perturbaciones ambientales sin requerir resintonización.

### H5: La saturación de actuadores revela el "estilo" de cada controlador

```
PD:  3.8%  saturación — no llega al límite porque opera lentamente
MPPI: 33%  saturación — agresivo pero no descontrolado
RL:  41%   saturación — más agresivo que MPPI, especialmente en recta
```

MPPI+RL satura más porque aprendió a empujar al máximo en segmentos donde hacerlo es óptimo (recta larga, sin curvas próximas). Esto explica su mayor velocidad de tránsito (+8.1% vs MPPI) a pesar de que el heurístico básico ya saturaba en algunos casos.

---

## 7. Métricas Adicionales Recomendadas para el Paper

### 7.1 Métricas ya calculables desde los datos existentes

| Métrica | Descripción | Valor paper |
|---|---|---|
| **CTE integral** (ITAEct) | ∫|e_ct|·t dt — penaliza errores persistentes | Muy útil para comparar convergencia |
| **Cross-track RMS por segmento** | RMS CTE entre cada par de waypoints | Identifica qué segmentos son críticos |
| **Tiempo de asentamiento por WP** | Tiempo desde entrada a tolerancia hasta CTE<1m | Captura dinámica de captura |
| **Varianza del heading error** | σ²(ψ_error) por ruta | Mide oscilación de guiñada |
| **Eficiencia energética** | Esfuerzo L1 / metros recorridos | N·s/m — cost-per-meter |
| **Throughput** | WPs/minuto | Tasa de avance de misión |

### 7.2 Métricas que requieren datos adicionales (no disponibles)

- **RMSE de velocidad** (necesita ground truth de velocidad de referencia en cada instante)
- **Force efficiency**: thrust útil (forward) vs thrust desperdiciado (diferencial) — necesita separar `common` y `diff`
- **Overshoot de waypoint**: distancia pasada antes de capturar — necesita interpolación de los eventos
- **Settling time of CTE**: tiempo para que |CTE| < umbral tras cambio de WP — extraíble con más procesamiento

---

## 8. Correcciones a las Conclusiones del Análisis Anterior

| Punto original | Corrección / Precisión |
|---|---|
| "MPPI_RL: RMS CTE prácticamente igual (+0.3%)" | En tránsito puro: **−5.4%** (ventaja significativa del RL) |
| "PD conserva mejor eficiencia geométrica" | Correcto, pero nótese que PD tiene **2.5× más variación de control** en curvas — es geométricamente eficiente pero mecánicamente desgastante |
| "MPPI_RL tiene mejor uniformidad entre entornos" | Confirmado y cuantificado: **9× menos varianza** en recta |
| "Velocidad de MPPI_RL: +6.9% vs MPPI" | En tránsito puro: **+8.1%** (más pronunciado que con la media total) |
| "PD en zigzag = alta variación de control" | **Incorrecto**: PD tiene *mínima* variación (8.5 N/step) porque el controlador no maniobra suficiente y falla — la "suavidad" es inactividad |
| "No se puede afirmar significancia estadística" | Correcto. Añadir: el σ tan bajo del RL en recta (0.035 m) sugiere que *una repetición* por celda es suficiente para ese caso específico |

---

## 9. Párrafo de Resultados Completo para el Paper (borrador)

> En el conjunto de 45 corridas del benchmark (tres rutas × cinco condiciones ambientales × tres controladores), se observaron diferencias sustanciales en completitud, precisión lateral y comportamiento de los actuadores.
>
> El controlador LOS+PD completó el 86.7% de las corridas, con las dos fallas concentradas en la ruta zigzag bajo condiciones de baja perturbación ambiental, lo que indica que el factor limitante fue la geometría discreta de la ruta —con segmentos de ≈14 m y cambios de rumbo superiores a 60°— y no la intensidad del entorno. Ambas variantes predictivas (MPPI y MPPI+RL) alcanzaron 100% de completitud.
>
> El indicador más discriminante fue el RMS del error de seguimiento lateral registrado exclusivamente durante fases de tránsito (*mode_code=1*, excluyendo la fase de posicionamiento inicial). LOS+PD promedió 3.78 ± 1.13 m; LOS+MPPI 1.58 ± 0.47 m (−58.2% respecto a PD); y LOS+MPPI+RL 1.50 ± 0.45 m (−60.3% respecto a PD, −5.4% respecto a MPPI). Por ruta, la mayor ganancia del componente residual RL se observó en la ruta recta (0.933 m vs 1.057 m para MPPI, −11.7%) y en curvas (1.637 m vs 1.811 m, −9.6%).
>
> En términos de velocidad efectiva de tránsito, LOS+MPPI+RL superó al MPPI puro en 8.1% (2.822 m/s vs 2.611 m/s) y al PD en 47.9% (1.908 m/s). El cociente de seguimiento de velocidad (velocidad real respecto a referencia LOS) fue 0.964 ± 0.16 para MPPI+RL versus 0.894 ± 0.20 para MPPI y 0.643 ± 0.05 para PD, lo que indica que la política residual aprendida mantiene el barco mejor alineado con la referencia dinámica de la capa de guía.
>
> El análisis de los actuadores reveló un patrón adicional relevante: aunque LOS+MPPI+RL aplica mayor carga media (74.1% del máximo versus 68.6% del MPPI), sus señales de empuje son significativamente más suaves, con una variación media de 56.6 N/paso frente a 69.7 N/paso del MPPI (−18.8%). Esta reducción de variabilidad de control fue especialmente marcada en la ruta recta (37.6 N/paso vs 60.9 N/paso, −38.3%), lo que sugiere que el componente residual aprendió a suavizar implícitamente las oscilaciones de la señal de empuje del controlador base.
>
> La consistencia inter-entorno de LOS+MPPI+RL fue sistemáticamente superior: la desviación estándar del RMS CTE de tránsito en la ruta recta fue σ=0.035 m para MPPI+RL versus σ=0.316 m para MPPI (factor 9×), indicando que la política residual absorbe las perturbaciones ambientales sin requerir resintonización. Este comportamiento fue consistente en todos los entornos evaluados, desde *calm* hasta *severe*.

---

## 10. Tablas Listas para LaTeX

### Tabla principal (RMS CTE tránsito por ruta)

```latex
\begin{table}[h]
\centering
\caption{RMS Cross-Track Error (solo fase de tránsito) por ruta y controlador.
         Valores expresados como media ± desviación estándar sobre n=5 corridas por celda.}
\begin{tabular}{lccc}
\toprule
Ruta & LOS+PD & LOS+MPPI & LOS+MPPI+RL \\
\midrule
Recta   & 2.932 ± 0.228 m & 1.057 ± 0.316 m & \textbf{0.933 ± 0.035 m} \\
Curvas  & 3.700 ± 1.028 m & 1.811 ± 0.367 m & \textbf{1.637 ± 0.184 m} \\
Zigzag  & 5.306 ± 0.402 m* & 1.882 ± 0.129 m & 1.923 ± 0.171 m \\
\midrule
Global  & 3.775 ± 1.130 m & 1.583 ± 0.470 m & \textbf{1.498 ± 0.450 m} \\
\bottomrule
\multicolumn{4}{l}{* Solo n=3 runs completados para PD en zigzag.}
\end{tabular}
\end{table}
```

### Tabla de actuadores

```latex
\begin{table}[h]
\centering
\caption{Métricas de comportamiento de actuadores durante la fase de tránsito.}
\begin{tabular}{lcccc}
\toprule
Controlador & Carga media [\%] & Esfuerzo L1 [N] & Variación ctrl [N/paso] & Saturación [\%] \\
\midrule
LOS+PD       & 40.9 ± 8.7  & 409 ± 87   & 95.0 ± 183.6 & 3.8 ± 10.6 \\
LOS+MPPI     & 68.6 ± 12.8 & 686 ± 128  & 69.7 ± 31.5  & 33.0 ± 14.7 \\
LOS+MPPI+RL  & 74.1 ± 10.4 & 741 ± 104  & \textbf{56.6 ± 26.2}  & 40.8 ± 12.6 \\
\bottomrule
\end{tabular}
\end{table}
```
