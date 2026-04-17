# vrx_experiment_benchmark — Contexto Completo del Proyecto

> **Versión de referencia:** abril 2026 · ROS 2 Humble · Gazebo Garden · WAM-V 16  
> **Propósito:** documentar toda la arquitectura, tópicos, parámetros y resultados del stack LOS+PD
> para servir de base a nuevos algoritmos (MPPI, RL, etc.).

---

## 1. Descripción General

El paquete `vrx_experiment_benchmark` implementa un stack modular de navegación autónoma para el
robot marino WAM-V 16 en el simulador VRX (Virtual RobotX). El sistema permite seguir rutas
definidas por waypoints GPS con diferentes geometrías (recto, curvas, zigzag) y registra métricas
de desempeño en CSV para benchmarking comparativo entre algoritmos.

**Arquitectura por capas:**

```
GPS/IMU ──► [state_estimator_2d] ──► [route_manager] ──► [guidance_los / MPPI / RL]
                                                              │
                                                              ▼
                                                        [controller_pd]
                                                              │
                                                              ▼
                                                      [thruster_commander]
                                                              │
                                                   ┌──────────┴──────────┐
                                             left_thrust            right_thrust
```

---

## 2. Estructura de Archivos

```
vrx_experiment_benchmark/
├── config/
│   ├── pd_config.yaml             # Ganancias y límites del controlador PD
│   ├── los_config.yaml            # Parámetros de guiado LOS (velocidades, lookahead)
│   ├── controller_limits.yaml     # Límites físicos de los actuadores
│   ├── route_manager_config.yaml  # Hold times, publish rate del route_manager
│   ├── metrics_config.yaml        # Configuración del logger de métricas
│   ├── state_estimator_2d.yaml    # Filtros y tópicos del estimador de estado
│   ├── mppi_config.yaml           # Placeholder para parámetros MPPI (futuro)
│   ├── routes/
│   │   ├── route_straight.yaml    # Ruta recta ~159 m, 10 WPs
│   │   ├── route_curves.yaml      # Ruta curva ~186 m, 10 WPs
│   │   └── route_zigzag.yaml      # Ruta zigzag, 10 WPs, segmentos ~14 m
│   └── env/                       # YAMLs de entornos de simulación (viento, olas)
├── launch/
│   ├── route_following_pd.launch.py   ← LAUNCH PRINCIPAL
│   ├── state_estimation.launch.py     # Solo estimador de estado
│   └── experiment.launch.py           # Launch de experimento completo (con env)
├── vrx_experiment_benchmark/     # Nodos Python
│   ├── state_estimator_2d.py
│   ├── route_manager.py
│   ├── guidance_los.py
│   ├── controller_pd.py
│   ├── thruster_commander.py
│   ├── metrics_logger.py
│   ├── route_utils.py             # Funciones matemáticas/geográficas compartidas
│   └── world_generator.py
├── metrics/
│   ├── raw/                       # CSVs timeseries y events por run
│   └── summary/                   # CSVs de resumen por run
├── scripts/
│   └── sync_config.sh             # Sincronización rápida src→install sin rebuild
└── worlds/                        # Mundos Jinja para Gazebo
```

---

## 3. Cómo Compilar y Ejecutar

### 3.1 Build completo (primera vez o al cambiar setup.py/package.xml)

```bash
cd ~/ros2_workspaces/diplomado_ws
colcon build --packages-select vrx_experiment_benchmark
source install/setup.bash
```

### 3.2 Sincronización rápida (cambios en .py o .yaml solamente)

```bash
bash src/vrx_experiment_benchmark/scripts/sync_config.sh
# No requiere colcon build
```

### 3.3 Lanzar la ruta (entorno VRX ya corriendo en otra terminal)

```bash
source install/setup.bash
ros2 launch vrx_experiment_benchmark route_following_pd.launch.py route_name:=route_straight.yaml
ros2 launch vrx_experiment_benchmark route_following_pd.launch.py route_name:=route_curves.yaml
ros2 launch vrx_experiment_benchmark route_following_pd.launch.py route_name:=route_zigzag.yaml
```

**Argumentos del launch:**

| Argumento | Default | Descripción |
|---|---|---|
| `route_name` | `route_curves.yaml` | Nombre del YAML en `config/routes/` |
| `route_file` | `""` | Ruta absoluta opcional a cualquier YAML |
| `guidance_mode` | `"los"` | `"los"` o `"mppi"` (futuro) |

---

## 4. Grafo de Tópicos

```
/wamv/sensors/gps/gps/fix         [sensor_msgs/NavSatFix]
/wamv/sensors/imu/imu/data        [sensor_msgs/Imu]
         │
         ▼
[state_estimator_2d]
         │
         ├──► /wamv/navigation/state2d        [nav_msgs/Odometry]
         └──► /wamv/navigation/local_origin   [sensor_msgs/NavSatFix]
                     │
                     ▼
              [route_manager]
                     │
                     ├──► /wamv/navigation/route_local          [nav_msgs/Path]
                     ├──► /wamv/navigation/active_waypoint      [geometry_msgs/PointStamped]
                     ├──► /wamv/navigation/active_waypoint_meta [std_msgs/Float64MultiArray]
                     └──► /wamv/navigation/route_status         [std_msgs/Float64MultiArray]
                                   │
                                   ▼
                            [guidance_los]  ← también suscrito a state2d
                                   │
                                   └──► /wamv/control/guidance_cmd [std_msgs/Float64MultiArray]
                                                │
                                                ▼
                                        [controller_pd]  ← también suscrito a state2d
                                                │
                                                └──► /wamv/control/thruster_cmd [std_msgs/Float64MultiArray]
                                                             │
                                                             ▼
                                                   [thruster_commander]
                                                             │
                                              ┌──────────────┼──────────────┐
                                              ▼              ▼              ▼
                             left/thrust  right/thrust  left/pos  right/pos
                             [Float64]    [Float64]    [Float64] [Float64]
```

**Suscriptores adicionales de metrics_logger:**
- `/wamv/navigation/state2d`
- `/wamv/navigation/route_status`
- `/wamv/navigation/active_waypoint_meta`
- `/wamv/control/guidance_cmd`
- `/wamv/control/controller_debug`
- `/wamv/control/thruster_cmd`

---

## 5. Definición de Tópicos Críticos

### 5.1 `/wamv/navigation/active_waypoint_meta` — `Float64MultiArray` (15 campos)

Este es el tópico de interfaz principal entre `route_manager` y el nodo de guiado.
**Todo nuevo algoritmo debe suscribirse a este tópico.**

| Índice | Campo | Tipo | Descripción |
|---|---|---|---|
| `[0]` | `wp_id` | int | ID del waypoint activo |
| `[1]` | `mode_code` | int | 0=start, 1=transit, 2=finish |
| `[2]` | `pos_tol` | float | Tolerancia de posición [m] |
| `[3]` | `wp_x` | float | Coordenada X del waypoint [m] |
| `[4]` | `wp_y` | float | Coordenada Y del waypoint [m] |
| `[5]` | `path_yaw` | float | Ángulo del segmento [rad] |
| `[6]` | `seg_x0` | float | X inicio del segmento [m] |
| `[7]` | `seg_y0` | float | Y inicio del segmento [m] |
| `[8]` | `seg_x1` | float | X fin del segmento [m] |
| `[9]` | `seg_y1` | float | Y fin del segmento [m] |
| `[10]` | `active_idx` | int | Índice interno en la lista |
| `[11]` | `total_wps` | int | Total de waypoints en la ruta |
| `[12]` | `hold_elapsed` | float | Tiempo en hold [s] |
| `[13]` | `hold_req` | float | Hold requerido [s] |
| `[14]` | `completed` | int | 1 si la ruta está completa, 0 si no |

### 5.2 `/wamv/control/guidance_cmd` — `Float64MultiArray` (12 campos)

Contrato de salida del nodo de guiado. **Todo nuevo algoritmo de guiado debe publicar este formato.**

| Índice | Campo | Tipo | Descripción |
|---|---|---|---|
| `[0]` | `psi_ref` | float | Heading deseado [rad] |
| `[1]` | `u_ref` | float | Velocidad de avance deseada [m/s] |
| `[2]` | `e_ct` | float | Error de pista lateral [m] |
| `[3]` | `chi_p` | float | Rumbo del segmento activo [rad] |
| `[4]` | `dist_to_wp` | float | Distancia al waypoint activo [m] |
| `[5]` | `delta` | float | Lookahead distance [m] |
| `[6]` | `heading_error` | float | `psi_ref - yaw` [rad] |
| `[7]` | `wp_id` | float | ID del waypoint activo |
| `[8]` | `mode_code` | float | 0=start, 1=transit, 2=finish |
| `[9]` | `pos_tol` | float | Tolerancia de posición [m] |
| `[10]` | `wp_x` | float | X del waypoint activo [m] |
| `[11]` | `wp_y` | float | Y del waypoint activo [m] |

> **Caso especial — ruta completada:** cuando `completed=1`, el nodo de guiado debe publicar
> `u_ref=0.0` y `psi_ref=yaw_actual` para que el controlador lleve los thrusters a cero.

### 5.3 `/wamv/control/thruster_cmd` — `Float64MultiArray` (4 campos)

| Índice | Campo | Unidad |
|---|---|---|
| `[0]` | `left_thrust` | N |
| `[1]` | `right_thrust` | N |
| `[2]` | `left_pos` | rad |
| `[3]` | `right_pos` | rad |

### 5.4 `/wamv/navigation/state2d` — `nav_msgs/Odometry`

| Campo | Descripción |
|---|---|
| `pose.pose.position.{x,y}` | Posición local [m] desde el origen GPS |
| `pose.pose.orientation` | Cuaternión → usar `route_utils.quaternion_to_yaw()` |
| `twist.twist.linear.{x,y}` | Velocidad lineal [m/s] |
| `twist.twist.angular.z` | Yaw rate [rad/s] |

---

## 6. Descripción de Nodos

### 6.1 `state_estimator_2d`

**Archivo:** `vrx_experiment_benchmark/state_estimator_2d.py`  
**Frecuencia:** 20 Hz

Funde GPS + IMU para producir un estado 2D en coordenadas locales ENU.
Usa el primer fix GPS como origen dinámico del frame local (`map_local`).

**Filtros (valores YAML activos):**

| Parámetro | Valor | Descripción |
|---|---|---|
| `position_alpha` | 0.85 | EMA de posición GPS |
| `velocity_alpha` | 0.20 | EMA de velocidad |
| `yaw_alpha` | 0.12 | EMA de yaw (IMU) |
| `min_speed_for_course_yaw` | 0.30 m/s | Umbral para actualizar yaw por curso |

---

### 6.2 `route_manager`

**Archivo:** `vrx_experiment_benchmark/route_manager.py`  
**Frecuencia:** 10 Hz  
**Fuente de verdad:** avance de waypoints, detección de hold, flag de ruta completada.

**Lógica de avance:**

- **Modo `transit`:**
  - **Criterio 1:** distancia al WP ≤ `pos_tolerance` (inside\_tolerance)
  - **Criterio 2:** proyección escalar `proj ≥ 1.05` sobre el segmento (overshoot)
  - Ambos criterios avanzan al siguiente WP sin hold.

- **Modo `start` / `finish`:**
  - Debe entrar en `pos_tolerance` Y mantener hold.
  - `start_hold_sec = 2.0 s`, `finish_hold_sec = 3.0 s`
  - Si sale de la tolerancia durante el hold: **reset** (hold\_started = False).
  - Al completar el hold de `finish`: `route_completed = True`.

**Mensajes de log visuales:**

```
╔════════════════════════════════════════════╗
║  ▶ WP N → WP N+1  [TRANSIT]  dist=X.Xm    ║
╚════════════════════════════════════════════╝

┌────────────────────────────────────────┐
│  ⏸ HOLD  idx=N  mode=START  espera=2.0s  │
└────────────────────────────────────────┘

╔════════════════════════════════════════════╗
║  ✅ HOLD OK  idx=N  hold=2.1s  → avanzando ║
╚════════════════════════════════════════════╝

██████████████████████████████████████████████
  ★ RUTA COMPLETADA  hold=3.1s  WP=9  ★
██████████████████████████████████████████████
```

**Parámetros (route_manager_config.yaml):**

| Parámetro | Valor | Descripción |
|---|---|---|
| `start_hold_sec` | 2.0 | Tiempo de hold en WP inicial [s] |
| `finish_hold_sec` | 3.0 | Tiempo de hold en WP final [s] |
| `publish_rate_hz` | 10.0 | Frecuencia de publicación [Hz] |
| `debug_interval_sec` | 3.0 | Intervalo de log periódico [s] |

---

### 6.3 `guidance_los`

**Archivo:** `vrx_experiment_benchmark/guidance_los.py`  
**Frecuencia:** 20 Hz  
**Algoritmo:** Line-of-Sight (LOS) con lookahead adaptativo y velocidad adaptativa por segmento.

**Lógica principal:**

1. Calcula el error de pista lateral `e_ct` sobre el segmento activo.
2. Calcula el lookahead `delta = clamp(lookahead_min + lookahead_speed_gain × |v|, min, max)`.
3. **Modo `transit`:** `psi_ref = chi_p - atan2(e_ct, delta)` (LOS puro).
4. **Modo `start`/`finish`:** `psi_ref = bearing_directo_al_WP`.
5. Calcula `u_ref` via `nominal_speed()`.
6. Si `completed=True`: publica `u_ref=0.0, psi_ref=yaw_actual` (parada suave).

**Velocidad adaptativa por segmento (novedad v2):**

```python
# En modo transit:
seg_speed_cap = transit_speed
if seg_len < 20.0:
    seg_speed_cap = max(2.0, transit_speed * (seg_len / 20.0))
# → segmento de 14m: cap = max(2.0, 4.0 × 14/20) = max(2.0, 2.8) = 2.8 m/s
```

Esto evita que en rutas con segmentos cortos (zigzag ~14 m) el barco no pueda frenar a tiempo.

**Parámetros (los_config.yaml):**

| Parámetro | Valor activo | Descripción |
|---|---|---|
| `publish_rate_hz` | 20.0 | Hz |
| `lookahead_min` | 6.0 m | Lookahead mínimo |
| `lookahead_max` | 14.0 m | Lookahead máximo |
| `lookahead_speed_gain` | 1.5 | `delta += gain × speed` |
| `start_speed` | 2.5 m/s | Velocidad en modo start |
| `transit_speed` | 4.0 m/s | Velocidad nominal en transit |
| `finish_speed` | 1.5 m/s | Velocidad en modo finish |
| `slowdown_radius` | 5.0 m | Radio de desaceleración al WP |
| `debug_interval_sec` | 3.0 s | Frecuencia de log |

---

### 6.4 `controller_pd`

**Archivo:** `vrx_experiment_benchmark/controller_pd.py`  
**Frecuencia:** 20 Hz

Controlador diferencial (dos thrusters fijos en popa). Recibe `psi_ref` y `u_ref` del nodo de guiado.

**Fórmulas:**

```
yaw_error    = wrap_angle(psi_ref - yaw)
yaw_effort   = kp_yaw × yaw_error - kd_yaw × yaw_rate
differential = yaw_to_diff_gain × yaw_effort
common       = surge_gain × u_ref
               × 0.7  (si mode ∈ {start, finish})

left_thrust  = clamp(common - differential, ±max_thrust)
right_thrust = clamp(common + differential, ±max_thrust)
```

**Parámetros (pd_config.yaml):**

| Parámetro | Valor activo | Descripción |
|---|---|---|
| `kp_yaw` | 2.3 | Ganancia proporcional de yaw |
| `kd_yaw` | 5.0 | Ganancia derivativa de yaw (amortiguación) |
| `surge_gain` | 110.0 | Conversión u_ref [m/s] → empuje [N] |
| `yaw_to_diff_gain` | 60.0 | Escala del esfuerzo de yaw a diferencial |
| `max_thrust` | 1000.0 N | Límite de saturación del controlador |
| `fixed_thruster_angle` | 0.0 rad | Ángulo fijo de los thrusters |
| `publish_rate_hz` | 20.0 | Hz |
| `debug_interval_sec` | 3.0 s | |

> **Nota de diseño:** `mode ∈ {start, finish}` aplica un factor 0.7 sobre `common` para reducir
> la velocidad de avance durante los holds, sin cambiar los gains de yaw.

---

### 6.5 `thruster_commander`

**Archivo:** `vrx_experiment_benchmark/thruster_commander.py`  
**Frecuencia:** 20 Hz

Puente entre el controlador y el simulador. Aplica clamp final, timeout de seguridad y publica a
los tópicos nativos VRX.

**Timeout de seguridad:** si no recibe comandos en `command_timeout_sec = 0.5 s`, publica `0.0 N`.

**Parámetros (controller_limits.yaml):**

| Parámetro | Valor activo | Descripción |
|---|---|---|
| `thrust_min` | -1000.0 N | Empuje mínimo real |
| `thrust_max` | +1000.0 N | Empuje máximo real |
| `pos_min` | -π/2 rad | Ángulo mínimo thruster |
| `pos_max` | +π/2 rad | Ángulo máximo thruster |
| `command_timeout_sec` | 0.5 s | Timeout de seguridad |
| `send_positions` | true | Publicar posición de thruster |
| `debug_interval_sec` | 3.0 s | |

---

### 6.6 `metrics_logger`

**Archivo:** `vrx_experiment_benchmark/metrics_logger.py`  
**Frecuencia:** 10 Hz flush / 20 Hz recepción

Registra tres archivos CSV por cada run:

1. **`_timeseries.csv`** — estado completo a 20 Hz (posición, velocidad, CTE, thrusters, etc.)
2. **`_events.csv`** — eventos discretos (waypoint_reached, hold_started, route_completed...)
3. **`_summary.csv`** — métricas agregadas actualizadas cada segundo

**Métricas calculadas:**

| Métrica | Descripción |
|---|---|
| `rms_cte` | RMS del error de pista lateral [m] |
| `mean_speed` | Velocidad media efectiva [m/s] |
| `efficiency` | `dist_directo / dist_recorrida` ∈ [0,1] |
| `completed` | 1 si la ruta fue completada |

**Ruta de salida (hardcoded en launch):**
```
src/vrx_experiment_benchmark/metrics/raw/
src/vrx_experiment_benchmark/metrics/summary/
```

**Formato del run_tag:** `{route_name}_{YYYYMMDD}_{HHMMSS}`

---

### 6.7 `route_utils` (librería compartida)

**Archivo:** `vrx_experiment_benchmark/route_utils.py`

Funciones matemáticas/geográficas compartidas entre todos los nodos:

| Función | Descripción |
|---|---|
| `wrap_angle(a)` | Normaliza ángulo a [-π, π] |
| `unwrap_angle(new, prev)` | Desenvuelve ángulo continuo |
| `quaternion_to_yaw(x,y,z,w)` | Extrae yaw de cuaternión |
| `geodetic_to_local_xy(lat,lon,lat0,lon0)` | Convierte GPS a XY local [m] |
| `segment_yaw(x0,y0,x1,y1)` | Rumbo de un segmento |
| `distance_xy(x0,y0,x1,y1)` | Distancia Euclidiana 2D |
| `load_route_yaml(path)` | Carga YAML de ruta |
| `compute_local_route_from_latlon(...)` | Convierte ruta GPS a XY local |
| `MODE_TO_CODE` | `{"start":0, "transit":1, "finish":2}` |
| `CODE_TO_MODE` | `{0:"start", 1:"transit", 2:"finish"}` |

> **Regla:** toda función matemática o geográfica nueva debe agregarse aquí, nunca inline en los nodos.

---

## 7. Definición de Rutas (YAML)

### Formato de un archivo de ruta

```yaml
route_id: route_straight     # Identificador único
frame_id: map_local          # Frame de referencia

waypoints:
  - id: 0
    lat: -33.72252784760515  # Latitud WGS84
    lon: 150.6739565151321   # Longitud WGS84
    mode: start              # "start" | "transit" | "finish"
    pos_tolerance: 3.0       # Radio de captura [m]

  - id: 1
    lat: -33.72238429782643
    lon: 150.6739929169588
    mode: transit
    pos_tolerance: 3.0
  # ... más waypoints ...
  - id: 9
    lat: -33.72112120852503
    lon: 150.6742753157646
    mode: finish
    pos_tolerance: 3.0
```

**Reglas:**
- El **primer WP** debe ser `mode: start`.
- El **último WP** debe ser `mode: finish`.
- Los WPs intermedios son `mode: transit`.
- Solo puede haber **un** WP con `mode: start` y **uno** con `mode: finish`.

### Rutas disponibles

| Ruta | WPs | Longitud aprox. | Geometría |
|---|---|---|---|
| `route_straight.yaml` | 10 | ~159 m | Recta a 80° aprox. |
| `route_curves.yaml` | 10 | ~186 m | Curva en S extendida |
| `route_zigzag.yaml` | 10 | ~120 m | Zigzag, segm. ~14 m |

---

## 8. Resultados de Referencia — Baseline LOS+PD

Condiciones: entorno sin viento ni corriente (calm), limite de thruster ±1000 N.

| Ruta | Tiempo | Dist. recorrida | Eficiencia | Mean speed | RMS CTE | Completada |
|---|---|---|---|---|---|---|
| `route_straight` | 143.7 s | 193.1 m | **0.950** | 1.53 m/s | 4.23 m | ✅ |
| `route_curves` | ~139 s | ~224 m | **0.800** | 1.78 m/s | **3.11 m** | ✅ |
| `route_zigzag` | ~115 s* | — | ~0.920 | ~1.62 m/s | ~7.3 m | ⚠️* |

> \* El zigzag fue interrumpido antes del hold final, pero capturaba todos los WPs correctamente.
> El RMS CTE elevado se debe a la inercia a alta velocidad en segmentos de 14 m.

### Observaciones clave:

- El limitante de RMS CTE en `route_straight` es el **desvío lateral inicial** durante el hold del WP0 start (el barco llega desde un ángulo y el LOS corrige la desviación en los primeros 2-3 WPs).
- En `route_curves`, el CTE de ~3-4 m en giros pronunciados es inherente a la velocidad de 4 m/s y la inercia del WAM-V.
- El **speed cap adaptativo** por longitud de segmento (`transit_speed × seg_len / 20.0` si `seg_len < 20 m`) mejora la captura de WPs en el zigzag.

---

## 9. Cómo Implementar un Nuevo Algoritmo de Guiado

Para reemplazar `guidance_los` por MPPI, RL residual u otro algoritmo:

### 9.1 Requisitos mínimos del nuevo nodo

```python
# Suscripciones requeridas:
/wamv/navigation/state2d              → nav_msgs/Odometry
/wamv/navigation/active_waypoint_meta → std_msgs/Float64MultiArray (15 campos, ver §5.1)

# Publicación requerida:
/wamv/control/guidance_cmd → std_msgs/Float64MultiArray (12 campos, ver §5.2)

# Al recibir data[14] == 1 (route_completed):
# Publicar u_ref=0.0, psi_ref=yaw_actual para parada suave.
```

### 9.2 Contrato de modo

```python
from vrx_experiment_benchmark.route_utils import CODE_TO_MODE

mode_code = int(data[1])    # 0=start, 1=transit, 2=finish
mode = CODE_TO_MODE[mode_code]
```

- **`start`** (idx=0): aceleración desde cero hacia el primer WP. Heading = bearing directo.
- **`transit`** (idx 1..N-2): algoritmo libre. El `route_manager` evalúa avance por tolerancia o proyección.
- **`finish`** (idx=N-1): desaceleración y hold. Heading = bearing directo. `u_ref` bajo.

### 9.3 Añadir entrypoint en setup.py

```python
# En entry_points → console_scripts:
'guidance_mppi = vrx_experiment_benchmark.guidance_mppi:main',
```

### 9.4 Añadir al launch

En `route_following_pd.launch.py`, reemplazar o condicionar el nodo `guidance_los_node` con el
nuevo nodo usando el argumento `guidance_mode`:

```python
if LaunchConfiguration("guidance_mode").perform(context) == "mppi":
    # nodo guidance_mppi
else:
    # nodo guidance_los (actual)
```

### 9.5 Para MPPI específicamente

El `mppi_config.yaml` ya existe como placeholder en `config/`. Completarlo con los parámetros
propios (horizonte, número de muestras, lambda de temperatura, etc.) y referenciarlo en el launch.

---

## 10. Cómo Implementar un Nuevo Controlador (reemplazar controller_pd)

Si el nuevo algoritmo produce directamente comandos de thruster (no psi_ref/u_ref):

1. Suscribirse a `/wamv/navigation/active_waypoint_meta` y `/wamv/navigation/state2d`.
2. Publicar directamente a `/wamv/control/thruster_cmd` (formato §5.3).
3. El `thruster_commander` sigue siendo necesario como puente al simulador.
4. En ese caso, el nodo `guidance_los` y `controller_pd` no son necesarios.

---

## 11. Scripts de Utilidad

### `scripts/sync_config.sh`

Copia archivos `.yaml` y `.py` de `src/` al `install/` sin necesidad de `colcon build`.
**Usar siempre después de modificar datos de configuración o lógica de Python.**

```bash
bash src/vrx_experiment_benchmark/scripts/sync_config.sh
```

Copia:
- `config/*.yaml` → `install/.../share/.../config/`
- `config/routes/*.yaml` → `install/.../share/.../config/routes/`
- `vrx_experiment_benchmark/*.py` → `install/.../site-packages/vrx_experiment_benchmark/`

---

## 12. QoS y Convenciones de Mensajería

Todos los nodos usan el mismo perfil QoS:

```python
QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
```

**Convenciones de frecuencia:**

| Capa | Frecuencia | Rationale |
|---|---|---|
| state_estimator_2d | 20 Hz | Frecuencia del GPS RTK |
| route_manager | 10 Hz | Lógica de avance, no requiere alta frecuencia |
| guidance_los | 20 Hz | Misma frecuencia que el estado |
| controller_pd | 20 Hz | Loop de control |
| thruster_commander | 20 Hz | Publicación a actuadores |
| metrics_logger | 10 Hz flush | 20 Hz recepción interna |

---

## 13. Reglas de Desarrollo

1. **No modificar contratos de tópicos** sin actualizar todos los consumidores.
2. **No cambiar índices del `Float64MultiArray`** sin actualizar la tabla del §5.
3. Toda función matemática/geográfica nueva va en `route_utils.py`.
4. Los parámetros activos vienen del YAML (el launch los pasa). Los defaults en el código son solo fallback.
5. Siempre verificar que el valor activo viene del YAML (no hardcodeado en el nodo).
6. No usar `print()` en loops rápidos. Usar `get_logger().info()` con `debug_interval_sec`.
7. No introducir trabajo pesado dentro de callbacks de 10–20 Hz.
8. Preservar timeout de seguridad en `thruster_commander` (`command_timeout_sec = 0.5 s`).
9. Usar `sync_config.sh` tras cada cambio en `.py` o `.yaml` antes de relanzar.
10. Toda modificación debe verificarse extremo a extremo en la cadena de tópicos.

---

## 14. Archivos de Entorno (config/env/)

Contienen parámetros de condiciones ambientales (viento, olas, corrientes) que se aplican al
mundo de simulación. Se usan en el `experiment.launch.py` para pruebas de robustez.

---

*Documento base (LOS+PD) generado el 14 de abril de 2026.*  
*Sección LOS+MPPI añadida el 14 de abril de 2026.*  
*Para actualizarlo, ejecutar `sync_config.sh` y verificar que los valores coincidan con los YAMLs vigentes.*

---

---

# Stack LOS + MPPI — Documentación Completa

> **Versión:** abril 2026 · ROS 2 Humble · Gazebo Garden · WAM-V 16  
> **Propósito:** documentar la implementación del controlador MPPI (Model Predictive Path Integral)
> integrado con la capa de guiado LOS ya existente, como alternativa de alto rendimiento al PD baseline.  
> **Estado:** funcional y benchmarkeado. Candidato a superar el baseline LOS+PD en tiempo y CTE.

---

## 15. Descripción General — LOS+MPPI

El stack LOS+MPPI mantiene las mismas capas de estimación, manejo de ruta y guiado que el LOS+PD,
pero **reemplaza el controlador PD por un controlador MPPI** que planifica directamente comandos de
thruster (Newton) sin pasar por el lazo PD.

**Diferencia clave respecto al LOS+PD:**

| Aspecto | LOS+PD | LOS+MPPI |
|---|---|---|
| Capa de guiado | `guidance_los` | `guidance_los` (igual) |
| Capa de control | `controller_pd` | `controller_mppi` |
| Entrada del controlador | `psi_ref`, `u_ref` | `psi_ref`, `u_ref`, `e_ct`, `chi_p`, `wp_x/y`, meta completo |
| Salida del controlador | `(tl, tr)` via ganancias PD | `(tl, tr)` via optimización MPPI |
| Modelo interno | Ninguno (realimentación pura) | Modelo dinámico surrogado del WAM-V |
| Planificación | Reactiva (1 paso) | Horizonte H=22 pasos × dt=0.15s = 3.3s |

**Arquitectura por capas LOS+MPPI:**

```
GPS/IMU ──► [state_estimator_2d] ──► [route_manager] ──► [guidance_los]
                                                               │
                                                    /wamv/control/guidance_cmd
                                                               │
                                                               ▼
                                                    [controller_mppi]  ← también suscrito a
                                                          │              state2d y active_waypoint_meta
                                                          │
                                               /wamv/control/thruster_cmd
                                                          │
                                                          ▼
                                                 [thruster_commander]
                                                          │
                                             ┌────────────┴────────────┐
                                       left_thrust              right_thrust
```

---

## 16. Archivos del Stack LOS+MPPI

```
vrx_experiment_benchmark/
├── config/
│   └── mppi_config.yaml                    # Todos los parámetros del controlador MPPI
├── launch/
│   └── route_following_mppi.launch.py      # Launch principal del stack LOS+MPPI
└── vrx_experiment_benchmark/
    └── controller_mppi.py                  # Nodo ROS 2 del controlador MPPI
```

**Nodos compartidos con LOS+PD (sin cambio):**

- `state_estimator_2d.py` — estimación GPS+IMU
- `route_manager.py` — avance de waypoints
- `guidance_los.py` — cálculo de `psi_ref` y `u_ref`
- `thruster_commander.py` — puente al simulador VRX
- `metrics_logger.py` — registro de métricas CSV

---

## 17. Cómo Lanzar el Stack LOS+MPPI

```bash
# Sincronizar cambios de config (siempre antes de lanzar)
bash src/vrx_experiment_benchmark/scripts/sync_config.sh

# Lanzar con ruta específica
source install/setup.bash
ros2 launch vrx_experiment_benchmark route_following_mppi.launch.py route_name:=route_straight.yaml
ros2 launch vrx_experiment_benchmark route_following_mppi.launch.py route_name:=route_curves.yaml
ros2 launch vrx_experiment_benchmark route_following_mppi.launch.py route_name:=route_zigzag.yaml
```

**Argumentos del launch `route_following_mppi.launch.py`:**

| Argumento | Default | Descripción |
|---|---|---|
| `route_name` | `route_curves.yaml` | Nombre del YAML en `config/routes/` |
| `route_file` | `""` | Ruta absoluta opcional a cualquier YAML |
| `guidance_mode` | `"los"` | Siempre `"los"` — el MPPI es el controlador, no la guía |

---

## 18. Grafo de Tópicos — LOS+MPPI

```
/wamv/sensors/gps/gps/fix        [sensor_msgs/NavSatFix]
/wamv/sensors/imu/imu/data       [sensor_msgs/Imu]
         │
         ▼
[state_estimator_2d]
         │
         ├──► /wamv/navigation/state2d        [nav_msgs/Odometry]
         └──► /wamv/navigation/local_origin   [sensor_msgs/NavSatFix]
                     │
                     ▼
              [route_manager]
                     │
                     ├──► /wamv/navigation/route_local          [nav_msgs/Path]
                     ├──► /wamv/navigation/active_waypoint      [geometry_msgs/PointStamped]
                     ├──► /wamv/navigation/active_waypoint_meta [std_msgs/Float64MultiArray]
                     └──► /wamv/navigation/route_status         [std_msgs/Float64MultiArray]
                                   │
                                   ▼
                            [guidance_los]  ← suscrito a state2d y active_waypoint_meta
                                   │
                    /wamv/control/guidance_cmd [Float64MultiArray, 12 campos]
                                   │
                    ┌──────────────┴──────────────────────────┐
                    │    (también suscrito a state2d           │
                    ▼     y active_waypoint_meta)              │
             [controller_mppi]  ◄──────────────────────────────┘
                    │
                    ├──► /wamv/control/thruster_cmd   [Float64MultiArray, 4 campos]
                    └──► /wamv/control/controller_debug [Float64MultiArray, 12 campos]
                                   │
                                   ▼
                         [thruster_commander]
                                   │
                    ┌──────────────┼──────────────┐
                    ▼              ▼              ▼
             left/thrust    right/thrust    left/pos  right/pos
```

**El `controller_mppi` se suscribe a tres tópicos simultáneamente:**

| Tópico | Uso en MPPI |
|---|---|
| `/wamv/navigation/state2d` | Estado actual `(x, y, ψ, u, r)` para iniciar el rollout |
| `/wamv/control/guidance_cmd` | Obtiene `psi_ref`, `u_ref`, `e_ct`, `chi_p`, `wp_x/y`, `mode_code` |
| `/wamv/navigation/active_waypoint_meta` | Obtiene geometría del segmento `(seg_x0, seg_y0, seg_x1, seg_y1)` para el cómputo de CTE interno |

---

## 19. Nodo `controller_mppi`

**Archivo:** `vrx_experiment_benchmark/controller_mppi.py`  
**Entrypoint:** `controller_mppi` (definido en `setup.py`)  
**Frecuencia:** 20 Hz  
**Clase:** `ControllerMPPI(Node)`

### 19.1 Algoritmo General (MPPI)

MPPI (Model Predictive Path Integral) es un controlador de horizonte rodante basado en muestreo
estocástico. En cada ciclo de control:

1. **Muestreo:** genera `N=256` trayectorias perturbando el plan nominal actual con ruido gaussiano
   correlacionado en el tiempo.
2. **Rollout:** propaga cada trayectoria `H=22` pasos adelante usando el modelo surrogado del WAM-V.
3. **Evaluación:** calcula el costo acumulado de cada trayectoria (stage cost + terminal cost).
4. **Promedio ponderado:** actualiza el plan nominal con promedio ponderado por `exp(-J/λ)`.
5. **Ejecución:** aplica el primer paso del plan nominal actualizado como comando de actuador.

### 19.2 Suscripciones y Publicaciones

**Suscripciones:**
- `/wamv/navigation/state2d` → `nav_msgs/Odometry`
- `/wamv/control/guidance_cmd` → `std_msgs/Float64MultiArray` (12 campos)
- `/wamv/navigation/active_waypoint_meta` → `std_msgs/Float64MultiArray` (15 campos)

**Publicaciones:**
- `/wamv/control/thruster_cmd` → `std_msgs/Float64MultiArray` (4 campos: L, R, pos_L, pos_R)
- `/wamv/control/controller_debug` → `std_msgs/Float64MultiArray` (12 campos, ver §19.5)

### 19.3 Modelo Surrogado del WAM-V

El modelo dinámico utilizado en el rollout MPPI es un modelo planar de 2º orden saturado:

```
── Dinámica de surge (avance) ──────────────────────────────────────────
du/dt = (F_common / m_eff) - (b_u_lin / m_eff)·u - (b_u_quad / m_eff)·|u|·u
du/dt = clamp(du/dt, ±surge_accel_limit)

── Dinámica de yaw (rotación) ──────────────────────────────────────────
F_common      = tl + tr          (empuje neto de avance)
F_differential = tr - tl         (par de giro)

dr/dt = (l·F_differential / Iz_eff) - (b_r_lin / Iz_eff)·r - (b_r_quad / Iz_eff)·|r|·r
dr/dt = clamp(dr/dt, ±yaw_accel_limit)

── Integración Euler (un paso dt) ──────────────────────────────────────
u_k+1  = clamp(u_k + dt·du/dt,  ±prediction_speed_limit)
r_k+1  = clamp(r_k + dt·dr/dt,  ±prediction_yaw_rate_limit)
ψ_k+1  = wrap(ψ_k + dt·r_k+1)
x_k+1  = x_k + dt·u_k+1·cos(ψ_k+1)
y_k+1  = y_k + dt·u_k+1·sin(ψ_k+1)
```

**Parámetros del modelo surrogado (mppi_config.yaml):**

| Parámetro | Valor | Descripción |
|---|---|---|
| `mass_eff` | 260.0 kg | Masa efectiva de surge |
| `iz_eff` | 270.0 kg·m² | Inercia efectiva de yaw |
| `thruster_half_spacing_m` | 0.98 m | Semi-separación de thrusters (b = 0.98 m) |
| `surge_drag_linear` | 100.0 | Coeficiente de arrastre lineal en surge |
| `surge_drag_quadratic` | 150.0 | Coeficiente de arrastre cuadrático en surge |
| `yaw_drag_linear` | 800.0 | Coeficiente de arrastre lineal en yaw |
| `yaw_drag_quadratic` | 800.0 | Coeficiente de arrastre cuadrático en yaw |
| `surge_accel_limit` | 2.5 m/s² | Límite de aceleración de surge por paso |
| `yaw_accel_limit` | 2.8 rad/s² | Límite de aceleración de yaw por paso |
| `prediction_speed_limit` | 6.5 m/s | Velocidad máxima en predicción |
| `prediction_yaw_rate_limit` | 2.0 rad/s | Yaw rate máximo en predicción |

### 19.4 Función de Costo

El costo total de una trayectoria es `J = Σ(t=0..H-1) stage_cost(t) + terminal_cost(H)`.

**Stage cost (por paso):**

```
J_stage = w_cte        · e_ct²
        + w_heading    · e_ψ²
        + w_speed      · (u - u_ref)²
        + w_yaw_rate   · r²
        + w_wp_dist    · (dist_wp / (pos_tol + 2.0))²
        - w_progress   · progress            ← premio (negativo = recompensa)
        + w_control    · (tl² + tr²) / T²
        + w_control_delta · (Δtl² + Δtr²) / T²
        + w_saturation · sat_pen             ← penaliza uso > 92% de saturación
        + w_reverse    · reverse_pen         ← penaliza thrust negativo

donde:
  e_ct     = -sin(chi_p)·(x - seg_x0) + cos(chi_p)·(y - seg_y0)  [CTE en segmento]
  e_ψ      = wrap(psi_ref - ψ)
  progress = max(0, dist_wp0 - dist_wp_actual)
  T        = max_thrust (normalización)
```

**Terminal cost (estado final del horizonte):**

```
J_terminal = terminal_multiplier × (w_cte·e_ct² + w_heading·e_ψ² + 0.5·w_wp_dist·dist_norm² + 0.5·w_speed·u²)
```

**Pesos en modo transit (mppi_config.yaml, valores activos):**

| Parámetro | Valor | Descripción |
|---|---|---|
| `w_cte` | 32.0 | Peso del error de pista lateral |
| `w_heading` | 20.0 | Peso del error de heading |
| `w_speed` | 8.0 | Peso del error de velocidad |
| `w_yaw_rate` | 4.5 | Peso de yaw rate (amortiguación de oscilación) |
| `w_progress` | 3.0 | Premio por progreso hacia el waypoint |
| `w_wp_distance` | 1.0 | Peso de distancia al waypoint |
| `w_control` | 0.010 | Penalización magnitud de control |
| `w_control_delta` | 0.28 | Penalización de cambio de control |
| `w_saturation` | 42.0 | Penalización de saturación (>92% del máx.) |
| `w_reverse` | 1.6 | Penalización de thrust negativo |
| `terminal_multiplier` | 4.5 | Multiplicador del costo terminal |

**Pesos en modo hold (start/finish):**

| Parámetro | Valor | Descripción |
|---|---|---|
| `w_cte_hold` | 14.0 | CTE durante hold |
| `w_heading_hold` | 28.0 | Heading durante hold (mayor para mantener orientación) |
| `w_speed_hold` | 18.0 | Penaliza velocidad residual durante hold |
| `w_yaw_rate_hold` | 18.0 | Fuerte amortiguación de yaw durante hold |
| `w_progress_hold` | 1.0 | Premio mínimo (no debe avanzar) |
| `w_wp_distance_hold` | 10.0 | Mantenerse cerca del WP |
| `w_control_hold` | 0.014 | Penalización control (ligeramente mayor) |
| `w_control_delta_hold` | 0.70 | Penaliza cambios bruscos durante hold |
| `w_saturation_hold` | 70.0 | Fuerte penalización de saturación durante hold |
| `w_reverse_hold` | 3.0 | Fuerte penalización de thrust negativo |
| `terminal_multiplier_hold` | 6.5 | Costo terminal más agresivo para estabilizar |

### 19.5 Parámetros Core MPPI (mppi_config.yaml, valores activos)

| Parámetro | Valor | Descripción |
|---|---|---|
| `horizon_steps` | 22 | Pasos de horizonte H |
| `dt` | 0.15 s | Paso de integración (horizonte = 3.3 s) |
| `num_samples` | 256 | Trayectorias muestreadas por ciclo |
| `lambda_temp` | 13.0 | Temperatura MPPI (mayor = más democrático entre trayectorias) |
| `sample_smoothing` | 0.82 | Correlación temporal del ruido (AR-1); mayor = trayectorias más suaves |
| `noise_sigma_left` | 105.0 N | Desviación estándar del ruido de exploración (thruster izquierdo, modo transit) |
| `noise_sigma_right` | 105.0 N | Desviación estándar del ruido (thruster derecho, modo transit) |
| `noise_sigma_left_hold` | 30.0 N | Ruido reducido durante hold |
| `noise_sigma_right_hold` | 30.0 N | Ruido reducido durante hold |
| `max_delta_thrust_per_step` | 110.0 N | Rate limiter: máximo cambio de thrust entre pasos |
| `warm_start_with_last_cmd` | true | Inicializa el plan nominal con el último comando |
| `random_seed` | 7 | Semilla para reproducibilidad |

**Ruido con correlación temporal (proceso AR-1):**

```python
noise_t = α · noise_{t-1} + sqrt(1 - α²) · gauss(0, σ)
# α = sample_smoothing = 0.82
# → ruido suave y temporalmente coherente; reduce reversales abruscos
```

### 19.6 Restricciones de Actuador

| Parámetro | Valor | Descripción |
|---|---|---|
| `min_thrust` | -1000.0 N | Límite inferior absoluto |
| `max_thrust` | +1000.0 N | Límite superior absoluto |
| `reverse_thrust_scale` | 0.85 | El thrust negativo se escala a `0.85 × min_thrust` |
| `finish_brake_distance` | 4.0 m | Cuando dist < 4m en modo finish, el costo de distancia se amplifica ×1.4 |
| `completion_zero_u_ref` | 0.05 m/s | Si u_ref ≤ 0.05 y en hold, aplica comando cero |

### 19.7 Detección de Modo (Transit vs Hold)

```python
# "hold_mode" = modo start (code=0) o finish (code=2)
def is_hold_mode(mode_code):
    return mode_code in (0, 2)
```

En modo hold se usan los pesos `*_hold` (más restrictivos) y `sigma_hold` (menos exploración)
para garantizar comportamiento suave en los puntos de inicio y llegada.

### 19.8 Warm-Start del Plan Nominal

Al iniciar (last_cmd ≈ 0 y u_ref > 0), el plan nominal se inicializa con un thrust base:

```python
base = mass_eff × max(u_ref, 0) × 0.45
if hold_mode: base *= 0.70
base = min(base, 0.55 × max_thrust)
# → Evita que el MPPI parta de un plan de "no moverse"
```

Tras cada ciclo, el horizonte se desplaza un paso (shift):
```python
nominal[t] ← nominal[t+1]  para t = 0..H-2
nominal[H-1] ← nominal[H-2]  (último paso se repite)
```

### 19.9 Log de Debug

**Mensaje de inicio:**
```
controller_mppi started | samples=256 horizon=22 dt=0.15 max_thrust=1000 lambda=13.0 sigma_L/R=105/105 rate=20Hz
```

**Log periódico (cada `debug_interval_sec=3.0s`):**
```
MPPI[m:1]|psi:1.42/1.37(e:0.05,r:0.12)|u:3.5|c:710.3,d:45.2|th:665.1,755.5|e_ct:-0.3|J:142.7
          │    │    │    │    │    │        │    │      │       │     │          │          └── costo best J
          │    │    │    │    │    │        │    │      │       │     └── th_R   └── CTE [m]
          │    │    │    │    │    │        │    │      └── diferencial └── th_L
          │    │    │    │    │    │        │    └── empuje común [N]
          │    │    │    │    │    └── yaw_rate [rad/s]
          │    │    │    │    └── heading_error [rad]
          │    │    │    └── yaw actual [rad]
          │    │    └── psi_ref [rad]
          │    └── modo (0=start, 1=transit, 2=finish)
          └── prefijo del nodo
```

**Tópico de debug `/wamv/control/controller_debug` (12 campos):**

| Índice | Campo | Descripción |
|---|---|---|
| `[0]` | `psi_ref` | Heading deseado del LOS [rad] |
| `[1]` | `yaw` | Heading actual [rad] |
| `[2]` | `yaw_error` | `wrap(psi_ref - yaw)` [rad] |
| `[3]` | `yaw_rate` | Velocidad angular actual [rad/s] |
| `[4]` | `u_ref` | Velocidad de avance deseada [m/s] |
| `[5]` | `common` | `0.5·(tl + tr)` — empuje medio [N] |
| `[6]` | `differential` | `0.5·(tr - tl)` — diferencial [N] |
| `[7]` | `left_thrust` | Comando thruster izquierdo [N] |
| `[8]` | `right_thrust` | Comando thruster derecho [N] |
| `[9]` | `e_ct` | Error de pista lateral [m] |
| `[10]` | `heading_error_from_guidance` | Error de heading del LOS [rad] |
| `[11]` | `mode_code` | Código de modo (0/1/2) |

---

## 20. Launch `route_following_mppi.launch.py`

**Nodos lanzados:**

| Nodo | Ejecutable | Config YAML |
|---|---|---|
| `state_estimator_2d` | vía `state_estimation.launch.py` | `state_estimator_2d.yaml` |
| `route_manager` | `route_manager` | `route_manager_config.yaml` |
| `guidance_los` | `guidance_los` | `los_config.yaml` |
| `controller_mppi` | `controller_mppi` | `mppi_config.yaml` |
| `thruster_commander` | `thruster_commander` | `controller_limits.yaml` |
| `metrics_logger` | `metrics_logger` | `metrics_config.yaml` |

**Diferencia con `route_following_pd.launch.py`:**

```diff
- controller_pd_node    ← executale: controller_pd, config: pd_config.yaml
+ controller_mppi_node  ← executable: controller_mppi, config: mppi_config.yaml
```

Todos los demás nodos y parámetros de tópicos son idénticos al launch PD.

**Ruta de métricas hardcodeada:**
```python
_METRICS_ROOT = "/home/misael/ros2_workspaces/diplomado_ws/src/vrx_experiment_benchmark/metrics"
```

---

## 21. Resultados de Referencia — LOS+MPPI

> Condiciones: entorno sin viento ni corriente (calm), límite de thruster ±1000 N.  
> Las métricas registradas son las del `metrics_logger` del run completo.

### 21.1 Mejores métricas observadas (configuración activa)

| Ruta | Tiempo | Dist. recorrida | Eficiencia | Mean speed | RMS CTE | Completada |
|---|---|---|---|---|---|---|
| `route_straight` | **~105–140 s** | ~200–210 m | ~0.88 | ~1.7–2.2 m/s | **~2.2 m** | ✅ |
| `route_curves` | **~140–170 s** | ~215–230 m | ~0.72–0.82 | ~1.7–2.0 m/s | **~2.1 m** | ✅ |

> **Nota:** El MPPI es un controlador estocástico. Los resultados varían run a run por el ruido
> ambiental del simulador (olas, viento). El RMS CTE es consistentemente mejor que el PD (~4.23m
> straight, ~3.11m curves), pero el tiempo de ruta presenta mayor varianza.

### 21.2 Comparativa con Baseline LOS+PD

| Métrica | LOS+PD | LOS+MPPI | Mejora |
|---|---|---|---|
| RMS CTE (straight) | 4.23 m | **~2.2 m** | -48% ✅ |
| RMS CTE (curves) | 3.11 m | **~2.1 m** | -32% ✅ |
| Tiempo (straight) | 143.7 s | ~105–140 s | Variable ⚠️ |
| Tiempo (curves) | ~139 s | ~140–170 s | Similar / peor ⚠️ |
| Eficiencia (straight) | 0.950 | ~0.88 | Ligeramente inferior |
| Eficiencia (curves) | 0.800 | ~0.72–0.82 | Similar |

**Conclusión:** El MPPI supera claramente al PD en seguimiento de trayectoria (CTE), pero la
planificación estocástica introduce varianza en el tiempo total de ruta, especialmente en rutas curvas.

---

## 22. Historial de Tuning — LOS+MPPI

El proceso de optimización de parámetros pasó por las siguientes iteraciones principales:

### Iteración 1 — Valores iniciales (conservadores)
- `sigma=120`, `w_yaw_rate=2.5`, `w_reverse=0.6`
- Resultado: comportamiento errático, alta exploración, comandos negativos frecuentes

### Iteración 2 — Ajuste de pesos de costo
- Subida de `w_cte`, `w_heading`, reducción de `lambda`
- Resultado: mejor seguimiento de ruta pero oscilación en yaw

### Iteración 3 — "Golden" baseline
- `sigma=120`, `w_yaw_rate=4.5`, `w_reverse=1.2`
- Resultado: straight ≈99s (mejor run), CTE≈2.2m. Alta varianza entre runs.

### Iteración 4 — Ajuste agresivo (revertido)
- `sigma=95`, `w_yaw_rate=6.0`, `w_reverse=2.5`
- Resultado: MPPI entra en mínimos locales en rutas curvas, tiempo curves ≈172s. **Revertido.**

### Iteración 5 — Configuración actual (activa)
- `sigma=105`, `sample_smoothing=0.82`, `w_reverse=1.6`
- Cambios respecto al golden: reducción moderada de sigma, correlación temporal aumentada,
  penalización de thrust negativo más firme
- Objetivo: reducir reversales abruptos de thrust sin bloquear la maniobrabilidad diferencial

### Parámetros que NO se deben modificar sin validación exhaustiva

| Parámetro | Motivo |
|---|---|
| `lambda_temp` < 8 | Con lambda muy bajo, el MPPI converge a una sola trayectoria y pierde robustez |
| `sigma` > 130 | Genera comandos extremos frecuentes (L:-800 / R:+900) que oscilan el barco |
| `sigma` < 90 | El muestreo es tan restrictivo que el MPPI queda atrapado en mínimos locales |
| `w_yaw_rate` > 6.0 | Causa que el MPPI penalice cualquier giro, generando oscillation loop |
| `w_reverse` > 2.5 | Bloquea el giro diferencial eficiente; el barco no puede girar rápido |
| `sample_smoothing` < 0.65 | Ruido no correlacionado → trayectorias con thrust reversales muy bruscos |
| `horizon_steps` < 15 | Horizonte corto → sin planificación anticipatoria, comportamiento reactivo |

---

## 23. Reglas de Desarrollo — LOS+MPPI

1. **Mismo sync_config.sh** que el PD: siempre ejecutar tras cambiar `.yaml` o `.py`.
2. **No cambiar contratos de tópicos**: el MPPI usa exactamente los mismos tópicos que el PD.
3. **No cambiar índices de `Float64MultiArray`**: los índices de `guidance_cmd` y `active_waypoint_meta`
   son compartidos entre stacks.
4. **Toda modificación de pesos de costo** debe hacerse en `mppi_config.yaml`, nunca en el código.
5. **Benchmarking**: comparar siempre contra las mismas rutas y condiciones que el LOS+PD para
   mantener la validez de las métricas comparativas.
6. **Alta varianza estocástica**: los resultados de un run único no son definitivos. Promediar al
   menos 3 runs por configuración antes de concluir mejora o regresión.
7. **Debug**: el tópico `/wamv/control/controller_debug` y el log `MPPI[m:X]|...` son las fuentes
   primarias de diagnóstico. No introducir `print()` en el loop de 20 Hz.
8. **Modelo surrogado**: los cambios en los parámetros del modelo (`mass_eff`, `iz_eff`, `drag*`)
   afectan la calidad de predicción y pueden causar comportamiento inesperado. Validar siempre
   en `route_straight` primero.

---

*Sección LOS+MPPI añadida el 14 de abril de 2026.*  
*Parámetros activos al momento de escritura: sigma=105, smoothing=0.82, w_reverse=1.6, w_yaw_rate=4.5.*

---

---

# Stack LOS + MPPI + RL Residual (TD3+BC) — Documentación Completa

> **Versión:** abril 2026 · ROS 2 Humble · Gazebo Garden · WAM-V 16  
> **Propósito:** documentar la implementación del controlador residual de Aprendizaje por Refuerzo
> Offline (TD3+BC) que se sobrepone al stack LOS+MPPI para reducir el error de pista lateral.  
> **Estado (v4 — activo):** entrenado con recompensa cuadrática de CTE y bonus de velocidad.
> Benchmark completo (45 runs, 3 rutas × 5 entornos): RMS CTE tránsito −5.4% vs MPPI,
> −11.7% en recta, −9.6% en curvas. Suavidad de control −18.8% vs MPPI. Consistencia 9× en recta.

---

## 24. Descripción General — LOS+MPPI+RL

El stack LOS+MPPI+RL **añade una capa de política aprendida sobre el LOS+MPPI existente**.
El controlador MPPI genera comandos de thrust base; el nodo `controller_rl_residual` intercepta
esos comandos y añade un **residual diferencial** calculado por una red neuronal política entrenada
con TD3+BC (Twin Delayed DDPG + Behavioral Cloning) offline.

**Idea clave — Residual Policy:**

```
thruster_final = thruster_MPPI + α · π_RL(estado)
```

donde `π_RL` es una red neuronal que aprendió cuándo y cuánto desviarse del MPPI para minimizar
el CTE a largo plazo. El factor `α` es **dinámico por modo de navegación** (ver §27.4).

**Diferencia clave respecto al LOS+MPPI:**

| Aspecto | LOS+MPPI | LOS+MPPI+RL |
|---|---|---|
| Capa de control | `controller_mppi` | `controller_mppi` + `controller_rl_residual` |
| Tipo de control | Heurístico optimizado | Heurístico + política aprendida |
| Adaptación online | No | No (offline RL) |
| Dataset requerido | No | Sí (130K+ transiciones de runs MPPI) |
| Entrenamiento | No | TD3+BC (PyTorch, CUDA opcional) |
| Modelo exportado | No | TorchScript `.ts` (cargado en ROS 2) |

**Arquitectura por capas LOS+MPPI+RL:**

```
GPS/IMU ──► [state_estimator_2d] ──► [route_manager] ──► [guidance_los]
                                                                │
                                               /wamv/control/guidance_cmd
                                                                │
                                                                ▼
                                                     [controller_mppi]
                                                                │
                                               /wamv/control/thruster_cmd_raw  ← nuevo tópico
                                                                │
                                                                ▼
                                                  [controller_rl_residual]  ← NUEVA CAPA
                                                                │              (suscrito a state2d,
                                                                │               guidance_cmd,
                                                                │               active_waypoint_meta,
                                                                │               thruster_cmd_raw)
                                               /wamv/control/thruster_cmd   ← salida final
                                                                │
                                                                ▼
                                                     [thruster_commander]
```

> **Tópico intermedio clave:** `controller_mppi` ahora publica en
> `/wamv/control/thruster_cmd_raw` (en lugar de `/wamv/control/thruster_cmd`).
> El `controller_rl_residual` lee ese raw, le suma el residual, y publica
> el comando final en `/wamv/control/thruster_cmd`.

---

## 25. Archivos del Stack LOS+MPPI+RL

```
vrx_experiment_benchmark/
├── config/
│   ├── rl_residual_config.yaml              # Parámetros del nodo controller_rl_residual
│   └── rl_models/
│       ├── mppi_rl_residual_policy.ts       # Modelo TorchScript para inferencia ROS 2
│       ├── mppi_rl_residual_policy.pt       # Checkpoint PyTorch (para re-entrenamiento)
│       └── norm_stats.pt                    # Estadísticas de normalización del estado
├── launch/
│   └── route_following_mppi_rl.launch.py   # Launch principal del stack LOS+MPPI+RL
├── metrics/
│   └── rl_dataset/
│       ├── raw/                             # CSVs rawde episodios (input al entrenador)
│       ├── splits/                          # Manifiestos train/val por episodio
│       └── training/
│           ├── training_history.csv         # Métricas de cada step de entrenamiento
│           └── training_summary.txt         # Resumen de hiperparámetros y resultado final
└── vrx_experiment_benchmark/
    ├── controller_rl_residual.py            # Nodo de inferencia y mezcla de residual
    └── train_rl_residual.py                 # Script de entrenamiento TD3+BC offline
```

**Nodos compartidos sin cambio:**
`state_estimator_2d`, `route_manager`, `guidance_los`, `controller_mppi`, `thruster_commander`,
`metrics_logger`.

---

## 26. Cómo Lanzar el Stack LOS+MPPI+RL

```bash
# Rebuild (solo si se modificaron archivos Python o setup.py)
cd ~/ros2_workspaces/diplomado_ws
colcon build --packages-select vrx_experiment_benchmark
source install/setup.bash

# Lanzar con política aprendida activa
ros2 launch vrx_experiment_benchmark route_following_mppi_rl.launch.py \
  route_name:=route_straight.yaml \
  use_learned_policy:=true \
  shadow_mode:=false \
  record_dataset:=false

# Lanzar con shadow mode (RL calcula pero no actúa — para diagnóstico)
ros2 launch vrx_experiment_benchmark route_following_mppi_rl.launch.py \
  route_name:=route_curves.yaml \
  use_learned_policy:=true \
  shadow_mode:=true \
  record_dataset:=false

# Lanzar para recolectar dataset (política heurística activa)
ros2 launch vrx_experiment_benchmark route_following_mppi_rl.launch.py \
  route_name:=route_straight.yaml \
  use_learned_policy:=false \
  shadow_mode:=false \
  record_dataset:=true
```

**Argumentos del launch `route_following_mppi_rl.launch.py`:**

| Argumento | Default | Descripción |
|---|---|---|
| `route_name` | `route_curves.yaml` | Nombre del YAML en `config/routes/` |
| `use_learned_policy` | `false` | `true` = política RL activa; `false` = MPPI puro |
| `shadow_mode` | `false` | `true` = RL calcula pero no modifica los comandos |
| `record_dataset` | `false` | `true` = guarda transiciones en `metrics/rl_dataset/raw/` |

---

## 27. Nodo `controller_rl_residual`

**Archivo:** `vrx_experiment_benchmark/controller_rl_residual.py`  
**Entrypoint:** `controller_rl_residual` (en `setup.py`)  
**Frecuencia:** 20 Hz  
**Clase:** `ControllerRLResidual(Node)`

### 27.1 Lógica de Operación

```
1. Recibe comando raw del MPPI: (tl_raw, tr_raw) en /wamv/control/thruster_cmd_raw
2. Construye vector de estado x ∈ ℝ²⁰ (ver §27.3)
3. Normaliza x usando norm_stats.pt (media y std por componente)
4. Evalúa la política: (a_c, a_d) = π(x) ∈ [-1, 1]²
5. Convierte a thrust:
     delta_c = a_c × max_residual_common_transit  [N]
     delta_d = a_d × max_residual_diff_transit    [N]
6. Aplica residual con blending:
     th_L = clip(tl_raw + alpha × (delta_c - delta_d), ±max_total_thrust)
     th_R = clip(tr_raw + alpha × (delta_c + delta_d), ±max_total_thrust)
7. Aplica rate limiter: |Δth| ≤ max_delta_residual_per_step por ciclo
8. Publica (th_L, th_R) en /wamv/control/thruster_cmd
```

En **shadow mode** (`shadow_mode=true`), el paso 8 publica `(tl_raw, tr_raw)` sin modificar.

### 27.2 Suscripciones y Publicaciones

**Suscripciones:**
- `/wamv/navigation/state2d` → `nav_msgs/Odometry`
- `/wamv/control/guidance_cmd` → `Float64MultiArray` (12 campos, §5.2)
- `/wamv/navigation/active_waypoint_meta` → `Float64MultiArray` (15 campos, §5.1)
- `/wamv/control/thruster_cmd_raw` → `Float64MultiArray` (4 campos)
- `/wamv/control/controller_debug_raw` → `Float64MultiArray` (debug del MPPI)

**Publicaciones:**
- `/wamv/control/thruster_cmd` → `Float64MultiArray` (4 campos: comando final)

### 27.3 Vector de Estado de la Política (20 componentes)

| Índice | Componente | Fuente | Descripción |
|---|---|---|---|
| `[0]` | `e_ct` | guidance_cmd[2] | Error de pista lateral [m] |
| `[1]` | `e_ct_prev` | memoria | CTE del ciclo anterior [m] |
| `[2]` | `heading_error` | guidance_cmd[6] | `psi_ref - yaw` [rad] |
| `[3]` | `yaw_rate` | state2d | `ω_z` [rad/s] |
| `[4]` | `u` | state2d | Velocidad de avance [m/s] |
| `[5]` | `u_ref` | guidance_cmd[1] | Velocidad deseada [m/s] |
| `[6]` | `vx` | state2d | Vel. en X local [m/s] |
| `[7]` | `vy` | state2d | Vel. en Y local [m/s] |
| `[8]` | `delta_c_raw` | thruster_cmd_raw | Empuje común del MPPI [N] (normalizado) |
| `[9]` | `delta_d_raw` | thruster_cmd_raw | Diferencial del MPPI [N] (normalizado) |
| `[10]` | `mode_start` | active_waypoint_meta | 1.0 si mode=start, si no 0.0 |
| `[11]` | `mode_transit` | active_waypoint_meta | 1.0 si mode=transit, si no 0.0 |
| `[12]` | `mode_finish` | active_waypoint_meta | 1.0 si mode=finish, si no 0.0 |
| `[13]` | `dist_to_wp` | guidance_cmd[4] | Distancia al waypoint activo [m] |
| `[14]` | `chi_p` | guidance_cmd[3] | Rumbo del segmento activo [rad] |
| `[15]` | `psi_ref` | guidance_cmd[0] | Heading deseado [rad] |
| `[16]` | `yaw` | state2d | Heading actual [rad] |
| `[17]` | `e_ct_rate` | calculado | `(e_ct - e_ct_prev) / dt` [m/s] |
| `[18]` | `sin_heading_err` | calculado | `sin(heading_error)` |
| `[19]` | `cos_heading_err` | calculado | `cos(heading_error)` |

### 27.4 Parámetros de Inferencia (rl_residual_config.yaml)

| Parámetro | Valor activo | Descripción |
|---|---|---|
| `enable_residual` | `true` | Activa la capa RL |
| `shadow_mode` | `false` | Si true, no modifica los comandos |
| `use_learned_policy` | **`true`** | Política RL activa (⚠️ cambiar a `false` para MPPI puro) |
| `policy_format` | `torchscript` | Formato del modelo cargado |
| `policy_path` | `mppi_rl_residual_policy.ts` | Nombre del archivo en `config/rl_models/` |
| `policy_device` | `cpu` | Dispositivo de inferencia (`cpu` o `cuda`) |
| `policy_input_dim` | `20` | Dimensión del vector de estado |
| `policy_hidden_dim` | `128` | Neuronas en capas ocultas del actor |
| `alpha_residual` | `0.50` | Fallback genérico (si no aplica ningún modo específico) |
| `alpha_residual_transit` | `0.50` | Blending en modo transit (tránsito normal) |
| `alpha_residual_hold` | `0.20` | Blending en modo start/finish (baja autoridad) |
| `alpha_residual_zigzag_diff` | `0.60` | Mayor autoridad diferencial en segmentos cortos |

> **Alpha dinámico por modo (implementado en v4):** el `controller_rl_residual` selecciona el
> factor `α` en función de `mode_code` y `seg_len`. En tránsito largo usa `alpha_transit=0.50`,
> en hold/start/finish usa `alpha_hold=0.20`, y en zigzag (seg_len < umbral) usa
> `alpha_zigzag_diff=0.60` para potenciar la corrección diferencial sin necesidad de reentrenar.

### 27.5 Límites de Seguridad del Residual

| Parámetro | Valor | Descripción |
|---|---|---|
| `max_residual_common_transit` | 120.0 N | Límite de delta_c en modo transit |
| `max_residual_diff_transit` | 150.0 N | Límite de delta_d en modo transit |
| `max_residual_common_hold` | 60.0 N | Límite de delta_c en modo start/finish |
| `max_residual_diff_hold` | 80.0 N | Límite de delta_d en modo start/finish |
| `max_delta_residual_per_step` | 40.0 N | Rate limiter: máx. cambio de residual por ciclo |
| `max_total_thrust` | 1000.0 N | Límite total positivo (igual al MPPI) |
| `min_total_thrust` | -1000.0 N | Límite total negativo |

### 27.6 Log de Debug del Nodo

**Mensaje de inicio:**
```
controller_rl_residual started | mode=learned:torchscript shadow=False alpha=0.50 rate=20Hz
  cmd_raw=/wamv/control/thruster_cmd_raw cmd_out=/wamv/control/thruster_cmd
```

**Log periódico en operación (cada `debug_interval_sec`):**
```
RL[m:1:lea]|psi:1.42/1.37(e:0.05,r:0.12)|u:3.5|c:850.0,d:45.0|raw:812.0,954.0|res:10.2,-5.8|th:828.0,962.0|e_ct:-0.3
      │    │   │    │    │    │    │       │    │     │       │     │            │              └── CTE [m]
      │    │   │    │    │    │    │       │    │     │       │     └────── th_L, th_R finales [N]
      │    │   │    │    │    │    │       │    │     │       └──────────── residual (delta_c, delta_d) [N]
      │    │   │    │    │    │    │       │    │     └──────────────────── raw MPPI (tl_raw, tr_raw) [N]
      │    │   │    │    │    │    │       │    └────────────────────────── common, differential [N]
      │    │   │    │    │    │    │       └────────────────────────────── u_actual [m/s]
      │    │   │    │    │    │    └───────────────────────────────────── yaw_rate [rad/s]
      │    │   │    │    │    └──────────────────────────────────────── heading_error [rad]
      │    │   │    │    └─────────────────────────────────────────── yaw actual [rad]
      │    │   │    └──────────────────────────────────────────────── psi_ref [rad]
      │    │   └───────────────────────────────────────────────────── modo (0/1/2)
      │    └───────────────────────────────────────────────────────── sub-modo (lea/sha/heu/don)
      └────────────────────────────────────────────────────────────── prefijo del nodo
```

**Sub-modos del log:**
| Código | Significado |
|---|---|
| `lea` | Política aprendida activa |
| `sha` | Shadow mode (calcula pero no actúa) |
| `heu` | Heurístico de bootstrap (sin política cargada) |
| `don` | Ruta completada, thrusters a cero |

---

## 28. Pipeline de Entrenamiento TD3+BC

### 28.1 Concepto: RL Offline

El entrenamiento es **completamente offline**: no requiere simulación durante el entrenamiento.
El dataset se recolectó previamente ejecutando el stack LOS+MPPI con `record_dataset:=true`.
El algoritmo TD3+BC (Fujimoto & Gu, 2021) aprende una política que:

1. **Imita** el comportamiento del MPPI (término BC — Behavioral Cloning).
2. **Optimiza** el Q-value para maximizar la reducción acumulada de CTE (término TD3).
3. **Penaliza** residuales innecesarios para forzar selectividad (término de regularización de acción).

**Fórmula de pérdida del actor:**

```
L_actor = -λ · Q(s, π(s)) + MSE(π(s), a_dataset)

donde:
  λ = td3bc_alpha / E[|Q(s, a_dataset)|]   ← normalización dinámica
  MSE = error cuadrático respecto a la acción del MPPI en el dataset
```

### 28.2 Diseño de la Recompensa (v4 — Cuadrática)

```
r_t = - w_cte  · (CTE_{t+1} / cte_norm)²       ← penalización cuadrática (gradiente proporcional)
      + w_speed · 1[speed ≥ 0.8 · u_ref]        ← bonus de velocidad (0.15 si mantiene ≥80% u_ref)
      ) · (1 / reward_scale)
```

| Componente | Peso activo | Descripción |
|---|---|---|
| `reward_cte_weight` | 1.0 | Peso de la penalización cuadrática |
| `reward_cte_norm` | 2.0 m | Distancia de normalización: (CTE/2.0)² |
| `reward_speed_weight` | 0.15 | Bonus cuando speed ≥ 0.8 × u_ref |
| `reward_scale` | **22.0** | Escala para que Q_escalado ≈ −10.5 → λ ≈ 0.29 |

> **Por qué `reward_scale=22`:** con CTE cuadrático, el Q real ≈ −231 (derivado del log:
> mean_r ≈ −2.31, γ=0.99 → Q = −2.31/0.01 = −231). Para λ = α/|Q_scaled| ≈ 3.0/10.5 ≈ 0.29,
> se necesita `reward_scale = 231/10.5 ≈ 22`.
>
> **Por qué cuadrática:** la penalización (CTE/2)² produce gradientes proporcionales al error
> (mayor corrección cuando el error es grande) y proporciona señal continua incluso cerca de CTE=0,
> a diferencia de la penalización lineal que satura el gradiente.

> **Evolución del diseño de recompensa:**
> - v1: lineal `|CTE|` con `reward_improve` — saturación de gradiente, BC dominaba.
> - v2: mismo lineal con `reward_action` — actor llegó a residuales contextuales pero λ inestable.
> - **v4 (activa):** cuadrática + speed bonus + `reward_scale=22` → λ≈0.29 estable, cte_align>0.

### 28.3 Arquitectura de la Red

**Actor (política):**
```
MLP: 20 → 256 → 256 → 2
Activación: ReLU (ocultas), tanh (salida)
Salida: (a_c, a_d) ∈ [-1, 1]²
```

**Crítico twin (Q-function):**
```
MLP: 22 → 256 → 256 → 1   (concatena estado + acción)
Se instancian 2 críticos independientes (twin); se usa el mínimo para evitar sobreestimación
```

**Parámetros de normalización:**
- El vector de estado se normaliza por media y std estimados del dataset antes de entrar al actor.
- Los estadísticos se guardan en `rl_models/norm_stats.pt` y se cargan en inferencia.

### 28.4 Hiperparámetros de Entrenamiento (v4 — valores activos)

| Parámetro ROS | Valor activo | Descripción |
|---|---|---|
| `n_gradient_steps` | **200 000** | Pasos de gradiente totales (+50K vs v3) |
| `batch_size` | 256 | Tamaño del minibatch |
| `lr_actor` | **2e-4** | Learning rate del actor (aumentado para mayor capacidad RL) |
| `lr_critic` | 3e-4 | Learning rate de los críticos |
| `weight_decay` | 1e-4 | Regularización L2 del optimizador |
| `gamma` | **0.99** | Factor de descuento (aumentado para horizonte largo) |
| `tau` | 0.005 | Tasa de soft-update de los target networks |
| `td3bc_alpha` | **3.0** | Mayor autoridad RL (con reward_scale=22, λ≈0.29) |
| `gradient_clip_norm` | 1.0 | Clipping de gradiente para estabilidad |
| `reward_scale` | **22.0** | Calculado para λ≈0.29 con recompensa cuadrática |
| `reward_cte_weight` | 1.0 | Peso de la penalización cuadrática de CTE |
| `reward_cte_norm` | 2.0 | Normalización: `(CTE / 2.0)²` |
| `reward_speed_weight` | 0.15 | Bonus velocidad (speed ≥ 0.8 × u_ref) |
| `warmup_steps` | **3000** | Pasos de warm-start BC (aumentado de 1000) |
| `exclude_stage1` | `true` | Excluye episodios sin política activa |

> **`td3bc_alpha=3.0` con `reward_scale=22`:** λ = 3.0 / |Q_scaled| ≈ 3.0/10.5 ≈ 0.29.
> El gradiente RL contribuye ~29% del ajuste del actor (vs ~8% en v3 con scale=6).

**Proceso warm-start BC (3 000 pasos pre-TD3):**

Antes del loop TD3+BC, se entrena el actor solo con pérdida BC durante 3 000 pasos.
Esto inicializa el actor cerca del comportamiento del MPPI y estabiliza la estimación del Q.
Resultado esperado: `bc_loss_final < 0.008`.

### 28.5 Estadísticas de Entrenamiento (v4 — Run Activo)

| Métrica | Valor observado |
|---|---|
| Episodios en dataset | 79 (excluidos 47 de stage1, 0 cortos) |
| Transiciones train | 164 180 |
| Transiciones val | 15 004 |
| Q escalado estimado | ≈ −10.49 (train) / −9.74 (val) |
| **λ estimado inicial** | **≈ 0.286** (objetivo: 0.2–1.0) ✅ |
| λ durante entrenamiento | 0.31–1.79 (mayormente 0.35–0.52) ✅ |
| `cte_align` inicial | **+0.38** (positivo = actor aprende dirección correcta) ✅ |
| `val_bc` mejor | **0.01251** (en step 10 000) |
| `critic_loss` estable | 0.003–0.12, sin divergencia ✅ |
| `bc_loss` warm-start | 0.00596 |
| Tiempo total GPU | ≈ 11 min (RTX 3070 Ti Laptop) |

> **`cte_align` positivo:** métrica que verifica que el actor aplica `delta_diff` en la misma
> dirección que `e_ct`, consistente con la ley `delta_diff = k_cte × tanh(e_ct)` del MPPI.
> Valores negativos indicarían un bug de signo o inversión del aprendizaje.

### 28.6 Comando de Entrenamiento (sin parámetros extra — defaults correctos)

```bash
source install/setup.bash
ros2 run vrx_experiment_benchmark train_rl_residual --ros-args \
  -p dataset_root:=/home/misael/ros2_workspaces/diplomado_ws/src/vrx_experiment_benchmark/metrics/rl_dataset/raw \
  -p output_model_dir:=/home/misael/ros2_workspaces/diplomado_ws/src/vrx_experiment_benchmark/config/rl_models \
  -p split_manifest_dir:=/home/misael/ros2_workspaces/diplomado_ws/src/vrx_experiment_benchmark/metrics/rl_dataset/splits \
  -p training_log_dir:=/home/misael/ros2_workspaces/diplomado_ws/src/vrx_experiment_benchmark/metrics/rl_dataset/training \
  -p exclude_stage1:=true \
  -p train_device:=auto
```

> **Todos los hiperparámetros v4 están como defaults en `train_rl_residual.py`**.
> No se require pasar argumentos adicionales. El entrenamiento tarda ≈11 min en GPU.

### 28.7 Señales de Convergencia para Validar el Entrenamiento

Al iniciar el entrenamiento verificar en el log:

```
# Líneas clave a buscar:
Recompensa escalada | mean=-0.10XX  →  Q estimado ≈ -10.X  →  λ estimado ≈ 0.2X (objetivo: 0.2-1.0)
# λ debe estar entre 0.20 y 1.0. Si λ < 0.10 → reward_scale insuficiente.

# Durante el entrenamiento:
step:  5000 | actor:3.01XXX λ:X.XX | cte_align:0.3XX
# cte_align debe ser POSITIVO. Negativo indica bug de signo.
# λ entre 0.3 y 1.8 es normal (fluctúa).
```

---

## 29. Dataset de RL

### 29.1 Estructura

El dataset se compone de episodios CSV recolectados al ejecutar el stack LOS+MPPI con
`record_dataset:=true`. Cada episodio es un archivo en `metrics/rl_dataset/raw/`.

**Columnas clave del CSV de episodio:**

| Columna | Descripción |
|---|---|
| `t` | Timestamp relativo [s] |
| `e_ct` | Error de pista lateral [m] |
| `delta_c` | Empuje común del MPPI [N] |
| `delta_d` | Diferencial del MPPI [N] |
| `mode_code` | 0=start, 1=transit, 2=finish |
| `stage` | `stage1` (sin política) o `stage2` (con política activa) |

### 29.2 Filtros Aplicados Durante la Construcción del Replay Buffer

1. **Excluir `stage1`** (`exclude_stage1:=true`): descarta runs del bootstrap heurístico.
2. **Excluir filas con residual cero** (`|delta_c| < 1e-4 AND |delta_d| < 1e-4`): indica que
   el nodo RL estaba inactivo (modo heurístico o hold absoluto).
3. **Episodios cortos** (`< 50 transiciones`): descartados automáticamente.

### 29.3 Recolección de Dataset

```bash
# Recolectar episodio con política MPPI pura:
ros2 launch vrx_experiment_benchmark route_following_mppi_rl.launch.py \
  route_name:=route_curves.yaml \
  use_learned_policy:=false \
  shadow_mode:=false \
  record_dataset:=true

# Recolectar con política RL activa (DAgger-style, para futura iteración):
ros2 launch vrx_experiment_benchmark route_following_mppi_rl.launch.py \
  route_name:=route_curves.yaml \
  use_learned_policy:=true \
  shadow_mode:=false \
  record_dataset:=true
```

---

## 30. Resultados de Referencia — Benchmark Completo (45 corridas)

> **Diseño experimental:** 3 stacks × 3 rutas × 5 entornos = 45 corridas.  
> **Entornos:** env_01_calm, env_02_low, env_03_medium, env_04_high, env_05_severe.  
> **Métrica principal:** RMS CTE solo en fase de tránsito (mode_code=1), que excluye el
> posicionamiento inicial y refleja la calidad real del seguimiento de trayectoria.

### 30.1 Métricas Globales (media ± std sobre runs completados)

| Métrica | LOS+PD | LOS+MPPI | **LOS+MPPI+RL** |
|---|---|---|---|
| Completados | 13/15 (86.7%) | 15/15 (100%) | **15/15 (100%)** |
| Tiempo total [s] | 147.5 ± 19.3 | 114.4 ± 28.3 | **103.9 ± 21.0** |
| **RMS CTE tránsito [m]** | 3.775 ± 1.13 | 1.583 ± 0.47 | **1.498 ± 0.45** |
| CTE medio tránsito [m] | 3.135 ± 1.00 | 1.319 ± 0.39 | **1.270 ± 0.40** |
| RMS heading error [rad] | 0.477 ± 0.27 | 0.403 ± 0.11 | **0.386 ± 0.11** |
| Vel. media tránsito [m/s] | 1.908 ± 0.12 | 2.611 ± 0.48 | **2.822 ± 0.31** |
| Speed tracking ratio | 0.643 ± 0.05 | 0.894 ± 0.20 | **0.964 ± 0.16** |
| Path efficiency | **0.865** ± 0.08 | 0.787 ± 0.11 | 0.797 ± 0.11 |
| Carga actuadores [%] | 40.9 ± 8.7 | 68.6 ± 12.8 | 74.1 ± 10.4 |
| **Suavidad ctrl [N/step]** | 95.0 ± 183.6 | 69.7 ± 31.5 | **56.6 ± 26.2** |
| Saturación [%] | 3.8 ± 10.6 | 33.0 ± 14.7 | 40.8 ± 12.6 |
| Intervalo entre WPs [s] | 11.75 ± 2.05 | 9.47 ± 3.12 | **8.41 ± 2.32** |

### 30.2 RMS CTE Tránsito por Ruta (métrica central)

| Ruta | LOS+PD | LOS+MPPI | **LOS+MPPI+RL** | Δ RL vs MPPI |
|---|---|---|---|---|
| Recta | 2.932 ± 0.228 m | 1.057 ± 0.316 m | **0.933 ± 0.035 m** | **−11.7%** ✅ |
| Curvas | 3.700 ± 1.028 m | 1.811 ± 0.367 m | **1.637 ± 0.184 m** | **−9.6%** ✅ |
| Zigzag | 5.306 ± 0.402 m* | 1.882 ± 0.129 m | 1.923 ± 0.171 m | +2.2% (marginal) |
| **Global** | **3.775 m** | **1.583 m** | **1.498 m** | **−5.4%** ✅ |

*Solo n=3 runs completados para PD en zigzag.

### 30.3 Consistencia Inter-Entorno (σ del RMS CTE tránsito)

| Stack | Recta σ | Curvas σ | Zigzag σ |
|---|---|---|---|
| LOS+PD | 0.228 m | 1.028 m | 0.402 m |
| LOS+MPPI | 0.316 m | 0.367 m | 0.129 m |
| **LOS+MPPI+RL** | **0.035 m** ⭐ | **0.184 m** | **0.171 m** |

> El RL es **9× más consistente** que MPPI en la ruta recta (σ=0.035 vs 0.316 m).
> Esto indica que la política residual absorbe implícitamente las perturbaciones ambientales.

### 30.4 Suavidad de Control por Ruta [N/paso, menor = más suave]

| Ruta | LOS+PD | LOS+MPPI | **LOS+MPPI+RL** | Δ RL vs MPPI |
|---|---|---|---|---|
| Recta | 61.1 | 60.9 | **37.6** | **−38.3%** ✅ |
| Curvas | **180.7** ❌ | 75.3 | **71.2** | −5.4% |
| Zigzag | 8.5 (inactivo) | 72.9 | **61.0** | −16.3% |

> PD en curvas produce variación de control **2.5× mayor** que RL/MPPI — refleja
> correcciones reactivas bruscas. PD en zigzag tiene variación mínima (8.5 N/paso) porque
> el controlador falla y no maniobra, no porque sea preciso.

---

## 31. Historial de Iteraciones — Entrenamiento TD3+BC

### v1 — BC puro (descartado)
- Solo Behavioral Cloning sin TD3.
- Problema: sin optimización de Q, el actor no mejoraba el CTE futuro.
- Resultado: `rms_cte ≈ 2.39m` straight; peor que MPPI en curves/zigzag.

### v2 — TD3+BC inicial (α=2.5, reward lineal, sin scale)
- `td3bc_alpha=2.5`, recompensa lineal `|CTE|`, sin `reward_scale`.
- Problema: λ colapsó a 0.04 (RL prácticamente desactivado; BC dominaba 96%).
- Consecuencia: `rms_cte curves = 2.201m`, **peor que MPPI**.

### v3 — TD3+BC con `reward_scale=10`, regularización de acción
- `td3bc_alpha=1.5`, `reward_action_weight=0.10`, `n_gradient_steps=150K`.
- `alpha_residual=0.35` en inferencia.
- Resultado: λ ≈ 0.25, `val_bc_best = 0.00302` en step 98K.
- **Superó MPPI en straight (−6%) y zigzag (−4.5%).**
- Problema pendiente: recompensa lineal producía gradientes débiles cerca de CTE=0.

### v4 — TD3+BC con recompensa cuadrática + bonus velocidad (ACTIVO)

**Cambios respecto a v3:**
- Recompensa: `(CTE/2.0)²` cuadrática + speed bonus (`+0.15` si speed ≥ 0.8×u_ref)
- `reward_scale`: 6→**22** (recalculado para λ≈0.29: Q_real≈−231, scale=231/10.5≈22)
- `td3bc_alpha`: 1.5→**3.0** (mayor autoridad RL al tener λ calibrado)
- `gamma`: 0.97→**0.99** (horizonte temporal más largo)
- `lr_actor`: 1e-4→**2e-4** (mayor capacidad de aprendizaje RL)
- `warmup_steps`: 1000→**3000** (inicialización BC más robusta)
- `n_gradient_steps`: 150K→**200K**
- **Alpha dinámico por modo:** transit=0.50, hold=0.20, zigzag_diff=0.60
- **Corrección `cte_align`:** signo corregido para consistencia con ley MPPI
- **`use_learned_policy: true`** en YAML (activo por defecto)

**Indicadores de entrenamiento v4:**
- λ inicial ≈ 0.286 ✅, λ durante training: 0.31–1.79 ✅
- `cte_align` = +0.38 (positivo = corrección en dirección correcta) ✅
- `val_bc_best` = 0.01251 (step 10K; sube después → RL diverge del BC puro, esperado)
- `critic_loss` estable 0.003–0.12 ✅

**Resultado benchmark completo (45 runs):**
- RMS CTE tránsito global: **1.498 m** (vs 1.583 MPPI, −5.4%) ✅
- Recta: **0.933 m** (vs 1.057 MPPI, −11.7%) ✅
- Curvas: **1.637 m** (vs 1.811 MPPI, −9.6%) ✅
- Suavidad ctrl: **56.6 N/paso** (vs 69.7 MPPI, −18.8%) ✅
- Consistencia recta: **σ=0.035 m** (vs 0.316 MPPI, 9× más estable) ✅

---

## 32. Launch `route_following_mppi_rl.launch.py`

**Nodos lanzados:**

| Nodo | Ejecutable | Config YAML |
|---|---|---|
| `state_estimator_2d` | vía `state_estimation.launch.py` | `state_estimator_2d.yaml` |
| `route_manager` | `route_manager` | `route_manager_config.yaml` |
| `guidance_los` | `guidance_los` | `los_config.yaml` |
| `controller_mppi` | `controller_mppi` | `mppi_config.yaml` |
| `controller_rl_residual` | `controller_rl_residual` | `rl_residual_config.yaml` |
| `thruster_commander` | `thruster_commander` | `controller_limits.yaml` |
| `metrics_logger` | `metrics_logger` | `metrics_config.yaml` |

**Diferencia con `route_following_mppi.launch.py`:**

```diff
  controller_mppi_node    ← ahora publica en thruster_cmd_raw (antes thruster_cmd)
+ controller_rl_residual_node  ← nuevo; lee thruster_cmd_raw, publica thruster_cmd
```

El `thruster_commander` sigue leyendo de `/wamv/control/thruster_cmd` (sin cambio en su interfaz).

---

## 33. Reglas de Desarrollo — LOS+MPPI+RL

1. **No romper la cadena de tópicos**: `thruster_cmd_raw` → `controller_rl_residual` →
   `thruster_cmd` → `thruster_commander`. Si se modifica este flujo, actualizar el launch y el YAML.
2. **El dataset es válido independientemente de `alpha_residual`**: alpha solo afecta la inferencia,
   nunca los archivos CSV de entrenamiento.
3. **Al reentrenar**, verificar en el log inicial: λ ≈ 0.2–1.0 ✅ y `cte_align` > 0 ✅.
   Si λ < 0.1 → `reward_scale` insuficiente. Si `cte_align` < 0 → bug de signo en la métrica.
4. **Nunca modificar el contrato de `Float64MultiArray`** de los tópicos existentes.
5. **El modelo exportado es TorchScript** (`.ts`): no requiere PyTorch en el nodo de inferencia
   más allá de `torch.jit.load`. Mantener esta interfaz para compatibilidad futura.
6. **El `alpha_residual` dinámico** admite ajuste sin reentrenar: modificar
   `alpha_residual_transit`, `alpha_residual_hold`, `alpha_residual_zigzag_diff` en el YAML.
   Si el zigzag muestra CTE mayor que MPPI → subir `alpha_residual_zigzag_diff` de 0.60 a 0.75.
7. **Diagnóstico**: observar `res:X,Y` en el log del controlador. Si siempre `|res| > 25` y es
   constante → actor saturado → reducir `alpha_residual_transit` o reentrenar con mayor `gamma`.
8. **Benchmarking estadístico**: ≥3 runs por ruta por entorno para significancia. La σ natural
   del entorno es ≈±0.1–0.3 m en RMS CTE tránsito (menor para RL que para MPPI).
9. **Parámetros que NO cambiar sin reentrenamiento**:
   - `policy_input_dim`: cambiar requiere reentrenar por completo.
   - `policy_hidden_dim`: idem.
   - `max_residual_*`: cambiar los límites cambia la escala de la acción normalizada.
10. **Si el rendimiento regresa a niveles BC** (rms_cte ≈ baseline), verificar:
    - Que `use_learned_policy:=true` esté activo en el launch.
    - Que el archivo `.ts` en `rl_models/` no esté corrupto.
    - Que `norm_stats.pt` corresponda al mismo entrenamiento que el `.ts`.

---

*Sección LOS+MPPI+RL (TD3+BC) añadida el 16 de abril de 2026.*  
*Actualizada el 17 de abril de 2026 con cambios de la sesión v4:*  
*recompensa cuadrática (CTE/2)² + speed bonus, reward_scale=22, td3bc_alpha=3.0,*  
*gamma=0.99, lr_actor=2e-4, warmup=3000, n_gradient_steps=200K.*  
*Alpha dinámico por modo: transit=0.50, hold=0.20, zigzag_diff=0.60.*  
*Benchmark completo (45 runs): RMS CTE tránsito −5.4% vs MPPI, suavidad ctrl −18.8%,*  
*consistencia recta 9× superior (σ=0.035 m). best_val_bc=0.01251 (step 10K).*
