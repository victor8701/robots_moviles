# Práctica SLAM — Robots Móviles

**Autores del simulador:** J. Neira, J. Tardós — Universidad de Zaragoza  
**Alumno:** Victor Martin

---

## Contenido de la práctica

La práctica cubre **5 sistemas SLAM en MATLAB** y se centra en experimentar con algoritmos de asociación de datos en un sistema EKF-SLAM. El trabajo principal está en la carpeta `SLAM-Zaragoza/`.

```
Slam_programs/
├── SLAM-Zaragoza/        ← ★ EJERCICIOS PRINCIPALES ★
│   ├── slam.m            ← Programa principal (modificado)
│   └── TOOLS/
│       ├── NN.m          ← Nearest Neighbour (dado)
│       ├── SINGLES.m     ← Only Neighbour ★ COMPLETADO POR EL ALUMNO ★
│       ├── JCBB.m        ← Joint Compatibility B&B (dado)
│       └── remove_features.m  ← ★ NUEVO — auxiliar de map maintenance ★
├── Slam-Salvi/           ← Demo interactiva
├── ekf-slam-matlab-master/ ← Demo con GUI
├── Slam_Bailey/          ← FastSLAM 1, 2 y 2r
└── Slamtb-Sola/          ← Toolbox SLAM completo
```

---

## Modificaciones realizadas

### 1. `TOOLS/SINGLES.m` — Algoritmo completado

El fichero estaba vacío (solo con comentarios). Se implementó el algoritmo **SINGLES** (Only Neighbour):

> Una observación `i` se empareja con el feature `j` **únicamente si** la observación solo tiene un vecino compatible (el feature `j`) **y** ese feature solo tiene una observación compatible (la observación `i`). El match es estrictamente biunívoco.

```matlab
for i = 1:observations.m
    compatible_features = find(compatibility.ic(i, :));
    if length(compatible_features) == 1
        j = compatible_features(1);
        compatible_observations = find(compatibility.ic(:, j));
        if length(compatible_observations) == 1
            H(i) = j;  % match único y unívoco
        end
    end
end
```

**Ventaja respecto a NN:** elimina matches ambiguos. Hace menos asociaciones, pero las que hace son correctas.

---

### 2. `slam.m` — Selector de algoritmo y Map Maintenance

Al inicio del fichero se controla todo con **tres variables**:

```matlab
algorithm            = 'NN';   % 'NN' | 'SINGLES' | 'JCBB'
configuration.people =  0;     % 1 = añadir outliers (personas)
configuration.odometry = 1;    % 0 = sin odometría
use_map_maintenance  =  1;     % 1 = activar eliminación de features
```

**Map Maintenance implementado:** al final de cada paso se eliminan los features que:
- Han sido vistos **solo una vez**, Y
- Su última observación fue **hace más de 2 steps**.

Esto evita que outliers (personas, ruido) queden permanentemente en el mapa.

---

### 3. `TOOLS/remove_features.m` — Nueva función auxiliar

Elimina features del mapa EKF actualizando correctamente:
- El vector de estado `map.x` (cada feature ocupa 2 filas, tras las 3 del robot)
- La matriz de covarianza `map.P`
- El array `map.ground_id`
- Los contadores de mantenimiento `map.seen` y `map.last_seen`

---

## Cómo ejecutar

### SLAM-Zaragoza (ejercicios principales)

```matlab
% En MATLAB, desde la carpeta SLAM-Zaragoza/
clear; slam
```

Cambia las variables al inicio de `slam.m` para cada experimento (ver tabla abajo). Pulsa **Enter** para avanzar paso a paso, o pon `configuration.step_by_step = 0` para ejecución continua.

### Secuencia de experimentos

| # | `algorithm` | `people` | `odometry` | `maintenance` | Qué observar |
|---|-------------|----------|------------|---------------|--------------|
| 1 | `'NN'` | 0 | 1 | 0 | Baseline: NN sin filtrado |
| 2 | `'SINGLES'` | 0 | 1 | 0 | Menos matches, más precisos |
| 3 | `'SINGLES'` | 1 | 1 | 0 | SINGLES vs. outliers de personas |
| 4 | `'JCBB'` | 0 | 1 | 0 | JCBB limpio |
| 5 | `'JCBB'` | 1 | 1 | 0 | JCBB robusto con personas |
| 6 | `'JCBB'` | 0 | 0 | 0 | Sin odometría → elipses más grandes |
| 7 | `'NN'` | 0 | 1 | 1 | Map maintenance activo |

### Figuras del simulador

| Figura | Descripción |
|--------|-------------|
| Fig. 1 | Ground Truth: posición real de features (puntos rojos) |
| Fig. 2 | SLAM Map: posición estimada + elipses de incertidumbre (azul) |
| Fig. 3 | Observations: medidas del sensor en el frame del robot |
| Fig. 6 | Compatibility: qué observaciones son compatibles con qué features |
| Fig. 7 | Hypothesis: qué matches ha decidido el algoritmo |

---

## Parámetros del sensor (experimentos opcionales)

En `slam.m`, sección SENSOR PARAMETERS, puedes probar:

| Parámetro | Valor base | Para probar | Efecto |
|-----------|-----------|-------------|--------|
| `sensor.srho` | 0.01 | 0.05 / 0.10 | Elipses más grandes en rango |
| `sensor.stita` | 0.125°→rad | 1°→rad | Elipses más grandes en ángulo |
| `sensor.range` | 5 | 3 / 8 | El robot ve más o menos lejos |

---

## Demos interactivas (sin código a modificar)

### Slam-Salvi
```matlab
cd Slam-Salvi
clear; Slam2D1
% click derecho = poner landmark/waypoint, click izquierdo = terminar
% Probar también: Slam2D2, Slam2D3, Slam2D4
```

### ekf-slam-matlab-master
```matlab
cd ekf-slam-matlab-master
clear; setup
% En el GUI: Load Map → elegir mapa de sample-maps/ → Execute SLAM Simulation
```

### Slam_Bailey — FastSLAM
```matlab
cd Slam_Bailey/fastslam
addpath(genpath('.'));
load example_webmap.mat        % doble clic en el archivo también vale
fastslam1_sim(Im, wp)
fastslam2_sim(Im, wp)
fastslam2r_sim(Im, wp)
```

### Slamtb-Sola
```matlab
cd Slamtb-Sola
clear; slamrc; slamtb
% Ver slamToolbox.pdf para más detalles
```

---

## Qué se entrega

> **No se requiere memoria escrita.** El enunciado (`5007439`) solo pide completar el código y ejecutar los experimentos.

### Ficheros entregables (los únicos modificados respecto al original)

| Fichero | Cambio |
|---------|--------|
| `SLAM-Zaragoza/TOOLS/SINGLES.m` | ⭐ **Core del ejercicio** — algoritmo completado |
| `SLAM-Zaragoza/slam.m` | Selector de algoritmo (NN/SINGLES/JCBB) + map maintenance |
| `SLAM-Zaragoza/TOOLS/remove_features.m` | Función auxiliar nueva (map maintenance) |
| `README.md` | Este fichero de documentación |

### Experimentos obligatorios (enunciado `5007439`)

| Ejercicio | Cómo hacerlo |
|-----------|--------------|
| 1. Probar **NN** | `algorithm = 'NN'` → `clear; slam` |
| 2. Completar y probar **SINGLES** | `algorithm = 'SINGLES'` → `clear; slam` |
| 3. **SINGLES con personas** | `algorithm = 'SINGLES'`, `configuration.people = 1` |
| 4. Probar **JCBB** | `algorithm = 'JCBB'` → `clear; slam` |

---

## Descripción del escenario de simulación

- **Robot:** con odometría de ruedas y sensor laser range-finder.
- **Entorno:** pasillo cuadrado con features puntuales en las paredes.
- **EKF-SLAM:** el estado incluye la posición del robot (3D: x, y, θ) y la posición de cada feature (2D: x, y).
- **Cierre de bucle:** al final del recorrido el robot reobserva features iniciales y la incertidumbre se reduce drásticamente — comportamiento SLAM clásico.

---

*Práctica basada en: J. Neira & J.D. Tardós, "Data Association in Stochastic Mapping Using the Joint Compatibility Test", IEEE T-RA 2001.*
