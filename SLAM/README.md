# Practica SLAM -- Robots Moviles

**Autores del simulador:** J. Neira, J. Tardos -- Universidad de Zaragoza  
**Alumno:** Victor Martin

---

## Contenido de la practica

La practica cubre **5 sistemas SLAM en MATLAB** y se centra en experimentar con algoritmos de asociacion de datos en un sistema EKF-SLAM. El trabajo principal esta en la carpeta `SLAM-Zaragoza/`.

```
Slam_programs/
├── SLAM-Zaragoza/        ← * EJERCICIOS PRINCIPALES *
│   ├── slam.m            ← Programa principal (modificado)
│   └── TOOLS/
│       ├── NN.m          ← Nearest Neighbour (dado)
│       ├── SINGLES.m     ← Only Neighbour * COMPLETADO POR EL ALUMNO *
│       ├── JCBB.m        ← Joint Compatibility B&B (dado)
│       └── remove_features.m  ← * NUEVO -- auxiliar de map maintenance *
├── Slam-Salvi/           ← Demo interactiva
├── ekf-slam-matlab-master/ ← Demo con GUI
├── Slam_Bailey/          ← FastSLAM 1, 2 y 2r
└── Slamtb-Sola/          ← Toolbox SLAM completo
```

---

## Modificaciones realizadas

### 1. `TOOLS/SINGLES.m` -- Algoritmo completado

El fichero estaba vacio (solo con comentarios). Se implemento el algoritmo **SINGLES** (Only Neighbour):

> Una observacion `i` se empareja con el feature `j` **unicamente si** la observacion solo tiene un vecino compatible (el feature `j`) **y** ese feature solo tiene una observacion compatible (la observacion `i`). El match es estrictamente biunivoco.

```matlab
for i = 1:observations.m
    compatible_features = find(compatibility.ic(i, :));
    if length(compatible_features) == 1
        j = compatible_features(1);
        compatible_observations = find(compatibility.ic(:, j));
        if length(compatible_observations) == 1
            H(i) = j;  % match unico y univoco
        end
    end
end
```

**Ventaja respecto a NN:** elimina matches ambiguos. Hace menos asociaciones, pero las que hace son correctas.

---

### 2. `slam.m` -- Selector de algoritmo y Map Maintenance

Al inicio del fichero se controla todo con **tres variables**:

```matlab
algorithm            = 'NN';   % 'NN' | 'SINGLES' | 'JCBB'
configuration.people =  0;     % 1 = anadir outliers (personas)
configuration.odometry = 1;    % 0 = sin odometria
use_map_maintenance  =  1;     % 1 = activar eliminacion de features
```

**Map Maintenance implementado:** al final de cada paso se eliminan los features que:
- Han sido vistos **solo una vez**, Y
- Su ultima observacion fue **hace mas de 2 steps**.

Esto evita que outliers (personas, ruido) queden permanentemente en el mapa.

---

### 3. `TOOLS/remove_features.m` -- Nueva funcion auxiliar

Elimina features del mapa EKF actualizando correctamente:
- El vector de estado `map.x` (cada feature ocupa 2 filas, tras las 3 del robot)
- La matriz de covarianza `map.P`
- El array `map.ground_id`
- Los contadores de mantenimiento `map.seen` y `map.last_seen`

---

## Como ejecutar

### SLAM-Zaragoza (ejercicios principales)

```matlab
% En MATLAB, desde la carpeta SLAM-Zaragoza/
clear; slam
```

Cambia las variables al inicio de `slam.m` para cada experimento (ver tabla abajo). Pulsa **Enter** para avanzar paso a paso, o pon `configuration.step_by_step = 0` para ejecucion continua.

### Secuencia de experimentos

| # | `algorithm` | `people` | `odometry` | `maintenance` | Que observar |
|---|-------------|----------|------------|---------------|--------------|
| 1 | `'NN'` | 0 | 1 | 0 | Baseline: NN sin filtrado |
| 2 | `'SINGLES'` | 0 | 1 | 0 | Menos matches, mas precisos |
| 3 | `'SINGLES'` | 1 | 1 | 0 | SINGLES vs. outliers de personas |
| 4 | `'JCBB'` | 0 | 1 | 0 | JCBB limpio |
| 5 | `'JCBB'` | 1 | 1 | 0 | JCBB robusto con personas |
| 6 | `'JCBB'` | 0 | 0 | 0 | Sin odometria -> elipses mas grandes |
| 7 | `'NN'` | 0 | 1 | 1 | Map maintenance activo |

### Figuras del simulador

| Figura | Descripcion |
|--------|-------------|
| Fig. 1 | Ground Truth: posicion real de features (puntos rojos) |
| Fig. 2 | SLAM Map: posicion estimada + elipses de incertidumbre (azul) |
| Fig. 3 | Observations: medidas del sensor en el frame del robot |
| Fig. 6 | Compatibility: que observaciones son compatibles con que features |
| Fig. 7 | Hypothesis: que matches ha decidido el algoritmo |

---

## Parametros del sensor (experimentos opcionales)

En `slam.m`, seccion SENSOR PARAMETERS, puedes probar:

| Parametro | Valor base | Para probar | Efecto |
|-----------|-----------|-------------|--------|
| `sensor.srho` | 0.01 | 0.05 / 0.10 | Elipses mas grandes en rango |
| `sensor.stita` | 0.125°->rad | 1°->rad | Elipses mas grandes en angulo |
| `sensor.range` | 5 | 3 / 8 | El robot ve mas o menos lejos |

---

## Demos interactivas (sin codigo a modificar)

### Slam-Salvi
```matlab
cd Slam-Salvi
clear; Slam2D1
% click derecho = poner landmark/waypoint, click izquierdo = terminar
% Probar tambien: Slam2D2, Slam2D3, Slam2D4
```

### ekf-slam-matlab-master
```matlab
cd ekf-slam-matlab-master
clear; setup
% En el GUI: Load Map -> elegir mapa de sample-maps/ -> Execute SLAM Simulation
```

### Slam_Bailey -- FastSLAM
```matlab
cd Slam_Bailey/fastslam
addpath(genpath('.'));
load example_webmap.mat        % doble clic en el archivo tambien vale
fastslam1_sim(Im, wp)
fastslam2_sim(Im, wp)
fastslam2r_sim(Im, wp)
```

### Slamtb-Sola
```matlab
cd Slamtb-Sola
clear; slamrc; slamtb
% Ver slamToolbox.pdf para mas detalles
```

---

## Que se entrega

> **No se requiere memoria escrita.** El enunciado (`5007439`) solo pide completar el codigo y ejecutar los experimentos.

### Ficheros entregables (los unicos modificados respecto al original)

| Fichero | Cambio |
|---------|--------|
| `SLAM-Zaragoza/TOOLS/SINGLES.m` | * **Core del ejercicio** -- algoritmo completado |
| `SLAM-Zaragoza/slam.m` | Selector de algoritmo (NN/SINGLES/JCBB) + map maintenance |
| `SLAM-Zaragoza/TOOLS/remove_features.m` | Funcion auxiliar nueva (map maintenance) |
| `README.md` | Este fichero de documentacion |

### Experimentos obligatorios (enunciado `5007439`)

| Ejercicio | Como hacerlo |
|-----------|--------------|
| 1. Probar **NN** | `algorithm = 'NN'` -> `clear; slam` |
| 2. Completar y probar **SINGLES** | `algorithm = 'SINGLES'` -> `clear; slam` |
| 3. **SINGLES con personas** | `algorithm = 'SINGLES'`, `configuration.people = 1` |
| 4. Probar **JCBB** | `algorithm = 'JCBB'` -> `clear; slam` |

---

## Descripcion del escenario de simulacion

- **Robot:** con odometria de ruedas y sensor laser range-finder.
- **Entorno:** pasillo cuadrado con features puntuales en las paredes.
- **EKF-SLAM:** el estado incluye la posicion del robot (3D: x, y, θ) y la posicion de cada feature (2D: x, y).
- **Cierre de bucle:** al final del recorrido el robot reobserva features iniciales y la incertidumbre se reduce drasticamente -- comportamiento SLAM clasico.

---

*Practica basada en: J. Neira & J.D. Tardos, "Data Association in Stochastic Mapping Using the Joint Compatibility Test", IEEE T-RA 2001.*

