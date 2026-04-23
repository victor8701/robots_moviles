# Contenido para la Memoria — Parte Topológica
# (máx. 4 páginas, entrega en grupos)

---

## 1. Introducción

A partir del mapa geométrico obtenido en la práctica anterior se ha implementado
un sistema de navegación topológica en dos fases: (1) generación automática del
grafo topológico y (2) planificación y ejecución de rutas en dicho grafo.

---

## 2. Pipeline de Generación del Mapa Topológico

El proceso se implementa en `topological_mapper.py` y consta de cinco pasos:

### 2.1 Extracción del espacio libre
La imagen PGM generada por gmapping asigna ~254 a espacio libre, ~205 a zona
desconocida y ~0 a obstáculo. Se aplica un umbral binario (≥220) que captura
únicamente el espacio libre confirmado, seguido de un cierre morfológico 3×3
para cerrar huecos internos pequeños.

### 2.2 Esqueletonización (Medial Axis)
Se calcula el eje medial del espacio libre con `skimage.morphology.skeletonize`.
El resultado es una estructura de 1 px de grosor que conserva la topología del
espacio: los pasillos se convierten en líneas y las habitaciones en regiones
conectadas. Esta representación captura la "columna vertebral" del entorno
navegable.

### 2.3 Detección de nodos con Transformada de Distancia + NMS
Sobre la máscara de espacio libre se calcula la Transformada de Distancia L2
(`cv2.distanceTransform`), cuyo valor en cada píxel es la distancia euclídea
al obstáculo más cercano. Los nodos del grafo se colocan en los **máximos
locales** de esta función *restringidos al esqueleto*, ya que representan los
puntos más alejados de las paredes sobre el eje central del espacio navegable
(posiciones óptimas para el robot).

Se aplica Supresión de No-Máximos (NMS) con ventana de radio 10 px
(= 0.5 m a la resolución de 0.05 m/px) para evitar nodos duplicados.
Solo se aceptan nodos con distancia a pared > 2 px.

### 2.4 Cálculo de aristas
Dos nodos se conectan si:
- Su distancia euclídea es ≤ 200 px (10 m), Y
- La línea recta entre ellos no cruza ningún obstáculo (muestreo cada 2 px).

Tras este paso se verifica la conectividad global del grafo y se añaden aristas
de emergencia entre los pares de nodos más cercanos de componentes
desconectadas, garantizando que el grafo sea siempre conexo.

### 2.5 Planificador A*
Para navegar de un nodo a otro se usa A* con heurística euclídea (distancia en
píxeles entre nodos). El coste de cada arista es la distancia euclídea entre
los nodos que conecta. En cada iteración el planificador expande el nodo con
menor f = g + h, garantizando optimalidad. La ruta resultante se convierte a
waypoints en metros usando la resolución y el origen del mapa YAML.

---

## 3. Resultados por Entorno

### Tabla resumen

| Entorno    | Nodos | Aristas | Resolución (m/px) | Grafo conexo |
|------------|------:|--------:|:-----------------:|:------------:|
| Escenario1 |     3 |       3 |       0.05        |      Sí      |
| Escenario2 |     8 |      14 |       0.05        |      Sí      |
| Escenario3 |     9 |      12 |       0.05        |      Sí      |
| Estudio    |     8 |      19 |       0.05        |      Sí      |

### Posición de nodos

**Escenario 1**
| Nodo | X (m) | Y (m) | Dist. pared (m) |
|------|------:|------:|----------------:|
|  0   |  0.45 | -1.85 |      1.26       |
|  1   |  5.30 | -1.65 |      1.08       |
|  2   |  2.50 | -1.95 |      0.83       |

**Escenario 2**
| Nodo | X (m)  | Y (m) | Dist. pared (m) |
|------|-------:|------:|----------------:|
|  0   |  -2.05 |  2.60 |      1.55       |
|  1   |   1.50 |  4.75 |      1.25       |
|  2   |  -2.35 |  4.45 |      1.25       |
|  3   |  -1.85 | -0.30 |      0.81       |
|  4   |   1.50 | -0.75 |      0.80       |
|  5   |   1.60 |  1.05 |      0.65       |
|  6   |   0.20 | -0.70 |      0.65       |
|  7   |   0.10 |  3.20 |      0.58       |

**Escenario 3**
| Nodo | X (m)  | Y (m)  | Dist. pared (m) |
|------|-------:|-------:|----------------:|
|  0   |   0.80 |  -5.30 |      1.19       |
|  1   |   0.20 |  -0.10 |      1.00       |
|  2   |   0.00 |   1.85 |      0.78       |
|  3   |   2.15 |   2.05 |      0.76       |
|  4   |   2.70 |  -4.45 |      0.75       |
|  5   |  -0.05 |  -3.05 |      0.74       |
|  6   |   4.30 |   0.80 |      0.69       |
|  7   |   2.20 |   0.80 |      0.65       |
|  8   |   2.85 |  -2.45 |      0.65       |

**Estudio**
| Nodo | X (m)  | Y (m)  | Dist. pared (m) |
|------|-------:|-------:|----------------:|
|  0   |   3.50 |  -3.35 |      1.56       |
|  1   |   3.50 |  -1.85 |      1.56       |
|  2   |   0.25 |  -4.05 |      1.52       |
|  3   |   4.70 |  -1.45 |      1.30       |
|  4   |  -1.15 |  -4.55 |      1.25       |
|  5   |  -2.15 |  -2.10 |      1.06       |
|  6   |   5.35 |  -3.85 |      1.05       |
|  7   |  -0.20 |  -2.20 |      1.00       |

*(Insertar aquí las figuras de `mapas/memoria/*_pipeline.png`)*
*(Cada figura muestra: espacio libre | esqueleto | mapa topológico)*

---

## 4. Estrategia de Navegación Topológica

### 4.1 Localización del robot en el grafo
Al iniciar la navegación el sistema obtiene la posición del robot en metros
mediante la transformación TF `map → base_footprint`. La convierte a coordenadas
de píxel y busca el nodo más cercano por distancia euclídea.

### 4.2 Planificación de ruta
El usuario introduce el ID del nodo destino por consola. El planificador A*
calcula el camino de mínimo coste. Ejemplo para Escenario 2:
- Robot en (0.5, -0.5) m → nodo más cercano: 6
- Destino nodo 5 → Ruta: **6 → 4 → 5** (dist. total: 3.10 m)

### 4.3 Ejecución nodo a nodo
Para cada nodo de la ruta (exceptuando el inicial, donde ya está el robot) se
calcula la orientación como el ángulo hacia el siguiente nodo:

    yaw = atan2(y_{i+1} - y_i, x_{i+1} - x_i)

El goal se envía a `move_base` con posición (x, y) y orientación en cuaternión.
El sistema espera confirmación de `SUCCEEDED` antes de avanzar al siguiente nodo.
La ruta completa se publica en `/topological_path` para visualizarla en RViz.

---

## 5. Archivos entregables

| Archivo | Descripción |
|---------|-------------|
| `src/topological_mapper.py`     | Generador del mapa topológico |
| `src/path_planner.py`           | Planificador A* topológico    |
| `scripts/topological_navigation.py` | Nodo ROS interactivo      |
| `scripts/generate_all_maps.py`  | Generación de los 4 entornos  |
| `mapas/grafos/*.json`           | Grafos de los 4 entornos      |
| `mapas/memoria/*_pipeline.png`  | Figuras para la memoria       |
