# Navegación Topológica

Módulo 2 del proyecto. Convierte los mapas geométricos (PGM) generados en el módulo anterior en **grafos topológicos** (nodos + aristas), implementa planificación de rutas con **A\*** y ofrece un nodo ROS para navegación interactiva nodo a nodo.

---

## Qué hace este módulo

| Componente | Archivo | Función |
|-----------|---------|---------|
| Generador de mapas | `src/topological_mapper.py` | PGM → grafo JSON (nodos, aristas, metadatos) |
| Planificador A\* | `src/path_planner.py` | Ruta óptima entre dos nodos del grafo |
| Nodo ROS | `scripts/topological_navigation.py` | Interfaz interactiva: el usuario elige nodo destino y el robot navega |
| Generador de figuras | `scripts/generate_all_maps.py` | PNG con el grafo superpuesto al mapa, para todos los entornos |
| Visualizador RViz | `scripts/graph_visualizer.py` | Publica markers del grafo en RViz en tiempo real |

---

## Algoritmo de generación del mapa topológico

El proceso se ejecuta en 5 pasos sobre el mapa PGM:

```
PGM del mapa
     │
     ▼
1. Máscara de espacio libre (umbral > 220)
     │
     ▼
2. Esqueletonización (scikit-image — thinning morfológico)
     │
     ▼
3. Transformada de Distancia (cv2.distanceTransform)
     │  Máximos locales = centros de espacios abiertos
     ▼
4. Detección de nodos (scipy.ndimage.maximum_filter)
     │
     ▼
5. Cálculo de aristas (línea de visión + distancia euclidiana)
     │
     ▼
escenarioN_grafo.json
```

### Nodo topológico

Cada nodo representa el **centro del espacio libre local** más alejado de las paredes. Su campo `distance_to_wall` (en píxeles) indica la amplitud del espacio alrededor.

### Aristas

Dos nodos se conectan si la distancia es < 100 px y existe línea de visión libre entre ellos. El grafo es no dirigido y bidireccional.

### Planificación A\*

- **Heurística**: distancia euclidiana al nodo destino.
- **Coste de arista**: distancia en píxeles (proporcional a metros).
- **Garantía**: camino óptimo en distancia.

---

## Resultados por entorno

| Entorno | Nodos | Aristas | Nodo más amplio (dtw) |
|---------|------:|--------:|----------------------:|
| escenario1 | 3 | ~4 | nodo 0 → 1.26 m |
| escenario2 | 8 | ~14 | nodo 0 → 1.55 m |
| escenario3 | 9 | ~12 | nodo 0 → 1.19 m |
| estudio    | 8 | ~10 | nodo 0 → 1.56 m |

---

## Instalación

```bash
pip install pyyaml opencv-python scikit-image numpy scipy
```

---

## Generar los grafos topológicos (sin ROS)

```bash
source ~/Ubuntu20noetic_ws/devel/setup.bash
cd $(rospack find navegacion_topologica)/scripts
python3 generate_all_maps.py
```

Genera en `mapas/topologicos/` las imágenes PNG y en `mapas/grafos/` los JSON.

Para generar las figuras de 3 paneles para la memoria:

```bash
python3 generar_figuras_memoria.py
```

---

## Ejecutar la simulación (navegación interactiva)

### Terminal 1 — Gazebo

```bash
source /opt/ros/noetic/setup.bash && source ~/Ubuntu20noetic_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch navegacion_geometrica turtlebot3_escenario3.launch
```

Esperar: `Spawn status: Successfully spawned entity`

### Terminal 2 — Navigation stack (mapa + AMCL + move_base)

```bash
source /opt/ros/noetic/setup.bash && source ~/Ubuntu20noetic_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch \
  map_file:=$(rospack find navegacion_geometrica)/mapas/exploration/mapa_escenario3.yaml
```

Esperar: nodos `/amcl`, `/move_base`, `/rviz` activos.

### Terminal 3 — Navegación topológica

```bash
source /opt/ros/noetic/setup.bash && source ~/Ubuntu20noetic_ws/devel/setup.bash
rosrun navegacion_topologica topological_navigation.py \
  $(rospack find navegacion_topologica)/mapas/grafos/escenario3_grafo.json
```

El sistema muestra la lista de nodos y pide un número de destino:

```
=== MAPA TOPOLOGICO — NODOS DISPONIBLES ===
  Nodo 0: ( 0.80,  4.50) m   dist_pared=1.19 m
  Nodo 1: ( 0.20, -0.70) m   dist_pared=1.00 m
  Nodo 2: ( 0.00, -2.65) m   dist_pared=0.78 m
  ...
  Nodo 8: ( 2.85,  1.65) m   dist_pared=0.65 m

Nodo actual: 1
Destino (0-8): _
```

Escribe el número de nodo destino y el robot planifica y navega.

### Lanzar todo con un launch file (alternativa)

```bash
roslaunch navegacion_topologica topological_navigation.launch escenario:=3
```

---

## Estructura del grafo JSON

```json
{
  "nodes": [
    { "id": 0, "x_pixel": 196, "y_pixel": 115, "distance_to_wall": 23.8 }
  ],
  "edges": [
    { "from_node": 0, "to_node": 5, "distance_pixels": 106.3 }
  ],
  "metadata": {
    "resolution_m_per_pixel": 0.05,
    "origin_x": -0.2,
    "origin_y": -0.2,
    "width": 384,
    "height": 384
  }
}
```

Conversión coordenadas píxel → metros:
```
x_m = origin_x + x_pixel * resolution
y_m = origin_y + (height - y_pixel) * resolution
```

---

## Parámetros ajustables

En `topological_mapper.py` se pueden modificar:

| Parámetro | Valor por defecto | Efecto |
|-----------|------------------|--------|
| `min_distance` | 15 px | Separación mínima entre nodos. ↑ = menos nodos |
| `max_distance` | 100 px | Distancia máxima para conectar nodos. ↑ = grafo más denso |
| `skeleton_dilation` | 3 px | Grosor del esqueleto. ↑ = más robusto, menos detallado |

---

## Estructura de archivos

```
navegacion_topologica/
├── src/
│   ├── topological_mapper.py      # Clase TopologicalMapper
│   ├── path_planner.py            # Clase TopologicalPathPlanner (A*)
│   └── semantic_integration.py    # (usado por módulo semántico)
├── scripts/
│   ├── generate_all_maps.py       # Genera grafos para los 4 entornos
│   ├── generar_figuras_memoria.py # Figuras de 3 paneles para la memoria
│   ├── graph_visualizer.py        # Markers RViz del grafo
│   ├── topological_navigation.py  # Nodo ROS principal
│   └── test_all.py                # Prueba planificador en todos los entornos
├── launch/
│   └── topological_navigation.launch
└── mapas/
    ├── grafos/                    # JSON con grafos (base y semánticos)
    ├── topologicos/               # PNG del grafo sobre el mapa
    └── memoria/                   # Figuras de 3 paneles
```

---

## Guía para el vídeo

### Demostración recomendada: escenario3 (9 nodos, el más complejo)

1. **Mostrar RViz** con el grafo publicado (`/topological_graph_markers`): se ven los nodos como esferas azules y las aristas como líneas.
2. En el terminal, la lista de nodos con posición y distancia a pared.
3. **Demo 1 — Ruta directa**: escribir nodo 0 (extremo del mapa). El planificador muestra la ruta `1 → 5 → 0`. El robot navega waypoint a waypoint.
4. **Demo 2 — Ruta larga**: escribir nodo 6. La ruta `1 → 6` pasa por el extremo opuesto del mapa.
5. Mostrar en pantalla: el path de move_base en RViz y los logs de la terminal con el estado de cada waypoint.

---

## Puntos clave para la memoria

- **Esqueletonización**: transforma el espacio libre en una línea central 1D que preserva la topología del entorno.
- **Máximos locales de la DT**: garantizan que los nodos están en los puntos más alejados de los obstáculos → posiciones robustas para navegar.
- **A\* vs. Dijkstra**: A\* es óptimo con la heurística euclidiana (admisible). Dijkstra encontraría la misma ruta pero explorando más nodos.
- **Limitación**: el grafo es estático (se genera una sola vez). Obstáculos dinámicos los maneja `move_base`, no el planificador topológico.
- Los grafos JSON generados aquí son la entrada del módulo de navegación semántica.
