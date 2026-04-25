# Robots Móviles — Segunda Entrega: Navegación Topológica y Semántica

Máster en Robótica y Automatización · Universidad de Zaragoza

Este repositorio implementa un sistema de navegación multinivel para TurtleBot3 en Gazebo/ROS Noetic:

```
Módulo 1 — Geométrico:   exploración autónoma por fronteras → mapa PGM
Módulo 2 — Topológico:   PGM → grafo de nodos → A* entre nodos
Módulo 3 — Semántico:    grafo + análisis 3D → costes semánticos → A* semántico
```

---

## Estructura del proyecto

```
robots_moviles/
├── navegacion_geometrica/     # Exploración autónoma + evaluación cobertura
├── navegacion_topologica/     # Generación de grafos + A* + nodo ROS
├── navegacion_semantica/      # Clasificación 3D + A* semántico + nodo ROS
├── SLAM/                      # Práctica EKF-SLAM en MATLAB (independiente)
└── doc/                       # Diapositivas del trabajo
```

---

## Entornos disponibles

| Entorno | Complejidad | Nodos | Aristas | Escena |
|---------|:-----------:|------:|--------:|--------|
| escenario1 | Baja  |  3 |  3 | Sala rectangular simple |
| escenario2 | Media |  8 | 14 | Pasillos en L con salas |
| escenario3 | Alta  |  9 | 12 | Red de pasillos estrechos |
| estudio    | Media |  8 | 19 | Planta con varias habitaciones |

**El escenario recomendado para el vídeo es escenario3** (el más complejo, con rutas topológica y semántica claramente distintas).

---

## Inicio rápido

### 1. Instalar dependencias Python

```bash
pip install pyyaml opencv-python scikit-image numpy scipy
```

### 2. Compilar el workspace

```bash
cd ~/Ubuntu20noetic_ws
catkin_make
source devel/setup.bash
```

### 3. Generar todos los grafos (topológicos y semánticos)

```bash
# Grafos topológicos
cd $(rospack find navegacion_topologica)/scripts
python3 generate_all_maps.py

# Grafos semánticos + figuras para la memoria
cd $(rospack find navegacion_semantica)
python3 semantic_enricher.py
python3 generate_semantic_results.py
```

---

## Guía de simulación por módulo

| Módulo | Guía detallada |
|--------|---------------|
| Navegación geométrica | [navegacion_geometrica/README.md](navegacion_geometrica/README.md) |
| Navegación topológica | [navegacion_topologica/README.md](navegacion_topologica/README.md) |
| Navegación semántica | [navegacion_semantica/SIMULACION.md](navegacion_semantica/SIMULACION.md) |

---

## Simulación rápida — escenario3 completo

Abre **4 terminales**. En cada una, hacer antes:

```bash
source /opt/ros/noetic/setup.bash && source ~/Ubuntu20noetic_ws/devel/setup.bash && export TURTLEBOT3_MODEL=burger
```

| Terminal | Comando |
|----------|---------|
| **1 — Gazebo** | `roslaunch navegacion_geometrica turtlebot3_escenario3.launch` |
| **2 — Nav stack** | `roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$(rospack find navegacion_geometrica)/mapas/exploration/mapa_escenario3.yaml` |
| **3 — Topológica** | `rosrun navegacion_topologica topological_navigation.py $(rospack find navegacion_topologica)/mapas/grafos/escenario3_grafo.json` |
| **4 — Semántica** | `rosrun navegacion_semantica semantic_navigation.py $(rospack find navegacion_topologica)/mapas/grafos/escenario3_grafo_semantico.json` |

> Solo lanzar **terminal 3 o 4**, no las dos a la vez (ambas usan `move_base`).

---

## Configurar RViz para el vídeo

Añadir en RViz:

| Topic | Tipo | Para |
|-------|------|------|
| `/map` | Map | Mapa de navegación |
| `/semantic_markers` | MarkerArray | Esferas de colores (módulo semántico) |
| `/semantic_path` | Path | Ruta planificada semántica |
| `/move_base/local_costmap/costmap` | Map | Costmap local |

---

## Demostración clave para el vídeo (semántica)

El par **nodo 2 → nodo 6** en escenario3 muestra rutas completamente distintas:

```
🔵 Topológica: [2 → 3 → 7 → 6]  — 5.51 m, todo cruces, coste sem. 4.8
🟢 Semántica:  [2 → 1 → 6]       — 6.16 m, pasa por habitación, coste sem. 3.1
```

La semántica elige una ruta 0.65 m más larga pero que pasa por una habitación (coste 0.70) evitando dos cruces (coste 1.20 cada uno). Resultado: −1.70 unidades de coste semántico.

---

## Archivos de resultados para la memoria

Todos los archivos necesarios para la memoria se generan automáticamente:

```
navegacion_topologica/mapas/
├── topologicos/
│   ├── escenario{1,2,3}_topologico.png   # Grafo sobre el mapa
│   └── estudio_topologico.png
└── memoria/
    └── escenario{1,2,3}_memoria.png      # Figuras de 3 paneles

navegacion_semantica/resultados/
├── escenario{1,2,3}_semantico.png        # Mapa coloreado por tipo de región
├── escenario{1,2,3}_pipeline_semantico.png  # Mapa geo vs. semántico
├── escenario{1,2,3}_comparacion_rutas.png   # Ruta topo vs. semántica
└── tabla_resultados.md                   # Tabla lista para copiar en la memoria
```

---

## Troubleshooting

| Problema | Solución |
|---------|---------|
| `ModuleNotFoundError: yaml` | `pip install pyyaml` |
| `ModuleNotFoundError: skimage` | `pip install scikit-image` |
| `move_base no disponible` | `sudo apt install ros-noetic-turtlebot3-navigation` |
| Grafo semántico no encontrado | Ejecutar `python3 semantic_enricher.py` primero |
| El robot no se mueve | Verificar que `/amcl` y `/move_base` están activos: `rosnode list` |
| RViz no muestra markers | Pulsar `m` en el terminal de navegación semántica para republicar |

---

## Rama de entrega

```
git checkout entrega_individual_topo_sem
```
