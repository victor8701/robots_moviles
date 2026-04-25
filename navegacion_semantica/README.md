# Navegación Semántica — Clasificación de Regiones 3D

Módulo 3 del proyecto. Enriquece el grafo topológico con **información semántica** derivada del análisis volumétrico 3D de cada nodo. El robot puede navegar hacia tipos de región ("ir a una habitación") y el planificador A\* usa costes semánticos que dan prioridad a espacios amplios y seguros.

---

## Concepto: clasificación semántica basada en regiones 3D

La navegación tradicional ve el espacio como obstáculos y espacio libre. La navegación semántica añade la pregunta: **¿qué tipo de espacio es este?**

```
Nivel geométrico:  "espacio libre de 384×384 píxeles"
Nivel topológico:  "nodo 1 en (0.20, -0.70) m"
Nivel semántico:   "nodo 1 es una HABITACIÓN — espacio amplio, bajo riesgo"
```

### Técnica implementada: proyección 2D→3D con `SemanticMapper3D`

El clasificador (`semantic_mapper_3d.py`) aplica razonamiento volumétrico 3D sobre los datos geométricos del mapa 2D:

1. **Extrae** el radio de espacio libre de cada nodo (`distance_to_wall` en metros).
2. **Proyecta** al espacio 3D: `width = distance_to_wall × 2`, `height = 2.5 m` (altura interior estándar), `volume = width² × height`.
3. **Clasifica** por dimensiones volumétricas:

| Etiqueta 3D | Ancho (width) | Altura | → Etiqueta topológica |
|-------------|--------------|--------|-----------------------|
| `wall`      | < 0.8 m      | > 1.5 m | `narrow`             |
| `corridor`  | 0.8 – 2.0 m  | > 1.5 m | `corridor` / `junction` |
| `room`      | 2.0 – 3.0 m  | > 1.5 m | `room`               |
| `large_room`| > 3.0 m      | > 1.5 m | `room`               |

4. **Refinamiento topológico**: si el nodo es `corridor` y tiene ≥ 3 direcciones abiertas (ray-casting cardinal) → se reclasifica como `junction` (cruce de pasillos).

### Atributos semánticos por tipo de región

| Tipo | Coste traversal | Dificultad | Riesgo |
|------|:--------------:|:----------:|:------:|
| `room`     | 0.70 | easy   | low    |
| `corridor` | 1.00 | medium | low    |
| `junction` | 1.20 | medium | medium |
| `narrow`   | 1.80 | hard   | high   |

---

## Pipeline completo

```
escenarioN_grafo.json  +  mapa_escenarioN.pgm
           │                       │
           └───────────────────────┘
                       │
               semantic_enricher.py
                       │  (SemanticMapper3D → classify_node_from_map)
                       ▼
          escenarioN_grafo_semantico.json
                       │
          ┌────────────┴────────────┐
          │                         │
  semantic_path_planner.py   semantic_navigation.py
  (A* con costes semánticos)   (nodo ROS interactivo)
          │
   resultados/
   ├── escenarioN_semantico.png         (mapa coloreado)
   ├── escenarioN_pipeline_semantico.png (mapa geo + semántico)
   ├── escenarioN_comparacion_rutas.png  (topológica vs. semántica)
   └── tabla_resultados.md
```

---

## Resultados de clasificación (4 entornos)

| Entorno | Nodos | Rooms | Junctions | Coste medio |
|---------|------:|------:|----------:|------------:|
| escenario1 | 3 | 2 | 1 | 0.87 |
| escenario2 | 8 | 3 | 5 | 1.01 |
| escenario3 | 9 | 2 | 7 | 1.09 |
| estudio    | 8 | 8 | 0 | 0.70 |

El entorno `estudio` tiene todos los nodos clasificados como `room` porque todos los puntos óptimos del grafo caen en habitaciones con radio > 1.0 m.

### Pares con rutas DIFERENTES en escenario3

El planificador semántico elige un camino distinto al topológico en 8 de los 36 pares posibles. El caso más llamativo:

| Par | Topológica | Semántica | Coste sem. |
|-----|-----------|-----------|:----------:|
| **2 → 6** | `[2,3,7,6]` 5.51 m (todo junctions) | `[2,1,6]` 6.16 m (pasa por room) | −1.70 |
| 2 → 7 | `[2,3,7]` | `[2,1,7]` | rodeo por room |
| 2 → 5 | `[2,5]` | `[2,1,5]` | rodeo por room |

La semántica prefiere rutas más largas en metros pero con menor coste semántico acumulado (menor riesgo, menor dificultad).

---

## Instalación

```bash
pip install pyyaml opencv-python scikit-image numpy scipy
```

---

## Paso 0: generar los grafos semánticos (una sola vez)

```bash
source ~/Ubuntu20noetic_ws/devel/setup.bash
cd $(rospack find navegacion_semantica)
python3 semantic_enricher.py
```

Genera `escenarioN_grafo_semantico.json` en `navegacion_topologica/mapas/grafos/` y las imágenes en `resultados/`.

Para regenerar las figuras para la memoria:

```bash
python3 generate_semantic_results.py
```

---

## Probar el planificador sin ROS

```bash
cd $(rospack find navegacion_semantica)

# Ver clasificación 3D de todos los nodos y comparar rutas
python3 semantic_path_planner.py \
  $(rospack find navegacion_topologica)/mapas/grafos/escenario3_grafo_semantico.json \
  2 6 --compare
```

Salida esperada:
```
NODOS CON INFORMACION SEMANTICA
  ID    Pos (m)         Tipo       Cost   Dif.   Riesgo
   0  ( 0.80,  4.50)  room       0.70   easy   low
   1  ( 0.20, -0.70)  room       0.70   easy   low
   2  ( 0.00, -2.65)  junction   1.20   medium medium
   ...

COMPARACION: Nodo 2 → Nodo 6
🔵 TOPOLOGICA: 2→3→7→6  (5.51m, coste 4.8)
🟢 SEMANTICA:  2→1→6    (6.16m, coste 3.1)
→ Rutas DIFERENTES: la semántica pasa por la habitación
```

---

## Simulación completa con ROS

Ver [SIMULACION.md](SIMULACION.md) para la secuencia detallada de 3 terminales, configuración de RViz y guía de grabación del vídeo.

### Resumen de comandos (escenario3)

```bash
# Terminal 1 — Gazebo
export TURTLEBOT3_MODEL=burger
roslaunch navegacion_geometrica turtlebot3_escenario3.launch

# Terminal 2 — Navigation stack
roslaunch turtlebot3_navigation turtlebot3_navigation.launch \
  map_file:=$(rospack find navegacion_geometrica)/mapas/exploration/mapa_escenario3.yaml

# Terminal 3 — Nodo semántico
rosrun navegacion_semantica semantic_navigation.py \
  $(rospack find navegacion_topologica)/mapas/grafos/escenario3_grafo_semantico.json
```

---

## Estructura de archivos

```
navegacion_semantica/
├── semantic_mapper_3d.py          # Clasificador 3D (SemanticMapper3D)
├── semantic_enricher.py           # Enriquece los 4 entornos → JSON semánticos
├── semantic_integration.py        # Integra SemanticMapper3D con el grafo
├── semantic_path_planner.py       # A* con costes semánticos
├── generate_semantic_results.py   # Genera figuras y tabla para la memoria
├── scripts/
│   └── semantic_navigation.py     # Nodo ROS interactivo
├── launch/
│   └── semantic_navigation.launch
├── resultados/
│   ├── escenario{1,2,3}_semantico.png
│   ├── escenario{1,2,3}_pipeline_semantico.png
│   ├── escenario{1,2,3}_comparacion_rutas.png
│   └── tabla_resultados.md
└── SIMULACION.md                  # Guía completa para el vídeo
```

---

## Guía para el vídeo

La secuencia completa está en [SIMULACION.md](SIMULACION.md). Resumen de los momentos clave:

1. **Mapa semántico en RViz**: esferas verdes (rooms) y moradas (junctions) sobre el mapa. Narrar: *"SemanticMapper3D clasifica cada nodo por su volumen 3D estimado"*.
2. **Demo ruta diferente** (par 2→6): mostrar la comparativa en terminal antes de confirmar. El robot elige el camino más largo pero más seguro pasando por la habitación.
3. **Navegación por tipo** (`r` en el menú): el sistema localiza automáticamente la habitación más amplia y navega hacia ella.
4. **Cambio de modo** (`t`): contrastar topológico vs. semántico con el mismo par de nodos.

---

## Puntos clave para la memoria

### Qué es la clasificación semántica basada en regiones 3D
- Extiende el mapa topológico añadiendo **significado** a cada región.
- La clase `SemanticMapper3D` proyecta datos 2D a espacio 3D (width, height, volume) y clasifica por dimensiones métricas.
- Reproduce la lógica que aplicaría sobre una nube de puntos real, usando la Transformada de Distancia como estimador del radio libre.

### Cómo mejora la navegación
- **Costes heterogéneos**: habitaciones cuestan 0.70, cruces 1.20 → el planificador encuentra rutas menos arriesgadas aunque sean métricamente más largas.
- **Navegación por semántica**: el usuario puede decir "ir a la habitación" sin conocer el ID del nodo.
- **Información para el operador**: el mapa coloreado muestra de un vistazo qué zonas son más complejas para navegar.

### Limitaciones y líneas futuras
- La clasificación usa datos geométricos 2D proyectados; una cámara de profundidad real (RGB-D) permitiría clasificación volumétrica directa.
- El nivel semántico actual (room/junction) es funcional pero no distingue tipo de habitación (cocina vs. dormitorio), para lo que haría falta detección de objetos (YOLO/Mask R-CNN).
- En grafos pequeños con una sola ruta posible, la semántica no cambia el camino pero sí la estimación del coste y riesgo.

### Referencias científicas clave
1. Thrun, Burgard & Fox (2005). *Probabilistic Robotics*. MIT Press. — Base teórica de mapas métricos y topológicos.
2. Rusu & Cousins (2011). "3D is here: PCL." ICRA. — Procesamiento de nubes de puntos y clustering 3D.
3. Felzenszwalb & Huttenlocher (2004). "Efficient graph-based image segmentation." IJCV. — Segmentación de regiones coherentes.
4. Boykov & Kolmogorov (2004). "Min-cut/max-flow for energy minimization in vision." IEEE TPAMI. — Particionamiento de regiones semánticas.
5. He et al. (2017). "Mask R-CNN." ICCV. — Aproximación deep learning para segmentación de instancias (línea futura).
