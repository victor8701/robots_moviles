# Comandos de simulación — Navegación Semántica

Secuencia completa para probar y grabar el vídeo. Usar **escenario2** como demo
principal (8 nodos, bien conectado, incluye nodo `room` y nodo `junction`).

> **Requisito previo**: tener los grafos semánticos generados. Si no existen,
> ejecutar primero el **Paso 0** (solo una vez).

---

## Paso 0 — Generar grafos semánticos (una sola vez, sin ROS)

```bash
source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash
cd $(rospack find navegacion_semantica)
python3 semantic_enricher.py
```

Esto crea en `navegacion_topologica/mapas/grafos/`:
- `escenario1_grafo_semantico.json`
- `escenario2_grafo_semantico.json`
- `escenario3_grafo_semantico.json`
- `estudio_grafo_semantico.json`

Y en `navegacion_semantica/resultados/`:
- Imágenes colorizadas por tipo de región
- `tabla_resultados.md` (para la memoria)

---

## Terminal 1 — Gazebo

```bash
source /opt/ros/noetic/setup.bash && source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash && export TURTLEBOT3_MODEL=burger && roslaunch navegacion_geometrica turtlebot3_escenario2.launch
```

---

## Terminal 2 — Navigation stack (mapa + AMCL + move_base)

La pose inicial correcta en el marco del mapa es (0,0), ya que `gmapping` generó el mapa tomando como origen la posición de inicio del robot. 

```bash
source /opt/ros/noetic/setup.bash && source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash && export TURTLEBOT3_MODEL=burger && roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$(rospack find navegacion_geometrica)/mapas/exploration/mapa_escenario2.yaml
```

---

## Terminal 3 — Navegación semántica interactiva

```bash
source /opt/ros/noetic/setup.bash && source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash && rosrun navegacion_semantica semantic_navigation.py $(rospack find navegacion_topologica)/mapas/grafos/escenario2_grafo_semantico.json
```

El sistema muestra una tabla con los nodos y su **tipo semántico**:

```
=======================================================================
NAVEGACION SEMANTICA — NODOS DISPONIBLES
=======================================================================
  ID  Posicion (m)          Tipo          Cost   Riesgo  Estado
  ---------------------------------------------------------------------
   0  ( -2.05,  -3.40)      corridor      1.00      low  <-- actual
   1  (  1.50,  -5.55)      room          0.70      low
   2  ( -2.35,  -5.25)      corridor      1.00      low
   3  ( -1.85,  -0.50)      corridor      1.00      low
   4  (  1.50,  -0.05)      corridor      1.00      low
   5  (  1.60,  -1.85)      junction      1.20   medium
   6  (  0.20,  -0.10)      corridor      1.00      low
   7  (  0.10,  -4.00)      corridor      1.00      low

Opciones:
  [0-N]   Navegar a nodo por ID
  [r]     Navegar a la habitacion (room) mas cercana
  [c]     Navegar al corredor mas cercano
  [t]     Alternar modo semantico / topologico
  [m]     Republica markers en RViz
  [q]     Salir
```

---

## Configurar RViz — Topics a añadir

En RViz, añadir los siguientes displays:

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/semantic_markers` | `MarkerArray` | Esferas coloreadas por tipo de región |
| `/semantic_path`    | `Path`        | Ruta planificada en curso |
| `/map`              | `Map`         | Mapa de navegación |
| `/move_base/local_costmap/costmap` | `Map` | Costmap local |

**Colores de los markers semánticos:**
- 🟢 Verde — `room` (habitación, coste 0.70)
- 🔵 Azul oscuro — `corridor` (corredor, coste 1.00)
- 🟣 Morado — `junction` (cruce, coste 1.20)
- 🟡 Amarillo — `narrow` (zona estrecha, coste 1.80)

---

## Alternativa — Launch file completo

```bash
# Escenario 2 (recomendado para demo)
roslaunch navegacion_semantica semantic_navigation.launch escenario:=escenario2

# Escenario 3
roslaunch navegacion_semantica semantic_navigation.launch escenario:=escenario3

# Estudio
roslaunch navegacion_semantica semantic_navigation.launch escenario:=estudio
```

> ⚠️ El launch file incluye Gazebo + el nodo semántico pero **no** el navigation stack.
> Hay que lanzar el Terminal 2 por separado.

---

## Secuencia de demo para el vídeo

1. **Mostrar RViz** con markers semánticos: esferas de colores sobre el mapa
2. La tabla del terminal muestra los 8 nodos con su tipo semántico
3. **Demo 1 — Navegar por ID**:
   - Escribir `1` → nodo `room` (el más económico, coste 0.70)
   - El sistema muestra comparativa: ruta topológica vs. semántica antes de navegar
   - Confirmar con `s` → el robot navega; en RViz se ve el path en `/semantic_path`
4. **Demo 2 — Navegar por tipo semántico**:
   - Escribir `r` → el sistema encuentra automáticamente la habitación (nodo 1)
   - Confirmar con `s`
5. **Demo 3 — Cambiar modo**:
   - Escribir `t` para cambiar a modo topológico clásico
   - Navegar al nodo 5 (`junction`, riesgo `medium`)
   - Mostrar que la ruta es la misma pero con coste semántico mayor
6. Volver a modo semántico con `t`

---

## Generar figuras de memoria (sin ROS, en cualquier momento)

```bash
cd $(rospack find navegacion_semantica)
python3 generate_semantic_results.py
```

Genera en `resultados/`:
- `escenario{1,2,3}_pipeline_semantico.png` — mapa + colores semánticos
- `escenario{1,2,3}_comparacion_rutas.png` — ruta topológica vs. semántica
- `tabla_resultados.md` — tabla lista para copiar en la memoria

---

## Probar el planificador semántico sin ROS

```bash
cd $(rospack find navegacion_semantica)

# Ruta con comparativa (topologica vs. semantica) — escenario2, nodo 0 → nodo 1 (room)
python3 semantic_path_planner.py \
  $(rospack find navegacion_topologica)/mapas/grafos/escenario2_grafo_semantico.json \
  0 1 --compare
```

---

## Notas

- Si `move_base` no arranca: `sudo apt install ros-noetic-turtlebot3-navigation`
- Si el grafo semántico no existe: ejecutar **Paso 0** primero
- El nodo semántico reutiliza el mismo `move_base` que la navegación topológica
- Para cambiar los umbrales de clasificación (room/corridor/narrow), editar las
  constantes al inicio de `semantic_integration.py`:
  ```python
  ROOM_MIN_DIST_WALL = 20   # px (1.0 m a 0.05 m/px)
  CORRIDOR_MIN_DIST  = 10   # px (0.5 m)
  ```
