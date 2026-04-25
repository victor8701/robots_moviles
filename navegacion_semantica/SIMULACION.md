# Guía de simulación — Navegación Semántica (escenario3)

Secuencia completa para probar y grabar el vídeo usando **escenario3**
(el más complejo: 9 nodos, 2 habitaciones, 7 cruces).

La clasificación semántica la realiza **SemanticMapper3D**: proyecta la
distancia-a-pared y la conectividad 2D al espacio 3D (ancho × alto × volumen)
y clasifica cada nodo en `room / corridor / junction / narrow`.

---

## Paso 0 — Generar grafos semánticos (una sola vez)

```bash
source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash
cd $(rospack find navegacion_semantica)
python3 semantic_enricher.py
```

Crea en `navegacion_topologica/mapas/grafos/`:
- `escenario3_grafo_semantico.json` ← el que usaremos

Y en `navegacion_semantica/resultados/`:
- Imágenes colorizadas y tabla de resultados

**Resultado esperado para escenario3:**

```
Distribución:  room × 2  |  junction × 7
Nodo  0 (0.80,  4.50):  room      cost=0.70  risk=low
Nodo  1 (0.20, -0.70):  room      cost=0.70  risk=low
Nodo  2 (0.00, -2.65):  junction  cost=1.20  risk=medium
...
Nodo  8 (2.85,  1.65):  junction  cost=1.20  risk=medium
```

---

## Lanzar la simulación (3 terminales)

### Terminal 1 — Gazebo con escenario3

```bash
source /opt/ros/noetic/setup.bash
source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch navegacion_geometrica turtlebot3_escenario3.launch
```

Esperar hasta ver en terminal: `Spawn status: Successfully spawned entity`

---

### Terminal 2 — Stack de navegación (AMCL + move_base)

```bash
source /opt/ros/noetic/setup.bash
source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch \
  map_file:=$(rospack find navegacion_geometrica)/mapas/exploration/mapa_escenario3.yaml
```

Esperar hasta ver: `/amcl`, `/move_base` y `/rviz` en `rosnode list`

---

### Terminal 3 — Nodo semántico interactivo

```bash
source /opt/ros/noetic/setup.bash
source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash
rosrun navegacion_semantica semantic_navigation.py \
  $(rospack find navegacion_topologica)/mapas/grafos/escenario3_grafo_semantico.json
```

El sistema muestra la tabla semántica y espera comandos:

```
======================================================================
NAVEGACION SEMANTICA — NODOS DISPONIBLES
======================================================================
  ID   Posicion (m)          Tipo          Cost   Riesgo  Estado
  --------------------------------------------------------------------
   0  (  0.80,   4.50)      room          0.70      low
   1  (  0.20,  -0.70)      room          0.70      low  <-- actual
   2  (  0.00,  -2.65)      junction      1.20   medium
   3  (  2.15,  -2.85)      junction      1.20   medium
   4  (  2.70,   3.65)      junction      1.20   medium
   5  ( -0.05,   2.25)      junction      1.20   medium
   6  (  4.30,  -1.60)      junction      1.20   medium
   7  (  2.20,  -1.60)      junction      1.20   medium
   8  (  2.85,   1.65)      junction      1.20   medium

Opciones:
  [0-N]   Navegar a nodo por ID
  [r]     Navegar a la habitacion (room) mas cercana
  [c]     Navegar al corredor mas cercano
  [t]     Alternar modo semantico / topologico
  [m]     Republica markers en RViz
  [q]     Salir
```

---

## Configurar RViz

En RViz añadir los siguientes displays:

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/semantic_markers` | `MarkerArray` | Esferas coloreadas por tipo de región |
| `/semantic_path`    | `Path`        | Ruta planificada en curso |
| `/map`              | `Map`         | Mapa de navegación |
| `/move_base/local_costmap/costmap` | `Map` | Costmap local |

**Colores de los markers semánticos:**
- 🟢 Verde — `room` (habitación, coste 0.70, riesgo bajo)
- 🔵 Azul — `corridor` (corredor, coste 1.00)
- 🟣 Morado — `junction` (cruce, coste 1.20, riesgo medio)
- 🟡 Amarillo — `narrow` (zona estrecha, coste 1.80, riesgo alto)

---

## Secuencia de demo para el vídeo

### Parte 1 — Mostrar el mapa semántico

1. Abrir RViz con el topic `/semantic_markers` activo
2. Se ven **2 esferas verdes** (rooms: nodo 0 y 1) y **7 esferas moradas** (junctions)
3. En el terminal, la tabla confirma los tipos y costes
4. Narrar: *"SemanticMapper3D clasifica cada nodo por su volumen 3D: los nodos 0 y 1 tienen radio libre > 1 m → width > 2 m → room. El resto son más estrechos → junction."*

---

### Parte 2 — Demo clave: rutas DIFERENTES (nodo 2 → nodo 6)

> Este es el par donde topológica y semántica eligen caminos distintos — el más
> llamativo para el vídeo.

En el terminal (modo **SEMANTICO** activo):

```
Comando: 2          ← navegar al nodo 2 primero si el robot no está ya ahí
```

Luego, una vez en nodo 2:

```
Comando: 6
```

El sistema muestra antes de confirmar:

```
COMPARACION: Nodo 2 → Nodo 6
=================================================================
🔵 TOPOLOGICA:  2 → 3 → 7 → 6    (5.51 m, coste sem. 4.8, todo junctions)
🟢 SEMANTICA:   2 → 1 → 6         (6.16 m, coste sem. 3.1, pasa por room)

DIFERENCIA: -1 nodo, -1.70 de coste semántico
→ Rutas DIFERENTES: la semántica rodea por la habitación
```

Confirmar con `s` → el robot sigue la ruta semántica `2 → 1 → 6`.

**Qué mostrar en pantalla dividida:**
- RViz: el path `/semantic_path` en rojo pasando por nodo 1 (esfera verde)
- Terminal: el log de waypoints con `[room]` en nodo 1

---

### Parte 3 — Navegar por tipo semántico

```
Comando: r          ← "ir a la habitación más amplia"
```

El sistema responde:
```
  -> Room mas amplia: nodo 0
  COMPARACION: Nodo actual → Nodo 0
  ...
Confirmar? [s/n]: s
```

El robot navega a la habitación más espaciosa (nodo 0, radio 1.19 m).

---

### Parte 4 — Cambio de modo topológico vs semántico

```
Comando: t          ← cambiar a modo TOPOLOGICO
Comando: 6          ← navegar a nodo 6 en modo topológico
```

Muestra la misma ruta `2 → 3 → 7 → 6` (sin rodeo por la habitación).
Cambiar de nuevo con `t` a modo semántico.

---

## Probar el planificador sin ROS (verificación rápida)

```bash
cd $(rospack find navegacion_semantica)

# Ver todos los nodos con su clasificación 3D
python3 semantic_path_planner.py \
  $(rospack find navegacion_topologica)/mapas/grafos/escenario3_grafo_semantico.json \
  2 6 --compare

# Regenerar todas las figuras para la memoria
python3 generate_semantic_results.py
```

**Pares con rutas diferentes en escenario3** (útiles para la memoria):

| Par | Topológica | Semántica | Diferencia |
|-----|-----------|-----------|-----------|
| 2 → 6 | `[2,3,7,6]` 5.51 m | `[2,1,6]` 6.16 m | -1 nodo, -1.70 coste |
| 2 → 7 | `[2,3,7]` | `[2,1,7]` | rodeo por room nodo 1 |
| 2 → 5 | `[2,5]` | `[2,1,5]` | rodeo por room nodo 1 |
| 3 → 4 | `[3,7,1,5,4]` | `[3,7,1,5,0,4]` | rodeo por room nodo 0 |

---

## Notas técnicas

- Si `move_base` no arranca: `sudo apt install ros-noetic-turtlebot3-navigation`
- Si el grafo semántico no existe: ejecutar **Paso 0** primero
- Los umbrales 3D de `SemanticMapper3D._classify_region`:
  - `width < 0.8 m` → pared/zona estrecha → `narrow`
  - `0.8 m ≤ width < 2.0 m` → corredor → `corridor` o `junction` (si ≥3 dir. abiertas)
  - `2.0 m ≤ width < 3.0 m` → habitación → `room`
  - `width ≥ 3.0 m` → sala grande → `room`
- Para cambiar los umbrales editar `_classify_region` en `semantic_mapper_3d.py`
