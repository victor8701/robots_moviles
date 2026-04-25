# Comandos de simulación — Navegación Topológica

Secuencia completa para probar y grabar el vídeo. Usar escenario2 como demo
principal (8 nodos, bien conectado). Abrir 4 terminales.

---

## Terminal 1 — Gazebo

```bash
source /opt/ros/noetic/setup.bash && source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash && export TURTLEBOT3_MODEL=burger && roslaunch navegacion_geometrica turtlebot3_escenario2.launch
```

## Terminal 2 — Navigation stack (mapa + AMCL + move_base)

La pose inicial debe coincidir con el spawn del robot en Gazebo (escenario2: x=4.0, y=-1.0).
Pasar los parámetros directamente evita tener que usar "2D Pose Estimate" en RViz.

```bash
source /opt/ros/noetic/setup.bash && source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash && export TURTLEBOT3_MODEL=burger && roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$(rospack find navegacion_geometrica)/mapas/exploration/mapa_escenario2.yaml initial_pose_x:=4.0 initial_pose_y:=-1.0
```

## Terminal 3 — Visualizador del grafo topológico en RViz

```bash
source /opt/ros/noetic/setup.bash && source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash && rosrun navegacion_topologica graph_visualizer.py $(rospack find navegacion_topologica)/mapas/grafos/escenario2_grafo.json
```

Añadir en RViz los topics:
- /topological_graph/nodes  (MarkerArray)
- /topological_graph/edges  (MarkerArray)
- /topological_path         (Path)

## Terminal 4 — Navegación topológica interactiva

```bash
source /opt/ros/noetic/setup.bash && source /home/ubuntu20/Ubuntu20noetic_ws/devel/setup.bash && rosrun navegacion_topologica topological_navigation.py $(rospack find navegacion_topologica)/mapas/grafos/escenario2_grafo.json
```

El sistema muestra los nodos disponibles y su posición, pide el nodo destino
por teclado y guía el robot de nodo en nodo mostrando el yaw calculado.

---

## Alternativa: usando el launch file (escenarios 1/2/3)

```bash
# Escenario 1
roslaunch navegacion_topologica topological_navigation.launch escenario:=1

# Escenario 2
roslaunch navegacion_topologica topological_navigation.launch escenario:=2

# Estudio
roslaunch navegacion_topologica topological_navigation.launch grafo:=estudio_grafo.json
```

---

## Secuencia de demo para el vídeo

1. Mostrar RViz con el grafo topológico superpuesto al mapa
2. En la terminal, el sistema muestra los 8 nodos con sus coordenadas
3. Escribir un nodo destino (p.ej. nodo 5)
4. El sistema imprime: "Ruta: 0 -> 2 -> 5" y comienza la navegación
5. El robot se mueve de nodo a nodo; en RViz se ve el path topológico en azul
6. Al llegar: el sistema pregunta de nuevo — navegar a otro nodo
7. Mostrar ruta más larga (p.ej. 0 -> 7) para ver múltiples waypoints

---

## Notas

- Si move_base no arranca: verificar que el paquete turtlebot3_navigation está instalado
  (`sudo apt install ros-noetic-turtlebot3-navigation`)
- Para el estudio, el robot aparece en distinta posición inicial (ver x_pos/y_pos en el
  launch de geometrica); pasar initial_pose_x/y al launch de navegación o usar
  "2D Pose Estimate" en RViz
