#!/usr/bin/env python3
"""
Nodo ROS de Navegacion Semantica con TurtleBot3.

Carga el grafo semantico-topologico y ofrece al usuario una interfaz
interactiva enriquecida donde puede:
  - Ver los nodos con su tipo semantico (room/corridor/junction/narrow)
  - Navegar por numero de nodo (como el nodo topologico)
  - Navegar por tipo semantico ("ir a la habitacion mas grande", etc.)
  - Comparar la ruta topologica vs. la semantica antes de ejecutar

La navegacion fisica reutiliza move_base exactamente igual que
topological_navigation.py (mismo mecanismo de waypoints).

Uso:
    rosrun navegacion_semantica semantic_navigation.py <grafo_semantico.json>
    # o via launch:
    roslaunch navegacion_semantica semantic_navigation.launch escenario:=escenario2
"""

import rospy
import actionlib
import json
import sys
import os
import math

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros

# Rutas de modulos locales
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SEM_DIR    = os.path.abspath(os.path.join(_SCRIPT_DIR, '..'))
_TOPO_SRC   = os.path.abspath(os.path.join(_SEM_DIR, '..', 'navegacion_topologica', 'src'))

# Cargar semantic_integration con ruta absoluta para evitar colision de nombres
# con el archivo homonimo en navegacion_topologica/src/
import importlib.util as _ilu
_si_spec = _ilu.spec_from_file_location(
    "sem_integration_local",
    os.path.join(_SEM_DIR, "semantic_integration.py")
)
_si_mod = _ilu.module_from_spec(_si_spec)
_si_spec.loader.exec_module(_si_mod)
REGION_COLORS = _si_mod.REGION_COLORS

# semantic_path_planner esta en _SEM_DIR y depende de path_planner en _TOPO_SRC
if _TOPO_SRC not in sys.path:
    sys.path.insert(0, _TOPO_SRC)
if _SEM_DIR not in sys.path:
    sys.path.insert(0, _SEM_DIR)

from semantic_path_planner import SemanticPathPlanner


# ---------------------------------------------------------------------------
# Colores en formato RGBa (0-1) para RViz markers
# ---------------------------------------------------------------------------

def _bgr_to_rgba(bgr: tuple, alpha: float = 1.0) -> ColorRGBA:
    b, g, r = bgr
    c = ColorRGBA()
    c.r = r / 255.0
    c.g = g / 255.0
    c.b = b / 255.0
    c.a = alpha
    return c


MARKER_COLORS = {
    k: _bgr_to_rgba(v) for k, v in REGION_COLORS.items()
}
MARKER_COLORS['unknown'] = _bgr_to_rgba((160, 160, 160))


# ---------------------------------------------------------------------------
# Clase principal
# ---------------------------------------------------------------------------

class SemanticNavigator:
    """Navegador semantico-topologico para TurtleBot3 en Gazebo."""

    def __init__(self, graph_json_path: str):
        self.planner     = SemanticPathPlanner(graph_json_path)
        self.graph       = self.planner.graph
        self.resolution  = self.graph['metadata']['resolution_m_per_pixel']
        self.origin_x    = self.graph['metadata']['origin_x']
        self.origin_y    = self.graph['metadata']['origin_y']
        self.height      = self.graph['metadata'].get('height', 384)
        self.failed_nodes = set()

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publicadores
        self.path_pub    = rospy.Publisher('/semantic_path',    Path,        queue_size=1, latch=True)
        self.marker_pub  = rospy.Publisher('/semantic_markers', MarkerArray, queue_size=1, latch=True)

        # Publicar path vacio inicial
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp    = rospy.Time.now()
        self.path_pub.publish(empty_path)

        # Publicar markers semanticos (se ven en RViz desde el inicio)
        self._publish_semantic_markers()

        # move_base
        rospy.loginfo("Esperando move_base...")
        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        connected = self.mb_client.wait_for_server(rospy.Duration(60.0))
        if not connected:
            rospy.logerr("move_base no disponible. Comprueba que la navegacion esta activa.")
            sys.exit(1)

        rospy.loginfo("Navegador semantico inicializado")

    # -----------------------------------------------------------------------
    # Conversion de coordenadas
    # -----------------------------------------------------------------------

    def _pixel_to_meters(self, x_px: int, y_px: int):
        x_m = self.origin_x + x_px * self.resolution
        y_m = self.origin_y + (self.height - y_px) * self.resolution
        return x_m, y_m

    # -----------------------------------------------------------------------
    # Posicion del robot
    # -----------------------------------------------------------------------

    def _get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
            return trans.transform.translation.x, trans.transform.translation.y
        except Exception as e:
            rospy.logwarn(f"TF no disponible: {e}")
            return None, None

    def _get_current_node(self):
        rx, ry = self._get_robot_position()
        if rx is None:
            return 0
        px = int((rx - self.origin_x) / self.resolution)
        py = self.height - int((ry - self.origin_y) / self.resolution)
        return self.planner.find_nearest_node(px, py)

    # -----------------------------------------------------------------------
    # Waypoints y navegacion
    # -----------------------------------------------------------------------

    def _compute_waypoints(self, path_ids: list):
        """Calcula waypoints (x, y, yaw) a partir de la lista de IDs."""
        coords = []
        for nid in path_ids:
            node = self.planner.nodes[nid]
            coords.append(self._pixel_to_meters(node['x_pixel'], node['y_pixel']))

        wps = []
        for i, (x, y) in enumerate(coords):
            if i < len(coords) - 1:
                dx = coords[i+1][0] - x
                dy = coords[i+1][1] - y
            else:
                dx = x - coords[i-1][0] if i > 0 else 1.0
                dy = y - coords[i-1][1] if i > 0 else 0.0
            wps.append((x, y, math.atan2(dy, dx)))
        return wps

    def _publish_path(self, waypoints: list):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp    = rospy.Time.now()
        for x, y, yaw in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp    = rospy.Time.now()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _navigate_waypoints(self, path_ids: list, waypoints: list) -> bool:
        """Navega secuencialmente por los waypoints via move_base."""
        to_visit = waypoints[1:]  # el primero es el nodo actual
        for i, (x, y, yaw) in enumerate(to_visit):
            node_id = path_ids[i + 1]
            is_final = (i == len(to_visit) - 1)
            rospy.loginfo(f"Waypoint {i+1}/{len(to_visit)}: "
                          f"nodo {node_id} ({x:.2f}m, {y:.2f}m) "
                          f"[{self.planner.nodes[node_id].get('semantic',{}).get('region_type','?')}]")

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp    = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            if is_final:
                goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
                goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                goal.target_pose.pose.orientation.z = 0.0
                goal.target_pose.pose.orientation.w = 1.0

            self.mb_client.send_goal(goal)
            reached = self.mb_client.wait_for_result(rospy.Duration(90))

            if not reached:
                rospy.logerr(f"Timeout en waypoint {i+1} (nodo {node_id})")
                self.mb_client.cancel_goal()
                self.failed_nodes.add(node_id)
                return False

            state = self.mb_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Nodo {node_id} alcanzado ✓")
            else:
                rospy.logerr(f"Fallo al alcanzar nodo {node_id} (estado={state})")
                self.failed_nodes.add(node_id)
                return False

        rospy.loginfo("Objetivo semantico alcanzado!")
        return True

    def navigate_to_node(self, start_id: int, goal_id: int,
                         use_semantic: bool = True) -> bool:
        """
        Navega del nodo start al nodo goal.

        Args:
            start_id:    ID del nodo inicial.
            goal_id:     ID del nodo objetivo.
            use_semantic: Si True, usa el planificador semantico; si False, el clasico.
        """
        if use_semantic:
            path = self.planner.plan_path_semantic(start_id, goal_id)
            mode_str = "SEMANTICO"
        else:
            path = self.planner.plan_path(start_id, goal_id)
            mode_str = "TOPOLOGICO"

        if not path:
            rospy.logerr(f"No se encontro camino {mode_str} de {start_id} a {goal_id}")
            return False

        rospy.loginfo(f"Ruta {mode_str}: {' -> '.join(map(str, path))}")

        waypoints = self._compute_waypoints(path)
        self._publish_path(waypoints)

        if not waypoints[1:]:
            rospy.loginfo("Ya en el nodo destino")
            return True

        return self._navigate_waypoints(path, waypoints)

    # -----------------------------------------------------------------------
    # Markers RViz
    # -----------------------------------------------------------------------

    def _publish_semantic_markers(self):
        """Publica spheres coloreadas por tipo semantico y texto con etiqueta."""
        ma = MarkerArray()
        stamp = rospy.Time.now()

        for node in self.planner.nodes:
            x_m, y_m = self._pixel_to_meters(node['x_pixel'], node['y_pixel'])
            sem = node.get('semantic', {})
            region_type = sem.get('region_type', 'unknown')
            color = MARKER_COLORS.get(region_type, MARKER_COLORS['unknown'])

            # Esfera
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = stamp
            m.ns   = 'semantic_nodes'
            m.id   = node['id']
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x_m
            m.pose.position.y = y_m
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            radius = 0.15 + sem.get('traversal_cost', 1.0) * 0.05
            m.scale.x = m.scale.y = m.scale.z = radius
            m.color = color
            ma.markers.append(m)

            # Texto
            t = Marker()
            t.header = m.header
            t.ns   = 'semantic_labels'
            t.id   = node['id'] + 1000
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x_m
            t.pose.position.y = y_m
            t.pose.position.z = 0.35
            t.pose.orientation.w = 1.0
            t.scale.z = 0.18
            t.color.r = t.color.g = t.color.b = 0.0
            t.color.a = 1.0
            t.text = f"{node['id']}:{region_type[:4]}"
            ma.markers.append(t)

        self.marker_pub.publish(ma)

    # -----------------------------------------------------------------------
    # Helpers: buscar nodos por tipo semantico
    # -----------------------------------------------------------------------

    def _nodes_by_type(self, region_type: str):
        return [n for n in self.planner.nodes
                if n.get('semantic', {}).get('region_type') == region_type]

    def _best_node_of_type(self, region_type: str):
        """Devuelve el nodo del tipo dado con mayor distance_to_wall."""
        candidates = self._nodes_by_type(region_type)
        if not candidates:
            return None
        return max(candidates, key=lambda n: n.get('distance_to_wall', 0))

    # -----------------------------------------------------------------------
    # Interfaz interactiva
    # -----------------------------------------------------------------------

    def _print_nodes_table(self, current_node: int):
        """Muestra tabla de nodos con informacion semantica."""
        print(f"\n{'='*70}")
        print("NAVEGACION SEMANTICA — NODOS DISPONIBLES")
        print(f"{'='*70}")
        print(f"  {'ID':>3}  {'Posicion (m)':^20}  {'Tipo':<12}  {'Cost':>5}  "
              f"{'Riesgo':>7}  {'Estado'}")
        print(f"  {'-'*68}")

        for node in self.planner.nodes:
            x_m, y_m = self._pixel_to_meters(node['x_pixel'], node['y_pixel'])
            sem = node.get('semantic', {})
            markers = []
            if node['id'] == current_node:
                markers.append('<-- actual')
            if node['id'] in self.failed_nodes:
                markers.append('[FALLO]')
            print(f"  {node['id']:>3}  "
                  f"({x_m:6.2f}, {y_m:6.2f})    "
                  f"{sem.get('region_type','?'):<12}  "
                  f"{sem.get('traversal_cost',1.0):>5.2f}  "
                  f"{sem.get('risk_level','?'):>7}  "
                  f"{'  '.join(markers)}")

        print(f"\nNodo actual: {current_node}")

    def _print_menu(self):
        print("\nOpciones:")
        print("  [0-N]   Navegar a nodo por ID")
        print("  [r]     Navegar a la habitacion (room) mas cercana")
        print("  [c]     Navegar al corredor mas cercano")
        print("  [t]     Alternar modo semantico / topologico")
        print("  [m]     Markers RViz (republica)")
        print("  [q]     Salir")

    def interactive_navigation(self):
        """Bucle interactivo de navegacion semantica."""
        use_semantic = True
        n_nodes = len(self.planner.nodes)

        while not rospy.is_shutdown():
            current = self._get_current_node()
            self._print_nodes_table(current)

            mode_str = "🟢 SEMANTICO" if use_semantic else "🔵 TOPOLOGICO"
            self._print_menu()
            print(f"\n  Modo actual: {mode_str}")

            try:
                entrada = input(f"\nComando (nodo 0-{n_nodes-1} o letra): ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if entrada == 'q':
                break

            elif entrada == 't':
                use_semantic = not use_semantic
                print(f"  Modo cambiado a: {'SEMANTICO' if use_semantic else 'TOPOLOGICO'}")

            elif entrada == 'm':
                self._publish_semantic_markers()
                print("  Markers republicados en /semantic_markers")

            elif entrada == 'r':
                best = self._best_node_of_type('room')
                if best:
                    print(f"  -> Room mas amplia: nodo {best['id']}")
                    # Mostrar comparacion antes de navegar
                    self.planner.print_comparison(current, best['id'])
                    confirm = input("  Confirmar navegacion? [s/n]: ").strip().lower()
                    if confirm == 's':
                        self.navigate_to_node(current, best['id'], use_semantic)
                else:
                    print("  No se encontraron nodos de tipo 'room'")

            elif entrada == 'c':
                corredores = self._nodes_by_type('corridor')
                if not corredores:
                    print("  No se encontraron nodos de tipo 'corridor'")
                    continue
                # Mas cercano
                best = min(corredores,
                           key=lambda n: self.planner._heuristic(n['id'], current))
                print(f"  -> Corredor mas cercano: nodo {best['id']}")
                self.planner.print_comparison(current, best['id'])
                confirm = input("  Confirmar navegacion? [s/n]: ").strip().lower()
                if confirm == 's':
                    self.navigate_to_node(current, best['id'], use_semantic)

            else:
                try:
                    goal = int(entrada)
                except ValueError:
                    print("  Entrada invalida")
                    continue

                if not (0 <= goal < n_nodes):
                    print(f"  ID fuera de rango (0-{n_nodes-1})")
                    continue

                # Mostrar comparacion antes de navegar
                self.planner.print_comparison(current, goal)
                confirm = input("  Confirmar navegacion? [s/n]: ").strip().lower()
                if confirm != 's':
                    continue

                # Avisar si la ruta pasa por nodos con fallos previos
                path_prev = self.planner.plan_path_semantic(current, goal) if use_semantic \
                    else self.planner.plan_path(current, goal)
                if path_prev and any(n in self.failed_nodes for n in path_prev):
                    prob = [n for n in path_prev if n in self.failed_nodes]
                    print(f"  AVISO: la ruta pasa por nodos que fallaron antes: {prob}")

                self.navigate_to_node(current, goal, use_semantic)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rospy.init_node('semantic_navigator', anonymous=False)

    graph_json = rospy.get_param('~graph_json', None)

    if not graph_json:
        if len(sys.argv) > 1:
            graph_json = sys.argv[1]
        else:
            rospy.logerr("Uso: rosrun navegacion_semantica semantic_navigation.py <grafo_semantico.json>")
            sys.exit(1)

    if not os.path.exists(graph_json):
        rospy.logerr(f"Grafo no encontrado: {graph_json}")
        sys.exit(1)

    navigator = SemanticNavigator(graph_json)
    navigator.interactive_navigation()


if __name__ == '__main__':
    main()
