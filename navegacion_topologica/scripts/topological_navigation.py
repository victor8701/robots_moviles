#!/usr/bin/env python3
"""
Navegacion Topologica con ROS.

Permite al usuario seleccionar un nodo topologico destino, planifica
una ruta y guia al robot hacia ese objetivo usando navegacion geometrica.
"""

import rospy
import actionlib
import json
import sys
import os
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import math

# Agregar src al path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from path_planner import TopologicalPathPlanner


class TopologicalNavigator:
    """Sistema de navegacion topologica."""

    def __init__(self, graph_json_path):
        """
        Inicializa navegador topologico.

        Args:
            graph_json_path: Ruta al archivo JSON del grafo topologico
        """
        self.planner = TopologicalPathPlanner(graph_json_path)
        self.resolution = self.planner.graph['metadata']['resolution_m_per_pixel']
        self.origin_x = self.planner.graph['metadata']['origin_x']
        self.origin_y = self.planner.graph['metadata']['origin_y']

        # Cliente de acciones move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # TF buffer para obtener posicion del robot
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publicador de camino (para visualizar en RViz)
        self.path_pub = rospy.Publisher('/topological_path', Path, queue_size=1)

        rospy.loginfo("Navegador topologico inicializado")

    def pixel_to_meters(self, x_pixel, y_pixel):
        """Convierte coordenadas pixel a metros."""
        x_m = self.origin_x + x_pixel * self.resolution
        y_m = self.origin_y + y_pixel * self.resolution
        return x_m, y_m

    def get_robot_position(self):
        """Obtiene posicion actual del robot."""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                     rospy.Time(0), rospy.Duration(1.0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except Exception as e:
            rospy.logwarn(f"Error obteniendo posicion del robot: {e}")
            return None, None

    def find_way_points(self, path_node_ids):
        """
        Convierte camino de nodos a waypoints en metros.

        Args:
            path_node_ids: Lista de IDs de nodos

        Returns:
            Lista de tuplas (x, y) en metros
        """
        waypoints = []
        for node_id in path_node_ids:
            node = self.planner.nodes[node_id]
            x_m, y_m = self.pixel_to_meters(node['x_pixel'], node['y_pixel'])
            waypoints.append((x_m, y_m))

        return waypoints

    def publish_path(self, waypoints):
        """Publica camino topologico para visualizar en RViz."""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        self.path_pub.publish(path)

    def navigate_topological(self, start_node_id, goal_node_id):
        """
        Ejecuta navegacion topologica.

        Args:
            start_node_id: ID del nodo inicial
            goal_node_id: ID del nodo objetivo
        """
        # Planificar camino
        rospy.loginfo(f"Planificando desde nodo {start_node_id} a nodo {goal_node_id}...")
        path = self.planner.plan_path(start_node_id, goal_node_id)

        if not path:
            rospy.logerr("No se encontro camino valido")
            return False

        rospy.loginfo(f"Camino encontrado: {' -> '.join(map(str, path))}")

        # Convertir a waypoints
        waypoints = self.find_way_points(path)
        self.publish_path(waypoints)

        # Navegar a cada waypoint
        for i, (x, y) in enumerate(waypoints):
            rospy.loginfo(f"Navegando a waypoint {i+1}/{len(waypoints)}: ({x:.2f}m, {y:.2f}m)")

            # Crear goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = 1.0

            # Enviar goal
            self.move_base_client.send_goal(goal)
            wait = self.move_base_client.wait_for_result(rospy.Duration(300))

            if not wait:
                rospy.logerr(f"Timeout llegando a waypoint {i}")
                self.move_base_client.cancel_goal()
                return False
            else:
                if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"Waypoint {i+1} alcanzado")
                else:
                    rospy.logerr(f"Fallo alcanando waypoint {i}")
                    return False

        rospy.loginfo("¡Objetivo topologico alcanzado!")
        return True

    def interactive_navigation(self):
        """Interfaz interactiva para seleccionar nodo destino."""
        print("\n" + "=" * 70)
        print("NAVEGACION TOPOLOGICA - SELECCIONAR NODO DESTINO")
        print("=" * 70)

        # Mostrar nodos disponibles
        print("\nNodos disponibles:")
        for node in self.planner.nodes:
            x_m, y_m = self.pixel_to_meters(node['x_pixel'], node['y_pixel'])
            print(f"  Nodo {node['id']:2d}: ({x_m:7.2f}m, {y_m:7.2f}m)")

        # Obtener nodo inicial
        robot_x, robot_y = self.get_robot_position()
        if robot_x is None:
            rospy.logwarn("No se pudo obtener posicion del robot, usando nodo 0 como inicial")
            start_node = 0
        else:
            start_node = self.planner.find_nearest_node(
                int((robot_x - self.origin_x) / self.resolution),
                int((robot_y - self.origin_y) / self.resolution)
            )
            rospy.loginfo(f"Nodo inicial detectado: {start_node}")

        # Seleccionar nodo destino
        while True:
            try:
                goal_node = int(input(f"\nSelecciona nodo destino (0-{len(self.planner.nodes)-1}): "))
                if 0 <= goal_node < len(self.planner.nodes):
                    break
                else:
                    print("ID de nodo invalido")
            except ValueError:
                print("Entrada invalida, ingresa un numero")

        # Navegar
        self.navigate_topological(start_node, goal_node)

    def list_all_nodes(self):
        """Lista todos los nodos del mapa topologico."""
        print("\n" + "=" * 70)
        print("NODOS DEL MAPA TOPOLOGICO")
        print("=" * 70)

        for node in self.planner.nodes:
            x_m, y_m = self.pixel_to_meters(node['x_pixel'], node['y_pixel'])
            print(f"Nodo {node['id']:2d}: ({x_m:7.2f}m, {y_m:7.2f}m) - "
                  f"Distancia a pared: {node['distance_to_wall']:.1f}px")

        print(f"\nTotal: {len(self.planner.nodes)} nodos")
        print(f"Aristas: {len(self.planner.edges)}")


def main():
    """Punto de entrada principal."""
    rospy.init_node('topological_navigator', anonymous=False)

    # Obtener ruta del grafo del parametro ROS o argumento
    graph_json = rospy.get_param('~graph_json', None)

    if not graph_json:
        if len(sys.argv) > 1:
            graph_json = sys.argv[1]
        else:
            rospy.logerr("Uso: rosrun navegacion_topologica topological_navigation.py <grafo_json>")
            rospy.logerr("O   rosrun navegacion_topologica topological_navigation.py _graph_json:=<ruta>")
            sys.exit(1)

    if not os.path.exists(graph_json):
        rospy.logerr(f"Archivo de grafo no encontrado: {graph_json}")
        sys.exit(1)

    # Crear navegador
    navigator = TopologicalNavigator(graph_json)

    # Interfaz interactiva
    navigator.list_all_nodes()
    navigator.interactive_navigation()


if __name__ == '__main__':
    main()
