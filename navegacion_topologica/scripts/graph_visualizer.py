#!/usr/bin/env python3
"""
Visualizador de Grafo Topologico en RViz.

Publica el grafo topologico como Markers para visualizacion en RViz.
"""

import rospy
import json
import sys
import os
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Agregar src al path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from path_planner import TopologicalPathPlanner


class TopologicalGraphVisualizer:
    """Visualiza el grafo topologico en RViz."""

    def __init__(self, graph_json_path):
        """
        Inicializa visualizador.

        Args:
            graph_json_path: Ruta al archivo JSON del grafo
        """
        self.planner = TopologicalPathPlanner(graph_json_path)

        # Publishers
        self.nodes_pub = rospy.Publisher('/topological_graph/nodes', MarkerArray, queue_size=1, latch=True)
        self.edges_pub = rospy.Publisher('/topological_graph/edges', MarkerArray, queue_size=1, latch=True)

        rospy.loginfo("Visualizador de grafo topologico inicializado")

    def publish_graph(self):
        """Publica nodos y aristas como markers."""
        nodes_markers = MarkerArray()
        edges_markers = MarkerArray()

        # Publicar nodos
        for node in self.planner.nodes:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "topological_nodes"
            marker.id = node['id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Posicion
            x_m, y_m = self.planner.get_node_position(node['id'])['x_meters'], \
                       self.planner.get_node_position(node['id'])['y_meters']
            marker.pose.position.x = x_m
            marker.pose.position.y = y_m
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0

            # Escala (tamaño de la esfera)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            # Color azul
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            nodes_markers.markers.append(marker)

            # Texto con ID del nodo
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "topological_labels"
            text_marker.id = node['id']
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = x_m
            text_marker.pose.position.y = y_m
            text_marker.pose.position.z = 0.3
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.15
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = str(node['id'])

            nodes_markers.markers.append(text_marker)

        # Publicar aristas
        marker_id = 0
        for edge in self.planner.edges:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "topological_edges"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker_id += 1

            # Nodos de la arista
            from_node = self.planner.nodes[edge['from_node']]
            to_node = self.planner.nodes[edge['to_node']]

            x1, y1 = self.planner.get_node_position(from_node['id'])['x_meters'], \
                     self.planner.get_node_position(from_node['id'])['y_meters']
            x2, y2 = self.planner.get_node_position(to_node['id'])['x_meters'], \
                     self.planner.get_node_position(to_node['id'])['y_meters']

            # Puntos de la línea
            p1 = Point()
            p1.x, p1.y, p1.z = x1, y1, 0.05

            p2 = Point()
            p2.x, p2.y, p2.z = x2, y2, 0.05

            marker.points.append(p1)
            marker.points.append(p2)

            # Escala de la línea
            marker.scale.x = 0.05  # Ancho de la línea

            # Color rojo
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            edges_markers.markers.append(marker)

        # Publicar
        self.nodes_pub.publish(nodes_markers)
        self.edges_pub.publish(edges_markers)

        rospy.loginfo(f"Grafo publicado: {len(self.planner.nodes)} nodos, {len(self.planner.edges)} aristas")


def main():
    """Punto de entrada."""
    rospy.init_node('topological_graph_visualizer', anonymous=False)

    # Obtener ruta del grafo
    graph_json = rospy.get_param('~graph_json', None)

    if not graph_json:
        if len(sys.argv) > 1:
            graph_json = sys.argv[1]
        else:
            rospy.logerr("Uso: rosrun navegacion_topologica graph_visualizer.py <grafo_json>")
            sys.exit(1)

    if not os.path.exists(graph_json):
        rospy.logerr(f"Archivo de grafo no encontrado: {graph_json}")
        sys.exit(1)

    # Crear visualizador y publicar
    visualizer = TopologicalGraphVisualizer(graph_json)
    visualizer.publish_graph()

    rospy.loginfo("Visualizador activo. Abre RViz para ver el grafo topologico.")
    rospy.spin()


if __name__ == '__main__':
    main()
