#!/usr/bin/env python3
"""
Planificador de Navegacion Topologica.

Implementa algoritmo A* para encontrar caminos optimos en el grafo topologico.
"""

import json
import heapq
from math import sqrt


class TopologicalPathPlanner:
    """Planificador A* para navegacion topologica."""

    def __init__(self, graph_json_path):
        """
        Inicializa el planificador.

        Args:
            graph_json_path: Ruta al archivo JSON del grafo topologico
        """
        with open(graph_json_path, 'r') as f:
            self.graph = json.load(f)

        self.nodes = self.graph['nodes']
        self.edges = self.graph['edges']
        self._build_adjacency_list()

    def _build_adjacency_list(self):
        """Construye lista de adyacencia para acceso rapido."""
        self.adjacency_list = {node['id']: [] for node in self.nodes}

        for edge in self.edges:
            from_id = edge['from_node']
            to_id = edge['to_node']
            distance = edge['distance_pixels']

            # Grafo no dirigido (bidireccional)
            self.adjacency_list[from_id].append((to_id, distance))
            self.adjacency_list[to_id].append((from_id, distance))

    def find_nearest_node(self, x_pixel, y_pixel):
        """
        Encuentra el nodo topologico mas cercano a una posicion.

        Args:
            x_pixel, y_pixel: Coordenadas en pixeles

        Returns:
            ID del nodo mas cercano
        """
        min_distance = float('inf')
        nearest_node_id = None

        for node in self.nodes:
            dx = node['x_pixel'] - x_pixel
            dy = node['y_pixel'] - y_pixel
            distance = sqrt(dx**2 + dy**2)

            if distance < min_distance:
                min_distance = distance
                nearest_node_id = node['id']

        return nearest_node_id

    def plan_path(self, start_node_id, goal_node_id):
        """
        Planifica una ruta usando A*.

        Args:
            start_node_id: ID del nodo inicial
            goal_node_id: ID del nodo objetivo

        Returns:
            Lista de IDs de nodos del camino, o None si no existe camino
        """
        if start_node_id == goal_node_id:
            return [start_node_id]

        # Validar que los nodos existan
        if start_node_id not in self.adjacency_list or \
           goal_node_id not in self.adjacency_list:
            return None

        # A* Search
        open_set = []
        heapq.heappush(open_set, (0, start_node_id))

        came_from = {}
        g_score = {node['id']: float('inf') for node in self.nodes}
        g_score[start_node_id] = 0

        closed_set = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_node_id:
                # Reconstruir camino
                path = []
                node = goal_node_id
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start_node_id)
                return list(reversed(path))

            if current in closed_set:
                continue

            closed_set.add(current)

            # Explorar vecinos
            for neighbor, distance in self.adjacency_list[current]:
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + distance

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score

                    h_score = self._heuristic(neighbor, goal_node_id)
                    f_score = g_score[neighbor] + h_score

                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def _heuristic(self, node_id, goal_id):
        """
        Heuristica para A*: distancia euclidiana entre nodos.

        Args:
            node_id: ID del nodo actual
            goal_id: ID del nodo objetivo

        Returns:
            Estimacion de distancia
        """
        node = self.nodes[node_id]
        goal = self.nodes[goal_id]

        dx = goal['x_pixel'] - node['x_pixel']
        dy = goal['y_pixel'] - node['y_pixel']

        return sqrt(dx**2 + dy**2)

    def get_path_distance(self, path_node_ids):
        """
        Calcula la distancia total de un camino.

        Args:
            path_node_ids: Lista de IDs del camino

        Returns:
            Distancia acumulada en pixeles
        """
        total_distance = 0

        for i in range(len(path_node_ids) - 1):
            current_id = path_node_ids[i]
            next_id = path_node_ids[i + 1]

            # Buscar arista correspondiente
            for neighbor, distance in self.adjacency_list[current_id]:
                if neighbor == next_id:
                    total_distance += distance
                    break

        return total_distance

    def get_node_position(self, node_id):
        """
        Obtiene la posicion de un nodo.

        Args:
            node_id: ID del nodo

        Returns:
            Diccionario con posicion en pixeles y metros
        """
        node = self.nodes[node_id]
        res = self.graph['metadata']['resolution_m_per_pixel']
        origin_x = self.graph['metadata']['origin_x']
        origin_y = self.graph['metadata']['origin_y']

        x_meters = origin_x + node['x_pixel'] * res
        y_meters = origin_y + node['y_pixel'] * res

        return {
            'x_pixel': node['x_pixel'],
            'y_pixel': node['y_pixel'],
            'x_meters': x_meters,
            'y_meters': y_meters,
            'distance_to_wall': node['distance_to_wall']
        }

    def print_path_info(self, path_node_ids):
        """Imprime informacion sobre un camino."""
        if not path_node_ids:
            print("No hay camino valido")
            return

        print(f"Camino encontrado con {len(path_node_ids)} nodos:")
        total_distance = 0

        for i, node_id in enumerate(path_node_ids):
            pos = self.get_node_position(node_id)
            print(f"  {i}: Nodo {node_id} -> ({pos['x_meters']:.2f}m, {pos['y_meters']:.2f}m)")

            if i < len(path_node_ids) - 1:
                next_id = path_node_ids[i + 1]
                for neighbor, distance in self.adjacency_list[node_id]:
                    if neighbor == next_id:
                        total_distance += distance
                        print(f"       -> distancia: {distance:.1f} pixeles ({distance * self.graph['metadata']['resolution_m_per_pixel']:.2f}m)")
                        break

        print(f"Distancia total: {total_distance:.1f} pixeles ({total_distance * self.graph['metadata']['resolution_m_per_pixel']:.2f}m)")
        print(f"Numero de transiciones entre nodos: {len(path_node_ids) - 1}")


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 4:
        print("Uso: python path_planner.py <grafo_json> <nodo_inicio> <nodo_objetivo>")
        sys.exit(1)

    graph_json = sys.argv[1]
    start_node = int(sys.argv[2])
    goal_node = int(sys.argv[3])

    planner = TopologicalPathPlanner(graph_json)
    path = planner.plan_path(start_node, goal_node)

    if path:
        planner.print_path_info(path)
    else:
        print("No se encontro camino valido")
