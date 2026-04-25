#!/usr/bin/env python3
"""
Planificador A* con Costes Semanticos.

Extiende la planificacion topologica clasica incorporando los atributos
semanticos de cada nodo (traversal_cost, risk_level) como peso adicional
en el calculo del coste de arista.

Uso standalone:
    python3 semantic_path_planner.py <grafo_semantico.json> <nodo_inicio> <nodo_fin>
    python3 semantic_path_planner.py <grafo_semantico.json> <n_ini> <n_fin> --compare
"""

import json
import heapq
import sys
import os
from math import sqrt
from typing import List, Optional, Dict, Tuple

# Añadir ruta a path_planner topologico
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_TOPO_SRC   = os.path.join(_SCRIPT_DIR, '..', 'navegacion_topologica', 'src')
sys.path.insert(0, _TOPO_SRC)

from path_planner import TopologicalPathPlanner


class SemanticPathPlanner(TopologicalPathPlanner):
    """
    Planificador A* semantico.

    La funcion de coste de cada arista (i -> j) es:
        g(i->j) = distancia_euclideana(i,j) * traversal_cost(j)

    Parametros de configuracion:
        prefer_rooms:   Reduce el coste de nodos 'room' (factor 0.6)
        avoid_narrow:   Penaliza nodos 'narrow' (factor x3)
        max_risk_level: Bloquea nodos con riesgo superior a este nivel
    """

    RISK_ORDER = {'low': 0, 'medium': 1, 'high': 2}

    def __init__(self, graph_json_path: str,
                 prefer_rooms: bool = True,
                 avoid_narrow: bool = True,
                 max_risk_level: str = 'high'):
        """
        Args:
            graph_json_path: Ruta al grafo semantico JSON.
            prefer_rooms:    Si True, reduce coste de nodos tipo 'room'.
            avoid_narrow:    Si True, penaliza nodos tipo 'narrow'.
            max_risk_level:  Nivel de riesgo maximo aceptable ('low'/'medium'/'high').
        """
        super().__init__(graph_json_path)
        self.prefer_rooms   = prefer_rooms
        self.avoid_narrow   = avoid_narrow
        self.max_risk       = self.RISK_ORDER.get(max_risk_level, 2)

    # -----------------------------------------------------------------------
    # Calculo de coste semantico de una arista
    # -----------------------------------------------------------------------

    def _semantic_edge_cost(self, from_id: int, to_id: int,
                            base_distance: float) -> float:
        """
        Coste semantico de la arista from_id -> to_id.

        Args:
            from_id:       ID del nodo origen.
            to_id:         ID del nodo destino.
            base_distance: Distancia euclidiana en pixeles.

        Returns:
            Coste total de la arista (float). Infinito si el nodo esta bloqueado.
        """
        node = self.nodes[to_id]
        sem  = node.get('semantic', {})

        # Comprobar nivel de riesgo
        risk = self.RISK_ORDER.get(sem.get('risk_level', 'low'), 0)
        if risk > self.max_risk:
            return float('inf')

        cost_mult = sem.get('traversal_cost', 1.0)

        # Preferencia de habitaciones
        if self.prefer_rooms and sem.get('region_type') == 'room':
            cost_mult *= 0.6

        # Penalizacion de zonas estrechas
        if self.avoid_narrow and sem.get('region_type') == 'narrow':
            cost_mult *= 3.0

        return base_distance * cost_mult

    # -----------------------------------------------------------------------
    # A* semantico
    # -----------------------------------------------------------------------

    def plan_path_semantic(self, start_node_id: int,
                           goal_node_id: int) -> Optional[List[int]]:
        """
        Planifica una ruta usando A* con costes semanticos.

        Args:
            start_node_id: ID del nodo inicial.
            goal_node_id:  ID del nodo objetivo.

        Returns:
            Lista de IDs de nodos del camino, o None si no existe camino.
        """
        if start_node_id == goal_node_id:
            return [start_node_id]

        if (start_node_id not in self.adjacency_list or
                goal_node_id not in self.adjacency_list):
            return None

        open_set = []
        heapq.heappush(open_set, (0.0, start_node_id))

        came_from: Dict[int, int] = {}
        g_score = {n['id']: float('inf') for n in self.nodes}
        g_score[start_node_id] = 0.0
        closed_set = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_node_id:
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

            for neighbor, base_dist in self.adjacency_list[current]:
                if neighbor in closed_set:
                    continue

                edge_cost = self._semantic_edge_cost(current, neighbor, base_dist)
                if edge_cost == float('inf'):
                    continue  # Nodo bloqueado

                tentative_g = g_score[current] + edge_cost

                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor]   = tentative_g
                    h = self._heuristic(neighbor, goal_node_id)
                    heapq.heappush(open_set, (tentative_g + h, neighbor))

        return None

    # -----------------------------------------------------------------------
    # Comparacion topologica vs semantica
    # -----------------------------------------------------------------------

    def compare_paths(self, start_id: int, goal_id: int) -> Dict:
        """
        Calcula y compara la ruta topologica clasica con la semantica.

        Args:
            start_id: ID del nodo inicial.
            goal_id:  ID del nodo objetivo.

        Returns:
            Diccionario con ambas rutas y sus estadisticas.
        """
        path_topo = self.plan_path(start_id, goal_id)
        path_sem  = self.plan_path_semantic(start_id, goal_id)

        def path_stats(path):
            if not path:
                return None
            res = self.graph['metadata'].get('resolution_m_per_pixel', 0.05)
            dist_px = self.get_path_distance(path)
            sem_cost = sum(
                self.nodes[nid].get('semantic', {}).get('traversal_cost', 1.0)
                for nid in path
            )
            types = [
                self.nodes[nid].get('semantic', {}).get('region_type', 'unknown')
                for nid in path
            ]
            return {
                'path':         path,
                'n_nodos':      len(path),
                'distancia_px': round(dist_px, 1),
                'distancia_m':  round(dist_px * res, 2),
                'costo_sem':    round(sem_cost, 2),
                'tipos':        types,
            }

        return {
            'start':      start_id,
            'goal':       goal_id,
            'topologica': path_stats(path_topo),
            'semantica':  path_stats(path_sem),
        }

    def print_comparison(self, start_id: int, goal_id: int):
        """Imprime la comparacion entre ruta topologica y semantica."""
        comp = self.compare_paths(start_id, goal_id)

        print(f"\n{'='*65}")
        print(f"COMPARACION: Nodo {start_id} → Nodo {goal_id}")
        print(f"{'='*65}")

        for mode in ('topologica', 'semantica'):
            stats = comp[mode]
            label = '🔵 TOPOLOGICA' if mode == 'topologica' else '🟢 SEMANTICA'
            print(f"\n{label}:")
            if not stats:
                print("  Sin camino.")
                continue
            print(f"  Ruta:       {' → '.join(map(str, stats['path']))}")
            print(f"  Nodos:      {stats['n_nodos']}")
            print(f"  Distancia:  {stats['distancia_m']} m ({stats['distancia_px']} px)")
            print(f"  Costo sem.: {stats['costo_sem']}")
            print(f"  Tipos:      {stats['tipos']}")

        # Diferencia
        t = comp['topologica']
        s = comp['semantica']
        if t and s:
            diff_nodos = s['n_nodos'] - t['n_nodos']
            diff_cost  = s['costo_sem'] - t['costo_sem']
            print(f"\n📊 DIFERENCIA (semantica - topologica):")
            print(f"  Nodos:      {diff_nodos:+d}")
            print(f"  Costo sem.: {diff_cost:+.2f}")
            if s['path'] == t['path']:
                print("  → Misma ruta (los costes semanticos no cambian el camino optimo)")
            else:
                print("  → Rutas DIFERENTES (la semantica evita zonas de dificil navegacion)")

    # -----------------------------------------------------------------------
    # Informacion semantica de nodos
    # -----------------------------------------------------------------------

    def print_nodes_semantic(self):
        """Lista todos los nodos con su informacion semantica."""
        res      = self.graph['metadata'].get('resolution_m_per_pixel', 0.05)
        origin_x = self.graph['metadata'].get('origin_x', 0.0)
        origin_y = self.graph['metadata'].get('origin_y', 0.0)
        height   = self.graph['metadata'].get('height', 384)

        print(f"\n{'='*70}")
        print("NODOS CON INFORMACION SEMANTICA")
        print(f"{'='*70}")
        print(f"  {'ID':>3}  {'Pos (m)':^18}  {'Tipo':<12}  {'Cost':>5}  "
              f"{'Dif.':>6}  {'Riesgo'}")
        print(f"  {'-'*65}")

        for node in self.nodes:
            x_m = origin_x + node['x_pixel'] * res
            y_m = origin_y + (height - node['y_pixel']) * res
            sem = node.get('semantic', {})
            print(f"  {node['id']:>3}  "
                  f"({x_m:6.2f}, {y_m:6.2f})  "
                  f"{sem.get('region_type','?'):<12}  "
                  f"{sem.get('traversal_cost',1.0):>5.2f}  "
                  f"{sem.get('difficulty','?'):>6}  "
                  f"{sem.get('risk_level','?')}")


# ---------------------------------------------------------------------------
# CLI standalone
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Uso: python3 semantic_path_planner.py <grafo_semantico.json> "
              "<nodo_inicio> <nodo_fin> [--compare]")
        sys.exit(1)

    graph_path = sys.argv[1]
    start_id   = int(sys.argv[2])
    goal_id    = int(sys.argv[3])
    do_compare = '--compare' in sys.argv

    planner = SemanticPathPlanner(graph_path)
    planner.print_nodes_semantic()

    if do_compare:
        planner.print_comparison(start_id, goal_id)
    else:
        path = planner.plan_path_semantic(start_id, goal_id)
        if path:
            planner.print_path_info(path)
        else:
            print("No se encontro camino valido con restricciones semanticas")
