#!/usr/bin/env python3
"""
Integracion de Navegacion Semantica con Topologica.

Asigna atributos semánticos a nodos topológicos basados en
análisis 3D del entorno.
"""

import json
import numpy as np
from typing import Dict, List


class SemanticTopologicalIntegration:
    """Integra información semántica 3D con navegación topológica."""

    def __init__(self, topological_graph_path, semantic_regions_path=None):
        """
        Inicializa integracion.

        Args:
            topological_graph_path: Ruta a grafo topológico (JSON)
            semantic_regions_path: Ruta a regiones semánticas (JSON, opcional)
        """
        with open(topological_graph_path, 'r') as f:
            self.topo_graph = json.load(f)

        self.semantic_regions = []
        if semantic_regions_path:
            with open(semantic_regions_path, 'r') as f:
                data = json.load(f)
                self.semantic_regions = data.get('regions', [])

        # Enriquecer nodos con atributos semánticos
        self._assign_semantic_attributes()

    def _assign_semantic_attributes(self):
        """Asigna atributos semánticos a cada nodo topológico."""
        for node in self.topo_graph['nodes']:
            # Inicializar atributos
            node['semantic'] = {
                'region_type': 'unknown',
                'traversability': 0.8,  # 0-1, qué tan fácil es pasar
                'difficulty': 'medium',  # easy/medium/hard
                'risk_level': 'low',  # low/medium/high
                'traversal_cost': 1.0,  # Multiplicador de costo
                'nearby_regions': []
            }

            # Si hay datos semánticos, asignarlos
            if self.semantic_regions:
                nearest_region = self._find_nearest_semantic_region(node)
                if nearest_region:
                    node['semantic'] = self._extract_semantic_attributes(nearest_region)

    def _find_nearest_semantic_region(self, node) -> Dict:
        """Encuentra la región semántica más cercana a un nodo."""
        min_distance = float('inf')
        nearest_region = None

        node_pos = np.array([node['x_pixel'], node['y_pixel']])

        for region in self.semantic_regions:
            region_pos = np.array(region['centroid'][:2])
            distance = np.linalg.norm(node_pos - region_pos)

            if distance < min_distance:
                min_distance = distance
                nearest_region = region

        return nearest_region

    def _extract_semantic_attributes(self, region: Dict) -> Dict:
        """Extrae atributos semánticos de una región."""
        attributes = {
            'region_type': region.get('label', 'unknown'),
            'traversability': 0.9 if region.get('is_traversable', False) else 0.1,
            'difficulty': region.get('difficulty', 'medium'),
            'risk_level': self._calculate_risk_level(region),
            'traversal_cost': self._calculate_traversal_cost(region),
            'nearby_regions': region.get('connected_to', [])
        }

        return attributes

    def _calculate_risk_level(self, region: Dict) -> str:
        """Calcula nivel de riesgo basado en características de región."""
        if not region.get('is_traversable', False):
            return 'high'
        elif region.get('label') == 'corridor' and region.get('width', 0) < 1.0:
            return 'medium'
        else:
            return 'low'

    def _calculate_traversal_cost(self, region: Dict) -> float:
        """Calcula costo de traversal para la región."""
        base_cost = 1.0

        # Multiplicar por dificultad
        difficulty_multiplier = {
            'easy': 0.8,
            'medium': 1.0,
            'hard': 1.5
        }
        base_cost *= difficulty_multiplier.get(region.get('difficulty', 'medium'), 1.0)

        # Ajustar por transitabilidad
        if not region.get('is_traversable', False):
            base_cost *= 10.0  # Muy costoso pasar

        return base_cost

    def modify_path_planner_with_semantics(self, path_node_ids: List[int]) -> float:
        """
        Calcula costo semántico de un camino.

        Considera no solo el número de nodos, sino también los atributos
        semánticos de las regiones por las que pasa.

        Args:
            path_node_ids: Lista de IDs de nodos en el camino

        Returns:
            Costo semántico total del camino
        """
        total_cost = 0

        for node_id in path_node_ids:
            node = self.topo_graph['nodes'][node_id]
            if 'semantic' in node:
                total_cost += node['semantic']['traversal_cost']
            else:
                total_cost += 1.0

        return total_cost

    def recommend_safe_path(self, start_id: int, goal_id: int,
                           max_risk_level: str = 'medium') -> Dict:
        """
        Recomienda camino considerando seguridad.

        Args:
            start_id: ID nodo inicial
            goal_id: ID nodo objetivo
            max_risk_level: Nivel máximo de riesgo aceptable

        Returns:
            Información sobre el camino recomendado
        """
        risk_level_order = {'low': 0, 'medium': 1, 'high': 2}
        max_risk = risk_level_order.get(max_risk_level, 1)

        # Filtrar nodos seguros
        safe_nodes = []
        for node in self.topo_graph['nodes']:
            if 'semantic' in node:
                risk = risk_level_order.get(node['semantic']['risk_level'], 1)
                if risk <= max_risk:
                    safe_nodes.append(node['id'])
            else:
                safe_nodes.append(node['id'])

        return {
            'start': start_id,
            'goal': goal_id,
            'safe_nodes': safe_nodes,
            'max_risk_level': max_risk_level,
            'total_safe_nodes': len(safe_nodes),
            'unsafe_nodes': len(self.topo_graph['nodes']) - len(safe_nodes)
        }

    def save_enriched_graph(self, output_path):
        """Guarda grafo enriquecido con atributos semánticos."""
        with open(output_path, 'w') as f:
            json.dump(self.topo_graph, f, indent=2)

        print(f"Grafo enriquecido guardado: {output_path}")

    def print_semantic_analysis(self):
        """Imprime análisis semántico del grafo."""
        print("\n" + "="*70)
        print("ANALISIS SEMANTICO DEL GRAFO TOPOLOGICO")
        print("="*70 + "\n")

        # Estadísticas por tipo de región
        region_types = {}
        for node in self.topo_graph['nodes']:
            if 'semantic' in node:
                region_type = node['semantic']['region_type']
                if region_type not in region_types:
                    region_types[region_type] = 0
                region_types[region_type] += 1

        print("Distribución de tipos de región:")
        for region_type, count in sorted(region_types.items()):
            print(f"  {region_type:<15}: {count:3d} nodos ({count/len(self.topo_graph['nodes'])*100:.1f}%)")

        # Estadísticas de dificultad
        difficulties = {}
        for node in self.topo_graph['nodes']:
            if 'semantic' in node:
                difficulty = node['semantic']['difficulty']
                if difficulty not in difficulties:
                    difficulties[difficulty] = 0
                difficulties[difficulty] += 1

        print("\nDistribución de dificultad:")
        for difficulty, count in sorted(difficulties.items()):
            print(f"  {difficulty:<15}: {count:3d} nodos ({count/len(self.topo_graph['nodes'])*100:.1f}%)")

        # Nodos de alto riesgo
        high_risk_nodes = [n for n in self.topo_graph['nodes']
                          if n.get('semantic', {}).get('risk_level') == 'high']

        print(f"\nNodos de alto riesgo: {len(high_risk_nodes)} ({len(high_risk_nodes)/len(self.topo_graph['nodes'])*100:.1f}%)")
        if high_risk_nodes:
            for node in high_risk_nodes[:5]:  # Mostrar primeros 5
                print(f"  Nodo {node['id']}: {node.get('semantic', {}).get('region_type', '?')}")

        # Costo promedio
        costs = [n.get('semantic', {}).get('traversal_cost', 1.0)
                for n in self.topo_graph['nodes']]
        avg_cost = sum(costs) / len(costs) if costs else 0

        print(f"\nCosto promedio de traversal: {avg_cost:.2f}")
        print(f"Costo mínimo: {min(costs):.2f}")
        print(f"Costo máximo: {max(costs):.2f}")


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print("Uso: python semantic_topological_integration.py <grafo_topologico.json> [regiones_semanticas.json]")
        sys.exit(1)

    topo_graph = sys.argv[1]
    semantic_regions = sys.argv[2] if len(sys.argv) > 2 else None

    integrator = SemanticTopologicalIntegration(topo_graph, semantic_regions)
    integrator.print_semantic_analysis()

    # Guardar grafo enriquecido
    output_path = topo_graph.replace('.json', '_enriched.json')
    integrator.save_enriched_graph(output_path)
