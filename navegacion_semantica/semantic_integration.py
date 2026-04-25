#!/usr/bin/env python3
"""
Integracion de Navegacion Semantica con Topologica.

Asigna atributos semanticos a nodos topologicos basados en analisis
geometrico del mapa 2D (ray-casting sobre mascara de espacio libre),
reproduciendo la logica de clasificacion de regiones 3D proyectada
al plano horizontal.

Uso standalone:
    python3 semantic_integration.py <grafo.json> <mapa.pgm> <mapa.yaml> [salida.json]
"""

import json
import cv2
import numpy as np
import os
import sys
from typing import Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Constantes de clasificacion
# ---------------------------------------------------------------------------

# Umbrales en PIXELES (resolucion tipica 0.05 m/px -> 1m = 20px)
ROOM_MIN_DIST_WALL    = 20   # >1.0 m de pared -> habitacion
CORRIDOR_MIN_DIST     = 10   # 0.5-1.0 m -> corredor
# Por debajo de CORRIDOR_MIN_DIST -> zona estrecha/junction

# Costes de traversal por tipo de region (multiplicador sobre distancia)
TRAVERSAL_COST = {
    "room":      0.7,   # espacio abierto, facil
    "corridor":  1.0,   # normal
    "junction":  1.2,   # cruce, atencion
    "narrow":    1.8,   # estrecho, mas dificil
    "unknown":   1.0,
}

# Dificultad de navegacion por tipo
DIFFICULTY = {
    "room":      "easy",
    "corridor":  "medium",
    "junction":  "medium",
    "narrow":    "hard",
    "unknown":   "medium",
}

# Nivel de riesgo por tipo
RISK_LEVEL = {
    "room":      "low",
    "corridor":  "low",
    "junction":  "medium",
    "narrow":    "high",
    "unknown":   "low",
}

# Colores BGR para visualizacion
REGION_COLORS = {
    "room":      (0,   200,  50),   # verde
    "corridor":  (220, 100,   0),   # azul
    "junction":  (180,   0, 180),   # morado
    "narrow":    (0,   200, 220),   # amarillo
    "unknown":   (160, 160, 160),   # gris
}


# ---------------------------------------------------------------------------
# Funciones de clasificacion geometrica 2D
# ---------------------------------------------------------------------------

def _load_free_mask(pgm_path: str) -> Optional[np.ndarray]:
    """
    Lee un mapa PGM de ROS/gmapping y devuelve la mascara binaria
    de espacio libre (umbral >= 220).
    """
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None
    _, mask = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)
    return mask


def _ray_cast(free_mask: np.ndarray, x: int, y: int,
              direction: Tuple[int, int], max_dist: int = 200) -> int:
    """
    Lanza un rayo desde (x, y) en la direccion dada y devuelve
    cuantos pixeles hay hasta el primer obstaculo (o hasta max_dist).
    """
    h, w = free_mask.shape
    dx, dy = direction
    dist = 0
    cx, cy = x, y
    while dist < max_dist:
        cx += dx
        cy += dy
        if not (0 <= cx < w and 0 <= cy < h):
            break
        if free_mask[cy, cx] == 0:
            break
        dist += 1
    return dist


def classify_node_geometry(free_mask: np.ndarray,
                           x_pixel: int, y_pixel: int,
                           distance_to_wall: float) -> str:
    """
    Clasifica un nodo del grafo topologico.

    Criterio principal: distancia a la pared mas cercana (distance_to_wall),
    ya calculada por el topological_mapper como maximo local de la
    Transformada de Distancia. Refleja directamente la amplitud del espacio.

    El ray-casting se usa SOLO para distinguir junction (cruce) de corridor.

    Returns:
        Etiqueta semantica: "room" | "corridor" | "junction" | "narrow"
    """
    # 1. ROOM: nodo con gran separacion de paredes -> espacio abierto/amplio
    #    ROOM_MIN_DIST_WALL px = ROOM_MIN_DIST_WALL * 0.05 m
    if distance_to_wall >= ROOM_MIN_DIST_WALL:
        return "room"

    # 2. Para nodos menos espaciosos: ray-casting para junction vs corridor
    if free_mask is not None:
        directions_cardinal = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        cardinal_dists = [
            _ray_cast(free_mask, x_pixel, y_pixel, d) for d in directions_cardinal
        ]
        # junction: abierto en 3+ direcciones cardinales (es un cruce)
        open_corridor = sum(1 for d in cardinal_dists if d >= CORRIDOR_MIN_DIST)
        if open_corridor >= 3:
            return "junction"

    # 3. CORRIDOR: distancia a pared suficiente para navegar
    if distance_to_wall >= CORRIDOR_MIN_DIST:
        return "corridor"

    # 4. NARROW: zona muy estrecha
    return "narrow"


# ---------------------------------------------------------------------------
# Clase principal
# ---------------------------------------------------------------------------

class SemanticTopologicalIntegration:
    """
    Integra informacion semantica con el grafo topologico.

    Si se proporciona un mapa PGM, la clasificacion se realiza por
    ray-casting geometrico 2D. Si no, se usa solo distance_to_wall.
    Opcionalmente acepta regiones semanticas 3D externas (JSON).
    """

    def __init__(self,
                 topological_graph_path: str,
                 map_pgm_path: Optional[str] = None,
                 semantic_regions_path: Optional[str] = None):
        """
        Args:
            topological_graph_path: Ruta al grafo topologico JSON.
            map_pgm_path:           Ruta al mapa PGM (opcional pero recomendado).
            semantic_regions_path:  Ruta a regiones semanticas 3D JSON (opcional).
        """
        with open(topological_graph_path, 'r') as f:
            self.topo_graph = json.load(f)

        # Mapa 2D para ray-casting
        self.free_mask: Optional[np.ndarray] = None
        if map_pgm_path and os.path.exists(map_pgm_path):
            self.free_mask = _load_free_mask(map_pgm_path)
            if self.free_mask is None:
                print(f"[WARNING] No se pudo cargar PGM: {map_pgm_path}")

        # Regiones 3D externas (opcionales)
        self.semantic_regions: List[Dict] = []
        if semantic_regions_path and os.path.exists(semantic_regions_path):
            with open(semantic_regions_path, 'r') as f:
                data = json.load(f)
                self.semantic_regions = data.get('regions', [])

        # Enriquecer grafo
        self._assign_semantic_attributes()

    # -----------------------------------------------------------------------
    # Enriquecimiento de nodos
    # -----------------------------------------------------------------------

    def _assign_semantic_attributes(self):
        """Asigna atributos semanticos a cada nodo topologico."""
        for node in self.topo_graph['nodes']:
            if self.semantic_regions:
                # Prioridad: regiones 3D externas
                nearest = self._find_nearest_semantic_region(node)
                if nearest:
                    node['semantic'] = self._extract_semantic_attributes_3d(nearest)
                    continue

            # Clasificacion geometrica 2D desde el mapa PGM
            region_type = classify_node_geometry(
                self.free_mask,
                node['x_pixel'],
                node['y_pixel'],
                node.get('distance_to_wall', 0.0)
            )
            node['semantic'] = self._build_semantic_dict(region_type)

    def _build_semantic_dict(self, region_type: str) -> Dict:
        """Construye el diccionario de atributos semanticos para un tipo de region."""
        return {
            'region_type':    region_type,
            'traversability': 0.9 if region_type in ('room', 'corridor', 'junction') else 0.4,
            'difficulty':     DIFFICULTY.get(region_type, 'medium'),
            'risk_level':     RISK_LEVEL.get(region_type, 'low'),
            'traversal_cost': TRAVERSAL_COST.get(region_type, 1.0),
            'nearby_regions': [],
        }

    # -----------------------------------------------------------------------
    # Soporte para regiones 3D externas
    # -----------------------------------------------------------------------

    def _find_nearest_semantic_region(self, node: Dict) -> Optional[Dict]:
        """Encuentra la region semantica 3D mas cercana a un nodo."""
        min_distance = float('inf')
        nearest = None
        node_pos = np.array([node['x_pixel'], node['y_pixel']])
        for region in self.semantic_regions:
            region_pos = np.array(region['centroid'][:2])
            dist = np.linalg.norm(node_pos - region_pos)
            if dist < min_distance:
                min_distance = dist
                nearest = region
        return nearest

    def _extract_semantic_attributes_3d(self, region: Dict) -> Dict:
        """Extrae atributos semanticos de una region 3D."""
        label = region.get('label', 'unknown')
        attrs = region.get('attributes', {})
        is_traversable = attrs.get('is_traversable', True)
        difficulty = attrs.get('difficulty', 'medium')
        cost = TRAVERSAL_COST.get(label, 1.0)
        if not is_traversable:
            cost *= 5.0
        return {
            'region_type':    label,
            'traversability': 0.9 if is_traversable else 0.1,
            'difficulty':     difficulty,
            'risk_level':     RISK_LEVEL.get(label, 'low'),
            'traversal_cost': cost,
            'nearby_regions': region.get('connected_to', []),
        }

    # -----------------------------------------------------------------------
    # Calculo de costo semantico de caminos
    # -----------------------------------------------------------------------

    def calculate_semantic_path_cost(self, path_node_ids: List[int]) -> float:
        """
        Calcula el costo semantico de un camino (suma de traversal_cost
        ponderada por la distancia de cada arista).

        Args:
            path_node_ids: Lista de IDs de nodos del camino.

        Returns:
            Costo semantico total del camino.
        """
        # Construir mapa id -> nodo para acceso rapido
        node_map = {n['id']: n for n in self.topo_graph['nodes']}
        # Construir mapa de aristas
        edge_dist = {}
        for e in self.topo_graph['edges']:
            a, b, d = e['from_node'], e['to_node'], e['distance_pixels']
            edge_dist[(a, b)] = d
            edge_dist[(b, a)] = d

        total = 0.0
        for i in range(len(path_node_ids) - 1):
            src = path_node_ids[i]
            dst = path_node_ids[i + 1]
            dist = edge_dist.get((src, dst), 0.0)
            cost_mult = node_map[dst].get('semantic', {}).get('traversal_cost', 1.0)
            total += dist * cost_mult
        return total

    def recommend_safe_path(self, start_id: int, goal_id: int,
                            max_risk_level: str = 'medium') -> Dict:
        """
        Informa sobre nodos seguros e inseguros en la ruta.

        Args:
            start_id:       ID nodo inicial.
            goal_id:        ID nodo objetivo.
            max_risk_level: Nivel maximo de riesgo aceptable.

        Returns:
            Diccionario con estadisticas de seguridad.
        """
        risk_order = {'low': 0, 'medium': 1, 'high': 2}
        max_risk = risk_order.get(max_risk_level, 1)

        safe_nodes, unsafe_nodes = [], []
        for node in self.topo_graph['nodes']:
            risk = risk_order.get(node.get('semantic', {}).get('risk_level', 'low'), 0)
            (safe_nodes if risk <= max_risk else unsafe_nodes).append(node['id'])

        return {
            'start':           start_id,
            'goal':            goal_id,
            'safe_nodes':      safe_nodes,
            'unsafe_nodes':    unsafe_nodes,
            'max_risk_level':  max_risk_level,
        }

    # -----------------------------------------------------------------------
    # Persistencia
    # -----------------------------------------------------------------------

    def save_enriched_graph(self, output_path: str):
        """Guarda el grafo enriquecido con atributos semanticos."""
        with open(output_path, 'w') as f:
            json.dump(self.topo_graph, f, indent=2, ensure_ascii=False)
        print(f"[SemanticIntegration] Grafo enriquecido guardado: {output_path}")

    # -----------------------------------------------------------------------
    # Reportes
    # -----------------------------------------------------------------------

    def print_semantic_analysis(self):
        """Imprime analisis semantico del grafo."""
        nodes = self.topo_graph['nodes']
        print("\n" + "=" * 70)
        print("ANALISIS SEMANTICO DEL GRAFO TOPOLOGICO")
        print("=" * 70)

        region_types: Dict[str, int] = {}
        for node in nodes:
            rt = node.get('semantic', {}).get('region_type', 'unknown')
            region_types[rt] = region_types.get(rt, 0) + 1

        print("\nDistribucion de tipos de region:")
        for rt, count in sorted(region_types.items()):
            pct = count / len(nodes) * 100
            bar = "█" * count
            print(f"  {rt:<12}: {count:3d} nodos ({pct:5.1f}%)  {bar}")

        print("\nDetalle por nodo:")
        res = self.topo_graph['metadata'].get('resolution_m_per_pixel', 0.05)
        origin_x = self.topo_graph['metadata'].get('origin_x', 0.0)
        origin_y = self.topo_graph['metadata'].get('origin_y', 0.0)
        height = self.topo_graph['metadata'].get('height', 384)
        for node in nodes:
            x_m = origin_x + node['x_pixel'] * res
            y_m = origin_y + (height - node['y_pixel']) * res
            sem = node.get('semantic', {})
            print(f"  Nodo {node['id']:2d} ({x_m:6.2f}m, {y_m:6.2f}m): "
                  f"{sem.get('region_type','?'):<12} | "
                  f"cost={sem.get('traversal_cost',1.0):.2f} | "
                  f"risk={sem.get('risk_level','?')}")

        costs = [n.get('semantic', {}).get('traversal_cost', 1.0) for n in nodes]
        print(f"\nCosto medio de traversal: {sum(costs)/len(costs):.2f}")
        print(f"Costo min: {min(costs):.2f}  |  Costo max: {max(costs):.2f}")


# ---------------------------------------------------------------------------
# CLI standalone
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Uso: python3 semantic_integration.py <grafo.json> <mapa.pgm> [salida.json]")
        sys.exit(1)

    graph_path = sys.argv[1]
    pgm_path   = sys.argv[2]
    out_path   = sys.argv[3] if len(sys.argv) > 3 else graph_path.replace('.json', '_semantico.json')

    integrator = SemanticTopologicalIntegration(graph_path, pgm_path)
    integrator.print_semantic_analysis()
    integrator.save_enriched_graph(out_path)
