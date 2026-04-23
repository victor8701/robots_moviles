#!/usr/bin/env python3
"""
Generador de Mapas Topologicos a partir de Mapas Geometricos.

Pipeline:
  1. Mascara espacio libre (umbral sobre imagen PGM de ROS)
  2. Esqueleto fino (medial axis / skeletonize) del espacio libre
  3. Transformada de distancia -> maximos locales sobre esqueleto = nodos
  4. Suprimir nodos duplicados (NMS por distancia minima)
  5. Aristas: verificar linea de vision libre entre pares de nodos
  6. Asegurar conectividad global del grafo
"""

import cv2
import numpy as np
import json
from collections import deque
from skimage import morphology
from scipy import ndimage
import os


class TopologicalMapper:
    """Convierte mapas geometricos en mapas topologicos."""

    def __init__(self, map_image_path, map_yaml_path, skeleton_dilation=3):
        self.map_image_path = map_image_path
        self.map_yaml_path  = map_yaml_path
        self.skeleton_dilation = skeleton_dilation

        self.map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
        self.metadata  = self._load_yaml_metadata()

        self.free_space_mask = None
        self.thin_skeleton   = None  # 1px de grosor, para detectar nodos
        self.skeleton        = None  # dilatado, para visualizacion
        self.dist_transform  = None
        self.nodes           = None
        self.edges           = None
        self.graph           = None

    # ------------------------------------------------------------------
    # Carga de metadatos
    # ------------------------------------------------------------------

    def _load_yaml_metadata(self):
        import yaml
        with open(self.map_yaml_path, 'r') as f:
            return yaml.safe_load(f)

    # ------------------------------------------------------------------
    # Paso 1: espacio libre
    # ------------------------------------------------------------------

    def _create_free_space_mask(self):
        """
        Umbral 220 captura espacio libre (~254) y excluye
        desconocido (~205) y obstaculos (~0) en mapas ROS/gmapping.
        """
        _, binary = cv2.threshold(self.map_image, 220, 255, cv2.THRESH_BINARY)
        # Cerrar huecos internos pequenos sin ensanchar paredes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.free_space_mask = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        return self.free_space_mask

    # ------------------------------------------------------------------
    # Paso 2: esqueleto
    # ------------------------------------------------------------------

    def _compute_skeleton(self):
        """
        Calcula el esqueleto fino (medial axis) del espacio libre.
        Guarda tambien version dilatada para visualizacion.
        """
        free_bool = self.free_space_mask > 0
        thin = morphology.skeletonize(free_bool).astype(np.uint8)
        self.thin_skeleton = thin

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.skeleton_dilation, self.skeleton_dilation)
        )
        self.skeleton = cv2.dilate(thin * 255, kernel, iterations=1)
        return self.skeleton

    # ------------------------------------------------------------------
    # Paso 3 + 4: deteccion y supresion de nodos
    # ------------------------------------------------------------------

    def _detect_nodes(self, min_distance=20):
        """
        Nodos = maximos locales de la transformada de distancia
        restringidos al esqueleto fino.

        min_distance: separacion minima entre nodos (pixeles).
        A 0.05 m/px, 20 px = 1 m.
        """
        # Transformada de distancia: valor = distancia al obstaculo mas cercano
        self.dist_transform = cv2.distanceTransform(
            self.free_space_mask, cv2.DIST_L2, 5
        )

        # Solo puntos del esqueleto fino
        dist_skel = self.dist_transform.copy()
        dist_skel[self.thin_skeleton == 0] = 0.0

        # Maximos locales con ventana de min_distance
        win = 2 * min_distance + 1
        local_max = ndimage.maximum_filter(dist_skel, size=win)
        candidates = np.argwhere(
            (dist_skel == local_max) & (dist_skel > 2.0)
        )  # shape (N, 2) -> (row, col)

        if len(candidates) == 0:
            self.nodes = []
            return self.nodes

        # Ordenar por distancia a pared descendente (los mejores primero)
        dists   = dist_skel[candidates[:, 0], candidates[:, 1]]
        order   = np.argsort(dists)[::-1]
        candidates = candidates[order]
        dists      = dists[order]

        # NMS: suprimir candidatos demasiado cercanos a uno ya aceptado
        kept = []
        suppressed = np.zeros(len(candidates), dtype=bool)
        min_d2 = (min_distance // 2) ** 2

        for i in range(len(candidates)):
            if suppressed[i]:
                continue
            kept.append(i)
            cy, cx = candidates[i]
            for j in range(i + 1, len(candidates)):
                if suppressed[j]:
                    continue
                dy = int(candidates[j, 0]) - int(cy)
                dx = int(candidates[j, 1]) - int(cx)
                if dy * dy + dx * dx < min_d2:
                    suppressed[j] = True

        self.nodes = []
        for idx in kept:
            y, x = candidates[idx]
            self.nodes.append({
                'id': len(self.nodes),
                'x_pixel': int(x),
                'y_pixel': int(y),
                'distance_to_wall': float(dists[idx])
            })

        return self.nodes

    # ------------------------------------------------------------------
    # Paso 5: aristas
    # ------------------------------------------------------------------

    def _is_free_line(self, x1, y1, x2, y2):
        """
        Comprueba que la linea recta entre (x1,y1) y (x2,y2) permanece
        en espacio libre. Muestrea un punto cada 2 pixeles.
        """
        dist = np.hypot(x2 - x1, y2 - y1)
        n_samples = max(5, int(dist / 2))
        h, w = self.free_space_mask.shape

        for t in np.linspace(0.0, 1.0, n_samples):
            x = int(round(x1 + t * (x2 - x1)))
            y = int(round(y1 + t * (y2 - y1)))
            if not (0 <= x < w and 0 <= y < h):
                return False
            if self.free_space_mask[y, x] == 0:
                return False
        return True

    def _compute_edges(self, max_distance=200):
        """
        Dos nodos se conectan con una arista si:
          - su distancia euclidiana <= max_distance, Y
          - la linea recta entre ellos no cruza ningun obstaculo.

        Tras calcular las aristas basicas, se garantiza conectividad global
        uniendo las componentes desconectadas con la pareja de nodos mas cercana.
        """
        self.edges = []
        n = len(self.nodes)
        if n < 2:
            return self.edges

        for i in range(n):
            for j in range(i + 1, n):
                ni = self.nodes[i]
                nj = self.nodes[j]
                dx = nj['x_pixel'] - ni['x_pixel']
                dy = nj['y_pixel'] - ni['y_pixel']
                dist = float(np.hypot(dx, dy))
                if dist > max_distance:
                    continue
                if self._is_free_line(ni['x_pixel'], ni['y_pixel'],
                                      nj['x_pixel'], nj['y_pixel']):
                    self.edges.append({
                        'from_node': i,
                        'to_node': j,
                        'distance_pixels': dist
                    })

        self._ensure_connectivity()
        return self.edges

    def _ensure_connectivity(self):
        """
        Detecta componentes conexas del grafo y las une con el par de nodos
        mas cercano entre componentes distintas, ignorando obstaculos
        (arista de emergencia). Itera hasta que el grafo sea conexo.
        """
        n = len(self.nodes)
        if n == 0:
            return

        def components():
            adj = {i: set() for i in range(n)}
            for e in self.edges:
                adj[e['from_node']].add(e['to_node'])
                adj[e['to_node']].add(e['from_node'])
            visited = set()
            comps   = []
            for start in range(n):
                if start in visited:
                    continue
                q, comp = deque([start]), set()
                while q:
                    v = q.popleft()
                    if v in visited:
                        continue
                    visited.add(v); comp.add(v)
                    q.extend(adj[v] - visited)
                comps.append(list(comp))
            return comps

        for _ in range(n):               # max n iteraciones garantiza fin
            comps = components()
            if len(comps) == 1:
                break
            # Unir las dos primeras componentes con el par mas cercano
            c0, c1 = comps[0], comps[1]
            best_d, best_pair = float('inf'), None
            for i in c0:
                for j in c1:
                    ni = self.nodes[i]; nj = self.nodes[j]
                    d = float(np.hypot(
                        nj['x_pixel'] - ni['x_pixel'],
                        nj['y_pixel'] - ni['y_pixel']
                    ))
                    if d < best_d:
                        best_d, best_pair = d, (i, j)
            if best_pair:
                self.edges.append({
                    'from_node': best_pair[0],
                    'to_node':   best_pair[1],
                    'distance_pixels': best_d
                })

    # ------------------------------------------------------------------
    # Paso 6: grafo
    # ------------------------------------------------------------------

    def _build_graph(self):
        origin = self.metadata.get('origin', [0, 0, 0])
        self.graph = {
            'nodes': self.nodes,
            'edges': self.edges,
            'metadata': {
                'map_image':             os.path.basename(self.map_image_path),
                'resolution_m_per_pixel': self.metadata.get('resolution', 0.05),
                'origin_x': origin[0],
                'origin_y': origin[1],
            }
        }
        return self.graph

    # ------------------------------------------------------------------
    # Pipeline completo
    # ------------------------------------------------------------------

    def generate(self):
        print("[TopologicalMapper] Mascara espacio libre...")
        self._create_free_space_mask()

        print("[TopologicalMapper] Esqueleto fino...")
        self._compute_skeleton()

        print("[TopologicalMapper] Deteccion de nodos (NMS)...")
        self._detect_nodes()
        print(f"  -> {len(self.nodes)} nodos")

        print("[TopologicalMapper] Calculo de aristas...")
        self._compute_edges()
        print(f"  -> {len(self.edges)} aristas")

        self._build_graph()
        return self.graph

    # ------------------------------------------------------------------
    # Guardado de imagenes y JSON
    # ------------------------------------------------------------------

    def save_topological_map_image(self, output_path):
        """Mapa con esqueleto (verde), nodos (azul) y aristas (rojo)."""
        vis = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2BGR)
        vis[self.skeleton > 0] = [0, 220, 0]

        for edge in self.edges:
            n1 = self.nodes[edge['from_node']]
            n2 = self.nodes[edge['to_node']]
            cv2.line(vis, (n1['x_pixel'], n1['y_pixel']),
                     (n2['x_pixel'], n2['y_pixel']), (0, 0, 220), 2)

        for node in self.nodes:
            x, y = node['x_pixel'], node['y_pixel']
            cv2.circle(vis, (x, y), 5, (220, 0, 0), -1)
            cv2.putText(vis, str(node['id']), (x + 6, y - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 0, 0), 1)

        cv2.imwrite(output_path, vis)
        print(f"[TopologicalMapper] Imagen topologica: {output_path}")

    def save_skeleton_image(self, output_path):
        """Esqueleto fino sobre el mapa original."""
        vis = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2BGR)
        vis[self.thin_skeleton > 0] = [0, 200, 0]
        cv2.imwrite(output_path, vis)
        print(f"[TopologicalMapper] Imagen traversabilidad: {output_path}")

    def save_free_space_image(self, output_path):
        """Mascara binaria del espacio libre."""
        cv2.imwrite(output_path, self.free_space_mask)
        print(f"[TopologicalMapper] Imagen espacio libre: {output_path}")

    def save_graph_json(self, output_path):
        with open(output_path, 'w') as f:
            json.dump(self.graph, f, indent=2)
        print(f"[TopologicalMapper] Grafo JSON: {output_path}")


# ------------------------------------------------------------------
# CLI standalone
# ------------------------------------------------------------------

if __name__ == '__main__':
    import sys

    if len(sys.argv) < 3:
        print("Uso: python topological_mapper.py <mapa.pgm> <mapa.yaml> "
              "[grafo.json] [topo.png]")
        sys.exit(1)

    map_pgm   = sys.argv[1]
    map_yaml  = sys.argv[2]
    out_graph = sys.argv[3] if len(sys.argv) > 3 else 'grafo.json'
    out_img   = sys.argv[4] if len(sys.argv) > 4 else 'mapa_topologico.png'

    mapper = TopologicalMapper(map_pgm, map_yaml)
    graph  = mapper.generate()
    mapper.save_graph_json(out_graph)
    mapper.save_topological_map_image(out_img)

    print(f"\nCompletado: {len(mapper.nodes)} nodos, {len(mapper.edges)} aristas")
