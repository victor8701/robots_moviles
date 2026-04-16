#!/usr/bin/env python3
"""
Generador de Mapas Topologicos a partir de Mapas Geometricos.

Realiza esqueletonizacion del espacio libre y genera un grafo topologico
con nodos y aristas que representan la conectividad del entorno.
"""

import cv2
import numpy as np
import json
from skimage import morphology
from scipy import ndimage
import os


class TopologicalMapper:
    """Convierte mapas geometricos en mapas topologicos."""

    def __init__(self, map_image_path, map_yaml_path, skeleton_dilation=3):
        """
        Inicializa el generador de mapas topologicos.

        Args:
            map_image_path: Ruta al archivo .pgm del mapa geometrico
            map_yaml_path: Ruta al archivo .yaml con metadatos del mapa
            skeleton_dilation: Dilacion del esqueleto para robustez (en pixeles)
        """
        self.map_image_path = map_image_path
        self.map_yaml_path = map_yaml_path
        self.skeleton_dilation = skeleton_dilation

        # Cargar mapa y metadatos
        self.map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
        self.metadata = self._load_yaml_metadata()

        # Procesamiento
        self.free_space_mask = None
        self.skeleton = None
        self.nodes = None
        self.edges = None
        self.graph = None

    def _load_yaml_metadata(self):
        """Carga metadatos del archivo YAML del mapa."""
        import yaml
        with open(self.map_yaml_path, 'r') as f:
            return yaml.safe_load(f)

    def _create_free_space_mask(self):
        """
        Crea mascara del espacio libre a partir del mapa.

        En los mapas ROS:
        - 0-50: espacio libre
        - 50-200: espacio desconocido
        - 200-255: obstaculo
        """
        # Usar threshold para crear mascara binaria
        # Los valores < ~180 son espacio libre
        _, binary = cv2.threshold(self.map_image, 180, 255, cv2.THRESH_BINARY_INV)
        self.free_space_mask = binary

        # Aplicar operaciones morfologicas para limpiar ruido
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.free_space_mask = cv2.morphologyEx(self.free_space_mask, cv2.MORPH_CLOSE, kernel)

        return self.free_space_mask

    def _compute_skeleton(self):
        """
        Calcula el esqueleto (linea central) del espacio libre.

        El esqueleto representa el "backbone" topologico del mapa.
        """
        # Normalizar mascara a valores 0-1
        free_normalized = self.free_space_mask.astype(float) / 255.0

        # Esqueletonizacion usando scikit-image
        # Thin realiza esqueletonizacion binaria
        skeleton = morphology.skeletonize(free_normalized > 0.5)

        # Dilatar ligeramente para robustez
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (self.skeleton_dilation, self.skeleton_dilation))
        skeleton_uint8 = (skeleton.astype(np.uint8) * 255)
        skeleton_dilated = cv2.dilate(skeleton_uint8, kernel, iterations=1)

        self.skeleton = skeleton_dilated
        return self.skeleton

    def _detect_nodes(self, min_distance=15):
        """
        Detecta nodos topologicos a partir del esqueleto.

        Los nodos son puntos notables:
        - Puntos de maxima distancia a obstaculos (puntos centrales)
        - Puntos de bifurcacion/interseccion

        Args:
            min_distance: Distancia minima entre nodos (en pixeles)
        """
        # Distance Transform: calcular distancia minima a obstaculo para cada pixel
        dist_transform = cv2.distanceTransform(self.skeleton, cv2.DIST_L2, cv2.DIST_2)

        # Detectar locales maximos (candidatos a nodos)
        local_max = ndimage.maximum_filter(dist_transform, size=2*min_distance + 1)
        is_local_max = (dist_transform == local_max) & (self.skeleton > 0)

        # Obtener coordenadas de los maximos locales
        coords = np.array(np.where(is_local_max)).T

        # Filtrar por distancia minima y distancia a obstaculo
        self.nodes = []
        for coord in coords:
            y, x = coord
            dist_to_obstacle = dist_transform[y, x]

            # Solo mantener si esta bastante alejado de obstaculos
            if dist_to_obstacle > 2:
                self.nodes.append({
                    'id': len(self.nodes),
                    'x_pixel': x,
                    'y_pixel': y,
                    'distance_to_wall': dist_to_obstacle
                })

        return self.nodes

    def _compute_edges(self, max_distance=100):
        """
        Calcula aristas conectando nodos cercanos.

        Dos nodos estan conectados si:
        1. Estan a distancia < max_distance
        2. El camino entre ellos en el esqueleto es viable

        Args:
            max_distance: Distancia maxima de conexion (en pixeles)
        """
        self.edges = []

        if not self.nodes:
            return self.edges

        n_nodes = len(self.nodes)
        for i in range(n_nodes):
            for j in range(i + 1, n_nodes):
                node_i = self.nodes[i]
                node_j = self.nodes[j]

                x1, y1 = node_i['x_pixel'], node_i['y_pixel']
                x2, y2 = node_j['x_pixel'], node_j['y_pixel']

                # Distancia euclidiana
                distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

                if distance < max_distance:
                    # Verificar si hay camino viable en el esqueleto
                    if self._is_path_viable(x1, y1, x2, y2):
                        self.edges.append({
                            'from_node': i,
                            'to_node': j,
                            'distance_pixels': distance
                        })

        return self.edges

    def _is_path_viable(self, x1, y1, x2, y2, num_samples=10):
        """
        Verifica si hay un camino viable en el esqueleto entre dos puntos.

        Muestrea puntos en la linea entre los dos nodos y verifica si
        estan sobre el esqueleto.
        """
        viable_count = 0
        for t in np.linspace(0, 1, num_samples):
            x = int(x1 + t * (x2 - x1))
            y = int(y1 + t * (y2 - y1))

            # Verificar limites
            if 0 <= x < self.skeleton.shape[1] and 0 <= y < self.skeleton.shape[0]:
                if self.skeleton[y, x] > 0:
                    viable_count += 1

        # Requerir que al menos 70% de los puntos esten en el esqueleto
        return viable_count >= int(0.7 * num_samples)

    def _build_graph(self):
        """Construye estructura de grafo a partir de nodos y aristas."""
        self.graph = {
            'nodes': self.nodes,
            'edges': self.edges,
            'metadata': {
                'map_image': os.path.basename(self.map_image_path),
                'resolution_m_per_pixel': self.metadata.get('resolution', 0.05),
                'origin_x': self.metadata.get('origin', [0, 0, 0])[0],
                'origin_y': self.metadata.get('origin', [0, 0, 0])[1],
            }
        }
        return self.graph

    def generate(self):
        """
        Pipeline completo: crea el mapa topologico.
        """
        print("[TopologicalMapper] Creando mascara de espacio libre...")
        self._create_free_space_mask()

        print("[TopologicalMapper] Calculando esqueleto...")
        self._compute_skeleton()

        print("[TopologicalMapper] Detectando nodos...")
        self._detect_nodes()
        print(f"  -> Detectados {len(self.nodes)} nodos")

        print("[TopologicalMapper] Calculando aristas...")
        self._compute_edges()
        print(f"  -> Calculadas {len(self.edges)} aristas")

        print("[TopologicalMapper] Construyendo grafo...")
        self._build_graph()

        return self.graph

    def save_topological_map_image(self, output_path):
        """
        Guarda una imagen del mapa topologico (esqueleto + nodos).
        """
        # Crear imagen RGB
        vis = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2BGR)

        # Dibujar esqueleto en verde
        vis[self.skeleton > 0] = [0, 255, 0]

        # Dibujar nodos como circulos azules
        for node in self.nodes:
            x, y = node['x_pixel'], node['y_pixel']
            cv2.circle(vis, (x, y), 5, (255, 0, 0), -1)
            cv2.putText(vis, str(node['id']), (x+8, y+8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0))

        # Dibujar aristas como lineas rojas
        for edge in self.edges:
            node_from = self.nodes[edge['from_node']]
            node_to = self.nodes[edge['to_node']]
            x1, y1 = node_from['x_pixel'], node_from['y_pixel']
            x2, y2 = node_to['x_pixel'], node_to['y_pixel']
            cv2.line(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)

        cv2.imwrite(output_path, vis)
        print(f"[TopologicalMapper] Imagen topologica guardada: {output_path}")

    def save_graph_json(self, output_path):
        """Guarda el grafo como archivo JSON."""
        with open(output_path, 'w') as f:
            json.dump(self.graph, f, indent=2)
        print(f"[TopologicalMapper] Grafo guardado: {output_path}")


if __name__ == '__main__':
    # Ejemplo de uso
    import sys

    if len(sys.argv) < 3:
        print("Uso: python topological_mapper.py <mapa_pgm> <mapa_yaml> [salida_grafo] [salida_imagen]")
        sys.exit(1)

    map_pgm = sys.argv[1]
    map_yaml = sys.argv[2]
    output_graph = sys.argv[3] if len(sys.argv) > 3 else "grafo_topologico.json"
    output_image = sys.argv[4] if len(sys.argv) > 4 else "mapa_topologico.png"

    # Generar
    mapper = TopologicalMapper(map_pgm, map_yaml)
    graph = mapper.generate()

    # Guardar resultados
    mapper.save_graph_json(output_graph)
    mapper.save_topological_map_image(output_image)

    print("\n[TopologicalMapper] ¡Completado!")
    print(f"  Nodos: {len(mapper.nodes)}")
    print(f"  Aristas: {len(mapper.edges)}")
