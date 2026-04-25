#!/usr/bin/env python3
"""
Clasificacion Semantica Basada en Regiones 3D.

Utiliza informacion de profundidad/3D para clasificar y asignar atributos
semanticos a diferentes regiones del entorno para navegacion mejorada.
"""

import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Dict
import json


@dataclass
class SemanticRegion:
    """Representa una region semantica del entorno."""
    region_id: int
    label: str  # "corridor", "room", "junction", etc.
    points_3d: np.ndarray  # Puntos 3D de la region (N, 3)
    bounds: Dict  # {"min_x", "max_x", "min_y", "max_y", "min_z", "max_z"}
    centroid: Tuple[float, float, float]  # Centro de la región
    volume: float  # Volumen aproximado
    width: float  # Ancho promedio
    height: float  # Altura promedio
    connectivity: List[int]  # IDs de regiones conectadas
    attributes: Dict  # {"is_traversable", "difficulty", etc.}


class SemanticMapper3D:
    """Mapea regiones semanticas basadas en dato 3D."""

    def __init__(self, depth_image, intrinsics=None, depth_scale=1.0):
        """
        Inicializa mapper semantico 3D.

        Args:
            depth_image: Imagen de profundidad (HxW)
            intrinsics: Matriz de intrinsecos de camara (3x3)
            depth_scale: Factor de escala para convertir valores a metros
        """
        self.depth_image = depth_image
        self.depth_scale = depth_scale

        # Intrinsicos por defecto (ajustar segun sensor)
        if intrinsics is None:
            self.intrinsics = np.array([
                [525.0, 0.0, 319.5],
                [0.0, 525.0, 239.5],
                [0.0, 0.0, 1.0]
            ])  # Parametros típicos Kinect
        else:
            self.intrinsics = intrinsics

        self.regions: List[SemanticRegion] = []

    def depth_to_3d(self, depth_image=None):
        """
        Convierte imagen de profundidad a nube de puntos 3D.

        Args:
            depth_image: Imagen de profundidad (si None, usa la del objeto)

        Returns:
            Array (H, W, 3) con coordenadas 3D
        """
        if depth_image is None:
            depth_image = self.depth_image

        h, w = depth_image.shape
        points_3d = np.zeros((h, w, 3), dtype=np.float32)

        fx = self.intrinsics[0, 0]
        fy = self.intrinsics[1, 1]
        cx = self.intrinsics[0, 2]
        cy = self.intrinsics[1, 2]

        for y in range(h):
            for x in range(w):
                z = depth_image[y, x] * self.depth_scale
                if z == 0:  # Valor invalido
                    continue

                px = (x - cx) * z / fx
                py = (y - cy) * z / fy

                points_3d[y, x] = [px, py, z]

        return points_3d

    def segment_by_height(self, points_3d, height_thresholds=None):
        """
        Segmenta regiones por altura (ej: suelo, obstaculos, techo).

        Args:
            points_3d: Nube de puntos 3D (H, W, 3)
            height_thresholds: Lista de thresholds de altura en metros

        Returns:
            Imagen segmentada (H, W) con IDs de región
        """
        if height_thresholds is None:
            height_thresholds = [0.2, 0.5, 1.0, 2.0]  # Ejemplos

        h, w = points_3d.shape[:2]
        segmentation = np.zeros((h, w), dtype=np.int32)

        for y in range(h):
            for x in range(w):
                z = points_3d[y, x, 2]
                if z == 0:
                    continue

                # Clasificar por altura
                for idx, threshold in enumerate(height_thresholds):
                    if z < threshold:
                        segmentation[y, x] = idx
                        break
                else:
                    segmentation[y, x] = len(height_thresholds)

        return segmentation

    def segment_by_occupancy(self, points_3d, occupancy_threshold=0.1):
        """
        Segmenta regiones por densidad de ocupancia (área ocupada vs libre).

        Args:
            points_3d: Nube de puntos 3D
            occupancy_threshold: Threshold de densidad

        Returns:
            Imagen segmentada binaria (ocupado/libre)
        """
        h, w = points_3d.shape[:2]
        valid_points = np.sum(points_3d[:, :, 2] > 0, axis=(0, 1))
        occupancy = np.where(points_3d[:, :, 2] > 0, 1, 0)

        return occupancy

    def segment_by_clustering(self, points_3d, distance_threshold=0.5):
        """
        Segmenta regiones usando clustering espacial 3D.

        Agrupa puntos 3D que estan cercanos entre si.

        Args:
            points_3d: Nube de puntos 3D
            distance_threshold: Distancia maxima entre puntos (metros)

        Returns:
            Imagen segmentada (H, W) con IDs de cluster
        """
        # Implementación simplificada: agrupar por proximidad
        h, w = points_3d.shape[:2]
        segmentation = np.zeros((h, w), dtype=np.int32)
        cluster_id = 0
        visited = np.zeros((h, w), dtype=bool)

        def flood_fill(y, x, current_cluster):
            """Flood fill para agrupar puntos cercanos."""
            if y < 0 or y >= h or x < 0 or x >= w or visited[y, x]:
                return
            if np.sum(points_3d[y, x]) == 0:  # Punto invalido
                return

            visited[y, x] = True
            segmentation[y, x] = current_cluster

            # Revisar vecinos
            for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and not visited[ny, nx]:
                    p1 = points_3d[y, x]
                    p2 = points_3d[ny, nx]

                    if np.sum(p2) > 0:  # Punto vaálido
                        distance = np.linalg.norm(p1 - p2)
                        if distance < distance_threshold:
                            flood_fill(ny, nx, current_cluster)

        for y in range(h):
            for x in range(w):
                if not visited[y, x] and np.sum(points_3d[y, x]) > 0:
                    flood_fill(y, x, cluster_id)
                    cluster_id += 1

        return segmentation

    def extract_regions(self, segmentation):
        """
        Extrae propiedades de cada región segmentada.

        Args:
            segmentation: Imagen segmentada (H, W)

        Returns:
            Lista de SemanticRegion
        """
        unique_ids = np.unique(segmentation)
        self.regions = []

        for region_id in unique_ids:
            if region_id == 0:  # Skipea fondo
                continue

            mask = segmentation == region_id
            indices = np.where(mask)
            points_3d = self.depth_image[mask]

            if len(points_3d) == 0:
                continue

            # Calcular propiedades
            centroid = np.mean(points_3d, axis=0)
            bounds = {
                'min_x': np.min(points_3d[:, 0]),
                'max_x': np.max(points_3d[:, 0]),
                'min_y': np.min(points_3d[:, 1]),
                'max_y': np.max(points_3d[:, 1]),
                'min_z': np.min(points_3d[:, 2]),
                'max_z': np.max(points_3d[:, 2]),
            }

            width = bounds['max_x'] - bounds['min_x']
            height = bounds['max_z'] - bounds['min_z']
            volume = width * (bounds['max_y'] - bounds['min_y']) * height

            region = SemanticRegion(
                region_id=int(region_id),
                label=self._classify_region(width, height, volume),
                points_3d=points_3d,
                bounds=bounds,
                centroid=tuple(centroid),
                volume=float(volume),
                width=float(width),
                height=float(height),
                connectivity=[],
                attributes={}
            )

            self.regions.append(region)

        return self.regions

    @staticmethod
    def _classify_region(width: float, height: float, volume: float) -> str:
        """
        Clasifica el tipo de región 3D según sus dimensiones métricas.

        Umbrales calibrados para interiores de robots móviles (TurtleBot3):
          - width  = diámetro del espacio libre en el plano horizontal (m)
          - height = altura estimada del espacio (m)
          - volume = aproximación del volumen libre (m³)

        Rangos típicos en nuestros entornos (resolución 0.05 m/px):
          Pasillos/cruces:  width  0.6 – 1.9 m  (dtw  0.3 – 0.95 m)
          Habitaciones:     width  2.0 – 3.1 m  (dtw  1.0 – 1.55 m)
          Grandes salas:    width > 3.0 m        (dtw > 1.5 m)
        """
        if height < 0.5:
            return "floor"
        elif width < 0.8 and height > 1.5:
            return "wall"
        elif width < 2.0 and height > 1.5:
            return "corridor"
        elif width >= 2.0 and width < 3.0 and height > 1.5:
            return "room"
        elif width >= 3.0 and height > 1.5:
            return "large_room"
        else:
            return "obstacle"

    @classmethod
    def classify_node_from_map(cls,
                               distance_to_wall_m: float,
                               n_open_directions: int,
                               room_height_m: float = 2.5) -> str:
        """
        Clasifica un nodo topológico con lógica 3D a partir de datos del mapa 2D.

        Proyecta la información geométrica 2D (distancia a pared, número de
        direcciones abiertas) al espacio 3D asumiendo altura interior estándar,
        replicando la clasificación volumétrica que haría SemanticMapper3D
        sobre una nube de puntos real.

        Args:
            distance_to_wall_m:  Radio del espacio libre (m), del mapa de distancias.
            n_open_directions:   Número de direcciones cardinales con paso libre
                                 (obtenido por ray-casting sobre el PGM).
            room_height_m:       Altura estimada del techo (m). Por defecto 2.5 m.

        Returns:
            Etiqueta semántica topológica: "room" | "corridor" | "junction" | "narrow"
        """
        width  = distance_to_wall_m * 2.0          # diámetro del espacio libre
        volume = width * room_height_m * width      # aproximación cúbica

        label_3d = cls._classify_region(width, room_height_m, volume)

        # Mapeo de etiquetas 3D a etiquetas topológicas
        topo_label_map = {
            "large_room": "room",
            "room":       "room",
            "corridor":   "corridor",
            "floor":      "narrow",
            "wall":       "narrow",
            "obstacle":   "narrow",
        }
        topo_label = topo_label_map.get(label_3d, "junction")

        # Refinamiento topológico: corredor abierto en ≥3 direcciones → junction
        if topo_label == "corridor" and n_open_directions >= 3:
            topo_label = "junction"

        return topo_label

    def assign_traversability(self):
        """Asigna atributo de transitabilidad a regiones."""
        for region in self.regions:
            # Heurística: corredores y habitaciones son transitables
            region.attributes['is_traversable'] = (
                region.label in ["corridor", "room", "large_room", "floor"]
            )

            # Dificultad de navigación (inversamente proporcional al tamaño)
            if region.width > 3.0:
                region.attributes['difficulty'] = "easy"
            elif region.width > 1.5:
                region.attributes['difficulty'] = "medium"
            else:
                region.attributes['difficulty'] = "hard"

    def build_connectivity_graph(self, segmentation, distance_threshold=0.3):
        """
        Construye grafo de conectividad entre regiones.

        Dos regiones están conectadas si tienen puntos cercanos.
        """
        # Encontrar pares de regiones adyacentes
        h, w = segmentation.shape
        connections = set()

        for y in range(h - 1):
            for x in range(w - 1):
                region_id = segmentation[y, x]
                neighbor_ids = [
                    segmentation[y + 1, x],
                    segmentation[y, x + 1],
                    segmentation[y + 1, x + 1]
                ]

                for neighbor_id in neighbor_ids:
                    if neighbor_id != region_id and neighbor_id > 0 and region_id > 0:
                        pair = tuple(sorted([region_id, neighbor_id]))
                        connections.add(pair)

        # Asignar conectividad a regiones
        for region in self.regions:
            connected_ids = set()
            for r1, r2 in connections:
                if r1 == region.region_id:
                    connected_ids.add(r2)
                elif r2 == region.region_id:
                    connected_ids.add(r1)
            region.connectivity = list(connected_ids)

    def save_regions_json(self, output_path):
        """Guarda información de regiones en JSON."""
        data = {
            'regions': [
                {
                    'id': r.region_id,
                    'label': r.label,
                    'centroid': list(r.centroid),
                    'bounds': r.bounds,
                    'volume': r.volume,
                    'width': r.width,
                    'height': r.height,
                    'connected_to': r.connectivity,
                    'attributes': r.attributes
                }
                for r in self.regions
            ]
        }

        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"Regiones semanticas guardadas en: {output_path}")


if __name__ == '__main__':
    print("Módulo de clasificación semántica 3D")
    print("Ver documentación en navegacion_semantica/README.md")
