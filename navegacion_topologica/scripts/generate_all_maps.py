#!/usr/bin/env python3
"""
Scripts para generar mapas topologicos de todos los entornos.
"""

import os
import sys

# Agregar la carpeta src al path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from topological_mapper import TopologicalMapper


def generate_all_topological_maps():
    """Genera mapas topologicos para los 4 entornos."""

    # Configuracion de los 4 entornos
    entornos = ['escenario1', 'escenario2', 'escenario3', 'estudio']
    base_map_dir = os.path.join(os.path.dirname(__file__),
                                '..', '..', 'navegacion_geometrica', 'mapas', 'sensores')
    output_dir = os.path.join(os.path.dirname(__file__), '..', 'mapas', 'topologicos')
    graph_dir = os.path.join(os.path.dirname(__file__), '..', 'mapas', 'grafos')

    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(graph_dir, exist_ok=True)

    print("=" * 70)
    print("GENERADOR DE MAPAS TOPOLOGICOS")
    print("=" * 70)

    for entorno in entornos:
        print(f"\n[{entorno.upper()}]")
        print("-" * 70)

        # Rutas de entrada
        map_pgm = os.path.join(base_map_dir, f'{entorno}_sensores.pgm')
        map_yaml = os.path.join(base_map_dir, f'{entorno}_sensores.yaml')

        # Verificar que existan
        if not os.path.exists(map_pgm) or not os.path.exists(map_yaml):
            print(f"ERROR: No se encontraron mapas para {entorno}")
            continue

        # Rutas de salida
        output_image = os.path.join(output_dir, f'{entorno}_topologico.png')
        output_graph = os.path.join(graph_dir, f'{entorno}_grafo.json')

        # Generar mapa topologico
        try:
            mapper = TopologicalMapper(map_pgm, map_yaml, skeleton_dilation=3)
            graph = mapper.generate()

            # Guardar resultados
            mapper.save_graph_json(output_graph)
            mapper.save_topological_map_image(output_image)

            print(f"✓ Mapa topologico guardado: {output_image}")
            print(f"✓ Grafo guardado: {output_graph}")

        except Exception as e:
            print(f"ERROR al procesar {entorno}: {e}")
            import traceback
            traceback.print_exc()

    print("\n" + "=" * 70)
    print("¡GENERACION COMPLETADA!")
    print("=" * 70)


if __name__ == '__main__':
    generate_all_topological_maps()
