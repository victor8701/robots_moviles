#!/usr/bin/env python3
"""
Genera mapas topologicos para los 4 entornos y produce imagenes para la memoria.
"""

import os
import sys
import json

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from topological_mapper import TopologicalMapper

ENTORNOS = ['escenario1', 'escenario2', 'escenario3', 'estudio']

BASE_DIR    = os.path.dirname(os.path.abspath(__file__))
MAP_DIR     = os.path.join(BASE_DIR, '..', '..', 'navegacion_geometrica', 'mapas', 'exploration')
OUTPUT_DIR  = os.path.join(BASE_DIR, '..', 'mapas')
GRAPH_DIR   = os.path.join(OUTPUT_DIR, 'grafos')

os.makedirs(GRAPH_DIR, exist_ok=True)


def procesar_entorno(entorno):
    map_pgm  = os.path.join(MAP_DIR, f'mapa_{entorno}.pgm')
    map_yaml = os.path.join(MAP_DIR, f'mapa_{entorno}.yaml')

    if not os.path.exists(map_pgm):
        print(f'  [ERROR] No encontrado: {map_pgm}')
        return None

    out_dir = os.path.join(OUTPUT_DIR, entorno)
    os.makedirs(out_dir, exist_ok=True)

    output_graph   = os.path.join(GRAPH_DIR, f'{entorno}_grafo.json')
    output_topo    = os.path.join(out_dir,   'mapa_topologico.png')
    output_skel    = os.path.join(out_dir,   'mapa_traversabilidad.png')
    output_free    = os.path.join(out_dir,   'mapa_libre.png')

    mapper = TopologicalMapper(map_pgm, map_yaml, skeleton_dilation=3)
    graph  = mapper.generate()

    mapper.save_graph_json(output_graph)
    mapper.save_topological_map_image(output_topo)
    mapper.save_skeleton_image(output_skel)
    mapper.save_free_space_image(output_free)

    n_nodes = len(graph['nodes'])
    n_edges = len(graph['edges'])
    res     = graph['metadata']['resolution_m_per_pixel']
    return {'entorno': entorno, 'nodos': n_nodes, 'aristas': n_edges, 'resolucion': res}


def main():
    print('=' * 65)
    print('GENERACION DE MAPAS TOPOLOGICOS - 4 ENTORNOS')
    print('=' * 65)

    resultados = []
    for entorno in ENTORNOS:
        print(f'\n[{entorno.upper()}]')
        print('-' * 65)
        try:
            r = procesar_entorno(entorno)
            if r:
                resultados.append(r)
        except Exception as e:
            print(f'  [ERROR] {e}')
            import traceback; traceback.print_exc()

    print('\n' + '=' * 65)
    print('RESUMEN (para la memoria)')
    print('=' * 65)
    print(f'{"Entorno":<15} {"Nodos":>7} {"Aristas":>9} {"Res (m/px)":>12}')
    print('-' * 65)
    for r in resultados:
        print(f'{r["entorno"]:<15} {r["nodos"]:>7} {r["aristas"]:>9} {r["resolucion"]:>12.4f}')
    print('=' * 65)


if __name__ == '__main__':
    main()
