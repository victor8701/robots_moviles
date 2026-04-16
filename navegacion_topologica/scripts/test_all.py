#!/usr/bin/env python3
"""
Script de demostracion y prueba de Navegacion Topologica.

Prueba la generacion de mapas y planificacion para todos los entornos.
"""

import os
import sys
import json

# Agregar src al path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from topological_mapper import TopologicalMapper
from path_planner import TopologicalPathPlanner


def print_separator(title=""):
    """Imprime separador con título."""
    if title:
        print(f"\n{'='*70}")
        print(f"  {title}")
        print(f"{'='*70}\n")
    else:
        print(f"\n{'-'*70}\n")


def test_topological_generation():
    """Prueba generacion de mapas topologicos."""
    print_separator("TEST 1: GENERACION DE MAPAS TOPOLOGICOS")

    entornos = ['escenario1', 'escenario2', 'escenario3', 'estudio']
    base_map_dir = os.path.join(os.path.dirname(__file__),
                                '..', '..', 'navegacion_geometrica', 'mapas', 'sensores')
    graph_dir = os.path.join(os.path.dirname(__file__), '..', 'mapas', 'grafos')

    resultados = []

    for entorno in entornos:
        print(f"Procesando {entorno.upper()}...")

        map_pgm = os.path.join(base_map_dir, f'{entorno}_sensores.pgm')
        map_yaml = os.path.join(base_map_dir, f'{entorno}_sensores.yaml')

        if not os.path.exists(map_pgm):
            print(f"  ❌ Mapa no encontrado: {map_pgm}")
            continue

        try:
            mapper = TopologicalMapper(map_pgm, map_yaml)
            graph = mapper.generate()

            resultados.append({
                'entorno': entorno,
                'nodos': len(mapper.nodes),
                'aristas': len(mapper.edges),
                'densidad': len(mapper.edges) / (len(mapper.nodes) * (len(mapper.nodes) - 1) / 2) if len(mapper.nodes) > 1 else 0
            })

            print(f"  ✓ Nodos: {len(mapper.nodes)}")
            print(f"  ✓ Aristas: {len(mapper.edges)}")
            print(f"  ✓ Densidad conectividad: {resultados[-1]['densidad']:.2%}")

        except Exception as e:
            print(f"  ❌ Error: {e}")

    print_separator("RESULTADOS COMPARATIVOS")
    print(f"{'Entorno':<15} {'Nodos':<10} {'Aristas':<10} {'Densidad':<15}")
    print("-" * 50)
    for r in resultados:
        print(f"{r['entorno']:<15} {r['nodos']:<10} {r['aristas']:<10} {r['densidad']:<15.2%}")


def test_path_planning():
    """Prueba planificacion de caminos."""
    print_separator("TEST 2: PLANIFICACION DE CAMINOS")

    entornos = ['escenario1', 'escenario2', 'escenario3', 'estudio']
    graph_dir = os.path.join(os.path.dirname(__file__), '..', 'mapas', 'grafos')

    for entorno in entornos:
        graph_json = os.path.join(graph_dir, f'{entorno}_grafo.json')

        if not os.path.exists(graph_json):
            print(f"{entorno.upper()}: ❌ Grafo no encontrado\n")
            continue

        print(f"{entorno.upper()}:")

        try:
            planner = TopologicalPathPlanner(graph_json)

            # Prueba 1: Nodo 0 a nodo final
            if planner.nodes:
                start = 0
                goal = len(planner.nodes) - 1

                path = planner.plan_path(start, goal)

                if path:
                    distance = planner.get_path_distance(path)
                    print(f"  Ruta {start}→{goal}: {len(path)} nodos, {distance:.1f}px")
                else:
                    print(f"  Ruta {start}→{goal}: ❌ No hay camino")

            # Prueba 2: Camino intermedio
            if len(planner.nodes) > 5:
                start = planner.nodes[0]['id']
                goal = planner.nodes[len(planner.nodes) // 2]['id']

                path = planner.plan_path(start, goal)

                if path:
                    distance = planner.get_path_distance(path)
                    print(f"  Ruta {start}→{goal}: {len(path)} nodos, {distance:.1f}px")

        except Exception as e:
            print(f"  ❌ Error: {e}")

        print()


def test_graph_structure():
    """Prueba estructura del grafo."""
    print_separator("TEST 3: ANALISIS DE ESTRUCTURA DEL GRAFO")

    entornos = ['escenario1', 'escenario2', 'escenario3', 'estudio']
    graph_dir = os.path.join(os.path.dirname(__file__), '..', 'mapas', 'grafos')

    for entorno in entornos:
        graph_json = os.path.join(graph_dir, f'{entorno}_grafo.json')

        if not os.path.exists(graph_json):
            print(f"{entorno.upper()}: ❌ Grafo no encontrado\n")
            continue

        print(f"{entorno.upper()}:")

        try:
            with open(graph_json, 'r') as f:
                graph = json.load(f)

            nodes = graph['nodes']
            edges = graph['edges']

            # Calcular estadísticas
            degrees = {node['id']: 0 for node in nodes}
            for edge in edges:
                degrees[edge['from_node']] += 1
                degrees[edge['to_node']] += 1

            avg_degree = sum(degrees.values()) / len(degrees) if degrees else 0
            max_degree = max(degrees.values()) if degrees else 0
            min_degree = min(degrees.values()) if degrees else 0

            print(f"  Nodos: {len(nodes)}")
            print(f"  Aristas: {len(edges)}")
            print(f"  Grado promedio: {avg_degree:.2f}")
            print(f"  Grado máximo: {max_degree}")
            print(f"  Grado mínimo: {min_degree}")
            print(f"  Nodos aislados: {sum(1 for d in degrees.values() if d == 0)}")

            # Distancias
            distances = [e['distance_pixels'] for e in edges]
            if distances:
                print(f"  Distancia promedio (aristas): {sum(distances) / len(distances):.1f}px")
                print(f"  Distancia máxima: {max(distances):.1f}px")
                print(f"  Distancia mínima: {min(distances):.1f}px")

        except Exception as e:
            print(f"  ❌ Error: {e}")

        print()


def main():
    """Ejecuta todos los tests."""
    print("\n" + "="*70)
    print("  NAVEGACION TOPOLOGICA - SUITE DE PRUEBAS")
    print("="*70)

    try:
        test_topological_generation()
        test_path_planning()
        test_graph_structure()

        print_separator("RESUMEN")
        print("✓ Todas las pruebas completadas")
        print("\nProximos pasos:")
        print("  1. Verificar mapas PNG generados en mapas/topologicos/")
        print("  2. Ejecutar navegacion topologica en ROS:")
        print("     roslaunch navegacion_topologica topological_navigation.launch")
        print("  3. Ver visualizacion en RViz")

    except Exception as e:
        print(f"\n❌ Error durante pruebas: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
