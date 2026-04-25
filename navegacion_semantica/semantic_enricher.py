#!/usr/bin/env python3
"""
Semantic Enricher — Enriquecimiento semantico de grafos topologicos.

Procesa los grafos JSON de los 4 entornos, clasifica cada nodo por tipo
de region (room/corridor/junction/narrow) usando ray-casting sobre el
mapa PGM, y genera:

  - <entorno>_semantico.json     Grafo enriquecido
  - resultados/<entorno>_semantico.png  Visualizacion coloreada

Uso:
    # Generar todos los entornos de una vez
    python3 semantic_enricher.py

    # Solo un entorno
    python3 semantic_enricher.py escenario2
"""

import os
import sys
import json
import cv2
import numpy as np
import yaml

# Añadir carpeta navegacion_semantica al path para importar semantic_integration
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

from semantic_integration import (
    SemanticTopologicalIntegration,
    REGION_COLORS,
    TRAVERSAL_COST,
)

# ---------------------------------------------------------------------------
# Rutas relativas al workspace
# ---------------------------------------------------------------------------

_WS = os.path.abspath(os.path.join(_SCRIPT_DIR, '..'))  # robots_moviles/

TOPO_GRAFOS_DIR = os.path.join(_WS, 'navegacion_topologica', 'mapas', 'grafos')
GEO_MAPAS_DIR   = os.path.join(_WS, 'navegacion_geometrica', 'mapas', 'exploration')
OUT_DIR         = os.path.join(_SCRIPT_DIR, 'resultados')

ENTORNOS = {
    'escenario1': {
        'grafo': os.path.join(TOPO_GRAFOS_DIR, 'escenario1_grafo.json'),
        'pgm':   os.path.join(GEO_MAPAS_DIR,   'mapa_escenario1.pgm'),
        'yaml':  os.path.join(GEO_MAPAS_DIR,   'mapa_escenario1.yaml'),
    },
    'escenario2': {
        'grafo': os.path.join(TOPO_GRAFOS_DIR, 'escenario2_grafo.json'),
        'pgm':   os.path.join(GEO_MAPAS_DIR,   'mapa_escenario2.pgm'),
        'yaml':  os.path.join(GEO_MAPAS_DIR,   'mapa_escenario2.yaml'),
    },
    'escenario3': {
        'grafo': os.path.join(TOPO_GRAFOS_DIR, 'escenario3_grafo.json'),
        'pgm':   os.path.join(GEO_MAPAS_DIR,   'mapa_escenario3.pgm'),
        'yaml':  os.path.join(GEO_MAPAS_DIR,   'mapa_escenario3.yaml'),
    },
    'estudio': {
        'grafo': os.path.join(TOPO_GRAFOS_DIR, 'estudio_grafo.json'),
        'pgm':   os.path.join(GEO_MAPAS_DIR,   'mapa_estudio.pgm'),
        'yaml':  os.path.join(GEO_MAPAS_DIR,   'mapa_estudio.yaml'),
    },
}


# ---------------------------------------------------------------------------
# Visualizacion
# ---------------------------------------------------------------------------

def _draw_semantic_map(graph: dict, pgm_path: str, output_path: str,
                       entorno_name: str):
    """
    Genera imagen coloreada del mapa topologico con semantica:
      - Fondo: mapa original en escala de grises
      - Aristas: lineas grises, grosor proporcional al traversal_cost del destino
      - Nodos: circulos coloreados por tipo de region
      - Etiquetas: ID + tipo semantico
      - Leyenda en esquina
    """
    img_bg = cv2.imread(pgm_path, cv2.IMREAD_COLOR)
    if img_bg is None:
        h = graph['metadata'].get('height', 384)
        w = graph['metadata'].get('width', 384)
        img_bg = np.full((h, w, 3), 200, dtype=np.uint8)

    vis = img_bg.copy()

    nodes = {n['id']: n for n in graph['nodes']}

    # Aristas
    for edge in graph['edges']:
        n1 = nodes[edge['from_node']]
        n2 = nodes[edge['to_node']]
        p1 = (n1['x_pixel'], n1['y_pixel'])
        p2 = (n2['x_pixel'], n2['y_pixel'])
        # Grosor segun costo del nodo destino (mas estrecho = mas grueso para destacar)
        cost = n2.get('semantic', {}).get('traversal_cost', 1.0)
        thickness = max(1, int(cost * 1.5))
        cv2.line(vis, p1, p2, (120, 120, 120), thickness)

    # Nodos
    for node in graph['nodes']:
        x, y = node['x_pixel'], node['y_pixel']
        sem = node.get('semantic', {})
        region_type = sem.get('region_type', 'unknown')
        color = REGION_COLORS.get(region_type, (160, 160, 160))

        # Circulo relleno
        radius = max(6, int(node.get('distance_to_wall', 10) * 0.4))
        cv2.circle(vis, (x, y), radius, color, -1)
        cv2.circle(vis, (x, y), radius, (0, 0, 0), 1)  # borde negro

        # Etiqueta: ID + tipo
        label = f"{node['id']}:{region_type[:3]}"
        cv2.putText(vis, label, (x + radius + 2, y - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 0, 0), 1)

    # Leyenda
    leyenda_items = list(REGION_COLORS.items())
    lx, ly = 8, 8
    for tipo, color in leyenda_items:
        cv2.rectangle(vis, (lx, ly), (lx + 12, ly + 12), color, -1)
        cv2.rectangle(vis, (lx, ly), (lx + 12, ly + 12), (0, 0, 0), 1)
        cv2.putText(vis, tipo, (lx + 16, ly + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
        ly += 16

    # Titulo
    cv2.putText(vis, f"Semantica: {entorno_name}", (lx, ly + 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    cv2.imwrite(output_path, vis)
    print(f"  Imagen semantica: {output_path}")


# ---------------------------------------------------------------------------
# Estadisticas de comparacion
# ---------------------------------------------------------------------------

def _print_comparison(graph: dict, entorno_name: str):
    """Muestra tabla comparativa de tipos de region para la memoria."""
    nodes = graph['nodes']
    tipos = {}
    for n in nodes:
        t = n.get('semantic', {}).get('region_type', 'unknown')
        tipos[t] = tipos.get(t, 0) + 1

    costs = [n.get('semantic', {}).get('traversal_cost', 1.0) for n in nodes]
    print(f"\n  {'Tipo':<12} {'Nodos':>6}  {'Cost medio':>10}")
    print(f"  {'-'*30}")
    for t in sorted(tipos.keys()):
        ns = [n for n in nodes if n.get('semantic', {}).get('region_type') == t]
        c_medio = sum(n['semantic']['traversal_cost'] for n in ns) / len(ns)
        print(f"  {t:<12} {len(ns):>6}  {c_medio:>10.2f}")
    print(f"  {'TOTAL':<12} {len(nodes):>6}  {sum(costs)/len(costs):>10.2f}")


# ---------------------------------------------------------------------------
# Pipeline por entorno
# ---------------------------------------------------------------------------

def enrich_entorno(name: str, paths: dict) -> dict:
    """
    Enriquece el grafo de un entorno y guarda los resultados.

    Returns:
        El grafo enriquecido como diccionario.
    """
    print(f"\n{'='*60}")
    print(f"Procesando: {name}")
    print(f"{'='*60}")

    grafo_path = paths['grafo']
    pgm_path   = paths['pgm']

    if not os.path.exists(grafo_path):
        print(f"  [SKIP] Grafo no encontrado: {grafo_path}")
        return {}

    if not os.path.exists(pgm_path):
        print(f"  [WARN] PGM no encontrado: {pgm_path} — usando solo distance_to_wall")
        pgm_path = None

    # Enriquecer
    integrator = SemanticTopologicalIntegration(grafo_path, pgm_path)
    integrator.print_semantic_analysis()
    _print_comparison(integrator.topo_graph, name)

    # Guardar grafo enriquecido junto a los originales
    out_grafo = grafo_path.replace('.json', '_semantico.json')
    integrator.save_enriched_graph(out_grafo)

    # Imagen de visualizacion
    out_img = os.path.join(OUT_DIR, f"{name}_semantico.png")
    _draw_semantic_map(integrator.topo_graph, paths.get('pgm', ''),
                       out_img, name)

    return integrator.topo_graph


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # Filtrar entornos si se pasa argumento
    if len(sys.argv) > 1:
        target = sys.argv[1].lower()
        entornos_to_run = {k: v for k, v in ENTORNOS.items() if target in k}
        if not entornos_to_run:
            print(f"Entorno '{target}' no reconocido. Disponibles: {list(ENTORNOS.keys())}")
            sys.exit(1)
    else:
        entornos_to_run = ENTORNOS

    grafos = {}
    for name, paths in entornos_to_run.items():
        grafos[name] = enrich_entorno(name, paths)

    print(f"\n{'='*60}")
    print("RESUMEN GLOBAL")
    print(f"{'='*60}")
    print(f"Entornos procesados: {len(grafos)}")
    print(f"Resultados en:       {OUT_DIR}")
    print(f"Grafos semanticos en: {TOPO_GRAFOS_DIR}")

    # Tabla resumen global para copiar a la memoria
    print("\nTabla para la memoria:")
    print(f"  {'Entorno':<12} {'Nodos':>6} {'Rooms':>6} {'Corr.':>6} "
          f"{'Junc.':>6} {'Narr.':>6} {'Cost.med':>9}")
    print(f"  {'-'*58}")
    for name, graph in grafos.items():
        if not graph:
            continue
        nodes = graph.get('nodes', [])
        tipo_count = {'room': 0, 'corridor': 0, 'junction': 0, 'narrow': 0}
        costs = []
        for n in nodes:
            t = n.get('semantic', {}).get('region_type', 'unknown')
            if t in tipo_count:
                tipo_count[t] += 1
            costs.append(n.get('semantic', {}).get('traversal_cost', 1.0))
        c_med = sum(costs) / len(costs) if costs else 0
        print(f"  {name:<12} {len(nodes):>6} {tipo_count['room']:>6} "
              f"{tipo_count['corridor']:>6} {tipo_count['junction']:>6} "
              f"{tipo_count['narrow']:>6} {c_med:>9.2f}")


if __name__ == '__main__':
    main()
