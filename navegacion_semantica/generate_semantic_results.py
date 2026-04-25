#!/usr/bin/env python3
"""
Generador de Resultados para la Memoria — Navegacion Semantica.

Genera sin ROS todas las figuras y estadisticas para la memoria:
  1. Imagenes de los 4 entornos coloreadas por tipo semantico
  2. Pipeline de comparacion (ruta topologica vs. semantica) por entorno
  3. Tabla resumen global en Markdown

Uso:
    python3 generate_semantic_results.py
"""

import os
import sys
import json
import cv2
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

from semantic_integration  import SemanticTopologicalIntegration, REGION_COLORS
from semantic_enricher     import ENTORNOS, OUT_DIR, enrich_entorno
from semantic_path_planner import SemanticPathPlanner

_WS         = os.path.abspath(os.path.join(_SCRIPT_DIR, '..'))
TOPO_GRAFOS = os.path.join(_WS, 'navegacion_topologica', 'mapas', 'grafos')
RESULTS_DIR = OUT_DIR   # navegacion_semantica/resultados/


# ---------------------------------------------------------------------------
# Figura 1: Imagen semantica comparada con imagen topologica
# ---------------------------------------------------------------------------

def generar_figura_pipeline(entorno_name: str, paths: dict) -> str:
    """
    Genera figura 3-paneles:
      Panel izquierdo:  Mapa original (PGM)
      Panel central:    Mapa topologico (grafo sin semantica)
      Panel derecho:    Mapa semantico (grafo con colores de region)

    Returns:
        Ruta a la imagen generada.
    """
    pgm = paths.get('pgm', '')
    img_orig = cv2.imread(pgm) if pgm and os.path.exists(pgm) else None

    sem_path = os.path.join(RESULTS_DIR, f"{entorno_name}_semantico.png")
    img_sem  = cv2.imread(sem_path) if os.path.exists(sem_path) else None

    # Fallback a imagen en blanco si no existen
    h = 384
    blank = np.full((h, h, 3), 200, dtype=np.uint8)

    panels = [
        img_orig if img_orig is not None else blank,
        img_sem  if img_sem  is not None else blank,
    ]

    # Escalar al mismo alto
    target_h = max(p.shape[0] for p in panels)
    resized = []
    for p in panels:
        scale = target_h / p.shape[0]
        new_w = int(p.shape[1] * scale)
        resized.append(cv2.resize(p, (new_w, target_h)))

    # Poner etiquetas
    labels = ['Mapa geometrico', 'Mapa semantico-topologico']
    for img, lbl in zip(resized, labels):
        cv2.putText(img, lbl, (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # Separador vertical blanco
    sep = np.full((target_h, 6, 3), 255, dtype=np.uint8)
    combined = np.hstack([resized[0], sep, resized[1]])

    # Titulo
    title_h = 28
    title_bar = np.full((title_h, combined.shape[1], 3), 240, dtype=np.uint8)
    cv2.putText(title_bar,
                f"Pipeline semantico: {entorno_name}",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    combined = np.vstack([title_bar, combined])

    out = os.path.join(RESULTS_DIR, f"{entorno_name}_pipeline_semantico.png")
    cv2.imwrite(out, combined)
    print(f"  Pipeline guardado: {out}")
    return out


# ---------------------------------------------------------------------------
# Figura 2: Comparacion de rutas (topologica vs semantica)
# ---------------------------------------------------------------------------

def _draw_path_on_map(base_img: np.ndarray, graph: dict,
                      path_ids: list, color: tuple, thickness: int = 2):
    """Dibuja un camino (lista de IDs) sobre una imagen."""
    nodes = {n['id']: n for n in graph['nodes']}
    for i in range(len(path_ids) - 1):
        n1 = nodes[path_ids[i]]
        n2 = nodes[path_ids[i + 1]]
        p1 = (n1['x_pixel'], n1['y_pixel'])
        p2 = (n2['x_pixel'], n2['y_pixel'])
        cv2.line(base_img, p1, p2, color, thickness)
    # Marcar inicio y fin
    if path_ids:
        n_ini = nodes[path_ids[0]]
        n_fin = nodes[path_ids[-1]]
        cv2.circle(base_img, (n_ini['x_pixel'], n_ini['y_pixel']), 7, (0, 200, 0), -1)
        cv2.circle(base_img, (n_fin['x_pixel'], n_fin['y_pixel']), 7, (0, 0, 200), -1)


def generar_figura_comparacion(entorno_name: str, paths: dict,
                                start_id: int, goal_id: int) -> str:
    """
    Genera figura de 2 paneles mostrando ruta topologica (azul)
    y ruta semantica (roja) sobre el mapa del entorno.

    Returns:
        Ruta a la imagen generada.
    """
    sem_grafo_path = paths['grafo'].replace('.json', '_semantico.json')
    if not os.path.exists(sem_grafo_path):
        print(f"  [SKIP comparacion] Grafo semantico no encontrado: {sem_grafo_path}")
        return ""

    planner = SemanticPathPlanner(sem_grafo_path)

    with open(sem_grafo_path) as f:
        graph = json.load(f)

    path_topo = planner.plan_path(start_id, goal_id)
    path_sem  = planner.plan_path_semantic(start_id, goal_id)

    pgm = paths.get('pgm', '')
    base = cv2.imread(pgm) if pgm and os.path.exists(pgm) else \
           np.full((384, 384, 3), 200, dtype=np.uint8)

    img_topo = base.copy()
    img_sem  = base.copy()

    # Dibujar nodos semanticos en ambos paneles
    nodes = {n['id']: n for n in graph['nodes']}
    for node in graph['nodes']:
        x, y = node['x_pixel'], node['y_pixel']
        sem = node.get('semantic', {})
        color = REGION_COLORS.get(sem.get('region_type', 'unknown'), (160, 160, 160))
        cv2.circle(img_topo, (x, y), 5, color, -1)
        cv2.circle(img_topo, (x, y), 5, (0,0,0), 1)
        cv2.circle(img_sem,  (x, y), 5, color, -1)
        cv2.circle(img_sem,  (x, y), 5, (0,0,0), 1)
        cv2.putText(img_topo, str(node['id']), (x+6, y-3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0,0,0), 1)
        cv2.putText(img_sem,  str(node['id']), (x+6, y-3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0,0,0), 1)

    # Rutas
    if path_topo:
        _draw_path_on_map(img_topo, graph, path_topo, (220, 50, 0), 3)  # azul
    if path_sem:
        _draw_path_on_map(img_sem,  graph, path_sem,  (0, 50, 220), 3)  # rojo

    # Etiquetas
    res  = graph['metadata'].get('resolution_m_per_pixel', 0.05)
    dist_t = planner.get_path_distance(path_topo) * res if path_topo else 0
    dist_s = planner.get_path_distance(path_sem)  * res if path_sem  else 0
    cv2.putText(img_topo, f"Topologica: {path_topo} ({dist_t:.2f}m)",
                (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0,0,0), 1)
    cv2.putText(img_sem,  f"Semantica: {path_sem} ({dist_s:.2f}m)",
                (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0,0,0), 1)

    sep = np.full((img_topo.shape[0], 6, 3), 255, dtype=np.uint8)
    combined = np.hstack([img_topo, sep, img_sem])
    title_bar = np.full((28, combined.shape[1], 3), 240, dtype=np.uint8)
    cv2.putText(title_bar,
                f"Rutas {entorno_name}: Nodo {start_id} -> {goal_id}",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 1)
    combined = np.vstack([title_bar, combined])

    out = os.path.join(RESULTS_DIR, f"{entorno_name}_comparacion_rutas.png")
    cv2.imwrite(out, combined)
    print(f"  Comparacion rutas guardada: {out}")
    return out


# ---------------------------------------------------------------------------
# Tabla Markdown para la memoria
# ---------------------------------------------------------------------------

def generar_tabla_markdown(stats_por_entorno: dict) -> str:
    """Genera tabla Markdown para incluir en la memoria."""
    lines = [
        "## Resultados de Clasificacion Semantica\n",
        "| Entorno | Nodos | Rooms | Corredores | Junctions | Estrechos | Costo medio |",
        "|---------|------:|------:|-----------:|----------:|----------:|------------:|",
    ]
    for name, stats in stats_por_entorno.items():
        if not stats:
            continue
        lines.append(
            f"| {name:<12} | {stats['total']:>5} | {stats['room']:>5} | "
            f"{stats['corridor']:>10} | {stats['junction']:>9} | "
            f"{stats['narrow']:>9} | {stats['cost_medio']:>11.2f} |"
        )
    return '\n'.join(lines) + '\n'


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    print("=" * 65)
    print("GENERANDO RESULTADOS PARA LA MEMORIA — NAVEGACION SEMANTICA")
    print("=" * 65)

    # PASO 1: Enriquecer grafos (si no existe ya el _semantico.json)
    print("\n[1/3] Enriqueciendo grafos topologicos...")
    grafos = {}
    for name, paths in ENTORNOS.items():
        sem_path = paths['grafo'].replace('.json', '_semantico.json')
        if os.path.exists(sem_path):
            print(f"  {name}: ya existe {os.path.basename(sem_path)}, reutilizando")
            with open(sem_path) as f:
                grafos[name] = json.load(f)
        else:
            grafos[name] = enrich_entorno(name, paths)

    # PASO 2: Figuras de pipeline
    print("\n[2/3] Generando figuras de pipeline...")
    for name, paths in ENTORNOS.items():
        generar_figura_pipeline(name, paths)

    # PASO 3: Figuras de comparacion de rutas
    print("\n[3/3] Generando figuras de comparacion de rutas...")
    # Pares inicio-fin representativos por entorno (los mas alejados)
    pares = {
        'escenario1': (0, 2),
        'escenario2': (0, 4),
        'escenario3': (0, 3),
        'estudio':    (4, 3),
    }
    for name, paths in ENTORNOS.items():
        start, goal = pares.get(name, (0, 1))
        sem_path = paths['grafo'].replace('.json', '_semantico.json')
        if os.path.exists(sem_path):
            planner = SemanticPathPlanner(sem_path)
            planner.print_comparison(start, goal)
        generar_figura_comparacion(name, paths, start, goal)

    # Tabla markdown
    stats_por_entorno = {}
    for name, graph in grafos.items():
        if not graph:
            continue
        nodes = graph.get('nodes', [])
        tipo_count = {t: 0 for t in ('room', 'corridor', 'junction', 'narrow')}
        costs = []
        for n in nodes:
            t = n.get('semantic', {}).get('region_type', 'unknown')
            if t in tipo_count:
                tipo_count[t] += 1
            costs.append(n.get('semantic', {}).get('traversal_cost', 1.0))
        stats_por_entorno[name] = {
            'total':      len(nodes),
            **tipo_count,
            'cost_medio': sum(costs) / len(costs) if costs else 0,
        }

    tabla = generar_tabla_markdown(stats_por_entorno)
    tabla_path = os.path.join(RESULTS_DIR, 'tabla_resultados.md')
    with open(tabla_path, 'w') as f:
        f.write(tabla)
    print(f"\n  Tabla Markdown: {tabla_path}")
    print("\n" + tabla)

    print("=" * 65)
    print(f"Resultados guardados en: {RESULTS_DIR}")
    print("=" * 65)


if __name__ == '__main__':
    main()
