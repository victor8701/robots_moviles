#!/usr/bin/env python3
"""
Genera una figura de 3 paneles por entorno para la memoria:
  Panel 1: espacio libre (mascara binaria)
  Panel 2: esqueleto de traversabilidad
  Panel 3: mapa topologico (grafo completo)
Tambien imprime la tabla resumen lista para copiar a la memoria.
"""

import os, sys, json
import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from topological_mapper import TopologicalMapper

ENTORNOS   = ['escenario1', 'escenario2', 'escenario3', 'estudio']
BASE_DIR   = os.path.dirname(os.path.abspath(__file__))
MAP_DIR    = os.path.join(BASE_DIR, '..', '..', 'navegacion_geometrica', 'mapas', 'exploration')
GRAPH_DIR  = os.path.join(BASE_DIR, '..', 'mapas', 'grafos')
MEM_DIR    = os.path.join(BASE_DIR, '..', 'mapas', 'memoria')
os.makedirs(MEM_DIR, exist_ok=True)

# Margen alrededor del espacio libre para recortar la imagen
MARGIN_PX = 20


def bbox_free_space(mask):
    """Bounding box del espacio libre con margen."""
    ys, xs = np.where(mask > 0)
    if len(ys) == 0:
        return None
    h, w = mask.shape
    y0 = max(0, ys.min() - MARGIN_PX)
    y1 = min(h, ys.max() + MARGIN_PX)
    x0 = max(0, xs.min() - MARGIN_PX)
    x1 = min(w, xs.max() + MARGIN_PX)
    return x0, y0, x1, y1


def recortar(img, bbox):
    x0, y0, x1, y1 = bbox
    return img[y0:y1, x0:x1]


def dibujar_grafo(mapper):
    """Panel 3: mapa original + esqueleto + grafo."""
    # Quitamos el offset, dibujamos directamente sobre la imagen original
    vis = cv2.cvtColor(mapper.map_image, cv2.COLOR_GRAY2BGR)
    
    # Esqueleto en verde claro
    skel_mask = mapper.thin_skeleton > 0
    vis[skel_mask] = [100, 220, 100]

    # Aristas en rojo
    for edge in mapper.edges:
        n1 = mapper.nodes[edge['from_node']]
        n2 = mapper.nodes[edge['to_node']]
        # Usamos las coordenadas exactas del nodo
        x1, y1 = n1['x_pixel'], n1['y_pixel']
        x2, y2 = n2['x_pixel'], n2['y_pixel']
        cv2.line(vis, (x1, y1), (x2, y2), (0, 0, 200), 2)

    # Nodos en azul con ID
    for node in mapper.nodes:
        # Usamos las coordenadas exactas del nodo
        x = node['x_pixel']
        y = node['y_pixel']
        cv2.circle(vis, (x, y), 6, (200, 50, 0), -1)
        cv2.circle(vis, (x, y), 6, (255, 255, 255), 1)
        cv2.putText(vis, str(node['id']), (x + 7, y - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 50, 0), 1)
    return vis


def procesar(entorno):
    pgm  = os.path.join(MAP_DIR, f'mapa_{entorno}.pgm')
    yaml = os.path.join(MAP_DIR, f'mapa_{entorno}.yaml')

    mapper = TopologicalMapper(pgm, yaml, skeleton_dilation=3)
    mapper._create_free_space_mask()
    mapper._compute_skeleton()
    mapper._detect_nodes(min_distance=20)
    mapper._compute_edges(max_distance=200)
    mapper._build_graph()

    bbox = bbox_free_space(mapper.free_space_mask)
    if bbox is None:
        print(f'  [WARN] {entorno}: sin espacio libre detectable')
        return None
    x0, y0, x1, y1 = bbox

    # --- Panel 1: espacio libre ---
    p1 = cv2.cvtColor(mapper.map_image, cv2.COLOR_GRAY2BGR)
    free_mask = mapper.free_space_mask > 0
    p1[free_mask]  = [255, 255, 255]
    p1[~free_mask] = [40,  40,  40]
    p1 = recortar(p1, bbox)

    # --- Panel 2: traversabilidad (esqueleto fino) ---
    p2 = cv2.cvtColor(mapper.map_image, cv2.COLOR_GRAY2BGR)
    p2[mapper.thin_skeleton > 0] = [0, 210, 0]
    p2 = recortar(p2, bbox)

    # --- Panel 3: mapa topologico ---
    p3 = dibujar_grafo(mapper)
    p3 = recortar(p3, bbox)

# --- Etiquetas ---
    def label(img, text):
        out = img.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        escala = 0.55
        grosor_blanco = 1
        
        # 1. Calcular en píxeles cuánto va a ocupar el texto
        (ancho_texto, alto_texto), baseline = cv2.getTextSize(text, font, escala, grosor_blanco)
        
        # 2. Si el texto es más ancho que la imagen, añadimos padding a la derecha
        margen = 10
        if ancho_texto + margen > out.shape[1]:
            extra_w = (ancho_texto + margen) - out.shape[1]
            # Rellenamos con un gris claro (similar a la zona desconocida del mapa)
            out = cv2.copyMakeBorder(out, 0, 0, 0, extra_w, 
                                     cv2.BORDER_CONSTANT, value=(205, 205, 205))
            
        # 3. Escribir el texto (borde negro para contraste y relleno blanco)
        cv2.putText(out, text, (5, 18), font, escala, (0, 0, 0), 3)
        cv2.putText(out, text, (5, 18), font, escala, (255, 255, 255), grosor_blanco)
        
        return out

    p1 = label(p1, 'Espacio libre')
    p2 = label(p2, 'Traversabilidad')
    p3 = label(p3, f'Mapa topologico ({len(mapper.nodes)}n, {len(mapper.edges)}a)')

    # Igualar altura antes de concatenar
    th = max(p1.shape[0], p2.shape[0], p3.shape[0])
    def pad_h(img, h):
        extra = h - img.shape[0]
        return cv2.copyMakeBorder(img, 0, extra, 0, 0,
                                  cv2.BORDER_CONSTANT, value=(60, 60, 60))
    p1 = pad_h(p1, th)
    p2 = pad_h(p2, th)
    p3 = pad_h(p3, th)

    # Separador vertical
    sep = np.full((th, 4, 3), 80, dtype=np.uint8)
    panel = np.concatenate([p1, sep, p2, sep, p3], axis=1)

# --- Titulo del entorno ---
    title_h = 28
    txt = f'Entorno: {entorno.upper()}   |   '
    res = mapper.graph['metadata']['resolution_m_per_pixel']
    txt += f'Resolucion: {res} m/px   |   '
    txt += f'Nodos: {len(mapper.nodes)}   |   Aristas: {len(mapper.edges)}'

    font_title = cv2.FONT_HERSHEY_SIMPLEX
    scale_title = 0.48
    thick_title = 1

    # 1. Calculamos el ancho real del texto del encabezado
    (ancho_txt, alto_txt), _ = cv2.getTextSize(txt, font_title, scale_title, thick_title)
    margen_titulo = 15

    # 2. Si el texto es más ancho que la suma de los 3 paneles, ensanchamos el panel base
    if ancho_txt + margen_titulo > panel.shape[1]:
        extra_w = (ancho_txt + margen_titulo) - panel.shape[1]
        # Añadimos un borde a la derecha (puedes cambiar el color si prefieres que no sea gris)
        panel = cv2.copyMakeBorder(panel, 0, 0, 0, extra_w, 
                                   cv2.BORDER_CONSTANT, value=(205, 205, 205))

    # 3. Creamos la barra del título con el ancho final del panel y escribimos el texto
    title = np.full((title_h, panel.shape[1], 3), 40, dtype=np.uint8)
    cv2.putText(title, txt, (6, 19), font_title, scale_title, (200, 200, 200), thick_title)
    
    # 4. Concatenamos el título arriba del panel
    panel = np.concatenate([title, panel], axis=0)

    out_path = os.path.join(MEM_DIR, f'{entorno}_pipeline.png')
    cv2.imwrite(out_path, panel)
    print(f'  Guardado: {out_path}')
    return {
        'entorno': entorno,
        'nodos': len(mapper.nodes),
        'aristas': len(mapper.edges),
        'resolucion': res,
        'nodes': mapper.nodes,
        'edges': mapper.edges,
        'meta': mapper.graph['metadata'],
    }

def tabla_memoria(resultados):
    print('\n' + '=' * 70)
    print('TABLA PARA LA MEMORIA')
    print('=' * 70)
    print(f'{"Entorno":<14} {"Nodos":>6} {"Aristas":>8} {"Res(m/px)":>10} {"Area libre approx.":>20}')
    print('-' * 70)
    for r in resultados:
        print(f'{r["entorno"]:<14} {r["nodos"]:>6} {r["aristas"]:>8} '
              f'{r["resolucion"]:>10.4f}  (ver imagen)')
    print('=' * 70)

    print('\nPOSICION DE NODOS (para la memoria)')
    for r in resultados:
        print(f'\n  {r["entorno"].upper()}')
        ox = r['meta']['origin_x']
        oy = r['meta']['origin_y']
        res = r['meta']['resolution_m_per_pixel']
        for n in r['nodes']:
            xm = ox + n['x_pixel'] * res
            ym = oy + n['y_pixel'] * res
            print(f'    Nodo {n["id"]}: ({xm:.2f} m, {ym:.2f} m)'
                  f'  dist_pared={n["distance_to_wall"]*res:.2f} m')


def main():
    print('=' * 60)
    print('GENERANDO FIGURAS PARA LA MEMORIA')
    print('=' * 60)
    resultados = []
    for entorno in ENTORNOS:
        print(f'\n[{entorno.upper()}]')
        r = procesar(entorno)
        if r:
            resultados.append(r)
    tabla_memoria(resultados)
    print(f'\nImagenes guardadas en: {MEM_DIR}')


if __name__ == '__main__':
    main()
