# Navegación Topológica - Instrucciones

Este módulo proporciona un sistema completo de navegación topológica basado en mapas geométricos existentes.

## Descripción General

El sistema transforma mapas geométricos de navegación en **mapas topológicos** que representan los espacios libres como grafos de nodos conectados. La navegación se realiza saltando de nodo en nodo usando el algoritmo A*.

### Componentes Principales

1. **topological_mapper.py** - Genera mapas topológicos a partir de mapas geométricos
2. **path_planner.py** - Implementa búsqueda de caminos con A*
3. **generate_all_maps.py** - Script que genera mapas para los 4 entornos
4. **topological_navigation.py** - ROS node para navegación interactiva

## Instalacion de Dependencias

Antes de usar este módulo, instala las siguientes librerías:

```bash
pip install pyyaml
pip install opencv-python
pip install scikit-image
pip install numpy scipy
```

## Uso Básico

### Paso 1: Generar Mapas Topológicos

Genera mapas topológicos para todos los entornos:

```bash
roscd navegacion_topologica/scripts
python3 generate_all_maps.py
```

Esto creará:
- Imágenes PNG con visualización del mapa topológico en `mapas/topologicos/`
- Archivos JSON con el grafo en `mapas/grafos/`

### Paso 2: Planificar Rutas

Prueba el planificador de rutas:

```bash
python3 path_planner.py mapas/grafos/escenario1_grafo.json 0 5
```

Salida:
```
Camino encontrado con 6 nodos:
  0: Nodo 0 -> (-10.00m, -10.00m)
       -> distancia: 45.2 pixeles (2.26m)
  1: Nodo 3 -> (-8.12m, -8.95m)
  ...
Distancia total: 234.5 pixeles (11.73m)
Número de transiciones entre nodos: 5
```

### Paso 3: Navegación Interactiva con ROS

En una terminal iniciando ROS:

```bash
roslaunch navegacion_geometrica turtlebot3_escenario1.launch
```

En otra terminal:

```bash
rosrun navegacion_topologica topological_navigation.py mapas/grafos/escenario1_grafo.json
```

Se mostrará una lista de nodos disponibles y podrás seleccionar uno para navegar hacia él.

## Estructura de Archivos

```
navegacion_topologica/
├── src/
│   ├── topological_mapper.py      # Generador de mapas topológicos
│   └── path_planner.py             # Planificador A*
├── scripts/
│   ├── generate_all_maps.py        # Genera mapas para 4 entornos
│   └── topological_navigation.py   # Node ROS principal
├── mapas/
│   ├── topologicos/                # Imágenes PNG del mapa topológico
│   │   ├── escenario1_topologico.png
│   │   ├── escenario2_topologico.png
│   │   ├── escenario3_topologico.png
│   │   └── estudio_topologico.png
│   └── grafos/                      # Grafos en formato JSON
│       ├── escenario1_grafo.json
│       ├── escenario2_grafo.json
│       ├── escenario3_grafo.json
│       └── estudio_grafo.json
└── resultados/                       # Resultados y análisis
```

## Estructura del Archivo JSON del Grafo

```json
{
  "nodes": [
    {
      "id": 0,
      "x_pixel": 150,
      "y_pixel": 200,
      "distance_to_wall": 15.3
    },
    ...
  ],
  "edges": [
    {
      "from_node": 0,
      "to_node": 3,
      "distance_pixels": 45.2
    },
    ...
  ],
  "metadata": {
    "map_image": "escenario1_sensores.pgm",
    "resolution_m_per_pixel": 0.05,
    "origin_x": -10.0,
    "origin_y": -10.0
  }
}
```

## Algoritmo de Generación de Mapas Topológicos

### Paso 1: Extracción de Espacio Libre
- Lee el mapa geométrico (.pgm)
- Crea mascara binaria separando espacio libre de obstáculos

### Paso 2: Esqueletonización
- Aplica algoritmo de esqueletonización (thinning) al espacio libre
- Genera línea central (espina dorsal) que representa la topología

### Paso 3: Detección de Nodos
- Calcula Distance Transform (distancia a obstáculo más cercano)
- Detecta máximos locales en el Distance Transform
- Estos son los nodos topológicos (puntos centrales del espacio)

### Paso 4: Cálculo de Aristas
- Conecta nodos cercanos (<100 píxeles)
- Verifica que exista camino viable en el esqueleto
- Genera grafo no dirigido bidireccional

### Paso 5: Búsqueda de Caminos (A*)
- Usa distancia euclidiana como heurística
- Encuentra camino óptimo (mínimo número de nodos)
- Convierte camino a waypoints en metros

## Parámetros Ajustables

### En topological_mapper.py:

```python
mapper = TopologicalMapper(
    map_pgm,
    map_yaml,
    skeleton_dilation=3  # Grosor del esqueleto (píxeles)
)
```

- **skeleton_dilation**: Mayor valor = esqueleto más robusto pero menos detallado
- Valores recomendados: 2-5

### En _detect_nodes():

```python
mapper._detect_nodes(min_distance=15)  # Distancia mínima entre nodos (píxeles)
```

- Aumentar para menos nodos (navegación más rápida)
- Disminuir para más nodos (navegación más precisa)

### En _compute_edges():

```python
mapper._compute_edges(max_distance=100)  # Distancia máxima de conexión (píxeles)
```

- Aumentar para grafo más conexo
- Disminuir para grafo más disperso

## Análisis de Resultados

Para cada entorno obtendrás:

1. **Imagen topológica** (PNG):
   - Fondo: mapa original (gris=libre, negro=obstáculo)
   - Verde: esqueleto
   - Azul: nodos (con ID)
   - Rojo: aristas/conexiones

2. **Estadísticas del grafo**:
   - Número de nodos
   - Número de aristas
   - Densidad de conexión
   - Distancia promedio entre nodos

3. **Archivo JSON**:
   - Grafo completo listo para consultas
   - Metadatos para conversión píxel↔metros

## Troubleshooting

### "ModuleNotFoundError: No module named 'yaml'"
```bash
pip install pyyaml
```

### "ModuleNotFoundError: No module named 'skimage'"
```bash
pip install scikit-image
```

### No se encuentran mapas geométricos
Verifica que los archivos existan en:
```
navegacion_geometrica/mapas/sensores/
  - escenario1_sensores.pgm/.yaml
  - escenario2_sensores.pgm/.yaml
  - escenario3_sensores.pgm/.yaml
  - estudio_sensores.pgm/.yaml
```

### El robot no navega entre nodos
- Verifica que move_base esté funcionando correctamente
- Comprueba que el grafo se haya generado correctamente
- Revisa los logs de ROS: `rosgraph`

## Referencias y Métodos

### Esqueletonización
- Implementación: scikit-image.morphology.skeletonize
- Método: Algoritmo topológico de adelgazamiento

### Detección de Nodos
- Distance Transform: OpenCV cv2.distanceTransform
- Máximos locales: scipy.ndimage.maximum_filter
- Criterio: Puntos de máxima distancia a obstáculos

### Búsqueda de Caminos
- Algoritmo A* (Dijkstra + heurística euclidiana)
- Complejidad: O(|E| log |V|) con heap
- Óptimalidad: Garantiza camino más corto en términos de nodos

## Próximos Pasos

Para integración con navegación semántica:
1. Clasificar regiones 3D del espacio
2. Asignar atributos semánticos a nodos
3. Usar información semántica en queries de navegación
