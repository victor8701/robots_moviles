# Proyecto: Navegación Topológica y Semántica - Guía Completa

**Máster en Robótica y Automatización - Robots Móviles**
**Segunda Entrega: Navegación Topológica y Semántica**

## 📋 Estado del Proyecto

✅ **Completado:**
- ✓ Navegación geométrica (sesión anterior)
- ✓ Módulo de generación de mapas topológicos
- ✓ Planificador de rutas con A*
- ✓ Sistema de navegación topológica con ROS
- ✓ Módulo de clasificación semántica 3D
- ✓ Documentación teórica completa

⏳ **Pendiente (por completar durante la práctica):**
- Generación de mapas topológicos para los 4 entornos
- Capturas de pantalla individuales en clase
- Video demostrativo del sistema
- Memorias finales (topológica + semántica)

## 🎯 Objetivos del Trabajo

### Parte 1: Navegación Topológica

**Objetivo**: Crear un sistema de navegación basado en un grafo de nodos topológicos.

Se genera un **mapa topológico** a partir del mapa geométrico mediante:
1. Esqueletonización del espacio libre
2. Detección de nodos (puntos notables)
3. Cálculo de conectividad entre nodos
4. Búsqueda de caminos óptimos (A*)

**Output esperado**:
- Mapa topológico para cada entorno (imagen PNG)
- Grafo (JSON) con nodos y aristas
- Video demostrativo del robot navegando entre nodos

### Parte 2: Navegación Semántica

**Tema asignado**: Clasificación semántica basada en regiones 3D

Se enriquece la navegación con comprensión del entorno:
1. Segmentación de regiones usando información de profundidad
2. Clasificación automática (corredor, habitación, etc.)
3. Asignación de atributos semánticos (transitabilidad, dificultad)
4. Integración con navegación topológica

**Output esperado**:
- Memoria teórica explicando el método
- Ejemplos y análisis de rentabilidad
- Presentación en clase

## 📁 Estructura del Proyecto

```
robots_moviles/
├── navegacion_geometrica/          [COMPLETADO - Sesión anterior]
│   ├── src/exploration.py          Exploración automática con fronteras
│   ├── mapas/sensores/             Mapas geométricos de 4 entornos
│   ├── worlds/                     Mundos Gazebo (.world)
│   └── launch/                     Archivos de lanzamiento ROS
│
├── navegacion_topologica/          [NUEVO - Navega entre nodos]
│   ├── README.md                   ← Documentación técnica
│   ├── src/
│   │   ├── topological_mapper.py   Generador de mapas topológicos
│   │   └── path_planner.py         Planificador A* para nodos
│   ├── scripts/
│   │   ├── generate_all_maps.py    Genera mapas para 4 entornos
│   │   └── topological_navigation.py ROS node para navegación
│   └── mapas/
│       ├── topologicos/            Imágenes PNG del mapa topológico
│       └── grafos/                 Archivos JSON con grafos
│
└── navegacion_semantica/           [NUEVO - Clasificación 3D]
    ├── README.md                   ← Documentación teórica
    ├── semantic_mapper_3d.py       Clasificación de regiones 3D
    └── resultados/                 Análisis y resultados
```

## 🚀 Instrucciones de Ejecución

### Prerequisitos

```bash
# Instalar librerías requeridas
pip install pyyaml opencv-python scikit-image shapely numpy scipy

# Asegurate que ROS esté instalado
# Ubuntu 20.04 Noetic: sudo apt install ros-noetic-desktop-full
```

### Paso 1: Generar Mapas Topológicos

**Para los 4 entornos:**

```bash
cd ~/catkin_ws/src/robots_moviles/navegacion_topologica/scripts
python3 generate_all_maps.py
```

**Output:**
- `navegacion_topologica/mapas/topologicos/*.png` - Visualización del mapa
- `navegacion_topologica/mapas/grafos/*.json` - Grafo topológico

**Lo que verás:**
```
======================================================================
GENERADOR DE MAPAS TOPOLOGICOS
======================================================================

[ESCENARIO1]
----------------------------------------------------------------------
[TopologicalMapper] Creando mascara de espacio libre...
[TopologicalMapper] Calculando esqueleto...
[TopologicalMapper] Detectando nodos...
  -> Detectados 15 nodos
[TopologicalMapper] Calculando aristas...
  -> Calculadas 32 aristas
[TopologicalMapper] Construyendo grafo...
✓ Mapa topologico guardado: .../escenario1_topologico.png
✓ Grafo guardado: .../escenario1_grafo.json

[ESCENARIO2]
...
```

### Paso 2: Probar el Planificador de Rutas

**Ejemplo: Navegar de nodo 0 a nodo 5 en escenario 1:**

```bash
cd navegacion_topologica/scripts
python3 path_planner.py mapas/grafos/escenario1_grafo.json 0 5
```

**Output:**
```
Camino encontrado con 6 nodos:
  0: Nodo 0 -> (-10.00m, -10.00m)
       -> distancia: 45.2 pixeles (2.26m)
  1: Nodo 3 -> (-8.12m, -8.95m)
       -> distancia: 32.1 pixeles (1.61m)
  ...
Distancia total: 234.5 pixeles (11.73m)
Número de transiciones entre nodos: 5
```

### Paso 3: Navegación Interactiva en ROS

**Terminal 1 - Lanzar simulación:**
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch navegacion_geometrica turtlebot3_escenario1.launch
```

**Terminal 2 - Ejecutar navegador topológico:**
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun navegacion_topologica topological_navigation.py \
  mapas/grafos/escenario1_grafo.json
```

**Seleccionar destino interactivamente:**
```
======================================================================
NODOS DEL MAPA TOPOLOGICO
======================================================================
Nodo  0: (-10.00m, -10.00m) - Distancia a pared: 12.3px
Nodo  1: (-8.50m, -8.75m)  - Distancia a pared: 11.8px
...

Selecciona nodo destino (0-14): 7
```

El robot navegará automáticamente pasando por los nodos intermedios.

## 📊 Análisis de Resultados Esperados

### Para cada entorno obtendrás:

**Nodos detectados:**
- Escenario 1 (pequeño): ~10-15 nodos
- Escenario 2 (mediano): ~20-30 nodos
- Escenario 3 (grande): ~25-35 nodos
- Estudio (complejo): ~30-40 nodos

**Propiedades del grafo:**
- Densidad de conexión: 30-50%
- Camino promedio entre nodos: 3-8 transiciones
- Distancia promedio: 2-4 metros

### Visualización del Mapa Topológico

En la imagen PNG generada:
- **Gris**: Mapa original (obstáculos en negro)
- **Verde**: Esqueleto (línea central del espacio libre)
- **Azul**: Nodos (con ID numerado)
- **Rojo**: Aristas (conexiones entre nodos)

## 📝 Requisitos de Entrega

### Parte Individual (En clase)
- [ ] Captura de pantalla del mapa topológico generado
- [ ] Captura de pantalla de la navegación en RViz
- [ ] Captura de ejecución del script

### Parte Topológica (Grupal)
- [ ] **Memoria** (máx 4 hojas):
  - Explicación del algoritmo de generación
  - Resultados para los 4 entornos (tabla comparativa)
  - Justificación del planificador elegido (A*)
  - Análisis de complejidad y rendimiento

- [ ] **Mapas topológicos**: 4 imágenes PNG (una por entorno)

- [ ] **Video** (2-3 min):
  - Demostración de navegación en 2-3 escenarios
  - Visualización en RViz del camino planificado
  - Robot ejecutando la navegación autónoma

- [ ] **Código** (Python scripts):
  - topological_mapper.py
  - path_planner.py
  - topological_navigation.py
  - generate_all_maps.py

### Parte Semántica (Grupal)
- [ ] **Memoria** (máx 6 hojas):
  - Explicación del método: Clasificación de regiones 3D
  - Fundamento teórico con 5+ referencias científicas
  - Ejemplos de segmentación y clasificación
  - **Análisis de rentabilidad**:
    - Cómo mejora la navegación topológica
    - Aplicaciones potenciales (búsqueda y rescate, limpieza, exploración)
    - Limitaciones y mejoras futuras

- [ ] **Presentación** (10-15 min):
  - Explicación del tema
  - Demostración visual (si es posible)
  - Vinculación con trabajos geométrico y topológico

## 🔗 Integración de los Tres Niveles

```
NAVEGACION SEMANTICA
├─ Información 3D (depth)
├─ Clasificación de regiones
└─ Atributos (transitabilidad, dificultad)
        ↓
NAVEGACION TOPOLOGICA
├─ Grafo de nodos
├─ Planificación A*
└─ Camino: Nodo 0 → 5 → 12 → 18
        ↓
NAVEGACION GEOMETRICA
├─ Mapa de ocupancia
├─ Evitar obstáculos
└─ Controlar cinemática del robot
        ↓
EJECUCION EN GAZEBO/REAL
└─ Robot alcanza objetivo
```

## 💡 Tips para el Trabajo

1. **Antes de la clase:**
   - Instala todas las dependencias
   - Ejecuta `generate_all_maps.py` para validar
   - Prueba `path_planner.py` con distintos nodos

2. **Durante la clase:**
   - Toma **capturas** de los mapas topológicos generados
   - **Captura** ejecución de la navegación en RViz
   - Anota **números** de nodos detectados

3. **Para la memoria:**
   - Incluye tabla comparativa de entornos (nodos, aristas, densidad)
   - Justifica parámetros (skeleton_dilation, min_distance, max_distance)
   - Analiza trade-off precisión vs. velocidad

4. **Para el video:**
   - Usa `rqt_graph` para mostrar nodos de ROS
   - Visualiza el grafo en RViz
   - Muestra el robot navegando (vista 3D en Gazebo)

5. **Para navegación semántica:**
   - Revisa las 5 referencias científicas proporcionadas
   - Enfatiza aplicabilidad al trabajo práctico
   - Propone futura integración (ej: YOLO para detección)

## 📚 Documentación Adicional

- `navegacion_topologica/README.md` - Detalles técnicos de topología
- `navegacion_semantica/README.md` - Teoría de clasificación 3D
- Scripts incluyen docstrings detallados

## ⚠️ Troubleshooting

**"ModuleNotFoundError: No module named 'yaml'"**
```bash
pip install pyyaml
```

**"No se encuentran mapas geométricos"**
- Verifica que `navegacion_geometrica/mapas/sensores/*.pgm` existan

**Imagen topológica sale en blanco/negro**
- Aumenta `skeleton_dilation` en `generate_all_maps.py`
- Ajusta parámetros de `_detect_nodes(min_distance=...)`

**El robot no navega en ROS**
- Verifica que `move_base` esté activo: `rosnode list`
- Comprueba que el grafo JSON existe y es válido
- Revisa logs: `rosbag record -a` y luego `rqt_bag`

## 🎓 Conceptos Clave

| Concepto | Descripción | Herramienta |
|----------|-------------|-----------|
| Esqueletonización | Extrae línea central del espacio | scikit-image |
| Distance Transform | Calcula distancia a obstáculo | OpenCV |
| A* Search | Búsqueda de camino óptimo | Implementación propia |
| Segmentación 3D | Agrupa puntos de profundidad | Flood-fill / DBSCAN |
| Clasificación | Etiqueta regiones semánticamente | Heurísticas + deep learning |

## ✅ Checklist Final

Antes de entregar:

- [ ] Todos los scripts ejecutables sin errores
- [ ] Mapas topológicos generados para 4 entornos
- [ ] Prueba path_planner.py en múltiples pares de nodos
- [ ] Video de navegación completado
- [ ] Memoria topológica con 4 hojas
- [ ] Memoria semántica con 6 hojas + referencias
- [ ] Código Python comentado y limpio
- [ ] Presentación preparada

---

**Entrega**: 3 de mayo
**Formato**: Python scripts + PDFs de memorias + Video
