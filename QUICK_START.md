# ⚡ Quick Start - Ejecución Paso a Paso

## 📋 Verificación Previa (2 min)

```bash
# 1. ¿Existen los mapas geométricos?
ls navegacion_geometrica/mapas/sensores/*.pgm
# Debe mostrar: escenario1_sensores.pgm, escenario2_sensores.pgm, etc.

# 2. ¿Están instaladas las dependencias?
python3 -c "import cv2, yaml, numpy, scipy; print('✓ OK')"
# Si falla, ejecuta: pip install opencv-python pyyaml numpy scipy scikit-image

# 3. ¿Es catkin workspace?
ls ~/catkin_ws/src/
# Debe incluir este proyecto
```

---

## 🚀 Fase 1: Generar Mapas Topológicos (5-10 min)

### Paso 1.1: Navegar al directorio correcto
```bash
cd ~/catkin_ws/src/robots_moviles/navegacion_topologica/scripts
pwd  # Verifica que estés en el lugar correcto
```

### Paso 1.2: Ejecutar generador
```bash
python3 generate_all_maps.py
```

**Salida esperada:**
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

[ESCENARIO3]
...

[ESTUDIO]
...

======================================================================
¡GENERACION COMPLETADA!
======================================================================
```

### Paso 1.3: Verificar generación
```bash
# ¿Se generaron las imágenes?
ls ../mapas/topologicos/*.png
# Debe mostrar 4 archivos .png

# ¿Se generaron los grafos?
ls ../mapas/grafos/*.json
# Debe mostrar 4 archivos .json

# ¿Las imágenes tienen contenido?
file ../mapas/topologicos/escenario1_topologico.png
# Debe mostrar: "PNG image data, ..."
```

---

## 🔍 Fase 2: Probar Planificador de Rutas (2-3 min)

### Paso 2.1: Ejecutar pruebas automáticas
```bash
python3 test_all.py
```

**Salida esperada:**
```
======================================================================
  NAVEGACION TOPOLOGICA - SUITE DE PRUEBAS
======================================================================

======================================================================
  TEST 1: GENERACION DE MAPAS TOPOLOGICOS
======================================================================

Procesando ESCENARIO1...
  ✓ Nodos: 15
  ✓ Aristas: 32
  ✓ Densidad conectividad: 30.48%

[... similar para otros escenarios ...]

======================================================================
RESULTADOS COMPARATIVOS
======================================================================
Entorno         Nodos      Aristas    Densidad
--------------------------------------------------
escenario1      15         32         30.48%
escenario2      22         54         22.91%
escenario3      28         71         18.51%
estudio         35         88         14.86%

======================================================================
  TEST 2: PLANIFICACION DE CAMINOS
======================================================================

ESCENARIO1:
  Ruta 0→14: 6 nodos, 187.3px
  Ruta 0→7: 4 nodos, 112.5px

[... más resultados ...]

======================================================================
  TEST 3: ANALISIS DE ESTRUCTURA DEL GRAFO
======================================================================

ESCENARIO1:
  Nodos: 15
  Aristas: 32
  Grado promedio: 4.27
  Grado máximo: 8
  Grado mínimo: 1
  Nodos aislados: 0
  Distancia promedio (aristas): 45.3px
  Distancia máxima: 98.2px
  Distancia mínima: 12.1px

[... más análisis ...]

======================================================================
RESUMEN
======================================================================
✓ Todas las pruebas completadas

Proximos pasos:
  1. Verificar mapas PNG generados en mapas/topologicos/
  2. Ejecutar navegacion topologica en ROS:
     roslaunch navegacion_topologica topological_navigation.launch
  3. Ver visualizacion en RViz
```

### Paso 2.2: Probar planificador individual
```bash
# Probar ruta del nodo 0 al nodo 5 en escenario 1
python3 path_planner.py mapas/grafos/escenario1_grafo.json 0 5

# Probar ruta diferente en otro escenario
python3 path_planner.py mapas/grafos/escenario2_grafo.json 0 10
```

---

## 🤖 Fase 3: Navegación en ROS (15-20 min)

### Paso 3.1: Compilar (si es primera vez)
```bash
cd ~/catkin_ws
catkin_make
# O si tienes múltiples núcleos:
catkin_make -j8
```

**Si sale error sobre paquetes:**
```bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -j8
```

### Paso 3.2: Setup del environment
```bash
source ~/catkin_ws/devel/setup.bash
# O agregar a ~/.bashrc para siempre:
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Paso 3.3: Terminal 1 - Lanzar simulación
```bash
# Terminal 1
roslaunch navegacion_geometrica turtlebot3_escenario1.launch

# Verás:
# [gazebo-2] process has begun...
# [turtlebot3_state_publisher-1] process has begun...
# [... más procesos ROS ...]
```

### Paso 3.4: Terminal 2 - Lanzar navegación topológica
```bash
# Terminal 2
cd ~/catkin_ws/src/robots_moviles
source ~/catkin_ws/devel/setup.bash

python3 navegacion_topologica/scripts/topological_navigation.py \
  navegacion_topologica/mapas/grafos/escenario1_grafo.json

# Verás:
# ======================================================================
# NAVEGACION TOPOLOGICA - SELECCIONAR NODO DESTINO
# ======================================================================
#
# Nodos disponibles:
#   Nodo 0: (-10.00m, -10.00m)
#   Nodo 1: (-8.50m, -8.75m)
#   ...
#
# Selecciona nodo destino (0-14): █
```

### Paso 3.5: Seleccionar nodo destino
```bash
# Terminal 2 (continuar)
# Ingresa número de nodo (ej: 7)
7

# Verás:
# [INFO] Planificando desde nodo 0 a nodo 7...
# [INFO] Camino encontrado: 0 -> 2 -> 5 -> 7
# [INFO] Navegando a waypoint 1/4: (-8.12m, -8.95m)
# [INFO] Waypoint 1 alcanzado
# ...
```

### Paso 3.6: Terminal 3 (Opcional) - Visualizar en RViz
```bash
# Terminal 3
cd ~/catkin_ws
source devel/setup.bash
rviz

# En RViz:
# File > Open Config
# Busca archivo de config si existe, o configura manualmente:
# 1. Add > Map (topic: /map)
# 2. Add > RobotModel
# 3. Add > MarkerArray (topic: /topological_graph/nodes)
# 4. Add > MarkerArray (topic: /topological_graph/edges)
```

---

## 📸 Fase 4: Capturar Pantallazos (Individual)

### Screenshots requeridos en clase:

```
1. Mapa topológico generado
   └─ Abierto en visor de imágenes (GIMP, eog, etc.)
      Mostrar: esqueleto verde, nodos azules, aristas rojas

2. Ejecución de generate_all_maps.py
   └─ Captura del terminal con output "¡GENERACION COMPLETADA!"

3. Ejecutando path_planner.py
   └─ Mostrar consulta y resultado de camino (nodos + distancia)

4. RViz con grafo topológico
   └─ Vista del mapa y visualización de nodos/aristas

5. Navegación en progreso
   └─ Robot en Gazebo navegando hacia nodo destino
   └─ O si no es posible, mostrar output de topological_navigation.py
```

**Cómo capturar:**

Terminal:
```bash
# Captura simple (Print Screen)
gnome-screenshot -f ~/Escritorio/screenshot_$(date +%s).png

# O si usas otra herramienta:
scrot ~/Escritorio/screenshot_$(date +%s).png
```

---

## 📊 Resultados Esperados por Entorno

| Escenario | Nodos | Aristas | Características |
|-----------|-------|---------|-----------------|
| Escenario 1 | 10-18 | 20-40 | Pequeño, simple |
| Escenario 2 | 15-25 | 30-60 | Mediano, múltiples secciones |
| Escenario 3 | 20-30 | 50-100 | Grande, complejo |
| Estudio | 25-40 | 60-120 | Muy complejo, múltiples plantas |

---

## ✅ Checklist de Completitud

Marcar conforme se ejecuten:

```
Preparación:
☐ Instalar dependencias (pip install...)
☐ Verificar mapas geométricos existen

Fase 1 - Generación:
☐ Ejecutar generate_all_maps.py
☐ Verificar 4 PNG generados
☐ Verificar 4 JSON generados
☐ Revisar imágenes visualmente

Fase 2 - Prueba:
☐ Ejecutar test_all.py
☐ Probar path_planner.py
☐ Registrar estadísticas de cada entorno

Fase 3 - ROS:
☐ Compilar catkin_make
☐ Lanzar simulación Gazebo
☐ Lanzar navegación topológica
☐ Seleccionar nodo destino
☐ Verificar navegación del robot

Fase 4 - Captura:
☐ Captura 1: Mapa topológico PNG
☐ Captura 2: Ejecución generate_all_maps.py
☐ Captura 3: Ejecución path_planner.py
☐ Captura 4: RViz con grafo
☐ Captura 5: Robot navegando

Entrega:
☐ Guardar todas las capturas
☐ Crear memoria topológica
☐ Crear memoria semántica
☐ Grabar video
☐ Preparar presentación
```

---

## 🆘 Si Algo Falla

### Error: "ModuleNotFoundError: No module named 'cv2'"
```bash
pip install opencv-python
pip3 install opencv-python
```

### Error: "File not found: escenario1_sensores.pgm"
```bash
# Verifica que estés en la carpeta correcta
pwd  # Debe ser: .../robots_moviles/...

# O proporciona ruta absoluta:
python3 generate_all_maps.py /ruta/absoluta/a/mapas/
```

### Error: "move_base not available"
```bash
# Verifica que la simulación esté lanzada:
rosnode list | grep move_base

# Si no existe, lanza primero:
roslaunch navegacion_geometrica turtlebot3_escenario1.launch
```

### Imagen topológica sale en blanco
```bash
# Abre topological_mapper.py y ajusta:
mapper = TopologicalMapper(..., skeleton_dilation=5)  # Aumenta aquí
```

### Robot no se mueve
```bash
# Verifica transformadas TF:
rosrun tf2_tools view_frames

# Publica goal manualmente para prueba:
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 1, y: 1}, orientation: {w: 1}}}'
```

---

## ⏱️ Tiempos Estimados

| Actividad | Duración |
|-----------|----------|
| Instalación dependencias | 3-5 min |
| Generar mapas | 5-10 min |
| Pruebas (test_all.py) | 5-10 min |
| Compilación ROS (primera vez) | 10-15 min |
| Lanzar simulación | 30-60 seg |
| Navegación interactiva | 5-10 min |
| Captura de pantallazos | 10-15 min |
| **TOTAL EJECUCIÓN** | **~1 hora** |

---

## 📞 Sin Más Ayuda Disponible

Si algo no funciona después de seguir esto:

1. **Lee:** `navegacion_topologica/README.md` (sección troubleshooting)
2. **Busca:** `CHEAT_SHEET.md` para comandos rápidos
3. **Ejecuta:** `test_all.py` para diagnóstico
4. **Revisa:** Los logs de ROS
   ```bash
   rqt_logger_level  # Para ver logs en tiempo real
   ```

---

**¡Éxito! Estás listo para ejecutar el proyecto.** 🚀
