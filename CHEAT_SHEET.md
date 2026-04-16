# Guía Rápida - Navegación Topológica y Semántica

## 🚀 Inicio Rápido (2 min)

### Instalar dependencias
```bash
pip install pyyaml opencv-python scikit-image numpy scipy
```

### Generar mapas para los 4 entornos
```bash
cd navegacion_topologica/scripts
python3 generate_all_maps.py
```

### Probar planificador
```bash
cd navegacion_topologica/scripts
python3 path_planner.py mapas/grafos/escenario1_grafo.json 0 5
```

---

## 📊 Verificar Resultados

**¿Se generaron correctamente los mapas?**
```bash
ls navegacion_topologica/mapas/topologicos/*.png  # Debe haber 4 imágenes
ls navegacion_topologica/mapas/grafos/*.json      # Debe haber 4 JSON
```

**¿Se vieron datos sensibles en los mapas?**
- Abre las imágenes PNG con un visor de imágenes
- Debes ver: mapa gris, esqueleto verde, nodos azules, aristas rojas

---

## 🤖 Con ROS (Navegación Real)

### Terminal 1: Lanzar simulación
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch navegacion_geometrica turtlebot3_escenario1.launch
```

### Terminal 2: Ejecutar navigator topológico
```bash
source ~/catkin_ws/devel/setup.bash
rosrun navegacion_topologica topological_navigation.py \
  ~/catkin_ws/src/robots_moviles/navegacion_topologica/mapas/grafos/escenario1_grafo.json
```

### Terminal 3 (Opcional): Visualizar grafo
```bash
rosrun navegacion_topologica graph_visualizer.py \
  ~/catkin_ws/src/robots_moviles/navegacion_topologica/mapas/grafos/escenario1_grafo.json

# En RViz:
# - Add > By topic > /topological_graph/nodes > Marker
# - Add > By topic > /topological_graph/edges > Marker
```

---

## 📈 Interpretar Resultados

### Estadísticas esperadas por entorno:

| Entorno | Nodos típicos | Aristas típicas | Densidad |
|---------|---------------|-----------------|----------|
| Escenario 1 | 10-15 | 20-40 | 20-40% |
| Escenario 2 | 15-25 | 30-60 | 15-35% |
| Escenario 3 | 20-30 | 50-100 | 15-30% |
| Estudio | 25-40 | 60-120 | 15-25% |

### Qué dicen estos números:

- **Nodos**: Puntos clave del espacio. Más = mapa más refinado
- **Aristas**: Conexiones viables. Más = grafo bien conectado
- **Densidad**: Porcentaje de conexiones posibles. 25-40% es típico

---

## 🔧 Ajustar Parámetros

### Para MÁS nodos (mapa más detallado):
En `topological_mapper.py`:
```python
mapper._detect_nodes(min_distance=5)  # Reducir este valor
```

### Para MENOS nodos (mapa más simplificado):
```python
mapper._detect_nodes(min_distance=25)  # Aumentar este valor
```

### Para aristas más densas:
```python
mapper._compute_edges(max_distance=150)  # Aumentar distancia máxima
```

### Para esqueleto más robusto:
```python
mapper = TopologicalMapper(..., skeleton_dilation=5)  # Aumentar dilatación
```

---

## 📝 Tareas para la Entrega

### Individual (en clase)
- [ ] Captura de pantalla: mapa topológico generado
- [ ] Captura de pantalla: navegación en RViz
- [ ] Captura de pantalla: ejecución de script

### Grupal - Parte Topológica
**Archivo: memoria_topologica.pdf (máx 4 hojas)**
```
1. Introducción al problema
2. Explicación algoritmo de generación (esqueletonización, detección nodos)
3. Tabla comparativa: nodos/aristas/densidad de los 4 entornos
4. Justificación A* vs otras opciones (Dijkstra, BFS)
5. Análisis de complejidad y rendimiento
6. Conclusiones
```

**Archivos de código:**
- `navegacion_topologica/src/topological_mapper.py`
- `navegacion_topologica/src/path_planner.py`
- `navegacion_topologica/scripts/topological_navigation.py`
- `navegacion_topologica/scripts/generate_all_maps.py`

**Video: navegacion_topologica.mp4 (2-3 min)**
```
[00:00-00:30] Visualización de los 4 mapas topológicos
[00:30-01:30] Ejecución de path_planner.py con ejemplos
[01:30-02:30] Navegación del robot en RViz/Gazebo
[02:30-03:00] Conclusiones y análisis
```

### Grupal - Parte Semántica
**Archivo: memoria_semantica.pdf (máx 6 hojas)**
```
1. Introducción: "¿Por qué es importante semántica en robots?"
2. Explicación técnica: clasificación de regiones 3D
3. Métodos:
   - Segmentación por altura
   - Segmentación por ocupancia
   - Clustering espacial
4. Clasificación de tipos (corredor, habitación, etc.)
5. Atributos semánticos (traversabilidad, dificultad, riesgo)
6. Ejemplos concretos:
   - Caso 1: Cómo se clasifica un corredor
   - Caso 2: Cómo se clasifica una habitación
7. Integración con navegación topológica:
   - ¿Cómo mejora la navegación?
   - ¿Qué decisiones se pueden tomar?
8. Aplicaciones potenciales:
   - Búsqueda y rescate
   - Limpieza automática
   - Exploración
9. Limitaciones y trabajo futuro
10. Referencias (5+ artículos científicos)
```

**Presentación: (10-15 min en clase)**
- Explicar el método
- Mostrar ejemplos visuales
- Demostración (si es posible con datos reales del robot)
- Análisis de rentabilidad
- Vinculación con trabajo geométrico y topológico

---

## 🐛 Troubleshooting Común

### ERROR: "ModuleNotFoundError: No module named 'cv2'"
```bash
pip install opencv-python
```

### ERROR: "No se encuentran mapas .pgm"
```bash
# Verifica que estos existen:
ls navegacion_geometrica/mapas/sensores/escenario*.pgm
# Si no, ejecuta generación de navegación geométrica primero
```

### ERROR: "El robot no se mueve en RViz"
1. Verifica que move_base esté activo: `rosnode list | grep move_base`
2. Comprueba que el grafo JSON es válido: `python3 -m json.tool mapas/grafos/escenario1_grafo.json`
3. Revisa logs: `rosbag record -a` y luego análisis

### ERROR: "Imagen topológica totalmente blanca/negra"
- Ajusta parámetros en generate_all_maps.py
- Prueba aumentando skeleton_dilation
- Verifica que el mapa .pgm tenga contraste

### ERROR: "Planificador no encuentra camino"
- Verifica que start_node y goal_node existen (< número de nodos)
- Comprueba que el grafo esté bien conectado
- Revisa que no estés usando nodos aislados

---

## 📚 Lecturas Recomendadas

### Esqueletonización
- Blum, H. (1967). A Transformation for Extracting New Descriptors of Shape

### A* Search
- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths

### Segmentación 3D (para parte semántica)
- Rusu, R. B., & Cousins, S. (2011). 3D is here: Point Cloud Library (PCL)

---

## 📞 Contacto y Soporte

**¿Qué hacer si algo no funciona?**
1. Revisa el README.md específico del módulo
2. Ejecuta test_all.py para diagnóstico
3. Revisa los logs detallados en el código
4. Consulta la guía de troubleshooting arriba

**Archivos importantes:**
- `GUIA_PROYECTO.md` - Guía completa
- `navegacion_topologica/README.md` - Documentación técnica
- `navegacion_semantica/README.md` - Teoría semántica
- `navegacion_topologica/scripts/test_all.py` - Diagnóstico

---

**¡Éxito en las entregas!** 🚀
