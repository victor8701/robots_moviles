# Navegación Semántica - Clasificación Basada en Regiones 3D

## Introducción

La **navegación semántica** enriquece los sistemas de navegación tradicionales (geométrica y topológica) con comprensión del significado y las características del entorno. Este trabajo se enfoca en **clasificación semántica basada en regiones 3D**, utilizando información de profundidad para segmentar y clasificar diferentes áreas del espacio.

## Concepto Fundamental

Mientras que:
- **Navegación geométrica**: Considera obstáculos como polígonos/puntos
- **Navegación topológica**: Representa el espacio como un grafo de nodos/conexiones

La **navegación semántica** añade un nivel de abstracción donde el robot entiende el "tipo" de espacio (corredor, habitación, zona abierta, etc.) y puede tomar decisiones basadas en estas categorías.

## Técnica: Clasificación de Regiones 3D

### Descripción

La clasificación de regiones 3D segmenta el entorno observable en regiones coherentes basadas en:

1. **Información de profundidad** (depth maps): Reconstruye geometría 3D
2. **Clustering espacial**: Agrupa puntos cercanos en regiones
3. **Análisis de características**: Calcula dimensiones y propiedades
4. **Clasificación heurística**: Asigna etiquetas semánticas

### Pipeline de Procesamiento

```
Imagen de Profundidad
        ↓
Conversión a Nube de Puntos 3D
        ↓
Segmentación por:
  - Altura
  - Ocupancia
  - Clustering espacial
        ↓
Extracción de Regiones
        ↓
Clasificación (corridor/room/obstacle)
        ↓
Análisis de Transitabilidad
        ↓
Construcción de Grafo Semántico
        ↓
Toma de Decisiones de Navegación
```

### Métodos de Segmentación

#### 1. Segmentación por Altura
- Clasifica puntos según su componente Z
- Típicamente: suelo (0-20cm), obstáculos (20cm-1m), pared (>1m)
- Sencillo pero rápido

#### 2. Segmentación por Ocupancia
- Identifica áreas con alta densidad de puntos (ocupadas) vs. vacías
- Útil para distinguir objetos vs. espacio libre
- Base para otros métodos

#### 3. Clustering Espacial 3D
- Agrupa puntos que están cercanos entre sí
- Métodos: Flood-fill, DBSCAN, K-means
- Genera regiones coherentes

### Clasificación de Regiones

Una vez segmentadas, se clasifican por dimensiones:

| Etiqueta | Ancho | Altura | Uso |
|----------|-------|--------|-----|
| Corredor | 1-3m | >1.5m | Pasillos conectores |
| Habitación | >3m | >1.5m | Espacios amplios (oficinas, salas) |
| Pared | <1m | >1m | Obstáculos verticales |
| Suelo | Variable | <0.5m | Superficies transitables |
| Obstáculo | Variable | Variable | Barreras |

### Atributos Semánticos

Cada región obtiene atributos que guían la navegación:

- **is_traversable**: ¿Es posible pasar por aquí?
- **difficulty**: Nivel de dificultad (easy/medium/hard)
- **connectivity**: Qué otras regiones conecta
- **traversal_cost**: Costo asociado (tiempo, energía, riesgo)

## Integración con Navegación Topológica

### Grafo Semántico-Topológico

Se pueden combinar mapas topológicos (nodos conectados) con información semántica:

```
Node 0 (Corredor)  --1.5m--  Node 3 (Room)
  └─ difficulty: hard         └─ difficulty: easy
  └─ width: 1.2m              └─ width: 4.5m
```

### Mejoras en Path Planning

1. **Preferencia de caminos**: Elegir corredores amplios (fáciles) vs. estrechos
2. **Análisis de riesgo**: Evitar regiones con obstáculos o baja ocupancia
3. **Optimización multicriterio**: Minimizar distancia Y dificultad
4. **Exploración adaptativa**: Priorizar regiones desconocidas

### Ejemplo Práctico

**Escenario**: Habitación con múltiples puertas y corredores

Navegación geométrica: "Evita polígonos de obstáculo"
Navegación topológica: "Pasa por nodos 5→8→12"
Navegación semántica: "Prefer el corredor ancho (1) sobre el estrecho (2)"

## Implementación en el Proyecto

### Archivo: semantic_mapper_3d.py

**Clase SemanticRegion**:
```python
region.label        # "corridor", "room", etc.
region.width        # Ancho en metros
region.height       # Altura en metros
region.volume       # Volumen aproximado
region.centroid     # Centro de la región
region.connectivity # Lista de regiones conectadas
region.attributes   # Dict con propiedades adicionales
```

**Método: segment_by_height()**
- Segmenta por altura relativa
- Ideal para suelo vs. obstáculos

**Método: segment_by_clustering()**
- Agrupa puntos 3D cercanos
- Genera regiones coherentes

**Método: extract_regions()**
- Calcula propiedades de cada región
- Clasifica automáticamente por dimensiones

## Ventajas de esta Aproximación

### Robustez
- No depende solo de forma (como topológico)
- Incorpora información física 3D (profundidad)
- Tolera incertidumbre en sensores

### Escalabilidad
- Se puede aplicar a diferentes sensores (Lidar, DeptCamera, etc.)
- No requiere mapa previo
- Funciona en tiempo real

### Interpretabilidad
- Las etiquetas (corridor, room) son comprensibles
- Los atributos permiten razonamiento de alto nivel
- Facilita aprendizaje y adaptación

### Aplicabilidad
- Búsqueda y rescate ("encontrar sala mayor")
- Limpieza automática ("prioritizar habitaciones amplias")
- Exploración ("mapear nuevas áreas desconocidas")

## Referencias Científicas

1. **Thrun, S., Burgard, W., & Fox, D. (2005).** Probabilistic Robotics. MIT Press.
   - Cap. 10-11: State estimation y mapping
   - Métodos fundamentales para fusion de sensores

2. **Boykov, Y., & Kolmogorov, V. (2004).** "An experimental comparison of min-cut/max-flow algorithms for energy minimization in vision." IEEE TPAMI.
   - Segmentación de imágenes usando graph cuts
   - Base teórica para particionamiento de regiones

3. **Rusu, R. B., & Cousins, S. (2011).** "3D is here: Point Cloud Library (PCL)." ICRA.
   - Procesamiento de nubes de puntos
   - Algoritmos de clustering y segmentación 3D

4. **Felzenszwalb, P. F., & Huttenlocher, D. P. (2004).** "Efficient graph-based image segmentation." IJCV.
   - Segmentación basada en similitud
   - Métodos para agrupar regiones coherentes

5. **He, K., Zhang, X., Ren, S., & Sun, J. (2017).** "Mask R-CNN." ICCV.
   - Deep learning para detección y segmentación de instancias
   - Aproximación moderna complementaria a métodos clásicos

## Aplicación al Trabajo Práctico

### Fase 1: Validación del Método
- [ ] Implementar segmentación básica por altura
- [ ] Probar con imágenes de profundidad reales del TurtleBot3
- [ ] Validar clasificación de regiones

### Fase 2: Integración con Topológica
- [ ] Asignar atributos semánticos a nodos topológicos
- [ ] Modificar A* para considerar dificultad/tipo de región
- [ ] Generar grafos semántico-topológicos

### Fase 3: Demostración
- [ ] Video mostrando clasificación automática de regiones
- [ ] Comparativa: navegación topológica vs. semántica
- [ ] Análisis de mejora en eficiencia y naturalidad

## Conclusiones y Análisis Personal

### Utilidad para el Trabajo Actual

La clasificación 3D de regiones es especialmente útil porque:

1. **Complementa navegación topológica**: Añade razomiento de alto nivel a decisiones basadas en nodos
2. **Usa sensores disponibles**: El TurtleBot3 tiene cámara RGB-D integrada (Kinect-style)
3. **Escalable**: Se puede enriquecer con más características (color, semántica deep-learning, etc.)

### Integración Futura

Para un sistema de navegación completo:

```
Nivel Semántico
    ↓ "Ir a habitación principal"
Nivel Topológico
    ↓ "Pasa por nodos 5→12→18"
Nivel Geométrico
    ↓ "Evita obstáculos, sigue trayectoria"
Actuadores del Robot
```

Este enfoque permite:
- Órdenes en lenguaje natural ("Lleva esto a la sala")
- Razonamiento sobre el entorno ("El corredor es demasiado estrecho")
- Adaptación a cambios ("Esta habitación ahora tiene mobiliario")

### Limitaciones Actuales

- Métodos heurísticos (requieren tuning por entorno)
- Depende calidad sensores profundidad
- No incorpora significado contextual real (ej: "oficina" vs. "almacén")
- Mejora potencial: Usar deep learning para clasificación más robusta

## Próximos Pasos

1. **Enriquecimiento semántico**: Integrar detectores de objetos (YOLO, Mask R-CNN)
2. **Aprendizaje**: Entrenar clasificadores con ejemplos del entorno
3. **Razonamiento**: Desarrollo de sistema experto para decisiones navigate
4. **Colaboración humano-robot**: Solicitar feedback del usuario para refinamiento
