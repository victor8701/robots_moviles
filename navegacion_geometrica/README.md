# Proyecto de Exploración Autónoma y Navegación Geométrica

Este paquete de ROS (Noetic) implementa un sistema completo de mapeo y exploración autónoma para un robot móvil (Turtlebot3) en entornos simulados con Gazebo. El objetivo principal es generar mapas precisos de espacios desconocidos sin intervención humana y evaluar el rendimiento del algoritmo.

## 1. Algoritmo de Exploración por Fronteras (`exploration.py`)

El núcleo de la autonomía reside en el script `exploration.py`, el cual utiliza una estrategia de **Exploración Basada en Fronteras (Frontier-Based Exploration)**.

### Lógica de Procesamiento de Imagen (OpenCV)
En lugar de tratar el mapa (`OccupancyGrid` de ROS) como un arreglo de datos simple, el algoritmo lo transforma en una matriz bidimensional (imagen) y aplica visión artificial para la toma de decisiones:
1. **Generación de Máscaras:** Se separan los píxeles en tres categorías: espacio libre (0), obstáculos (100) y espacio desconocido (-1).
2. **Dilatación de Obstáculos (Margen de Seguridad):** Mediante `cv2.dilate`, se engrosan las paredes virtuales para garantizar que el robot no seleccione objetivos demasiado cercanos a colisiones.
3. **Detección de Fronteras:** Se dilata la máscara de espacio libre y se busca su intersección (`cv2.bitwise_and`) con la máscara de zonas desconocidas. Las líneas resultantes representan el límite exacto entre lo mapeado y lo oculto.
4. **Clustering y Selección:** Se extraen los contornos (`cv2.findContours`) y se calculan sus centros de masa. Utilizando el árbol de transformadas (`tf2_ros`) desde `map` hasta `base_footprint`, se selecciona la frontera segura más cercana mediante distancia euclídea.

### Robustez y Criterios de Parada
- **Lista Negra:** Si el sistema de navegación (`move_base`) falla al intentar llegar a una frontera o agota el tiempo de espera (timeout de 30s), esa coordenada se almacena en una lista negra y el algoritmo busca la siguiente mejor opción, evitando bucles infinitos en espacios muy estrechos.
- **Fin de Exploración:** El mapeo concluye de forma automática cuando el algoritmo de visión ya no detecta fronteras seguras o cuando todas las fronteras restantes pertenecen a la lista negra (asumiendo el espacio como mapeado al máximo posible).

## 2. Evaluación de Rendimiento (`comparacion.py`)

Para medir la eficacia real de la exploración autónoma, se ha implementado un script de validación mediante visión por computador. 

Se compara el mapa generado autónomamente contra un "Ground Truth" (un mapa de referencia mapeado manualmente al 100% mediante teleoperación). El script:
1. Lee ambos mapas (`.pgm`) en escala de grises.
2. Aplica una umbralización binaria (`cv2.threshold`) para aislar exclusivamente los píxeles de espacio libre (valor ~254).
3. Calcula el porcentaje real descubierto mediante la relación: `(Píxeles Libres Autónomos / Píxeles Libres Referencia) * 100`.

## 3. Resultados Experimentales

El sistema fue evaluado en 4 escenarios de distinta complejidad. Los resultados demuestran una cobertura casi total en tiempos altamente competitivos:

| Escenario | Tiempo de Exploración | Porcentaje Descubierto | Notas |
| :--- | :--- | :--- | :--- |
| **Escenario 1** | 3 min 09 seg | 100.11% | *Supera al Ground Truth ligeramente por optimización en bordes.* |
| **Escenario 2** | 7 min 13 seg | 99.76% | *Mapeo casi perfecto.* |
| **Escenario 3** | 7 min 50 seg | 91.57% | *Alta complejidad de pasillos.* |
| **Estudio** | 7 min 02 seg | 99.85% | *Excelente navegación inter-habitaciones.* |
