# Navegación Geométrica — Exploración Autónoma por Fronteras

Módulo 1 del proyecto. Implementa la exploración autónoma de entornos desconocidos con un TurtleBot3 en Gazebo mediante la estrategia **Frontier-Based Exploration**, y evalúa la calidad del mapa resultante contra un ground truth teleoperado.

---

## Algoritmo: Exploración por Fronteras

El robot detecta continuamente el límite entre espacio libre (conocido) y espacio desconocido. Ese límite se llama **frontera**. El robot navega hacia la frontera más cercana y repite el proceso hasta que no quedan fronteras accesibles.

### Pasos del algoritmo (`exploration.py`)

1. **Máscaras del mapa**: separa espacio libre, obstáculos y desconocido desde el `OccupancyGrid` de ROS.
2. **Dilatación de obstáculos**: engrosa las paredes con `cv2.dilate` para mantener margen de seguridad.
3. **Detección de fronteras**: intersección entre el espacio libre dilatado y el espacio desconocido.
4. **Selección de objetivo**: extrae contornos con `cv2.findContours`, calcula centroides y elige el más cercano al robot (vía TF `map→base_footprint`).
5. **Lista negra**: si `move_base` falla o supera 30 s, la frontera se descarta y se busca la siguiente.
6. **Parada automática**: cuando no quedan fronteras, el mapa se considera completo.

### Evaluación (`comparacion.py`)

Compara el mapa explorado autónomamente con el ground truth (teleoperado al 100%):

```
% descubierto = (píxeles libres autónomos / píxeles libres ground truth) × 100
```

---

## Resultados experimentales

| Escenario | Tiempo | Cobertura | Notas |
|-----------|--------|-----------|-------|
| Escenario 1 | 3 min 09 s | 100.11 % | Supera ligeramente al GT por bordes |
| Escenario 2 | 7 min 13 s | 99.76 % | Mapeo casi perfecto |
| Escenario 3 | 7 min 50 s | 91.57 % | Pasillos estrechos dificultan la cobertura |
| Estudio     | 7 min 02 s | 99.85 % | Excelente navegación inter-habitaciones |

*(Resultados adicionales con script de sensores: E1 99.93 % / E2 99.35 % / E3 99.35 % / Est. 100.03 %)*

---

## Instalación

```bash
pip install pyyaml opencv-python scikit-image numpy scipy
```

---

## Ejecutar la simulación

### Terminal 1 — Gazebo

```bash
source /opt/ros/noetic/setup.bash
source ~/Ubuntu20noetic_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger

# Cambiar escenario1/2/3/estudio según corresponda
roslaunch navegacion_geometrica turtlebot3_escenario2.launch
```

### Terminal 2 — SLAM (gmapping)

```bash
source /opt/ros/noetic/setup.bash
source ~/Ubuntu20noetic_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

### Terminal 3 — Exploración autónoma

```bash
source /opt/ros/noetic/setup.bash
source ~/Ubuntu20noetic_ws/devel/setup.bash
rosrun navegacion_geometrica exploration.py
```

### Guardar el mapa al terminar

```bash
rosrun map_server map_saver -f ~/Ubuntu20noetic_ws/src/robots_moviles/navegacion_geometrica/mapas/exploration/mapa_escenario2
```

### Evaluar cobertura

```bash
cd ~/Ubuntu20noetic_ws/src/robots_moviles/navegacion_geometrica/scripts
python3 comparacion.py \
  ../mapas/exploration/mapa_escenario2.pgm \
  ../mapas/teleoperacion/escenario2_keyboard.pgm
```

---

## Estructura de archivos

```
navegacion_geometrica/
├── src/
│   └── exploration.py          # Nodo ROS de exploración autónoma
├── scripts/
│   └── comparacion.py          # Evaluación de cobertura vs. ground truth
├── launch/
│   ├── turtlebot3_escenario1.launch
│   ├── turtlebot3_escenario2.launch
│   ├── turtlebot3_escenario3.launch
│   └── turtlebot3_estudio.launch
├── mapas/
│   ├── exploration/            # Mapas generados autónomamente (PGM + YAML)
│   ├── teleoperacion/          # Ground truth teleoperado
│   └── sensores/               # Mapas de cobertura de sensores
└── worlds/                     # Mundos Gazebo (.world)
```

---

## Guía para el vídeo

1. **Mostrar Gazebo y RViz** en pantalla dividida (el mapa se va construyendo en tiempo real).
2. **Narrar** que el robot identifica las fronteras (borde azul en RViz si se configura el costmap) y navega autónomamente hacia ellas.
3. **Mostrar la terminal** cuando el robot detecta que no quedan fronteras y para.
4. **Ejecutar `comparacion.py`** en pantalla y leer el porcentaje de cobertura.
5. Repetir para al menos 2 escenarios (se recomienda escenario2 y escenario3).

---

## Puntos clave para la memoria

- El mapa geométrico (PGM) es la entrada para los módulos siguientes (topológico y semántico).
- La exploración por fronteras es un método clásico (Yamauchi 1997) que garantiza cobertura completa siempre que el entorno sea convexo o tenga pasillos accesibles.
- La lista negra y el timeout de 30 s son mecanismos de robustez que evitan que el robot quede bloqueado en pasillos muy estrechos (escenario 3 baja al 91.57 % precisamente por eso).
