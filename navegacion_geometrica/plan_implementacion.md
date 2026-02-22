# Proyecto de Exploración Autónoma: Guía Maestra

Este documento contiene la hoja de ruta definitiva para el desarrollo del sistema de exploración. Sigue estos pasos en orden estricto para completar el proyecto con éxito.

## Introducción: La Estrategia de Exploración por Fronteras (Frontier Exploration)

La estrategia elegida se basa en el concepto de **frontera**: la línea divisoria entre el espacio que el robot ya ha confirmado como **libre** (blanco en RViz) y el espacio que aún es **desconocido** (gris en RViz).

### ¿Cómo funciona el algoritmo?
1.  **Visión por Computador (VC)**: El robot trata el mapa no como un conjunto de datos, sino como una imagen binaria.
2.  **Identificación**: El algoritmo busca píxeles que sean "candidatos": deben ser celdas libres que tengan al menos un vecino desconocido.
3.  **Movimiento Inteligente**: El robot selecciona la frontera más prometedora (normalmente la más cercana), calcula una pose de navegación y se la envía a `move_base`.
4.  **Bucle Infinito hasta la completitud**: El proceso se repite. Al llegar a una frontera, el LIDAR descubre una zona nueva, el mapa se actualiza, la frontera anterior desaparece y aparecen otras nuevas más lejanas. La exploración termina cuando ya no quedan fronteras (el entorno está cerrado o totalmente mapeado).

---

## 1. Herramientas Necesarias
*   **ROS Noetic / Python 3**: Base del middleware.
*   **NumPy**: Para manipulación eficiente de las matrices del mapa.
*   **OpenCV (cv2)**: Herramienta de **Visión por Computador** esencial para detectar contornos y procesar el mapa como una imagen.
*   **ActionLib**: Para gestionar el envío de objetivos a `move_base` y saber si el robot ha llegado.

---

## 2. Plan de Acción Pasos a Seguir

### Paso 1: Configuración de la Percepción (Escuchar al sistema)
*   **Suscripción al Mapa**: Suscribirse a `/map` (`nav_msgs/OccupancyGrid`). Debes extraer el `width`, `height`, `resolution` y el `origin`.
    *   *Nota*: Extraer estos metadatos es necesario para que OpenCV pueda procesar el mapa como una matriz. Aunque el entorno sea desconocido, Gmapping expande estas dimensiones dinámicamente; leerlas no significa conocer el mapa final, sino el tamaño de la "hoja de dibujo" actual.
*   **Localización (TF)**: Implementar un `tf2_ros.TransformListener`. Necesitas saber dónde está el robot (`base_link`) respecto al mapa (`map`) en todo momento para calcular distancias a las fronteras.

### Paso 2: Tratamiento de Imagen con Visión por Computador (VC)
*   **Matriz de Imagen**: Convertir los datos de `/map` en una matriz de NumPy.
*   **Segmentación**: Usar OpenCV para crear máscaras:
    *   Zona Libre = 255 (Blanco)
    *   Zona Ocupada / Desconocida = 0 (Negro)
*   **Detección de Bordes**: Aplicar filtros (como Canny o dilatación/erosión) para resaltar solo los bordes donde el blanco toca al gris.

### Paso 3: Clustering y Selección de Objetivo
*   **findContours**: Usar esta función de OpenCV sobre la imagen de bordes para agrupar píxeles de frontera en "objetos" reales.
*   **Centro de Masas**: Para cada contorno (frontera), calcular su centro. Ese será nuestro punto candidato.
*   **Selección**: Calcular la distancia euclídea desde la posición actual del robot (Paso 1) hasta cada centro. Elegir el más cercano que sea alcanzable.

### Paso 4: Navegación y Control
*   **Transformación**: Convertir el centro de la frontera (píxel `col, row`) a coordenadas de ROS (metros `x, y`) usando la resolución y el origen del mapa.
*   **Envío**: Mandar un `MoveBaseGoal`.
*   **Timeout y Recuperación**: Si `move_base` no puede llegar en X segundos, cancelar el objetivo y marcar esa zona como "prohibida" temporalmente para no entrar en un bucle infinito.

### Paso 5: Criterio de Finalización (Parada)
*   Implementar una comprobación: Si tras procesar el mapa con OpenCV el número de contornos (fronteras) detectados es **cero**, el robot ha terminado.
*   **Resultados Finales**: Al detectar el fin, guardar el tiempo total de ejecución y mostrar el % de celdas libres (valor 0) registradas.

---

## 3. Lógica Matemática de Conversión
Para traducir entre el mundo de la Visión (OpenCV) y el mundo de ROS:
*   `pose_x = (pixel_x * resolución) + origen_x`
*   `pose_y = (pixel_y * resolución) + origen_y`
*   *Nota: Las coordenadas de imagen y las de ROS suelen estar invertidas en el eje Y (filas vs columnas).*

---

## 4. Estimación de Tiempos y Evaluación
*   Implementación completa: **15-20 horas**.
*   Obligatorio probar en los **4 escenarios**. Un algoritmo robusto debe ser capaz de salir de habitaciones cerradas y explorar pasillos largos sin intervención humana.
