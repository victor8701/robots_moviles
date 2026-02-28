#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import tf2_ros
import math
import time # Importar el módulo time

# Variables globales
map_data = None
tf_buffer = None
lista_negra = []  # Almacena las coordenadas (x, y) de las fronteras inalcanzables

# --- Temporizador ---
start_time = None # Variable para almacenar el tiempo de inicio

def map_callback(map_msg):
    global map_data
    map_data = map_msg

def select_and_publish_goal():
    global map_data, tf_buffer, lista_negra, start_time
    
    if map_data is None:
        return

    print('Procesando mapa con OpenCV...')
    
    # PASO 1: Extraer metadatos
    width = map_data.info.width
    height = map_data.info.height
    resolution = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    # Localización (TF): Obtener la posición actual del robot
    try:
        trans = tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        robot_x = trans.transform.translation.x
        robot_y = trans.transform.translation.y
    except Exception as e:
        rospy.logwarn("Esperando a la transformada TF de base_footprint...")
        return

    # PASO 2: Tratamiento de Imagen (Visión por Computador)
    mapa = np.array(map_data.data).reshape((height, width))

    # Crear máscaras base
    free_mask = np.uint8(mapa == 0) * 255
    unknown_mask = np.uint8(mapa == -1) * 255
    obstacle_mask = np.uint8(mapa == 100) * 255

    # MEJORA: Engrosar los obstáculos para alejar las fronteras de las paredes
    kernel_obs = np.ones((5,5), np.uint8) # Un kernel de 5x5 equivale a inflar unos 25cm
    obstacle_dilated = cv2.dilate(obstacle_mask, kernel_obs, iterations=2)
    
    # Restar los obstáculos engrosados del espacio libre para obtener una "zona segura"
    safe_free_mask = cv2.bitwise_and(free_mask, cv2.bitwise_not(obstacle_dilated))

    # Detección de Bordes (Fronteras) usando la zona segura
    kernel_free = np.ones((3,3), np.uint8)
    free_dilated = cv2.dilate(safe_free_mask, kernel_free, iterations=1)
    frontiers_mask = cv2.bitwise_and(free_dilated, unknown_mask)

    # PASO 3: Clustering y Selección de Objetivo
    contours, _ = cv2.findContours(frontiers_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # PASO 5: Criterio de Finalización
    if not contours:
        rospy.loginfo("¡No se detectaron más fronteras seguras! Exploración terminada.")
        total_cells = width * height
        free_cells = np.sum(mapa == 0)
        print("Porcentaje de mapa libre descubierto: {:.2f}%".format((free_cells/float(total_cells))*100))
        
        # Calcular y mostrar el tiempo transcurrido
        if start_time is not None:
            end_time = time.time()
            elapsed_time = end_time - start_time
            minutes = int(elapsed_time // 60)
            seconds = int(elapsed_time % 60)
            print("Tiempo total de exploración del mapa: {} minutos y {} segundos.".format(minutes, seconds))
        
        rospy.signal_shutdown("Exploración Completada")
        return

    mejor_distancia = float('inf')
    mejor_objetivo_x = None
    mejor_objetivo_y = None

    for contour in contours:
        # Calcular el Centro de Masas de la frontera
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX_pixel = int(M["m10"] / M["m00"])
            cY_pixel = int(M["m01"] / M["m00"])
        else:
            cX_pixel = contour[0][0][0]
            cY_pixel = contour[0][0][1]

        # Transformación a coordenadas de ROS
        frontera_x_ros = (cX_pixel * resolution) + origin_x
        frontera_y_ros = (cY_pixel * resolution) + origin_y

        # MEJORA: Comprobar la Lista Negra
        es_punto_malo = False
        for bx, by in lista_negra:
            # Si el punto está a menos de 0.5 metros de un punto fallido anterior, lo ignoramos
            if math.sqrt((frontera_x_ros - bx)**2 + (frontera_y_ros - by)**2) < 0.5:
                es_punto_malo = True
                break
        
        if es_punto_malo:
            continue # Saltar a la siguiente frontera

        # Selección: Distancia euclídea desde el robot
        distancia = math.sqrt((frontera_x_ros - robot_x)**2 + (frontera_y_ros - robot_y)**2)

        # Filtro: Ignorar fronteras demasiado cerca del robot (ej. < 0.2m)
        if 0.2 < distancia < mejor_distancia:
            mejor_distancia = distancia
            mejor_objetivo_x = frontera_x_ros
            mejor_objetivo_y = frontera_y_ros

    if mejor_objetivo_x is None:
        rospy.logwarn("Fronteras detectadas pero todas están en la lista negra o son inalcanzables.")
        map_data = None
        return

    print('Frontera seleccionada a {:.2f}m. Enviando objetivo...'.format(mejor_distancia))

    # PASO 4: Navegación y Control (Envío a move_base)
    goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    goal_client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = mejor_objetivo_x
    goal.target_pose.pose.position.y = mejor_objetivo_y
    # Orientación fija hacia adelante (en un proyecto más avanzado se calcularía el ángulo hacia la zona desconocida)
    goal.target_pose.pose.orientation.w = 1.0

    goal_client.send_goal(goal)
    
    # Timeout de 30 segundos
    finished_within_time = goal_client.wait_for_result(rospy.Duration(30.0))
    state = goal_client.get_state()
    
    if not finished_within_time:
        print("Timeout: El robot se atascó o tardó demasiado. Cancelando y añadiendo a la lista negra...")
        goal_client.cancel_goal()
        lista_negra.append((mejor_objetivo_x, mejor_objetivo_y))
    else:
        if state == actionlib.GoalStatus.SUCCEEDED:
            print('¡Frontera alcanzada con éxito! Descubriendo nueva zona...')
        else:
            print('Fallo de move_base (probablemente obstáculo invisible o ruta bloqueada). Añadiendo a la lista negra...')
            lista_negra.append((mejor_objetivo_x, mejor_objetivo_y))

    # Limpiar mapa para procesar uno nuevo en la siguiente iteración
    map_data = None

if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True)
        
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(2.0) # Dar tiempo a que el buffer de transformaciones se llene

        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback)
        rate = rospy.Rate(1) # Evaluar a 1 Hz

        start_time = time.time() # Registrar el tiempo de inicio aquí

        while not rospy.is_shutdown():
            select_and_publish_goal()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo de exploración detenido.")
        # Asegurarse de que el tiempo se registre incluso si se interrumpe manualmente
        if start_time is not None:
            end_time = time.time()
            elapsed_time = end_time - start_time
            minutes = int(elapsed_time // 60)
            seconds = int(elapsed_time % 60)
            rospy.loginfo("Exploración interrumpida. Tiempo total: {} minutos y {} segundos.".format(minutes, seconds))