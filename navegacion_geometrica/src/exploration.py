#!/usr/bin/env python

import rospy
import actionlib
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import tf2_ros
import math

# ---------------------------------------------------------------------------
# Variables globales
# ---------------------------------------------------------------------------
map_data = None
tf_buffer = None
lista_negra = []  # Almacena las coordenadas (x, y) de las fronteras inalcanzables

# --- Detección reactiva de obstáculos ---
obstacle_detected = False          # Flag compartido entre hilos
goal_client_global = None          # Referencia al cliente de move_base activo

OBSTACLE_THRESHOLD = 0.35          # Distancia mínima frontal (metros)
ESCAPE_LINEAR  = -0.10             # Velocidad lineal de escape (retroceso)
ESCAPE_ANGULAR =  0.50             # Velocidad angular de escape (giro)

cmd_vel_pub = None                 # Publicador /cmd_vel (inicializado en main)


# ---------------------------------------------------------------------------
# Maniobra de escape — se ejecuta en un hilo separado para no bloquear /scan
# ---------------------------------------------------------------------------
def execute_escape():
    """Retrocede y gira el robot para alejarse del obstáculo detectado.
    Se llama desde scan_callback en un hilo daemon."""
    global obstacle_detected

    rate = rospy.Rate(10)
    twist = Twist()

    # Fase 1: retroceder 0.8 s
    twist.linear.x = ESCAPE_LINEAR
    twist.angular.z = 0.0
    t_end = rospy.Time.now() + rospy.Duration(0.8)
    while rospy.Time.now() < t_end and not rospy.is_shutdown():
        cmd_vel_pub.publish(twist)
        rate.sleep()

    # Fase 2: girar en el sitio 0.7 s
    twist.linear.x = 0.0
    twist.angular.z = ESCAPE_ANGULAR
    t_end = rospy.Time.now() + rospy.Duration(0.7)
    while rospy.Time.now() < t_end and not rospy.is_shutdown():
        cmd_vel_pub.publish(twist)
        rate.sleep()

    # Parar y liberar el flag para que el bucle principal retome la exploración
    cmd_vel_pub.publish(Twist())
    obstacle_detected = False
    rospy.loginfo("Maniobra de escape completada. Reanudando exploración...")


# ---------------------------------------------------------------------------
# Callback del LiDAR — actúa como interrupción reactiva
# ---------------------------------------------------------------------------
def scan_callback(scan_msg):
    """Monitoriza /scan continuamente. Si detecta un obstáculo demasiado
    cerca en el sector frontal, cancela el goal de move_base y escapa."""
    global obstacle_detected, goal_client_global

    if obstacle_detected:
        return  # Ya estamos gestionando un obstáculo

    ranges = np.array(scan_msg.ranges)

    # Sector frontal: ±30° → índices 0..29 y 330..359
    front = np.concatenate([ranges[0:30], ranges[330:]])
    front = front[np.isfinite(front)]  # Filtrar inf/nan

    if len(front) > 0 and np.min(front) < OBSTACLE_THRESHOLD:
        rospy.logwarn(
            "¡OBSTÁCULO a {:.2f}m! Cancelando goal y ejecutando escape...".format(np.min(front))
        )
        obstacle_detected = True

        # Cancelar goal activo → move_base deja de publicar /cmd_vel
        if goal_client_global is not None:
            goal_client_global.cancel_goal()

        # Lanzar la maniobra en un hilo separado para no bloquear el callback
        threading.Thread(target=execute_escape, daemon=True).start()


# ---------------------------------------------------------------------------
# Callback del mapa
# ---------------------------------------------------------------------------
def map_callback(map_msg):
    global map_data
    map_data = map_msg


# ---------------------------------------------------------------------------
# Selección y envío de objetivo de exploración
# ---------------------------------------------------------------------------
def select_and_publish_goal():
    global map_data, tf_buffer, lista_negra, goal_client_global

    # No procesar mientras se resuelve un obstáculo
    if obstacle_detected:
        rospy.loginfo_throttle(2.0, "Esperando a que se resuelva el obstáculo...")
        return

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

    # Exponer cliente al callback de /scan para que pueda cancelarlo
    goal_client_global = goal_client

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
        elif state == actionlib.GoalStatus.PREEMPTED:
            # Cancelado por scan_callback (obstáculo detectado) — no añadir a lista negra
            rospy.loginfo("Goal cancelado por detección de obstáculo. Esperando maniobra de escape...")
        else:
            print('Fallo de move_base (probablemente obstáculo invisible o ruta bloqueada). Añadiendo a la lista negra...')
            lista_negra.append((mejor_objetivo_x, mejor_objetivo_y))

    # Limpiar mapa para procesar uno nuevo en la siguiente iteración
    goal_client_global = None
    map_data = None

if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True)
        
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(2.0) # Dar tiempo a que el buffer de transformaciones se llene

        # Publicador de velocidad para la maniobra de escape
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Suscriptores
        map_subscriber  = rospy.Subscriber('/map',  OccupancyGrid, map_callback)
        scan_subscriber = rospy.Subscriber('/scan', LaserScan,     scan_callback)

        rate = rospy.Rate(1) # Evaluar a 1 Hz

        while not rospy.is_shutdown():
            select_and_publish_goal()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo de exploración detenido.")