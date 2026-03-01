import cv2
import numpy as np

def calcular_porcentaje(mapa_referencia_path, mapa_autonomo_path):
    # Leer las imágenes en escala de grises
    img_ref = cv2.imread(mapa_referencia_path, cv2.IMREAD_GRAYSCALE)
    img_aut = cv2.imread(mapa_autonomo_path, cv2.IMREAD_GRAYSCALE)

    if img_ref is None or img_aut is None:
        print("Error: No se pudieron cargar las imágenes.")
        return

    # En los mapas de ROS, el espacio libre suele tener un valor de 254
    # Creamos una máscara que solo tenga en cuenta los píxeles blancos (espacio libre)
    _, mascara_ref = cv2.threshold(img_ref, 250, 255, cv2.THRESH_BINARY)
    _, mascara_aut = cv2.threshold(img_aut, 250, 255, cv2.THRESH_BINARY)

    # Contar cuántos píxeles de espacio libre hay en cada mapa
    pixeles_libres_ref = cv2.countNonZero(mascara_ref)
    pixeles_libres_aut = cv2.countNonZero(mascara_aut)

    # Calcular el porcentaje
    porcentaje = (pixeles_libres_aut / pixeles_libres_ref) * 100

    print(f"Píxeles libres (Teleoperado - Referencia): {pixeles_libres_ref}")
    print(f"Píxeles libres (Autónomo): {pixeles_libres_aut}")
    print(f"PORCENTAJE EXPLORADO: {porcentaje:.2f}%")

# Ejemplo de uso (cambia los nombres por los tuyos)
print("Comparación entre mapa teleoperado y mapa autónomo escenario 1:")
calcular_porcentaje('escenario1_keyboard.pgm', 'mapas/exploration/mapa_escenario1.pgm')
print("\n")
print("Comparación entre mapa teleoperado y mapa autónomo escenario 2:")
calcular_porcentaje('escenario2_keyboard.pgm', 'mapas/exploration/mapa_escenario2.pgm')
print("\n")
print("Comparación entre mapa teleoperado y mapa autónomo escenario 3:")
calcular_porcentaje('escenario3_keyboard.pgm', 'mapas/exploration/mapa_escenario3.pgm')
print("\n")
print("Comparación entre mapa teleoperado y mapa autónomo escenario 4:")
calcular_porcentaje('estudio.pgm', 'mapas/exploration/mapa_estudio.pgm')