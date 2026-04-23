import cv2
import numpy as np
from datetime import datetime

# Abrir archivo para guardar resultados
output_file = open('resultados_comparacion.md', 'w', encoding='utf-8')

def escribir(texto):
    """Escribe en terminal y en archivo"""
    print(texto)
    output_file.write(texto + '\n')

def calcular_porcentaje(mapa_referencia_path, mapa_autonomo_path):
    # Leer las imágenes en escala de grises
    img_ref = cv2.imread(mapa_referencia_path, cv2.IMREAD_GRAYSCALE)
    img_aut = cv2.imread(mapa_autonomo_path, cv2.IMREAD_GRAYSCALE)

    if img_ref is None or img_aut is None:
        escribir("Error: No se pudieron cargar las imágenes.")
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

    escribir(f"Píxeles libres (Teleoperado - Referencia): {pixeles_libres_ref}")
    escribir(f"Píxeles libres (Autónomo): {pixeles_libres_aut}")
    escribir(f"PORCENTAJE EXPLORADO: {porcentaje:.2f}%")

# Agregar cabecera al archivo
escribir("# Resultados de Comparación de Mapas\n")
escribir(f"Generado: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
escribir("---\n")

# Ejemplo de uso (cambia los nombres por los tuyos)
escribir("## Comparación entre mapa teleoperado y mapa autónomo escenario 1:")
calcular_porcentaje('mapas/teleoperacion/escenario1_keyboard.pgm', 'mapas/exploration/mapa_escenario1.pgm')
escribir("")
escribir("## Comparación entre mapa teleoperado y mapa autónomo escenario 2:")
calcular_porcentaje('mapas/teleoperacion/escenario2_keyboard.pgm', 'mapas/exploration/mapa_escenario2.pgm')
escribir("")
escribir("## Comparación entre mapa teleoperado y mapa autónomo escenario 3:")
calcular_porcentaje('mapas/teleoperacion/escenario3_keyboard.pgm', 'mapas/exploration/mapa_escenario3.pgm')
escribir("")
escribir("## Comparación entre mapa teleoperado y mapa autónomo escenario 4:")
calcular_porcentaje('mapas/teleoperacion/estudio_keyboard.pgm', 'mapas/exploration/mapa_estudio.pgm')

escribir("\n---\n")
escribir("## Comparación entre mapa teleoperado y mapa autónomo (con sensores para evitar colisiones) escenario 1:")
calcular_porcentaje('mapas/teleoperacion/escenario1_keyboard.pgm', 'mapas/sensores/escenario1_sensores.pgm')
escribir("")
escribir("## Comparación entre mapa teleoperado y mapa autónomo (con sensores para evitar colisiones) escenario 2:")
calcular_porcentaje('mapas/teleoperacion/escenario2_keyboard.pgm', 'mapas/sensores/escenario2_sensores.pgm')
escribir("")
escribir("## Comparación entre mapa teleoperado y mapa autónomo (con sensores para evitar colisiones) escenario 3:")
calcular_porcentaje('mapas/teleoperacion/escenario3_keyboard.pgm', 'mapas/sensores/escenario3_sensores.pgm')
escribir("")
escribir("## Comparación entre mapa teleoperado y mapa autónomo (con sensores para evitar colisiones) escenario 4:")
calcular_porcentaje('mapas/teleoperacion/estudio_keyboard.pgm', 'mapas/sensores/estudio_sensores.pgm')

# Cerrar archivo
output_file.close()
print("\n✓ Resultados guardados en 'resultados_comparacion.md'")