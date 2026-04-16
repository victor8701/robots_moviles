# Instrucciones: Ejecutar Notebook Jupyter (topoMap.ipynb)

## 🎯 Objetivo

Ejecutar el notebook completado con los TODO rellenados para generar mapas topológicos usando **Watershed segmentation**.

## 📋 Opciones de Ejecución

### Opción 1: Google Colab (RECOMENDADO - Más fácil)

**Ventajas:**
- No requiere instalación local
- Interfaz web intuitiva
- Procesamiento en la nube

**Pasos:**

1. **Subir notebook a Google Drive**
   - Descarga `topoMap.ipynb` de la carpeta del proyecto
   - Abre Google Drive
   - Sube el archivo

2. **Abrir en Colab**
   - Click derecho → "Abrir con" → "Google Colaboratory"

3. **Subir mapas**
   - En la celda de carga de archivos (primera)
   - Sube `escenario1.pgm` y `escenario1.yaml`

4. **Ejecutar todo**
   - Menú → "Ejecutar todas las celdas" (Ctrl+F9)
   - Ver resultados gráficos inline

### Opción 2: Jupyter Local

**Requisitos:**
```bash
pip install jupyter opencv-python numpy scipy scikit-image matplotlib shapely pyyaml
```

**Pasos:**

1. **Navegar a la carpeta del proyecto**
   ```bash
   cd robots_moviles
   ```

2. **Iniciar Jupyter**
   ```bash
   jupyter notebook topoMap.ipynb
   ```

3. **Se abre en navegador (http://localhost:8888)**

4. **Ejecutar celdas:**
   - Click en celda
   - Presionar Shift+Enter
   - O usar el botón "Run All Cells"

5. **Ver resultados**
   - Las gráficas matplotlib aparecen bajo cada celda

## 📊 Qué Esperar en Cada Paso

### Celda 1-2: Cargar mapa
```
✓ Se carga escenario1.pgm
✓ Se procesa en imagen binaria
✓ Se aplica filtro MORPH_OPEN para limpiar ruido
```

**Visualización esperada:** Mapa en blanco y negro

### Celda 3-5: Watershed segmentation
```
✓ Distance Transform: calcula distancia a obstáculo
✓ Threshold: convierte a píxeles de "frente"
✓ Watershed: segmenta el mapa en regiones
✓ Se extraen centroides de cada región
```

**Visualización esperada:** Mapa coloreado con múltiples colores (cada región diferente)

### Celda 6-7: Mapa topológico
```
✓ Se extraen perímetros de cada región (findContours)
✓ Se detectan píxeles que dividen regiones (-1)
✓ Se calculan puntos centrales de separaciones
```

**Visualización esperada:** Grafo con:
- Puntos rojos: centroides de regiones
- Puntos azules: puntos centrales de conexiones
- Líneas azules: conexiones entre nodos

### Celda 8-9: Sub-nodos y traversabilidad
```
✓ Se extrae esqueleto con medial_axis
✓ Se селekcionan puntos importantes (esquinas/bifurcaciones)
✓ Se asignan sub-nodos a regiones usando Polygon.contains()
```

**Visualización esperada:** Mapa con:
- Esqueleto en negro
- Sub-nodos en verde (pequeños)
- Overlay del mapa coloreado

### Celda 10-11: Guardar resultados
```
✓ mapaTopo.txt: [X Y zona1 zona2] - nodos de conexión
✓ subNodos.txt: [X Y zona] - sub-nodos dentro de regiones
```

**Archivos generados:**
```
mapaTopo.txt:
[150 200 2 3]
[215 160 3 4]
...

subNodos.txt:
[155 195 2]
[220 165 3]
...
```

## 🔧 Parámetros Ajustables

### Threshold (Celda 5, línea ~15):
```python
_, frente = cv2.threshold(dist_transform, 0.2 * dist_transform.max(), 255, 0)
```

- **0.2**: Valor por defecto
- **Aumentar (0.3-0.5)**: Menos regiones, menos detalles
- **Disminuir (0.1)**: Más regiones, más detalles

**Ejemplo para más nodos:**
```python
_, frente = cv2.threshold(dist_transform, 0.1 * dist_transform.max(), 255, 0)
```

### Dilatación (Celda 5, línea ~10):
```python
fondo = cv2.dilate(mapa_binario, np.ones((7,7),np.uint8), iterations=3)
```

- **iterations=3**: Por defecto (conecta regiones cercanas)
- **Aumentar**: Más conectividad entre regiones
- **Disminuir**: Menos conectividad

### Erosión (Celda 9, línea ~6):
```python
mapa_erosionado = cv2.erode(mapa_binario, np.ones((5,5), np.uint8), iterations=2)
```

- **iterations=2**: Por defecto (aleja sub-nodos de paredes)
- **Aumentar**: Sub-nodos más alejados de paredes
- **Disminuir**: Sub-nodos más próximos a paredes

## 📸 Capturas Requeridas (Para Entrega Individual)

**En clase, debes capturar 3-5 imágenes:**

1. **Mapa binario procesado** (Celda 2)
   ```
   Mostrar: Espacio blanco (libre) y negro (obstáculos)
   ```

2. **Mapa segmentado** (Celda 5)
   ```
   Mostrar: Múltiples colores, una región por color
   Anotar: Número de regiones encontradas (ej: 8 regiones)
   ```

3. **Mapa topológico completo** (Celda 7)
   ```
   Mostrar: Nodos rojos con centroides, nodos azules de conexión, líneas de conexión
   Anotar: Número de nodos principales y de conexión
   ```

4. **Mapa de traversabilidad** (Celda 9)
   ```
   Mostrar: Sub-nodos en verde, esqueleto en negro
   Anotar: Número de sub-nodos encontrados
   ```

5. **Archivos de salida** (Celda 10)
   ```
   Mostrar: Contenido de mapaTopo.txt y subNodos.txt
   Anotar: Líneas totales de cada archivo
   ```

## ✅ Checklist de Ejecución

```
Preparación:
☐ Descargar o clonar el repositorio
☐ Tener escenario1.pgm y escenario1.yaml

Ejecución:
☐ Opción A: Abrir en Google Colab
  ☐ Subir notebook
  ☐ Subir mapas (pgm + yaml)
  ☐ Ejecutar todo

☐ Opción B: Jupyter local
  ☐ Instalar dependencias
  ☐ Ejecutar jupyter notebook
  ☐ Ejecutar celdas en orden

Captura:
☐ Captura 1: Mapa binario
☐ Captura 2: Mapa segmentado (anotar # regiones)
☐ Captura 3: Mapa topológico (anotar # nodos)
☐ Captura 4: Mapa traversabilidad (anotar # sub-nodos)
☐ Captura 5: Archivos de salida (contenido de .txt)

Opcional - Probar parámetros diferentes:
☐ Cambiar threshold a 0.15 (más nodos)
☐ Cambiar threshold a 0.3 (menos nodos)
☐ Capturar resultados de ambas variantes
```

## 🆘 Troubleshooting

### Error: "FileNotFoundError: escenario1.yaml"
```
Solución: Cambiar ruta en celda 2
De:   archivo_yaml = 'escenario1.yaml'
A:    archivo_yaml = './navegacion_geometrica/mapas/sensores/escenario1.yaml'
```

### Error: "Module 'shapely' has no attribute 'geometry'"
```bash
pip install shapely --upgrade
```

### Error: "The truth value of an array with more than one element is ambiguous"
- Verificar que px es un tuple, no un array
- Usar `px[0]` y `px[1]` en lugar de acceso directo

### Error: "cv2.findContours expected cv2.UMat for argument 'image'"
```python
# Asegurar que mascara es uint8
mascara = (etiquetas == etiqueta).astype(np.uint8)
contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

### Las gráficas no aparecen
```python
# Agregar al inicio del notebook
%matplotlib inline
```

### Matplotlib no muestra correctamente en Colab
- Colab renderiza automáticamente
- Si no aparece, agregar: `plt.show()`

## 🚀 Próximos Pasos Después de Ejecutar

1. **Guardar captura** de resultados
2. **Anotar estadísticas**:
   - Número de regiones
   - Número de nodos principales
   - Número de sub-nodos
   - Threshold usado
   - Iteraciones de dilatación/erosión

3. **Probar con otros escenarios**:
   - Cambiar `archivo_yaml = 'escenario2.yaml'`
   - Volver a ejecutar todo
   - Comparar resultados

4. **Para entrega grupal**:
   - Usar su implementación alternativa para navegar
   - Combinar resultados de ambos enfoques
   - Documentar en memoria

## 📖 Referencia Rápida de Comandos Jupyter

| Acción | Atajo |
|--------|-------|
| Ejecutar celda | Shift+Enter |
| Ejecutar todas | Ctrl+F9 |
| Nueva celda arriba | Esc, A |
| Nueva celda abajo | Esc, B |
| Borrar celda | Esc, D, D |
| Deshacer | Ctrl+Z |
| Renombrar notebook | Ctrl+S |

---

**¡Listo para ejecutar!** 🎓
