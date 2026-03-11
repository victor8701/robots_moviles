# ComparaciÃ³n de Algoritmos de AsociaciÃ³n de Datos: NN vs SINGLES vs JCBB

**Simulador:** EKF-SLAM â€” Universidad de Zaragoza (Neira & TardÃ³s)  
**Escenario:** Robot recorre un pasillo cuadrado con 88 features (44 steps). Sensor laser + odometrÃ­a con ruido.

---

## Escenario de referencia: Ground Truth

El entorno de simulaciÃ³n es un pasillo cuadrado de 10Ã—10 metros con **88 features** (puntos rojos) distribuidos por las paredes. La trayectoria del robot sigue el perÃ­metro â€” en rojo.

![Ground Truth â€” 88 features reales en el entorno](images/1%20-%20NN%20-%20Figure1.png)

La tarea del algoritmo EKF-SLAM es estimar la posiciÃ³n del robot **y** construir el mapa de features simultÃ¡neamente, a partir Ãºnicamente de las observaciones del sensor y la odometrÃ­a (ambas con ruido).

---

## 1. Nearest Neighbour (NN)

### DescripciÃ³n
Para cada observaciÃ³n `i`, busca el feature del mapa con la **menor distancia de Mahalanobis** y lo empareja si estÃ¡ dentro del gate de compatibilidad individual (umbral chiÂ²). No tiene en cuenta las demÃ¡s observaciones.

- **Complejidad:** O(m Ã— n) â€” muy rÃ¡pido
- **FilosofÃ­a:** *"Emparejo con el mÃ¡s cercano, aunque haya ambigÃ¼edad"*

### Mapa final (Step 44)

![Mapa NN al finalizar â€” 108 features, 20 fantasmas](images/1%20-%20NN%20-%20Figure2.png)

> **ObservaciÃ³n clave:** El mapa final tiene **108 features** cuando la realidad son 88. Los **20 features extra** son "fantasmas" creados por matches incorrectos (false positives) que el EKF integrÃ³ como si fueran features reales. Las elipses de incertidumbre (azul) son grandes y en muchos casos no coinciden con los features reales (rojo).

### MÃ©tricas de asociaciÃ³n a lo largo de la simulaciÃ³n

![GrÃ¡ficas TP/TN/FP/FN de NN a lo largo de los 44 steps](images/1%20-%20NN%20-%20Figure3.png)

> En el grÃ¡fico de **False Positives** se aprecia el problema: aumenta progresivamente, llegando al **65% en el Ãºltimo step**. Cada match incorrecto aÃ±ade un feature mal posicionado al mapa, que compite con los features reales en los siguientes steps, generando mÃ¡s errores en cadena.

### Error de posiciÃ³n del vehÃ­culo

![Error de posiciÃ³n NN â€” azul=error real, rojo=sigma estimado](images/1%20-%20NN%20-%20Figure4.png)

> El error real del vehÃ­culo (lÃ­nea azul) **sale del intervalo de confianza sigma** (lÃ­neas rojas) a partir del step ~35. Esto indica que el EKF ha **perdido la consistencia** â€” sus estimaciones de incertidumbre ya no representan la realidad, debido a los matches incorrectos que han corrompido el estado del filtro.

---

## 2. SINGLES (Only Neighbour)

### DescripciÃ³n
Solo acepta un match cuando es **estrictamente biunÃ­voco**: la observaciÃ³n `i` tiene exactamente un Ãºnico vecino compatible `j`, **y** ese feature `j` tiene a `i` como su Ãºnico vecino compatible.

```matlab
compatible_features = find(compatibility.ic(i, :));    % vecinos de obs i
if length(compatible_features) == 1
    j = compatible_features(1);
    compatible_observations = find(compatibility.ic(:, j));  % vecinos de feat j
    if length(compatible_observations) == 1
        H(i) = j;   % match biunÃ­voco seguro
    end
end
```

- **Complejidad:** O(m Ã— n) â€” igual de rÃ¡pido que NN
- **FilosofÃ­a:** *"Si hay ambigÃ¼edad, no me arriesgo"*

### Mapa final (Step 44)

![Mapa SINGLES al finalizar â€” 88 features, sin fantasmas](images/2%20-%20SINGLES%20-%20Figure2.png)

> El mapa final tiene exactamente **88 features** â€” el nÃºmero correcto. No hay features fantasma. Sin embargo, las elipses de incertidumbre son **mÃ¡s grandes** que con JCBB, porque el robot integrÃ³ menos observaciones (muchos matches fueron descartados por ambigÃ¼edad).

### MÃ©tricas de asociaciÃ³n

![GrÃ¡ficas TP/TN/FP/FN de SINGLES a lo largo de los 44 steps](images/2%20-%20SINGLES%20Figure3.png)

> El grÃ¡fico de **False Positives es completamente negro (0 en todos los steps)**. SINGLES nunca hace un match incorrecto. El coste es visible en **False Negatives**: hay muchos steps donde features correctos no se integran por ser ambiguos.

### Error de posiciÃ³n del vehÃ­culo

![Error de posiciÃ³n SINGLES](images/2%20-%20SINGLES%20-%20Figure4.png)

> El error real (azul) permanece **dentro del intervalo sigma** durante toda la simulaciÃ³n. El EKF es consistente â€” sus estimaciones de incertidumbre son fiables. El filtro no se ha corrompido.

---

## 3. JCBB (Joint Compatibility Branch & Bound)

### DescripciÃ³n
Busca el **conjunto mÃ¡ximo de matches mutuamente compatibles** a nivel global. Usa backtracking (branch & bound) para explorar el espacio de hipÃ³tesis, podando ramas que no pueden superar la mejor soluciÃ³n encontrada hasta el momento.

- **Complejidad:** Exponencial en el peor caso, pero el pruning lo hace muy eficiente en prÃ¡ctica
- **FilosofÃ­a:** *"Busco el conjunto de matches globalmente consistente mÃ¡s grande posible"*

### Mapa final (Step 44)

![Mapa JCBB al finalizar â€” 88 features, incertidumbre mÃ­nima](images/3%20-%20JCBB%20-%20Figure2.png)

> El mapa final tiene exactamente **88 features**, con elipses de incertidumbre mucho mÃ¡s pequeÃ±as que SINGLES. JCBB ha podido integrar mÃ¡s observaciones correctas gracias a que resuelve las ambigÃ¼edades que SINGLES descarta.

### MÃ©tricas de asociaciÃ³n

![GrÃ¡ficas TP/TN/FP/FN de JCBB a lo largo de los 44 steps](images/3%20-%20JCBB%20-%20Figure3.png)

> **False Positives: 0 absoluto** en todos los steps (grÃ¡fico completamente vacÃ­o). **False Negatives: casi 0** â€” solo 2 steps con 1 false negative aislado (steps 9 y 43). True Positives consistentemente altos durante toda la simulaciÃ³n.

### Error de posiciÃ³n del vehÃ­culo

![Error de posiciÃ³n JCBB](images/3%20-%20JCBB%20-%20Figure4.png)

> El error permanece dentro del sigma estimado en toda la simulaciÃ³n, y las bandas de incertidumbre son **mÃ¡s estrechas** que en SINGLES, porque se han integrado mÃ¡s observaciones correctas. El cierre de bucle al final (step ~40) se aprecia como una reducciÃ³n brusca de la incertidumbre â€” comportamiento SLAM clÃ¡sico.

---

## 4. Tabla resumen cuantitativa

| Criterio | NN | SINGLES | JCBB |
|----------|:---:|:-------:|:----:|
| **Features en mapa final** | **108** âŒ | 88 âœ… | 88 âœ… |
| **Features fantasma** | ~20 | 0 | 0 |
| **False Positives totales** | ~30â€“40 | **0** âœ… | **0** âœ… |
| **False Negatives totales** | ~10 | ~80â€“100 | **~3** âœ… |
| **Steps con H == GT** | ~5 | ~2 | **~40** âœ… |
| **EKF consistente** | âŒ Sale del sigma | âœ… | âœ… |
| **Error posiciÃ³n final** | >0.5m en x | <0.1m | <0.1m |
| **Complejidad** | O(mÂ·n) | O(mÂ·n) | Exp. (prÃ¡ctico: ms) |

---

## 5. AnÃ¡lisis de causas

### Por quÃ© NN falla progresivamente

```
Match incorrecto â†’ Feature mal colocado en mapa
                 â†’ Ese feature compite con reales la siguiente vez
                 â†’ MÃ¡s ambigÃ¼edad â†’ MÃ¡s matches incorrectos
                 â†’ EKF corrupto â†’ Robot cree estar donde no estÃ¡
```

En los steps finales (40â€“44) NN tiene **hasta 8 FP por step** y el error de posiciÃ³n supera el sigma estimado â€” el filtro ya no se puede recuperar.

### Por quÃ© SINGLES es seguro pero incompleto

En cuanto una observaciÃ³n tiene 2+ vecinos compatibles (IC â‰¥ 2), SINGLES descarta el match. Esto ocurre frecuentemente en zonas con features prÃ³ximos. El resultado: **0 FP pero muchos FN** â€” el mapa crece lentamente y las elipses de incertidumbre son mÃ¡s grandes de lo necesario.

### Por quÃ© JCBB es el mejor

Cuando hay ambigÃ¼edad individual (lo que SINGLES descarta, lo que NN resuelve mal), JCBB prueba **todas las combinaciones posibles** de matches y selecciona la que maximiza los matches **conjuntamente compatibles**. Una asignaciÃ³n `H(i)=j` solo se acepta si es consistente con **todo el resto** de la hipÃ³tesis.

---

## 6. ConclusiÃ³n

| Algoritmo | Veredicto |
|-----------|-----------|
| **NN** | âŒ No usar en entornos con ambigÃ¼edad. El mapa se degrada irrecuperablemente. |
| **SINGLES** | âš ï¸ Seguro pero conservador. Ãštil como filtro, no como soluciÃ³n definitiva. |
| **JCBB** | âœ… Ã“ptimo. Cero FP, mÃ­nimos FN, mapa de mÃ¡xima calidad. ElecciÃ³n correcta en SLAM. |

---

*Basado en: J. Neira & J.D. TardÃ³s, "Data Association in Stochastic Mapping Using the Joint Compatibility Test", IEEE Transactions on Robotics and Automation, 2001.*

