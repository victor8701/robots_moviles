🔷 Navegación geométrica

Es la más “clásica” y precisa.

Representación: el entorno se modela con coordenadas métricas (mapas en 2D o 3D).
Ejemplo: mapas de ocupación (grid maps) donde cada celda indica si hay obstáculo.
Cómo navega: calcula rutas exactas usando distancias y ángulos (algoritmos como A* o RRT).

👉 Un robot con navegación geométrica sabe cosas como:

“Estoy en (x=2.3, y=5.1) y debo ir a (x=10, y=8) evitando obstáculos”.

✔️ Ventajas:

Muy precisa
Ideal para evitar obstáculos en tiempo real

❌ Desventajas:

Computacionalmente costosa
Sensible a errores de localización
🔷 Navegación topológica

Más abstracta y eficiente.

Representación: el entorno se modela como un grafo (nodos y conexiones).
Nodos → lugares (habitaciones, pasillos)
Aristas → conexiones entre ellos
Cómo navega: el robot se mueve de nodo a nodo.

👉 El robot piensa así:

“Estoy en la cocina → voy al pasillo → luego al salón”.

✔️ Ventajas:

Más simple y eficiente
Robusta ante pequeños errores métricos

❌ Desventajas:

Menos precisa (no sabe exactamente dónde está dentro de un nodo)
Puede tener problemas con obstáculos dinámicos
🔷 Navegación semántica

Es la más “inteligente” y cercana al lenguaje humano.

Representación: añade significado a los lugares y objetos.
“cocina”, “mesa”, “puerta”, “zona peligrosa”
Cómo navega: usa ese significado para tomar decisiones.

👉 El robot razona así:

“Ve a la cocina y busca la mesa”
“Evita zonas donde haya personas”

✔️ Ventajas:

Interacción natural con humanos
Permite tareas complejas y contextuales

❌ Desventajas:

Requiere percepción avanzada (visión, IA)
Más compleja de implementar
🧠 Resumen rápido
| Tipo       | Representación       | Nivel | Ejemplo de pensamiento |
| ---------- | -------------------- | ----- | ---------------------- |
| Geométrica | Coordenadas métricas | Bajo  | “Voy a (x,y)”          |
| Topológica | Grafo de lugares     | Medio | “Voy de nodo A a B”    |
| Semántica  | Significado/contexto | Alto  | “Voy a la cocina”      |

🧩 Idea clave
Geométrica = precisión
Topológica = estructura
Semántica = significado

Y en la práctica:

🤖 Los robots modernos combinan las tres (por ejemplo, usan semántica para decidir el objetivo, topología para planificar rutas generales, y geometría para moverse con precisión).