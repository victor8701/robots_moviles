"""
Navegacion Topologica para Robots Moviles.

Modulo para:
- Generacion de mapas topologicos desde mapas geometricos
- Planificacion de rutas usando A*
- Navegacion autonoma entre nodos topologicos
- Integracion con informacion semantica 3D
"""

from .topological_mapper import TopologicalMapper
from .path_planner import TopologicalPathPlanner
from .semantic_integration import SemanticTopologicalIntegration

__version__ = "1.0.0"
__author__ = "Robotics Team"

__all__ = [
    'TopologicalMapper',
    'TopologicalPathPlanner',
    'SemanticTopologicalIntegration'
]
