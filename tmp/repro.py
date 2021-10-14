assert __name__ == "__main__"

from pydrake.geometry import DrakeVisualizer, SceneGraph
from pydrake.systems.framework import DiagramBuilder
DrakeVisualizer.AddToBuilder(DiagramBuilder(), SceneGraph())
