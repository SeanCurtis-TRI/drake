assert __name__ == "__main__"

from pydrake.geometry import DrakeVisualizer, SceneGraph
from pydrake.systems.framework import DiagramBuilder
builder = DiagramBuilder()
scene_graph = builder.AddSystem(SceneGraph())
DrakeVisualizer.AddToBuilder(builder, scene_graph)
