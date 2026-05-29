from spherical_wrist import Mesh
from scipy.spatial.transform import RigidTransform, Rotation


def test_mesh_from_arrays_collides_with_safety_distances() -> None:
    mesh_a = Mesh.from_arrays(_cube_vertices(), _cube_triangles())
    mesh_b = Mesh.from_arrays(_cube_vertices(), _cube_triangles())

    overlapping = mesh_b.transformed(_translation(0.25, 0.0, 0.0))
    separated = mesh_b.transformed(_translation(1.5, 0.0, 0.0))

    assert mesh_a.vertex_count == 8
    assert mesh_a.triangle_count == 12
    assert mesh_a.collides(overlapping)
    assert not mesh_a.collides(separated)
    assert not mesh_a.collides(separated, safety_distance=0.49)
    assert mesh_a.collides(separated, safety_distance=0.51)
    assert mesh_a.collides(separated, safety_distance=1.0)


def _translation(x: float, y: float, z: float) -> RigidTransform:
    return RigidTransform.from_components(
        rotation=Rotation.identity(),
        translation=[x, y, z],
    )


def _cube_vertices() -> list[tuple[float, float, float]]:
    return [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 1.0),
        (1.0, 1.0, 1.0),
        (0.0, 1.0, 1.0),
    ]


def _cube_triangles() -> list[tuple[int, int, int]]:
    return [
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ]
