use super::*;

#[pyclass(frozen)]
#[derive(Clone)]
pub(crate) struct Mesh {
    mesh: Arc<TriMesh>,
    pose: ParryPose,
}

#[pymethods]
impl Mesh {
    #[new]
    #[pyo3(signature = (path, scale=1.0, pose=None))]
    fn new(path: &str, scale: f64, pose: Option<[[f64; 4]; 4]>) -> PyResult<Self> {
        Self::from_path_with_pose(path, scale, pose)
    }

    #[staticmethod]
    #[pyo3(signature = (path, scale=1.0))]
    fn from_path(path: &str, scale: f64) -> PyResult<Self> {
        Self::from_path_with_pose(path, scale, None)
    }

    #[staticmethod]
    #[pyo3(signature = (vertices, triangles, pose=None))]
    fn from_arrays(
        vertices: Vec<(f64, f64, f64)>,
        triangles: Vec<(u32, u32, u32)>,
        pose: Option<[[f64; 4]; 4]>,
    ) -> PyResult<Self> {
        Self::from_vertices_and_triangles(vertices, triangles, pose)
    }

    #[getter]
    fn vertex_count(&self) -> usize {
        self.mesh.vertices().len()
    }

    #[getter]
    fn triangle_count(&self) -> usize {
        self.mesh.indices().len()
    }

    #[pyo3(signature = (pose))]
    fn transformed(&self, pose: [[f64; 4]; 4]) -> PyResult<Self> {
        Ok(Self {
            mesh: Arc::clone(&self.mesh),
            pose: matrix_to_parry_pose(pose)? * self.pose,
        })
    }

    #[pyo3(signature = (pose))]
    fn transformed_by(&self, pose: [[f64; 4]; 4]) -> PyResult<Self> {
        self.transformed(pose)
    }

    #[pyo3(signature = (other, safety_distance=0.0))]
    fn collides(&self, other: &Mesh, safety_distance: f64) -> PyResult<bool> {
        let safety_distance = validate_nonnegative_f32(safety_distance, "safety_distance")?;
        let intersects = parry3d::query::intersection_test(
            &self.pose,
            self.mesh.as_ref(),
            &other.pose,
            other.mesh.as_ref(),
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        if intersects || safety_distance == 0.0 {
            return Ok(intersects);
        }

        let distance = parry3d::query::distance(
            &self.pose,
            self.mesh.as_ref(),
            &other.pose,
            other.mesh.as_ref(),
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        Ok(distance <= safety_distance)
    }

    fn __repr__(&self) -> String {
        format!(
            "Mesh(vertex_count={}, triangle_count={})",
            self.vertex_count(),
            self.triangle_count()
        )
    }
}

impl Mesh {
    fn from_path_with_pose(path: &str, scale: f64, pose: Option<[[f64; 4]; 4]>) -> PyResult<Self> {
        let scale = validate_positive_f32(scale, "scale")?;
        let mesh = load_trimesh(path, scale).map_err(PyValueError::new_err)?;

        Ok(Self {
            mesh: Arc::new(mesh),
            pose: transpose_option(pose, matrix_to_parry_pose)?.unwrap_or_else(ParryPose::identity),
        })
    }

    fn from_vertices_and_triangles(
        vertices: Vec<(f64, f64, f64)>,
        triangles: Vec<(u32, u32, u32)>,
        pose: Option<[[f64; 4]; 4]>,
    ) -> PyResult<Self> {
        let vertices = vertices
            .into_iter()
            .map(|(x, y, z)| {
                Ok(Vec3::new(
                    validate_f32(x, "vertices")?,
                    validate_f32(y, "vertices")?,
                    validate_f32(z, "vertices")?,
                ))
            })
            .collect::<PyResult<Vec<_>>>()?;
        let triangles = triangles
            .into_iter()
            .map(|(a, b, c)| [a, b, c])
            .collect::<Vec<_>>();
        let mesh = TriMesh::new(vertices, triangles)
            .map_err(|err| PyValueError::new_err(err.to_string()))?;

        Ok(Self {
            mesh: Arc::new(mesh),
            pose: transpose_option(pose, matrix_to_parry_pose)?.unwrap_or_else(ParryPose::identity),
        })
    }

    pub(crate) fn clone_trimesh(&self) -> TriMesh {
        self.mesh.as_ref().clone()
    }

    pub(crate) fn local_trimesh(&self) -> TriMesh {
        transform_mesh(self.mesh.as_ref(), &self.pose32())
    }

    pub(crate) fn pose32(&self) -> Pose32 {
        Pose32::from_parts(self.pose.translation, self.pose.rotation)
    }
}
