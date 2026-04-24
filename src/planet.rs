use bevy::prelude::*;
use bevy::pbr::{Material, MaterialPipeline, MaterialPipelineKey};
use bevy::render::render_resource::{AsBindGroup, ShaderRef, RenderPipelineDescriptor};
use bevy::render::mesh::{VertexAttributeValues, Indices, MeshVertexBufferLayoutRef};
use bevy::render::render_resource::SpecializedMeshPipelineError;

use crate::constants::*;

// ===== Noise Settings =====

#[derive(Clone, Copy)]
pub struct NoiseSettings {
    pub octaves: u32,
    pub frequency: f32,
    pub persistence: f32,
    pub lacunarity: f32,
    pub exponent: f32, // For ridged noise sharpness
    pub strength: f32,
    pub offset: Vec3,
}

impl Default for NoiseSettings {
    fn default() -> Self {
        Self {
            octaves: 5,
            frequency: 1.0,
            persistence: 0.5,
            lacunarity: 2.0,
            exponent: 1.0,
            strength: 1.0,
            offset: Vec3::ZERO,
        }
    }
}

// ===== Noise Functions =====

/// Simple hash-based value noise on 3D integer grid.
pub fn hash3(x: i32, y: i32, z: i32) -> f32 {
    let mut h = (x as u32)
        .wrapping_mul(374761393)
        .wrapping_add((y as u32).wrapping_mul(668265263))
        .wrapping_add((z as u32).wrapping_mul(1274126177));
    h = (h ^ (h >> 13)).wrapping_mul(1274126177);
    h = (h ^ (h >> 16)).wrapping_mul(668265263);
    h as f32 / u32::MAX as f32
}

pub fn vnoise(p: Vec3) -> f32 {
    let ix = p.x.floor() as i32;
    let iy = p.y.floor() as i32;
    let iz = p.z.floor() as i32;
    let fx = p.x - ix as f32;
    let fy = p.y - iy as f32;
    let fz = p.z - iz as f32;
    // Smoothstep
    let sx = fx * fx * (3.0 - 2.0 * fx);
    let sy = fy * fy * (3.0 - 2.0 * fy);
    let sz = fz * fz * (3.0 - 2.0 * fz);

    let mut acc = 0.0;
    for dz in 0..=1i32 {
        for dy in 0..=1i32 {
            for dx in 0..=1i32 {
                let v = hash3(ix + dx, iy + dy, iz + dz);
                let wx = if dx == 0 { 1.0 - sx } else { sx };
                let wy = if dy == 0 { 1.0 - sy } else { sy };
                let wz = if dz == 0 { 1.0 - sz } else { sz };
                acc += v * wx * wy * wz;
            }
        }
    }
    // 將 0.0 ~ 1.0 映射到 -1.0 ~ 1.0，這樣才有「零點」
    acc * 2.0 - 1.0
}

pub fn fbm(p: Vec3, settings: &NoiseSettings) -> f32 {
    let mut val = 0.0;
    let mut amp = 1.0;
    let mut freq = settings.frequency;
    let mut pos = p + settings.offset;
    for _ in 0..settings.octaves {
        // vnoise 現在是 -1 ~ 1
        val += amp * vnoise(pos * freq);
        freq *= settings.lacunarity;
        amp *= settings.persistence;
        pos = Vec3::new(pos.z * 1.1, pos.x * 0.9, pos.y * 1.05);
    }
    val * settings.strength
}

pub fn ridged_fbm(p: Vec3, settings: &NoiseSettings) -> f32 {
    let mut val = 0.0;
    let mut amp = 1.0;
    let mut freq = settings.frequency;
    let mut pos = p + settings.offset;
    let mut weight = 1.0;

    for _ in 0..settings.octaves {
        // 1.0 - abs(-1~1) 產生的結果是 0~1，且波峰在 1 處尖銳，凹陷處是 0
        let mut n = 1.0 - vnoise(pos * freq).abs();
        n = n.powf(settings.exponent);
        n *= weight;
        weight = n.clamp(0.0, 1.0);

        val += n * amp;
        freq *= settings.lacunarity;
        amp *= settings.persistence;
        pos = Vec3::new(pos.z * 1.1, pos.x * 0.9, pos.y * 1.05);
    }
    val * settings.strength
}

// ===== Atmosphere Material =====

#[derive(Asset, TypePath, AsBindGroup, Clone)]
pub struct AtmosphereMaterial {
    #[uniform(0)]
    pub color: LinearRgba,
    #[uniform(0)]
    pub camera_and_rim: Vec4, // xyz = camera position, w = rim power
    #[uniform(0)]
    pub sun_dir: Vec4,       // Use Vec4 for 16-byte alignment (w is unused)
    #[uniform(0)]
    pub radii: Vec4,         // x = planet/ocean radius, y = atmosphere radius
}

impl Material for AtmosphereMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/atmosphere.wgsl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Blend
    }

    fn specialize(
        _pipeline: &MaterialPipeline<Self>,
        descriptor: &mut RenderPipelineDescriptor,
        _layout: &MeshVertexBufferLayoutRef,
        _key: MaterialPipelineKey<Self>,
    ) -> Result<(), SpecializedMeshPipelineError> {
        // Render both front and back faces for atmosphere glow
        descriptor.primitive.cull_mode = None;
        Ok(())
    }
}

// ===== Ocean Material =====

#[derive(Asset, TypePath, AsBindGroup, Clone)]
pub struct OceanMaterial {
    #[uniform(0)]
    pub color: LinearRgba,
    #[uniform(0)]
    pub time: f32,
    #[uniform(0)]
    pub wave_speed: f32,
    #[uniform(0)]
    pub wave_height: f32,
    #[uniform(0)]
    pub wave_frequency: f32,
    #[uniform(0)]
    pub sun_dir: Vec4,
}

impl Material for OceanMaterial {
    fn vertex_shader() -> ShaderRef {
        "shaders/ocean.wgsl".into()
    }
    fn fragment_shader() -> ShaderRef {
        "shaders/ocean.wgsl".into()
    }
    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Blend
    }
}

#[derive(Component)]
pub struct OceanMarker;

// ===== Systems =====

pub fn update_ocean_time(
    time: Res<Time>,
    sun_q: Query<&Transform, With<crate::SunMarker>>,
    mut materials: ResMut<Assets<OceanMaterial>>,
    ocean_q: Query<&MeshMaterial3d<OceanMaterial>>,
) {
    let sun_dir = sun_q.get_single()
        .map(|tf| tf.translation.normalize_or(Vec3::X))
        .unwrap_or(Vec3::X);

    for handle in ocean_q.iter() {
        if let Some(mat) = materials.get_mut(&handle.0) {
            mat.time = time.elapsed_secs();
            mat.sun_dir = sun_dir.extend(0.0);
        }
    }
}

#[derive(Component)]
pub struct AtmosphereMarker;

// ===== Systems =====

pub fn update_atmosphere_camera(
    camera_q: Query<&GlobalTransform, With<Camera3d>>,
    sun_q: Query<&Transform, With<crate::SunMarker>>,
    terrain_config: Res<crate::TerrainConfig>,
    mut materials: ResMut<Assets<AtmosphereMaterial>>,
    atmosphere_q: Query<&MeshMaterial3d<AtmosphereMaterial>>,
) {
    let Ok(cam_gt) = camera_q.get_single() else { return };
    let cam_pos = cam_gt.translation();

    let sun_dir = sun_q.get_single()
        .map(|tf| tf.translation.normalize_or(Vec3::X))
        .unwrap_or(Vec3::X);

    // Base surface for atmosphere calculations is either the solid planet or the ocean surface, whichever is higher.
    // This assumes the ocean completely covers the lowlands (offset by sea_level).
    let surface_radius = crate::constants::KERBIN_RADIUS + terrain_config.sea_level.max(0.0);
    // Make sure atmosphere radius shrinks or expands appropriately if surface_radius changes significantly
    let atmosphere_radius = surface_radius + terrain_config.atmosphere_height * 0.7;

    for handle in atmosphere_q.iter() {
        if let Some(mat) = materials.get_mut(&handle.0) {
            mat.camera_and_rim.x = cam_pos.x;
            mat.camera_and_rim.y = cam_pos.y;
            mat.camera_and_rim.z = cam_pos.z;
            mat.sun_dir = sun_dir.extend(0.0); // Convert Vec3 to Vec4
            mat.radii = Vec4::new(surface_radius, atmosphere_radius, 0.0, 0.0);
        }
    }
}

// ===== Math Helpers =====

pub fn smooth_max(a: f32, b: f32, k: f32) -> f32 {
    let h = (0.5 + 0.5 * (a - b) / k).clamp(0.0, 1.0);
    a * h + b * (1.0 - h) + k * h * (1.0 - h)
}

// ===== Terrain Mesh Generation =====

pub fn generate_terrain_sphere_mesh(
    radius: f32,
    segments: u32,
    rings: u32,
    max_height_ratio: f32, // 改為比例 (例如 0.05 代表山高為半徑的 5%)
    ocean_depth_multiplier: f32,
    ocean_floor_smoothing: f32,
    mountain_blend: f32,
    continent_settings: &NoiseSettings,
    mountain_settings: &NoiseSettings,
    mask_settings: &NoiseSettings,
) -> Mesh {
    let mut mesh = Sphere::new(radius).mesh().uv(segments, rings);

    let floor_depth = 0.3;

    // 計算絕對高度偏移量，使其隨半徑縮放
    let absolute_max_height = radius * max_height_ratio;

    if let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    {
        for pos in positions.iter_mut() {
            let dir = Vec3::from_array(*pos).normalize();

            // 1. Continent Shape (base landmasses)
            let raw_continent = fbm(dir, continent_settings);

            // [Reference Logic] Smooth the floor to create the basin
            let mut continent_val = smooth_max(raw_continent, -floor_depth, ocean_floor_smoothing);

            // [Reference Logic] Only multiply if it's below sea level (0.0)
            // This keeps the islands and coastlines stable while the deep basins sink
            if continent_val < 0.0 {
                continent_val *= 1.0 + ocean_depth_multiplier;
            }

            // 2. Mountain Ridge detail
            let ridge_val = ridged_fbm(dir, mountain_settings);

            // 3. Mask logic
            let raw_mask = fbm(dir, mask_settings);
            let mask = (raw_mask * 0.5 + 0.5).powf(2.0).clamp(0.0, 1.0);

            // [Strict Isolation] Mountains ONLY appear where raw_continent > 0
            // We use a very sharp threshold to prevent mountain stretching into the sea
            let mountain_weight = (raw_continent * 50.0).clamp(0.0, 1.0) * mask * mountain_blend;

            let final_noise = continent_val + (ridge_val * mountain_weight);

            // 位移量現在與半徑掛鉤
            let displacement = final_noise * absolute_max_height;

            *pos = (dir * (radius + displacement)).to_array();
        }
    }

    recalculate_normals(&mut mesh);
    mesh
}

fn recalculate_normals(mesh: &mut Mesh) {
    let positions = match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
        Some(VertexAttributeValues::Float32x3(pos)) => pos,
        _ => return,
    };

    let indices: Vec<u32> = match mesh.indices() {
        Some(Indices::U32(idx)) => idx.clone(),
        Some(Indices::U16(idx)) => idx.iter().map(|&i| i as u32).collect(),
        _ => return,
    };

    let mut normals = vec![Vec3::ZERO; positions.len()];

    for chunk in indices.chunks_exact(3) {
        let v0 = chunk[0] as usize;
        let v1 = chunk[1] as usize;
        let v2 = chunk[2] as usize;

        let p0 = Vec3::from_array(positions[v0]);
        let p1 = Vec3::from_array(positions[v1]);
        let p2 = Vec3::from_array(positions[v2]);

        // 計算面法線 (注意順序，確保朝外)
        let edge1 = p1 - p0;
        let edge2 = p2 - p0;
        let face_normal = edge1.cross(edge2);

        // 將面法線累加到每個頂點
        normals[v0] += face_normal;
        normals[v1] += face_normal;
        normals[v2] += face_normal;
    }

    // 將累加的法線歸一化，並轉回陣列格式
    let final_normals: Vec<[f32; 3]> = normals
        .into_iter()
        .map(|n| n.normalize_or(Vec3::Y).to_array())
        .collect();

    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, final_normals);
}
