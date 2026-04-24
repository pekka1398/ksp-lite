#import bevy_pbr::forward_io::VertexOutput

// 引入 Bevy 的視圖視圖綁定
#import bevy_pbr::mesh_view_bindings::view

struct AtmosphereParams {
    color: vec4<f32>,
    camera_and_rim: vec4<f32>,
    sun_dir: vec4<f32>, // Must be vec4 for alignment
    radii: vec4<f32>,   // x = planet radius, y = atmosphere radius
};

@group(2) @binding(0) var<uniform> params: AtmosphereParams;

// Ray-Sphere intersection helper
// Returns distance to entry and exit points. Negative if no intersection.
fn ray_sphere_intersect(ray_origin: vec3<f32>, ray_dir: vec3<f32>, sphere_radius: f32) -> vec2<f32> {
    let a = dot(ray_dir, ray_dir);
    let b = 2.0 * dot(ray_origin, ray_dir);
    let c = dot(ray_origin, ray_origin) - sphere_radius * sphere_radius;
    let d = b * b - 4.0 * a * c;

    if (d < 0.0) {
        return vec2<f32>(-1.0, -1.0);
    }

    let sqrt_d = sqrt(d);
    return vec2<f32>((-b - sqrt_d) / (2.0 * a), (-b + sqrt_d) / (2.0 * a));
}

@fragment
fn fragment(
    in: bevy_pbr::forward_io::VertexOutput,
) -> @location(0) vec4<f32> {
    let world_pos = in.world_position.xyz;
    let camera_pos = view.world_position; // Use Bevy's built-in camera position

    // View ray from camera through the current fragment
    let view_dir = normalize(world_pos - camera_pos);

    // Atmosphere radii
    // We assume the planet center is at (0,0,0) for now.
    // If not, we should subtract planet center from camera_pos.
    let planet_radius = params.radii.x;
    let atmosphere_radius = params.radii.y;

    // 1. Calculate intersection with atmosphere sphere
    let atmo_hit = ray_sphere_intersect(camera_pos, view_dir, atmosphere_radius);

    // 2. Calculate intersection with planet sphere (to handle occlusion)
    let planet_hit = ray_sphere_intersect(camera_pos, view_dir, planet_radius);

    // Distances
    var d_entry = max(0.0, atmo_hit.x);
    var d_exit = atmo_hit.y;

    // Handle planet occlusion and camera being "underground"
    if (planet_hit.y > 0.0) {
        if (planet_hit.x > 0.0) {
            // Outside looking at planet: cap at entry
            d_exit = min(d_exit, planet_hit.x);
        } else {
            // Inside planet: atmosphere starts where we leave the planet
            d_entry = max(d_entry, planet_hit.y);
        }
    }

    let path_length = max(0.0, d_exit - d_entry);

    // 3. Density/Thickness model
    // Simple linear approximation of thickness
    // We can make this more complex later with exponential falloff
    let max_thickness = (atmosphere_radius - planet_radius) * 2.0;
    let thickness = path_length / (atmosphere_radius - planet_radius);

    // 4. Sun lighting & Scattering
    // Calculate optical depth to sun from a representative point
    let test_point_dist = d_entry + path_length * 0.4; // Sample a bit into the atmosphere
    let test_point = camera_pos + view_dir * test_point_dist;
    let test_dir = normalize(test_point);

    // How much atmosphere is between this point and the sun?
    let sun_dir_vec3 = params.sun_dir.xyz;
    let sun_factor = dot(test_dir, sun_dir_vec3);

    // Day/Night mask - much more aggressive now
    let sun_mask = smoothstep(-0.25, 0.05, sun_factor);

    // Rayleigh scattering approximation:
    // When looking at the sun through a lot of atmosphere (low sun_factor),
    // light shifts to red/orange.
    let scatter_blue = vec3<f32>(0.2, 0.5, 1.0);
    let scatter_red = vec3<f32>(1.0, 0.4, 0.1);

    // Sunset happens when sun_factor is low but positive
    let sunset_strength = smoothstep(0.4, -0.1, sun_factor);

    // Final color shifts from blue to orange/red
    var final_color = mix(scatter_blue, scatter_red, sunset_strength);

    // Add extra brightness for the sun disk area
    let view_sun_dot = dot(view_dir, sun_dir_vec3);
    let sun_glow = pow(max(0.0, view_sun_dot), 20.0) * 0.5 * sun_mask;
    final_color += vec3<f32>(1.0, 0.9, 0.7) * sun_glow;

    // Use thickness to drive alpha
    let alpha = saturate(thickness * 0.4) * sun_mask;

    return vec4<f32>(final_color, alpha * params.color.a);
}
