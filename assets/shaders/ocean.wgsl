#import bevy_pbr::forward_io::{VertexOutput, Vertex}
#import bevy_pbr::mesh_functions
#import bevy_pbr::mesh_view_bindings::view

struct OceanParams {
    @size(16) color: vec4<f32>,
    time: f32,
    wave_speed: f32,
    wave_height: f32,
    wave_frequency: f32,
    sun_dir: vec4<f32>,
};

@group(2) @binding(0) var<uniform> params: OceanParams;

// 擴展 VertexOutput 以包含波浪強度
struct OceanVertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) world_position: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) wave_intensity: f32, // 用於畫浪花
};

@vertex
fn vertex(vertex: Vertex) -> OceanVertexOutput {
    var out: OceanVertexOutput;

    var pos = vertex.position;
    let world_normal = normalize(vertex.position);

    // 疊加波浪
    let wave1 = sin(pos.x * params.wave_frequency + params.time * params.wave_speed);
    let wave2 = sin(pos.z * params.wave_frequency * 0.8 + params.time * params.wave_speed * 1.2);
    let total_wave_raw = (wave1 + wave2) * 0.5; // 範圍約 -1.0 ~ 1.0

    let displaced_pos = pos + world_normal * total_wave_raw * params.wave_height;

    var model_matrix = mesh_functions::get_world_from_local(vertex.instance_index);
    out.world_position = vec4<f32>(mesh_functions::mesh_position_local_to_world(model_matrix, vec4<f32>(displaced_pos, 1.0)));
    out.world_normal = mesh_functions::mesh_normal_local_to_world(vertex.normal, vertex.instance_index);
    out.position = mesh_functions::mesh_position_local_to_clip(model_matrix, vec4<f32>(displaced_pos, 1.0));

    // 將波浪強度傳給 fragment，映射到 0.0 ~ 1.0
    out.wave_intensity = total_wave_raw * 0.5 + 0.5;

    return out;
}

@fragment
fn fragment(
    in: OceanVertexOutput,
) -> @location(0) vec4<f32> {
    let normal = normalize(in.world_normal);
    let view_dir = normalize(view.world_position - in.world_position.xyz);
    let sun_dir = params.sun_dir.xyz;

    // Fresnel
    let fresnel = pow(1.0 - max(dot(normal, view_dir), 0.0), 4.0);

    // Diffuse daylight: dot(normal, sun_dir)
    let sun_factor = dot(normal, sun_dir);
    let day_mask = smoothstep(-0.2, 0.3, sun_factor);

    // Sunset reflection factor: high when sun is near horizon
    let sunset_factor = smoothstep(0.5, -0.1, sun_factor) * day_mask;

    // Specular (Sun reflection)
    let reflect_dir = reflect(-sun_dir, normal);
    let spec = pow(max(dot(view_dir, reflect_dir), 0.0), 32.0);

    // 浪花邏輯 (Foam)
    let foam_threshold = 0.75;
    let foam_amount = smoothstep(foam_threshold, 1.0, in.wave_intensity);
    let foam_color = vec3<f32>(0.8, 0.9, 1.0);

    // Base color: shifts toward golden-orange at sunset (sky reflection)
    let ocean_blue = params.color.rgb;
    let ocean_gold = vec3<f32>(0.9, 0.55, 0.25); // Golden sunset color
    let ocean_sunset = mix(ocean_blue, ocean_gold, sunset_factor * 0.7);

    // Base color: very dark at night, full color at day
    let lit_color = mix(ocean_sunset * 0.03, ocean_sunset * 1.2, day_mask);

    // Fresnel also picks up sunset color
    let fresnel_color = mix(vec3<f32>(0.5, 0.8, 1.0), vec3<f32>(1.0, 0.7, 0.4), sunset_factor);

    // 混合基礎色、Fresnel 亮度、浪花和高光
    var final_color = mix(lit_color, fresnel_color, fresnel);
    final_color = mix(final_color, foam_color, foam_amount * 0.6);
    // Specular turns orange at sunset
    let spec_color = mix(vec3<f32>(1.0, 0.95, 0.9), vec3<f32>(1.0, 0.6, 0.3), sunset_factor);
    final_color += spec_color * spec * 0.6 * day_mask;

    return vec4<f32>(final_color, params.color.a);
}
