#import bevy_pbr::forward_io::VertexOutput

struct AtmosphereParams {
    color: vec4<f32>,
    camera_and_rim: vec4<f32>,
};

@group(2) @binding(0) var<uniform> params: AtmosphereParams;

@fragment
fn fragment(
    in: VertexOutput,
    @builtin(front_facing) is_front: bool,
) -> @location(0) vec4<f32> {
    let world_pos = in.world_position.xyz;
    let camera_pos = params.camera_and_rim.xyz;
    let rim_power = params.camera_and_rim.w;

    let view_dir = normalize(camera_pos - world_pos);
    let normal = normalize(in.world_normal);

    // Flip normal for back faces so Fresnel works from both sides
    let n = select(-normal, normal, is_front);

    // Fresnel: 0 at center (looking straight down), 1 at edge (rim glow)
    let cos_theta = clamp(dot(view_dir, n), 0.0, 1.0);
    let fresnel = pow(1.0 - cos_theta, rim_power);

    return vec4<f32>(params.color.rgb, params.color.a * fresnel);
}
