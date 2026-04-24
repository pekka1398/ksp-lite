use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use crate::{AppState, Rocket, CelestialBody};
use crate::flight::FlightEntity;
use crate::orbit;

const NB_SIZE: u32 = 256;
const NB_PX: usize = NB_SIZE as usize;
const NB_HALF: f32 = NB_SIZE as f32 / 2.0;

#[derive(Component)]
pub struct Navball;

pub fn spawn_navball(commands: &mut Commands, images: &mut Assets<Image>) {
    let data = vec![0u8; NB_PX * NB_PX * 4];
    let image = Image::new(
        Extent3d {
            width: NB_SIZE,
            height: NB_SIZE,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        bevy::asset::RenderAssetUsages::RENDER_WORLD | bevy::asset::RenderAssetUsages::MAIN_WORLD,
    );
    let handle = images.add(image);

    // Navball
    commands.spawn((
        ImageNode::new(handle),
        Node {
            width: Val::Px(200.0),
            height: Val::Px(200.0),
            position_type: PositionType::Absolute,
            bottom: Val::Px(20.0),
            left: Val::Percent(50.0),
            margin: UiRect::left(Val::Px(-100.0)),
            ..default()
        },
        Navball,
        FlightEntity,
    ));

    // Legend
    commands.spawn((
        Text::new("● PRO  ● RET\n● RAD+ ● RAD-\n● NRM+ ● NRM-"),
        TextFont { font_size: 14.0, ..default() },
        TextColor(Color::srgb(0.7, 0.7, 0.7)),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(30.0),
            left: Val::Percent(50.0),
            margin: UiRect::left(Val::Px(110.0)),
            ..default()
        },
        FlightEntity,
    ));
}

pub fn navball_system(
    rocket_q: Query<(&Transform, &crate::sim::SimState), With<Rocket>>,
    planet_q: Query<(&CelestialBody, &crate::sim::SimState), Without<Rocket>>,
    mut images: ResMut<Assets<Image>>,
    navball_q: Query<&ImageNode, With<Navball>>,
    state: Res<State<AppState>>,
) {
    if *state.get() != AppState::Flight && *state.get() != AppState::MapView { return; }

    let Ok((rocket_tf, rocket_sim)) = rocket_q.get_single() else { return };
    let Ok(image_node) = navball_q.get_single() else { return };

    let handle = image_node.image.clone();
    let image = match images.get_mut(&handle) {
        Some(img) => img,
        None => return,
    };

    // Ship orientation axes
    let ship_fwd: Vec3 = *rocket_tf.up();
    let ship_right: Vec3 = *rocket_tf.right();
    let ship_up = ship_fwd.cross(ship_right);

    // Orbital reference frame
    let (_soi_body, soi_sim) = orbit::find_soi_body(rocket_sim.position, planet_q.iter(), false);
    let rel_pos_d = rocket_sim.position.0 - soi_sim.position.0;
    let orbital_vel_d = rocket_sim.velocity.0 - soi_sim.velocity.0;
    let orbital_vel = orbital_vel_d.as_vec3();
    let rel_pos = rel_pos_d.as_vec3();

    let radial_out = rel_pos.normalize_or(Vec3::Y);
    let prograde = if orbital_vel.length() > 1.0 {
        orbital_vel.normalize()
    } else {
        ship_fwd
    };
    let normal = prograde.cross(radial_out).normalize_or(Vec3::Y);

    // Project orbital axes onto ship's local frame
    let rad_s = v3_proj(radial_out, ship_right, ship_up, ship_fwd);
    let pro_s = v3_proj(prograde, ship_right, ship_up, ship_fwd);
    let nrm_s = v3_proj(normal, ship_right, ship_up, ship_fwd);

    let data = &mut image.data;

    // --- Draw hemisphere background + horizon + grid ---
    for py in 0..NB_SIZE {
        for px in 0..NB_SIZE {
            let nx = (px as f32 - NB_HALF) / NB_HALF;
            let ny = (NB_HALF - py as f32) / NB_HALF;
            let r_sq = nx * nx + ny * ny;
            let idx = (py as usize * NB_PX + px as usize) * 4;

            // Border ring
            if r_sq > 0.96 {
                let a = if r_sq > 1.0 { 0 } else { ((1.0 - r_sq) / 0.04 * 200.0) as u8 };
                data[idx] = 25;
                data[idx + 1] = 25;
                data[idx + 2] = 35;
                data[idx + 3] = a;
                continue;
            }

            let depth = (1.0 - r_sq).sqrt();
            let rad_comp = nx * rad_s.0 + ny * rad_s.1 + depth * rad_s.2;

            // Edge fade
            let edge_alpha = if r_sq > 0.88 {
                ((1.0 - r_sq) / 0.08 * 230.0).min(230.0) as u8
            } else {
                230
            };

            // Blue (sky / radial-out) or Orange (ground / radial-in)
            let (r, g, b) = if rad_comp > 0.0 {
                let t = rad_comp.min(1.0);
                (30.0 + t * 30.0, 55.0 + t * 50.0, 130.0 + t * 70.0)
            } else {
                let t = (-rad_comp).min(1.0);
                (130.0 + t * 70.0, 65.0 + t * 30.0, 15.0 + t * 20.0)
            };

            // Horizon line
            if rad_comp.abs() < 0.03 {
                data[idx] = 200;
                data[idx + 1] = 200;
                data[idx + 2] = 200;
                data[idx + 3] = 240;
                continue;
            }

            // Pitch grid at ±30° and ±60°
            let pitch_deg = rad_comp.asin().to_degrees();
            let mut is_grid = false;
            for &g_deg in &[30.0_f32, 60.0] {
                if (pitch_deg - g_deg).abs() < 1.8 || (pitch_deg + g_deg).abs() < 1.8 {
                    is_grid = true;
                    break;
                }
            }
            if is_grid {
                data[idx] = (r * 0.6 + 140.0 * 0.4) as u8;
                data[idx + 1] = (g * 0.6 + 140.0 * 0.4) as u8;
                data[idx + 2] = (b * 0.6 + 140.0 * 0.4) as u8;
                data[idx + 3] = edge_alpha;
                continue;
            }

            data[idx] = r as u8;
            data[idx + 1] = g as u8;
            data[idx + 2] = b as u8;
            data[idx + 3] = edge_alpha;
        }
    }

    // --- Center crosshair ---
    let c = NB_HALF as i32;
    for d in -14i32..=14 {
        if d.abs() > 2 {
            set_px(data, c + d, c, 220, 220, 220, 255);
            set_px(data, c, c + d, 220, 220, 220, 255);
        }
    }
    // Diagonal wings
    for d in 3..=9 {
        set_px(data, c + d, c - d, 180, 180, 180, 200);
        set_px(data, c - d, c - d, 180, 180, 180, 200);
    }

    // --- Direction markers ---
    draw_marker(data, pro_s, [255, 220, 50], true);           // Prograde
    draw_marker(data, neg3(pro_s), [255, 180, 30], false);    // Retrograde
    draw_marker(data, rad_s, [100, 210, 255], true);          // Radial out
    draw_marker(data, neg3(rad_s), [60, 150, 210], false);    // Radial in
    draw_marker(data, nrm_s, [200, 100, 255], true);          // Normal
    draw_marker(data, neg3(nrm_s), [150, 60, 210], false);    // Anti-normal
}

// --- Helpers ---

fn v3_proj(v: Vec3, right: Vec3, up: Vec3, fwd: Vec3) -> (f32, f32, f32) {
    (v.dot(right), v.dot(up), v.dot(fwd))
}

fn neg3((x, y, z): (f32, f32, f32)) -> (f32, f32, f32) {
    (-x, -y, -z)
}

fn set_px(data: &mut [u8], x: i32, y: i32, r: u8, g: u8, b: u8, a: u8) {
    if x < 0 || x >= NB_SIZE as i32 || y < 0 || y >= NB_SIZE as i32 { return; }
    let idx = (y as usize * NB_PX + x as usize) * 4;
    data[idx] = r;
    data[idx + 1] = g;
    data[idx + 2] = b;
    data[idx + 3] = a;
}

fn draw_marker(data: &mut [u8], proj: (f32, f32, f32), color: [u8; 3], filled: bool) {
    let (xp, yp, zp) = proj;
    if zp < -0.5 { return; }

    let (px, py, radius) = if zp >= 0.0 {
        ((NB_HALF + xp * NB_HALF) as i32, (NB_HALF - yp * NB_HALF) as i32, 7)
    } else {
        let len = (xp * xp + yp * yp).sqrt();
        if len < 0.01 { return; }
        let s = 0.85 / len;
        ((NB_HALF + xp * s * NB_HALF) as i32, (NB_HALF - yp * s * NB_HALF) as i32, 5)
    };

    for dy in -radius..=radius {
        for dx in -radius..=radius {
            let d_sq = dx * dx + dy * dy;
            if d_sq > radius * radius { continue; }
            let dist = (d_sq as f32).sqrt() / radius as f32;

            let (r, g, b) = if !filled && dist < 0.55 {
                // Hollow center for retro/anti markers
                (10, 10, 10)
            } else {
                (color[0], color[1], color[2])
            };
            set_px(data, px + dx, py + dy, r, g, b, 255);
        }
    }
}
