use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::{AppState, CelestialBody, Rocket};

// ===== Resources =====

#[derive(Resource)]
pub struct ManeuverNode {
    pub prograde: f32,
    pub normal: f32,
    pub radial: f32,
    pub ut: f64,
}

impl Default for ManeuverNode {
    fn default() -> Self {
        Self {
            prograde: 0.0,
            normal: 0.0,
            radial: 0.0,
            ut: -1.0,
        }
    }
}

impl ManeuverNode {
    pub fn is_active(&self) -> bool {
        self.ut >= 0.0
    }
}

// ===== Helpers =====

/// Propagate a Keplerian orbit forward by `dt` seconds.
pub fn propagate_kepler(pos: Vec3, vel: Vec3, mu: f32, dt: f32) -> (Vec3, Vec3) {
    let steps = ((dt / 0.5).ceil() as usize).max(1);
    let h = dt / steps as f32;

    let mut r = pos;
    let mut v = vel;

    for _ in 0..steps {
        let accel = |r: Vec3| -> Vec3 {
            let r_mag = r.length();
            if r_mag < 0.001 { return Vec3::ZERO; }
            -mu * r / (r_mag * r_mag * r_mag)
        };

        let k1v = accel(r);
        let k1r = v;

        let k2v = accel(r + k1r * (h * 0.5));
        let k2r = v + k1v * (h * 0.5);

        let k3v = accel(r + k2r * (h * 0.5));
        let k3r = v + k2v * (h * 0.5);

        let k4v = accel(r + k3r * h);
        let k4r = v + k3v * h;

        r += (k1r + 2.0 * k2r + 2.0 * k3r + k4r) * (h / 6.0);
        v += (k1v + 2.0 * k2v + 2.0 * k3v + k4v) * (h / 6.0);
    }

    (r, v)
}

/// Find the dominant SOI body for a given world position.
pub fn find_soi_body<'a>(
    pos: Vec3,
    bodies: impl Iterator<Item = (&'a CelestialBody, &'a Transform)>,
) -> (&'a CelestialBody, &'a Transform) {
    let mut soi: Option<(&CelestialBody, &Transform)> = None;
    let mut soi_ratio = f32::MAX;
    let mut nearest: Option<(&CelestialBody, &Transform)> = None;
    let mut nearest_dist = f32::MAX;

    for (body, body_tf) in bodies {
        let dist = (pos - body_tf.translation).length();
        if dist < nearest_dist {
            nearest_dist = dist;
            nearest = Some((body, body_tf));
        }
        if dist < body.soi_radius {
            let ratio = dist / body.soi_radius;
            if ratio < soi_ratio {
                soi_ratio = ratio;
                soi = Some((body, body_tf));
            }
        }
    }

    soi.or(nearest).unwrap()
}

/// Compute the world-space velocity of a celestial body (orbital motion).
pub fn soi_body_velocity(body: &CelestialBody, body_tf: &Transform) -> Vec3 {
    if body.orbit_radius > 0.0 && body.orbit_speed > 0.0 {
        body.orbit_speed * Vec3::new(-body_tf.translation.z, 0.0, body_tf.translation.x)
    } else {
        Vec3::ZERO
    }
}

/// Compute the surface rotation velocity at a world position on a rotating body.
pub fn surface_rotation_velocity(body: &CelestialBody, pos: Vec3) -> Vec3 {
    if body.rotation_speed == 0.0 { return Vec3::ZERO; }
    body.rotation_speed * Vec3::new(-pos.z, 0.0, pos.x)
}

// ===== Orbit Drawing =====

struct ApPe {
    apoapsis: Option<Vec3>,
    periapsis: Option<Vec3>,
}

fn draw_orbit_gizmo(
    gizmos: &mut Gizmos,
    pos: Vec3,
    vel: Vec3,
    mu: f32,
    planet_center: Vec3,
    planet_radius: f32,
    soi_radius: f32,
    color: Color,
) -> ApPe {
    let altitude = pos.length() - planet_radius;
    let speed = vel.length();
    if altitude < 10.0 || speed < 10.0 {
        if speed > 0.0 {
            warn!("draw_orbit_gizmo skipped: altitude={altitude:.1}, speed={speed:.1}");
        }
        return ApPe { apoapsis: None, periapsis: None };
    }

    let h = pos.cross(vel);
    let h_mag = h.length();
    if h_mag < 0.001 {
        warn!("draw_orbit_gizmo skipped: h_mag={h_mag:.4} (degenerate orbit)");
        return ApPe { apoapsis: None, periapsis: None };
    }

    let e_vec = (vel.cross(h) / mu) - pos.normalize();
    let e = e_vec.length();
    let energy = vel.length_squared() / 2.0 - mu / pos.length();

    let a = if e < 1.0 {
        -mu / (2.0 * energy)
    } else {
        mu / (2.0 * energy.abs())
    };

    let p = if e > 0.0001 {
        e_vec.normalize()
    } else {
        pos.normalize_or(Vec3::X)
    };
    let q = h.cross(p).normalize();

    let ap_pe = if e < 1.0 && e > 0.001 {
        let r_ap = a * (1.0 + e);
        let r_pe = a * (1.0 - e);
        ApPe {
            apoapsis: Some(planet_center - p * r_ap),
            periapsis: Some(planet_center + p * r_pe),
        }
    } else {
        ApPe { apoapsis: None, periapsis: None }
    };

    let num_segments = 128;

    if e < 1.0 {
        let b = a * (1.0 - e * e).sqrt();
        let center = planet_center - p * (a * e);

        let mut points = Vec::with_capacity(num_segments + 1);
        for i in 0..=num_segments {
            let angle = (i as f32 / num_segments as f32) * std::f32::consts::TAU;
            let local_pos = p * (a * angle.cos()) + q * (b * angle.sin());
            points.push(center + local_pos);
        }
        gizmos.linestrip(points, color);
    } else {
        let mut points = Vec::new();
        let max_nu = (-1.0 / e).acos();
        let render_nu = max_nu * 0.99;

        for i in 0..=num_segments {
            let nu = -render_nu + (i as f32 / num_segments as f32) * (2.0 * render_nu);
            let r_mag = (a * (e * e - 1.0)) / (1.0 + e * nu.cos());
            if r_mag > soi_radius || r_mag < 0.0 { continue; }
            let local_pos = p * (r_mag * nu.cos()) + q * (r_mag * nu.sin());
            points.push(planet_center + local_pos);
        }
        if points.len() > 1 {
            gizmos.linestrip(points, color);
        }
    }

    ap_pe
}

// ===== Systems =====

pub fn orbit_prediction_system(
    mut gizmos: Gizmos,
    planet_q: Query<(&CelestialBody, &Transform)>,
    rocket_q: Query<(&Transform, &Velocity, &ColliderMassProperties, Entity), With<Rocket>>,
    part_q: Query<(Entity, &Transform, &ColliderMassProperties, &Velocity), Without<Rocket>>,
    joint_q: Query<&ImpulseJoint>,
    state: Res<State<AppState>>,
    time: Res<Time<Virtual>>,
    mut maneuver: ResMut<ManeuverNode>,
) {
    if *state.get() != AppState::MapView { return; }

    let now = time.elapsed_secs_f64();

    let Ok((rocket_tf, rocket_vel, rocket_mass_props, rocket_entity)) = rocket_q.get_single() else { return; };

    let mut total_mass = 0.0;
    let mut weighted_pos = Vec3::ZERO;
    let mut weighted_vel = Vec3::ZERO;

    let m0 = if let ColliderMassProperties::Mass(m) = *rocket_mass_props { m } else { 1.0 };
    total_mass += m0;
    weighted_pos += rocket_tf.translation * m0;
    weighted_vel += rocket_vel.linvel * m0;

    for (part_entity, part_tf, part_mass_props, part_vel) in part_q.iter() {
        let is_attached = joint_q.iter().any(|j| j.parent == rocket_entity && part_entity != rocket_entity);
        if is_attached {
            let m = if let ColliderMassProperties::Mass(m) = *part_mass_props { m } else { 1.0 };
            total_mass += m;
            weighted_pos += part_tf.translation * m;
            weighted_vel += part_vel.linvel * m;
        }
    }

    let com_pos = weighted_pos / total_mass;
    let com_vel = weighted_vel / total_mass;

    let (planet, planet_tf) = find_soi_body(com_pos, planet_q.iter());

    let pos = com_pos - planet_tf.translation;
    let vel = com_vel - soi_body_velocity(planet, planet_tf) + surface_rotation_velocity(planet, pos);
    let mu = planet.mu;

    let ap_pe = draw_orbit_gizmo(&mut gizmos, pos, vel, mu, planet_tf.translation, planet.radius, planet.soi_radius, Color::srgb(0.0, 0.8, 1.0));

    gizmos.sphere(com_pos, 10.0, Color::srgb(0.0, 1.0, 0.0));

    let marker_size = 12.0;
    if let Some(ap_pos) = ap_pe.apoapsis {
        gizmos.sphere(ap_pos, marker_size, Color::srgb(1.0, 0.3, 0.3));
    }
    if let Some(pe_pos) = ap_pe.periapsis {
        gizmos.sphere(pe_pos, marker_size, Color::srgb(0.3, 0.6, 1.0));
    }

    let time_to_node = (maneuver.ut - now) as f32;
    if maneuver.is_active() && time_to_node <= 0.0 {
        warn!("Maneuver node expired (time_to_node={time_to_node:.1}s), auto-removing");
        *maneuver = ManeuverNode::default();
    }
    if maneuver.is_active() {
        let speed = vel.length();
        if speed > 0.1 {
            let (future_pos, future_vel) = propagate_kepler(pos, vel, mu, time_to_node);

            let mut trail_points = vec![planet_tf.translation + pos];
            let trail_steps = 50;
            for i in 1..=trail_steps {
                let t = time_to_node * (i as f32 / trail_steps as f32);
                let (rp, _) = propagate_kepler(pos, vel, mu, t);
                trail_points.push(planet_tf.translation + rp);
            }
            gizmos.linestrip(trail_points, Color::srgb(0.5, 0.5, 0.5));

            let future_speed = future_vel.length();
            if future_speed > 0.1 {
                let prograde_dir = future_vel.normalize();
                let h = future_pos.cross(future_vel);
                let h_mag = h.length();
                if h_mag > 0.001 {
                    let normal_dir = h.normalize();
                    let radial_dir = prograde_dir.cross(normal_dir).normalize();

                    let maneuver_dv = prograde_dir * maneuver.prograde
                        + normal_dir * maneuver.normal
                        + radial_dir * maneuver.radial;

                    let post_maneuver_vel = future_vel + maneuver_dv;
                    let mnv_ap_pe = draw_orbit_gizmo(
                        &mut gizmos,
                        future_pos,
                        post_maneuver_vel,
                        mu,
                        planet_tf.translation,
                        planet.radius,
                        planet.soi_radius,
                        Color::srgb(1.0, 0.9, 0.0),
                    );

                    let node_world = planet_tf.translation + future_pos;
                    gizmos.sphere(node_world, 12.0, Color::srgb(1.0, 0.3, 0.0));

                    if let Some(ap_pos) = mnv_ap_pe.apoapsis {
                        gizmos.sphere(ap_pos, marker_size, Color::srgb(1.0, 0.5, 0.3));
                    }
                    if let Some(pe_pos) = mnv_ap_pe.periapsis {
                        gizmos.sphere(pe_pos, marker_size, Color::srgb(0.3, 0.5, 1.0));
                    }
                } else {
                    warn!("Maneuver orbit skipped: h_mag={h_mag:.4} at propagated position (degenerate)");
                }
            } else {
                warn!("Maneuver orbit skipped: future_speed={future_speed:.2} (too low after propagation)");
            }
        } else {
            warn!("Maneuver orbit skipped: current speed={speed:.2} (too low to propagate)");
        }
    }
}

pub fn maneuver_node_system(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time<Virtual>>,
    mut node: ResMut<ManeuverNode>,
) {
    let now = time.elapsed_secs_f64();

    if keys.just_pressed(KeyCode::KeyN) {
        if node.is_active() {
            *node = ManeuverNode::default();
        } else {
            *node = ManeuverNode {
                prograde: 100.0,
                normal: 0.0,
                radial: 0.0,
                ut: now + 300.0,
            };
        }
    }

    if keys.just_pressed(KeyCode::Delete) || keys.just_pressed(KeyCode::Backspace) {
        *node = ManeuverNode::default();
    }

    if node.is_active() {
        let dv_step = 50.0;
        if keys.just_pressed(KeyCode::KeyI) { node.prograde += dv_step; }
        if keys.just_pressed(KeyCode::KeyK) { node.prograde -= dv_step; }
        if keys.just_pressed(KeyCode::KeyJ) { node.radial -= dv_step; }
        if keys.just_pressed(KeyCode::KeyL) { node.radial += dv_step; }
        if keys.just_pressed(KeyCode::KeyU) { node.normal -= dv_step; }
        if keys.just_pressed(KeyCode::KeyO) { node.normal += dv_step; }

        let time_step = 30.0_f64;
        if keys.just_pressed(KeyCode::KeyT) { node.ut += time_step; }
        if keys.just_pressed(KeyCode::KeyG) { node.ut = (node.ut - time_step).max(now); }
    }
}
