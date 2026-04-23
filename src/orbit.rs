use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::{AppState, CelestialBody, Rocket};
use crate::constants::*;

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
    let steps = ((dt / KEPLER_MAX_STEP_SIZE).ceil() as usize).max(1);
    let h = dt / steps as f32;

    debug!(
        "propagate_kepler: dt={:.2} steps={} h={:.4} | pos=({:.1},{:.1},{:.1}) vel=({:.1},{:.1},{:.1}) mu={:.1}",
        dt, steps, h,
        pos.x, pos.y, pos.z,
        vel.x, vel.y, vel.z,
        mu,
    );

    let mut r = pos;
    let mut v = vel;

    let mut hit_min_radius = false;

    for _ in 0..steps {
        let accel = |r: Vec3| -> Vec3 {
            let r_mag = r.length();
            if r_mag < MIN_RADIUS_FOR_KEPLER { return Vec3::ZERO; }
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

        if r.length() < MIN_RADIUS_FOR_KEPLER && !hit_min_radius {
            hit_min_radius = true;
            debug!("propagate_kepler: hit MIN_RADIUS at r.length()={:.4}", r.length());
        }
    }

    debug!(
        "propagate_kepler result: pos=({:.1},{:.1},{:.1}) vel=({:.1},{:.1},{:.1}) | hit_min_radius={}",
        r.x, r.y, r.z,
        v.x, v.y, v.z,
        hit_min_radius,
    );

    (r, v)
}

/// Find the dominant SOI body for a given world position.
pub fn find_soi_body<'a>(
    pos: Vec3,
    bodies: impl Iterator<Item = (&'a CelestialBody, &'a Transform)>,
    should_log: bool,
) -> (&'a CelestialBody, &'a Transform) {
    let mut soi: Option<(&CelestialBody, &Transform)> = None;
    let mut soi_ratio = f32::MAX;
    let mut nearest: Option<(&CelestialBody, &Transform)> = None;
    let mut nearest_dist = f32::MAX;

    for (body, body_tf) in bodies {
        let dist = (pos - body_tf.translation).length();
        if should_log {
            debug!(
                "SOI candidate: {} | dist={:.1} | soi_radius={:.1} | inside={}",
                body.name, dist, body.soi_radius, dist < body.soi_radius,
            );
        }
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

    let result = soi.or(nearest).unwrap();
    if should_log {
        let is_soi = soi.is_some();
        debug!(
            "SOI selected: {} | via={} | nearest_dist={:.1}",
            result.0.name,
            if is_soi { "soi_radius" } else { "nearest" },
            nearest_dist,
        );
    }
    result
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
    should_log: bool,
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

    if should_log {
        debug!(
            "orbit elements: a={:.1} e={:.6} energy={:.2} | r={:.1} v={:.1} h_mag={:.2} | mu={:.1}",
            a, e, energy,
            pos.length(), vel.length(), h_mag,
            mu,
        );
    }

    let p = if e > 0.0001 {
        e_vec.normalize()
    } else {
        pos.normalize_or(Vec3::X)
    };
    let q = h.cross(p).normalize();

    let ap_pe = if e > 0.001 {
        if e < 1.0 {
            let r_ap = a * (1.0 + e);
            let r_pe = a * (1.0 - e);
            if should_log {
                debug!(
                    "Ap/Pe: r_ap={:.1} (alt={:.1}) r_pe={:.1} (alt={:.1}) | planet_radius={:.1}",
                    r_ap, r_ap - planet_radius,
                    r_pe, r_pe - planet_radius,
                    planet_radius,
                );
            }
            ApPe {
                apoapsis: Some(planet_center - p * r_ap),
                periapsis: Some(planet_center + p * r_pe),
            }
        } else {
            let r_pe = a * (e - 1.0);
            if should_log {
                debug!(
                    "Pe (hyperbolic): r_pe={:.1} (alt={:.1}) | planet_radius={:.1}",
                    r_pe, r_pe - planet_radius,
                    planet_radius,
                );
            }
            ApPe {
                apoapsis: None,
                periapsis: Some(planet_center + p * r_pe),
            }
        }
    } else {
        if should_log {
            debug!("No Ap/Pe: e={:.6} (circular)", e);
        }
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

// ===== Screen-space constant markers =====

/// Sphere radius that appears the same size on screen regardless of camera distance.
fn screen_marker_radius(cam_pos: Vec3, marker_pos: Vec3) -> f32 {
    const SCREEN_FACTOR: f32 = 0.015;
    (cam_pos - marker_pos).length() * SCREEN_FACTOR
}

// ===== Systems =====

pub fn orbit_prediction_system(
    mut gizmos: Gizmos,
    planet_q: Query<(&CelestialBody, &Transform)>,
    rocket_q: Query<(&Transform, &Velocity, &ColliderMassProperties, Entity), With<Rocket>>,
    part_q: Query<(Entity, &Transform, &ColliderMassProperties, &Velocity), Without<Rocket>>,
    joint_q: Query<&ImpulseJoint>,
    camera_q: Query<&Transform, With<Camera3d>>,
    state: Res<State<AppState>>,
    time: Res<Time<Virtual>>,
    mut maneuver: ResMut<ManeuverNode>,
    mut log_timer: Local<Timer>,
) {
    if *state.get() != AppState::MapView { return; }

    if log_timer.duration().as_secs_f32() == 0.0 {
        *log_timer = Timer::from_seconds(2.0, TimerMode::Repeating);
    }
    let should_log = log_timer.tick(time.delta()).just_finished();

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

    let (planet, planet_tf) = find_soi_body(com_pos, planet_q.iter(), should_log);

    let pos = com_pos - planet_tf.translation;
    let vel = com_vel - soi_body_velocity(planet, planet_tf);
    let mu = planet.mu;

    if should_log {
        debug!(
            "orbit_prediction: SOI={} | com=({:.1},{:.1},{:.1}) total_mass={:.1} | rel_pos=({:.1},{:.1},{:.1}) rel_vel=({:.1},{:.1},{:.1}) | mu={:.1}",
            planet.name,
            com_pos.x, com_pos.y, com_pos.z,
            total_mass,
            pos.x, pos.y, pos.z,
            vel.x, vel.y, vel.z,
            mu,
        );
    }

    let ap_pe = draw_orbit_gizmo(&mut gizmos, pos, vel, mu, planet_tf.translation, planet.radius, planet.soi_radius, Color::srgb(0.0, 0.8, 1.0), should_log);

    let cam_pos = camera_q.get_single().map(|t| t.translation).unwrap_or(com_pos);

    gizmos.sphere(com_pos, screen_marker_radius(cam_pos, com_pos), Color::srgb(0.0, 1.0, 0.0));

    if let Some(ap_pos) = ap_pe.apoapsis {
        gizmos.sphere(ap_pos, screen_marker_radius(cam_pos, ap_pos), Color::srgb(1.0, 0.3, 0.3));
    }
    if let Some(pe_pos) = ap_pe.periapsis {
        gizmos.sphere(pe_pos, screen_marker_radius(cam_pos, pe_pos), Color::srgb(0.3, 0.6, 1.0));
    }

    // Draw orbital paths for celestial bodies that orbit another body
    for (body, _) in planet_q.iter() {
        if body.orbit_radius > 0.0 {
            // Find parent position — the body this one orbits
            // For now, assume parent is at the position of the closest body with orbit_radius == 0
            let parent_pos = planet_q.iter()
                .find(|(b, _)| b.orbit_radius == 0.0 && b.name != body.name)
                .map(|(_, tf)| tf.translation)
                .unwrap_or(Vec3::ZERO);

            let segments = 128;
            let mut points = Vec::with_capacity(segments + 1);
            for i in 0..=segments {
                let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
                let p = parent_pos + Vec3::new(
                    body.orbit_radius * angle.cos(),
                    0.0,
                    body.orbit_radius * angle.sin(),
                );
                points.push(p);
            }
            gizmos.linestrip(points, Color::srgba(0.4, 0.4, 0.4, 0.5));
        }
    }

    let time_to_node = (maneuver.ut - now) as f32;
    if maneuver.is_active() && time_to_node <= 0.0 {
        warn!("Maneuver node expired (time_to_node={time_to_node:.1}s), auto-removing");
        *maneuver = ManeuverNode::default();
    }
    if maneuver.is_active() {
        let speed = vel.length();
        if should_log {
            debug!(
                "maneuver: active=true | time_to_node={:.1}s | prograde={:.1} normal={:.1} radial={:.1} | current_speed={:.1}",
                time_to_node,
                maneuver.prograde, maneuver.normal, maneuver.radial,
                speed,
            );
        }
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
            if should_log {
                debug!(
                    "maneuver propagated: future_pos=({:.1},{:.1},{:.1}) future_speed={:.1}",
                    future_pos.x, future_pos.y, future_pos.z,
                    future_speed,
                );
            }
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

                    if should_log {
                        debug!(
                            "maneuver dv: |dv|={:.1} | post_maneuver_speed={:.1}",
                            maneuver_dv.length(),
                            post_maneuver_vel.length(),
                        );
                    }
                    let mnv_ap_pe = draw_orbit_gizmo(
                        &mut gizmos,
                        future_pos,
                        post_maneuver_vel,
                        mu,
                        planet_tf.translation,
                        planet.radius,
                        planet.soi_radius,
                        Color::srgb(1.0, 0.9, 0.0),
                        should_log,
                    );

                    let node_world = planet_tf.translation + future_pos;
                    gizmos.sphere(node_world, screen_marker_radius(cam_pos, node_world), Color::srgb(1.0, 0.3, 0.0));

                    if let Some(ap_pos) = mnv_ap_pe.apoapsis {
                        gizmos.sphere(ap_pos, screen_marker_radius(cam_pos, ap_pos), Color::srgb(1.0, 0.5, 0.3));
                    }
                    if let Some(pe_pos) = mnv_ap_pe.periapsis {
                        gizmos.sphere(pe_pos, screen_marker_radius(cam_pos, pe_pos), Color::srgb(0.3, 0.5, 1.0));
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
            debug!("maneuver_node: clearing active node");
            *node = ManeuverNode::default();
        } else {
            *node = ManeuverNode {
                prograde: DEFAULT_MANEUVER_PROGRADE,
                normal: 0.0,
                radial: 0.0,
                ut: now + DEFAULT_MANEUVER_TIME_OFFSET,
            };
            debug!("maneuver_node: created node at ut={:.1} (now={:.1})", node.ut, now);
        }
    }

    if keys.just_pressed(KeyCode::Delete) || keys.just_pressed(KeyCode::Backspace) {
        debug!("maneuver_node: deleted via key");
        *node = ManeuverNode::default();
    }

    if node.is_active() {
        let dv_step = MANEUVER_DV_STEP;
        if keys.just_pressed(KeyCode::KeyI) { node.prograde += dv_step; debug!("maneuver_node: prograde+={dv_step} -> {}", node.prograde); }
        if keys.just_pressed(KeyCode::KeyK) { node.prograde -= dv_step; debug!("maneuver_node: prograde-={dv_step} -> {}", node.prograde); }
        if keys.just_pressed(KeyCode::KeyJ) { node.radial -= dv_step; debug!("maneuver_node: radial-={dv_step} -> {}", node.radial); }
        if keys.just_pressed(KeyCode::KeyL) { node.radial += dv_step; debug!("maneuver_node: radial+={dv_step} -> {}", node.radial); }
        if keys.just_pressed(KeyCode::KeyU) { node.normal -= dv_step; debug!("maneuver_node: normal-={dv_step} -> {}", node.normal); }
        if keys.just_pressed(KeyCode::KeyO) { node.normal += dv_step; debug!("maneuver_node: normal+={dv_step} -> {}", node.normal); }

        let time_step = MANEUVER_TIME_STEP;
        if keys.just_pressed(KeyCode::KeyT) { node.ut += time_step; debug!("maneuver_node: ut+={time_step} -> {}", node.ut); }
        if keys.just_pressed(KeyCode::KeyG) { node.ut = (node.ut - time_step).max(now); debug!("maneuver_node: ut-={time_step} -> {}", node.ut); }
    }
}
