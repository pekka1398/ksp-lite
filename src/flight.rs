use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::{AppState, CelestialBody, Rocket, OrbitCamera, StageMarker, ExhaustFlame, FuelTank, Engine, FloatingOrigin, OrbitAngle, OrbitParent, SasMode};
use crate::DebugLaunched;
use crate::vab::RocketConfig;
use crate::constants::*;

// ===== Components =====

#[derive(Component)]
pub struct FlightEntity;

#[derive(Component)]
pub struct PauseMenuUI;

#[derive(Component)]
pub struct TelemetryUI;

// ===== Resources =====

#[derive(Resource, Default)]
pub struct PrePauseView(pub AppState);

// ===== Flight Spawning =====

pub fn spawn_flight(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    config: Res<RocketConfig>,
    mut camera_q: Query<&mut OrbitCamera>,
    rocket_q: Query<(), With<Rocket>>,
) {
    if !rocket_q.is_empty() {
        return;
    }

    for mut orbit in camera_q.iter_mut() {
        orbit.distance = 20.0;
        orbit.pitch = 0.2;
        orbit.yaw = 0.0;
    }

    let equator_rot = Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2);
    let n_stages = config.stages.len();
    if n_stages == 0 { return; }

    let pad_top = LAUNCH_PAD_SURFACE_Y;
    let mut y_positions = Vec::with_capacity(n_stages);
    let mut y = pad_top + PAD_CLEARANCE;
    for i in (0..n_stages).rev() {
        let h = config.stages[i].height;
        y += h / 2.0;
        y_positions.insert(0, y);
        y += h / 2.0;
        y += STAGE_GAP;
    }

    let mat_upper = materials.add(UPPER_STAGE_COLOR);
    let mat_lower = materials.add(BOOSTER_COLOR);
    let mat_heavy = materials.add(HEAVY_BOOSTER_COLOR);
    let mat_nose = materials.add(NOSE_CONE_COLOR);
    let mat_engine = materials.add(ENGINE_COLOR);
    let mat_flame = materials.add(EXHAUST_FLAME_COLOR);

    let mut stage_entities = Vec::with_capacity(n_stages);

    for i in 0..n_stages {
        let stage = &config.stages[i];
        let is_top = i == 0;
        let stage_mat = match stage.stage_type {
            crate::vab::StageType::UpperStage => mat_upper.clone(),
            crate::vab::StageType::Booster => mat_lower.clone(),
            crate::vab::StageType::HeavyBooster => mat_heavy.clone(),
        };
        let y = y_positions[i];

        let mut entity_cmds = commands.spawn((
            Mesh3d(meshes.add(Cylinder::new(stage.radius, stage.height))),
            MeshMaterial3d(stage_mat),
            Transform::from_xyz(y, 0.0, 0.0).with_rotation(equator_rot),
            RigidBody::Dynamic,
            GravityScale(0.0),
            Collider::cylinder(stage.height / 2.0, stage.radius),
            ColliderMassProperties::Mass(stage.dry_mass + stage.fuel_mass),
            ExternalForce::default(),
            Velocity::default(),
            FuelTank {
                fuel_mass: stage.fuel_mass,
                dry_mass: stage.dry_mass,
            },
            Engine {
                max_thrust: stage.max_thrust,
                fuel_burn_rate: stage.fuel_burn_rate,
                stage: i,
            },
            StageMarker(i),
            FlightEntity,
        ));

        if is_top {
            entity_cmds.insert((
                Rocket {
                    throttle: 0.0,
                    is_launched: false,
                    current_stage: n_stages - 1,
                    sas_mode: SasMode::default(),
                },
                FloatingOrigin,
            ));
            entity_cmds.with_child((
                Mesh3d(meshes.add(Cone { radius: stage.radius, height: stage.radius * NOSE_CONE_HEIGHT_FACTOR })),
                MeshMaterial3d(mat_nose.clone()),
                Transform::from_xyz(0.0, stage.height / 2.0 + stage.radius, 0.0),
                Collider::cone(stage.radius, stage.radius),
                ColliderMassProperties::Mass(1.0),
            ));
        }

        let nozzle_height = stage.radius * NOZZLE_HEIGHT_FACTOR;
        entity_cmds.with_child((
            Mesh3d(meshes.add(ConicalFrustum {
                radius_top: stage.radius * NOZZLE_RADIUS_TOP_FACTOR,
                radius_bottom: stage.radius * NOZZLE_RADIUS_BOTTOM_FACTOR,
                height: nozzle_height,
            })),
            MeshMaterial3d(mat_engine.clone()),
            Transform::from_xyz(0.0, -stage.height / 2.0 - nozzle_height / 2.0, 0.0),
            Collider::cylinder(nozzle_height / 2.0, stage.radius * NOZZLE_COLLIDER_RADIUS_FACTOR),
            ColliderMassProperties::Mass(1.0),
        ));

        let flame_height = stage.height;
        entity_cmds.with_child((
            Mesh3d(meshes.add(Cone { radius: stage.radius * EXHAUST_FLAME_RADIUS_FACTOR, height: flame_height })),
            MeshMaterial3d(mat_flame.clone()),
            Transform::from_xyz(0.0, -stage.height / 2.0 - nozzle_height - flame_height / 2.0, 0.0),
            Visibility::Hidden,
            ExhaustFlame(i),
        ));

        stage_entities.push(entity_cmds.id());
    }

    for i in 0..n_stages - 1 {
        let upper = stage_entities[i];
        let lower = stage_entities[i + 1];
        let lower_height = config.stages[i + 1].height;
        let lower_anchor_y = lower_height / 2.0;
        let meeting_y = y_positions[i + 1] + lower_anchor_y;
        let upper_anchor_y = meeting_y - y_positions[i];

        let joint = FixedJointBuilder::new()
            .local_anchor1(Vec3::new(0.0, upper_anchor_y, 0.0))
            .local_anchor2(Vec3::new(0.0, lower_anchor_y, 0.0));
        commands.entity(lower).insert(ImpulseJoint::new(upper, joint));
    }

    // Telemetry UI
    commands.spawn((
        Text::new("Telemetry Data..."),
        TextFont { font_size: 20.0, ..default() },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        TelemetryUI,
        FlightEntity,
    ));

    // Pause menu overlay
    commands.spawn((
        Node {
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            position_type: PositionType::Absolute,
            top: Val::Px(0.0),
            left: Val::Px(0.0),
            flex_direction: FlexDirection::Column,
            align_items: AlignItems::Center,
            justify_content: JustifyContent::Center,
            row_gap: Val::Px(16.0),
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)),
        Visibility::Hidden,
        PauseMenuUI,
        FlightEntity,
    )).with_children(|parent| {
        parent.spawn((
            Text::new("PAUSED"),
            TextFont { font_size: 48.0, ..default() },
            TextColor(Color::WHITE),
        ));
        parent.spawn((
            Text::new("[ESC] Resume\n[1] Flight View\n[2] Main Menu\n[3] Quit"),
            TextFont { font_size: 24.0, ..default() },
            TextColor(Color::srgb(0.8, 0.8, 0.8)),
        ));
    });

    // Navball
    crate::navball::spawn_navball(&mut commands, &mut images);
}

pub fn cleanup_game(
    mut commands: Commands,
    game_entities: Query<Entity, With<FlightEntity>>,
    mut time_warp: ResMut<crate::TimeWarp>,
    mut time: ResMut<Time<Virtual>>,
    mut maneuver: ResMut<crate::orbit::ManeuverNode>,
    mut offset: ResMut<crate::FloatingOriginOffset>,
    mut transform_q: Query<&mut Transform>,
) {
    // Reset floating origin: shift everything back to true world coordinates
    if offset.0.length() > 0.0 {
        let shift = offset.0;
        for mut tf in transform_q.iter_mut() {
            tf.translation += shift;
        }
        offset.0 = Vec3::ZERO;
    }

    for entity in game_entities.iter() {
        commands.entity(entity).despawn_recursive();
    }
    time_warp.index = 0;
    time.set_relative_speed(1.0);
    time.unpause();
    *maneuver = crate::orbit::ManeuverNode::default();
}

// ===== Flight Systems =====

pub fn rocket_flight_system(
    real_time: Res<bevy::time::Time<bevy::time::Real>>,
    keys: Res<ButtonInput<KeyCode>>,
    time_warp: Res<crate::TimeWarp>,
    mut commands: Commands,
    planet_q: Query<(&CelestialBody, &Transform), Without<Rocket>>,
    mut rocket_q: Query<(Entity, &mut Rocket, &Transform, &mut ExternalForce, &Velocity, &mut FuelTank, &Engine, &mut ColliderMassProperties)>,
    mut part_q: Query<(Entity, &mut FuelTank, &Engine, &mut ExternalForce, &mut ColliderMassProperties, &Transform, &Velocity), Without<Rocket>>,
    mut flame_q: Query<(&mut Visibility, &ExhaustFlame)>,
    joint_q: Query<(Entity, &ImpulseJoint)>,
) {
    let real_dt = real_time.delta_secs();
    let warp_rate = time_warp.rate();

    let Ok((rocket_entity, mut rocket, rocket_tf, mut rocket_ext_force, rocket_vel, mut rocket_fuel, rocket_engine, mut rocket_mass_props)) = rocket_q.get_single_mut() else { return; };

    if warp_rate > 1.0 {
        rocket.throttle = 0.0;
    }

    // Launch / Stage logic
    if keys.just_pressed(KeyCode::Space) {
        if !rocket.is_launched {
            rocket.is_launched = true;
        } else if rocket.current_stage > 0 {
            let mut attached_entities: Vec<Entity> = Vec::new();
            let mut current = rocket_entity;
            loop {
                let mut found_next = None;
                for (joint_entity, joint) in joint_q.iter() {
                    if joint.parent == current && joint_entity != rocket_entity {
                        found_next = Some(joint_entity);
                        break;
                    }
                }
                match found_next {
                    Some(next) => {
                        attached_entities.push(next);
                        current = next;
                    }
                    None => break,
                }
            }
            if let Some(bottom) = attached_entities.last() {
                commands.entity(*bottom).remove::<ImpulseJoint>();
                rocket.current_stage -= 1;
            }
        }
    }

    if !rocket.is_launched { return; }

    if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
        rocket.throttle += THROTTLE_CHANGE_RATE * real_dt;
    }
    if keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight) {
        rocket.throttle -= THROTTLE_CHANGE_RATE * real_dt;
    }
    if keys.just_pressed(KeyCode::KeyZ) { rocket.throttle = 1.0; }
    if keys.just_pressed(KeyCode::KeyX) { rocket.throttle = 0.0; }
    rocket.throttle = rocket.throttle.clamp(0.0, 1.0);

    // SAS mode toggle
    if keys.just_pressed(KeyCode::KeyT) {
        rocket.sas_mode = match rocket.sas_mode {
            SasMode::Stability => SasMode::Prograde,
            SasMode::Prograde => SasMode::Retrograde,
            SasMode::Retrograde => SasMode::Stability,
        };
    }

    let mut target_torque = Vec3::ZERO;
    let mut manual_input = false;

    if keys.pressed(KeyCode::KeyW) { target_torque.x += ROCKET_TORQUE; manual_input = true; }
    if keys.pressed(KeyCode::KeyS) { target_torque.x -= ROCKET_TORQUE; manual_input = true; }
    if keys.pressed(KeyCode::KeyA) { target_torque.z += ROCKET_TORQUE; manual_input = true; }
    if keys.pressed(KeyCode::KeyD) { target_torque.z -= ROCKET_TORQUE; manual_input = true; }
    if keys.pressed(KeyCode::KeyQ) { target_torque.y += ROCKET_TORQUE; manual_input = true; }
    if keys.pressed(KeyCode::KeyE) { target_torque.y -= ROCKET_TORQUE; manual_input = true; }

    // Compute orbital velocity for SAS alignment
    let (soi_body, soi_tf) = crate::orbit::find_soi_body(rocket_tf.translation, planet_q.iter(), false);

    let world_torque = if manual_input {
        rocket_tf.rotation * target_torque
    } else {
        match rocket.sas_mode {
            SasMode::Stability => -rocket_vel.angvel * SAS_DAMPING,
            SasMode::Prograde | SasMode::Retrograde => {
                let rel_pos = rocket_tf.translation - soi_tf.translation;
                let soi_vel = crate::orbit::soi_body_velocity(soi_body, soi_tf);
                let orbital_vel = rocket_vel.linvel
                    + crate::orbit::surface_rotation_velocity(soi_body, rel_pos)
                    - soi_vel;
                let speed = orbital_vel.length();
                if speed < 1.0 {
                    -rocket_vel.angvel * SAS_DAMPING
                } else {
                    let desired_forward = if rocket.sas_mode == SasMode::Prograde {
                        orbital_vel.normalize()
                    } else {
                        -orbital_vel.normalize()
                    };
                    // Align rocket's up() to desired_forward
                    let current_up = rocket_tf.up();
                    let cross = current_up.cross(desired_forward);
                    let dot = current_up.dot(desired_forward);
                    let correction = cross * SAS_DAMPING * 2.0
                        + (-rocket_vel.angvel) * SAS_DAMPING;
                    if dot < -0.9 {
                        // Nearly opposite — use a kick to break symmetry
                        correction + rocket_tf.right() * ROCKET_TORQUE * 0.3
                    } else {
                        correction
                    }
                }
            }
        }
    };

    rocket_ext_force.torque = world_torque;

    // N-body gravity: collect all planet positions so the closure can iterate them
    let planet_gravity: Vec<(f32, Vec3)> = planet_q.iter()
        .map(|(b, tf)| (b.mu, tf.translation))
        .collect();

    let compute_gravity_and_drag = |pos: Vec3, mass: f32, vel: &Velocity| -> Vec3 {
        let mut force = Vec3::ZERO;
        // Gravity from all celestial bodies (not just SOI)
        for (mu, body_pos) in &planet_gravity {
            if *mu == 0.0 { continue; }
            let to_body = *body_pos - pos;
            let dist_sq = to_body.length_squared();
            if dist_sq > MIN_DIST_SQ_FOR_GRAVITY {
                let dist = dist_sq.sqrt();
                force += to_body * (*mu * mass / (dist_sq * dist));
            }
        }
        // Atmospheric drag from SOI body only
        let to_soi = soi_tf.translation - pos;
        let soi_dist = to_soi.length();
        let altitude = soi_dist - soi_body.radius;
        if altitude < soi_body.atmosphere_height && soi_body.atmosphere_height > 0.0 {
            let scale_height = soi_body.atmosphere_height / ATMOSPHERE_SCALE_HEIGHT_DIVISOR;
            let density = (-altitude / scale_height).exp();
            let velocity_sq = vel.linvel.length_squared();
            if velocity_sq > MIN_VELOCITY_SQ_FOR_DRAG {
                let drag_mag = DRAG_COEFFICIENT * density * velocity_sq * 0.5;
                force += -vel.linvel.normalize() * drag_mag;
            }
        }
        force
    };

    {
        let mut total_force = Vec3::ZERO;

        if rocket_engine.stage == rocket.current_stage && rocket.throttle > 0.0 && rocket_fuel.fuel_mass > 0.0 {
            let virtual_dt = real_dt * warp_rate;
            let burnt = rocket_engine.fuel_burn_rate * rocket.throttle * virtual_dt;
            rocket_fuel.fuel_mass = (rocket_fuel.fuel_mass - burnt).max(0.0);
            *rocket_mass_props = ColliderMassProperties::Mass(rocket_fuel.dry_mass + rocket_fuel.fuel_mass);
            total_force += rocket_tf.up() * rocket.throttle * rocket_engine.max_thrust;
        }

        let body_mass = if let ColliderMassProperties::Mass(m) = *rocket_mass_props { m } else { 1.0 };
        total_force += compute_gravity_and_drag(rocket_tf.translation, body_mass, rocket_vel);

        rocket_ext_force.force = total_force;
    }

    for (_entity, mut fuel, engine, mut ext_force, mut mass_props, transform, velocity) in part_q.iter_mut() {
        let mut total_force = Vec3::ZERO;

        if engine.stage == rocket.current_stage && rocket.throttle > 0.0 && fuel.fuel_mass > 0.0 {
            let virtual_dt = real_dt * warp_rate;
            let burnt = engine.fuel_burn_rate * rocket.throttle * virtual_dt;
            fuel.fuel_mass = (fuel.fuel_mass - burnt).max(0.0);
            *mass_props = ColliderMassProperties::Mass(fuel.dry_mass + fuel.fuel_mass);
            total_force += transform.up() * rocket.throttle * engine.max_thrust;
        }

        let body_mass = if let ColliderMassProperties::Mass(m) = *mass_props { m } else { 1.0 };
        total_force += compute_gravity_and_drag(transform.translation, body_mass, &velocity);

        if ext_force.force.distance_squared(total_force) > FORCE_UPDATE_THRESHOLD_SQ {
            ext_force.force = total_force;
        }
    }

    for (mut vis, flame) in flame_q.iter_mut() {
        let stage = flame.0;
        let is_active_stage = stage == rocket.current_stage;
        let has_fuel = if rocket_engine.stage == stage {
            rocket_fuel.fuel_mass > 0.0
        } else {
            true
        };
        *vis = if is_active_stage && rocket.throttle > 0.0 && has_fuel {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

pub fn telemetry_system(
    planet_q: Query<(&CelestialBody, &Transform), Without<Rocket>>,
    rocket_q: Query<(&Rocket, &Transform, &Velocity)>,
    part_q: Query<(&FuelTank, &Engine)>,
    mut text_q: Query<&mut Text, With<TelemetryUI>>,
    time_warp: Res<crate::TimeWarp>,
    time: Res<Time<Virtual>>,
    maneuver: Res<crate::orbit::ManeuverNode>,
) {
    let Ok((rocket, transform, velocity)) = rocket_q.get_single() else { return; };
    let Ok(mut text) = text_q.get_single_mut() else { return; };

    let mut current_fuel = 0.0;
    let mut current_thrust = 0.0;
    for (tank, engine) in part_q.iter() {
        if engine.stage == rocket.current_stage {
            current_fuel += tank.fuel_mass;
            if tank.fuel_mass > 0.0 {
                current_thrust += rocket.throttle * engine.max_thrust;
            }
        }
    }

    let (planet, p_transform) = crate::orbit::find_soi_body(transform.translation, planet_q.iter(), false);

    let planet_center = p_transform.translation;
    let to_planet = transform.translation - planet_center;
    let dist = to_planet.length();
    let local_up = if dist > 0.0 { to_planet / dist } else { Vec3::Y };
    let altitude = dist - planet.radius;

    let rel_pos = transform.translation - planet_center;
    let soi_vel = crate::orbit::soi_body_velocity(planet, p_transform);
    let inertial_vel = velocity.linvel + crate::orbit::surface_rotation_velocity(planet, rel_pos) - soi_vel;
    let orbital_vel_mag = inertial_vel.length();

    let pitch_deg = transform.up().dot(local_up).asin().to_degrees();
    let vertical_vel = velocity.linvel.dot(local_up);
    let surface_vel = (velocity.linvel - local_up * vertical_vel).length();

    let density = if altitude < planet.atmosphere_height && planet.atmosphere_height > 0.0 {
        let scale_height = planet.atmosphere_height / ATMOSPHERE_SCALE_HEIGHT_DIVISOR;
        (-altitude / scale_height).exp()
    } else {
        0.0
    };

    let ap_pe_str = {
        let h = rel_pos.cross(inertial_vel);
        let h_mag = h.length();
        let e_vec = if h_mag > 0.001 { (inertial_vel.cross(h) / planet.mu) - rel_pos.normalize() } else { Vec3::ZERO };
        let e = e_vec.length();
        let energy = inertial_vel.length_squared() / 2.0 - planet.mu / rel_pos.length();
        let a = if e < 1.0 && energy.abs() > 0.001 { -planet.mu / (2.0 * energy) } else { 0.0 };
        if e > 0.001 && e < 1.0 && a > 0.0 {
            let r_ap = a * (1.0 + e);
            let r_pe = a * (1.0 - e);
            let ap_alt = r_ap - planet.radius;
            let pe_alt = r_pe - planet.radius;
            format!("\nAp: {:.0} m\nPe: {:.0} m", ap_alt, pe_alt)
        } else {
            "".to_string()
        }
    };

    let warp_str = if time_warp.rate() > 1.0 {
        format!(" (Warp x{:.0})", time_warp.rate())
    } else {
        "".to_string()
    };

    let maneuver_str = if maneuver.is_active() {
        let dv = f32::sqrt(maneuver.prograde * maneuver.prograde + maneuver.normal * maneuver.normal + maneuver.radial * maneuver.radial);
        let time_to_node = (maneuver.ut - time.elapsed_secs_f64()).max(0.0);
        let t_min = (time_to_node / 60.0).floor();
        let t_sec = time_to_node % 60.0;
        format!(
            "\n--- Maneuver ---\nT-: {:.0}m {:.0}s\ndV: {:.0} m/s\n  Prograde: {:.0}\n  Normal: {:.0}\n  Radial: {:.0}",
            t_min, t_sec, dv, maneuver.prograde, maneuver.normal, maneuver.radial,
        )
    } else {
        "".to_string()
    };

    let sas_str = match rocket.sas_mode {
        SasMode::Stability => "SAS: Stability".to_string(),
        SasMode::Prograde => "SAS: Prograde".to_string(),
        SasMode::Retrograde => "SAS: Retrograde".to_string(),
    };

    text.0 = format!(
        "SOI: {}\nStage: {}\nAltitude: {:.1} m\nPitch (vs Horizon): {:.1} deg\nOrbital Vel: {:.1} m/s\nSurface Vel: {:.1} m/s\nV.Speed: {:.1} m/s\nThrottle: {:.0}%{}\nThrust: {:.0} N\nFuel: {:.0} kg\nAir Density: {:.3}{}\n{}{}",
        planet.name,
        rocket.current_stage,
        altitude,
        pitch_deg,
        orbital_vel_mag,
        surface_vel,
        vertical_vel,
        rocket.throttle * 100.0,
        warp_str,
        current_thrust,
        current_fuel,
        density,
        sas_str,
        ap_pe_str,
        maneuver_str,
    );
}

pub fn time_warp_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut time_warp: ResMut<crate::TimeWarp>,
    mut time: ResMut<Time<Virtual>>,
    rocket_q: Query<&Transform, With<Rocket>>,
    planet_q: Query<(&CelestialBody, &Transform), Without<Rocket>>,
) {
    if let Ok(rocket_tf) = rocket_q.get_single() {
        let (body, body_tf) = crate::orbit::find_soi_body(rocket_tf.translation, planet_q.iter(), false);
        let altitude = (rocket_tf.translation - body_tf.translation).length() - body.radius;
        if altitude < body.atmosphere_height && body.atmosphere_height > 0.0 {
            if time_warp.index > 0 {
                time_warp.index = 0;
                time.set_relative_speed(1.0);
            }
            return;
        }
    }

    let mut changed = false;
    if keys.just_pressed(KeyCode::Period) {
        if time_warp.index < time_warp.rates.len() - 1 {
            time_warp.index += 1;
            changed = true;
        }
    }
    if keys.just_pressed(KeyCode::Comma) {
        if time_warp.index > 0 {
            time_warp.index -= 1;
            changed = true;
        }
    }
    if changed {
        time.set_relative_speed(time_warp.rate());
    }
}

pub fn celestial_orbit_system(
    time: Res<Time<Virtual>>,
    mut orbit_q: Query<(&mut OrbitAngle, &OrbitParent, &CelestialBody, &mut Transform)>,
    parent_q: Query<&Transform, (With<CelestialBody>, Without<OrbitAngle>)>,
) {
    let dt = time.delta_secs();
    for (mut angle, parent, body, mut transform) in orbit_q.iter_mut() {
        if body.orbit_radius > 0.0 && body.orbit_speed > 0.0 {
            angle.0 += body.orbit_speed * dt;
            let parent_pos = parent_q.get(parent.0)
                .map(|t| t.translation)
                .unwrap_or(Vec3::ZERO);
            transform.translation = parent_pos + Vec3::new(
                body.orbit_radius * angle.0.cos(),
                0.0,
                body.orbit_radius * angle.0.sin(),
            );
        }
    }
}

pub fn pause_menu_system(
    keys: Res<ButtonInput<KeyCode>>,
    state: Res<State<AppState>>,
    mut next_state: ResMut<NextState<AppState>>,
    mut time: ResMut<Time<Virtual>>,
    mut menu_q: Query<&mut Visibility, With<PauseMenuUI>>,
    mut exit_events: EventWriter<AppExit>,
    mut pre_pause: ResMut<PrePauseView>,
) {
    let Ok(mut menu_vis) = menu_q.get_single_mut() else { return };

    match state.get() {
        AppState::Paused => {
            if keys.just_pressed(KeyCode::Escape) {
                time.unpause();
                *menu_vis = Visibility::Hidden;
                next_state.set(pre_pause.0);
            }
            if keys.just_pressed(KeyCode::Digit1) {
                time.unpause();
                *menu_vis = Visibility::Hidden;
                next_state.set(AppState::Flight);
            }
            if keys.just_pressed(KeyCode::Digit2) {
                time.unpause();
                *menu_vis = Visibility::Hidden;
                next_state.set(AppState::MainMenu);
            }
            if keys.just_pressed(KeyCode::Digit3) {
                exit_events.send(AppExit::Success);
            }
        }
        AppState::Flight | AppState::MapView => {
            if keys.just_pressed(KeyCode::Escape) {
                pre_pause.0 = *state.get();
                time.pause();
                *menu_vis = Visibility::Visible;
                next_state.set(AppState::Paused);
            }
        }
        _ => {}
    }
}

pub fn map_view_toggle_system(
    keys: Res<ButtonInput<KeyCode>>,
    state: Res<State<AppState>>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    if keys.just_pressed(KeyCode::KeyM) {
        match state.get() {
            AppState::Flight => next_state.set(AppState::MapView),
            AppState::MapView => next_state.set(AppState::Flight),
            _ => {}
        }
    }
}

// ===== Debug Orbit Spawn =====

pub fn debug_orbit_apply_system(
    mut commands: Commands,
    launched: Option<ResMut<DebugLaunched>>,
    planet_q: Query<(&CelestialBody, &Transform), Without<FlightEntity>>,
    mut transforms: ParamSet<(
        Query<&Transform, (With<Rocket>, With<FlightEntity>)>,
        Query<&mut Transform, With<FlightEntity>>,
    )>,
    mut rocket_mut: Query<&mut Rocket>,
    mut vel_q: Query<&mut Velocity, With<FlightEntity>>,
) {
    if launched.is_none() { return; }

    let rocket_pos = match transforms.p0().get_single() {
        Ok(tf) => tf.translation,
        Err(_) => return,
    };

    let (mun_body, mun_tf) = match planet_q.iter().find(|(b, _)| b.name == "Mun") {
        Some(pair) => pair,
        None => return,
    };

    // Circular orbit around Mun at DEBUG_ORBIT_ALTITUDE above surface
    // mu_mun = g × r² = 1.0 × 200² = 40,000
    // orbit_r = MUN_RADIUS + altitude = 300
    // v_circular = sqrt(40,000 / 300) ≈ 11.55 m/s
    let orbit_r = mun_body.radius + DEBUG_ORBIT_ALTITUDE;
    let v_circular = (mun_body.mu / orbit_r).sqrt();

    // Mun's orbital velocity in world frame
    // At t=0 Mun is at (14000,0,0), angular_speed ≈ 0.0027 rad/s
    // Mun velocity = (0, 0, ~37.8) m/s
    let mun_vel = crate::orbit::soi_body_velocity(mun_body, mun_tf);

    // Place rocket on the Mun-far-from-Kerbin side, radial outward
    // At t=0: mun_dir = (1,0,0), orbit_pos = (14300, 0, 0)
    let mun_dir = mun_tf.translation.normalize_or(Vec3::X);
    let orbit_pos = mun_tf.translation + mun_dir * orbit_r;

    // Prograde: tangent to counterclockwise circular orbit
    // For radial (cos θ, 0, sin θ), prograde = (-sin θ, 0, cos θ)
    // At t=0: prograde = (0, 0, 1)
    let prograde = Vec3::new(-mun_dir.z, 0.0, mun_dir.x);
    let orbit_vel = mun_vel + prograde * v_circular;

    let offset = orbit_pos - rocket_pos;

    for mut tf in transforms.p1().iter_mut() {
        tf.translation += offset;
    }

    for mut vel in vel_q.iter_mut() {
        vel.linvel = orbit_vel;
    }

    for mut rocket in rocket_mut.iter_mut() {
        rocket.is_launched = true;
    }

    commands.remove_resource::<DebugLaunched>();
}
