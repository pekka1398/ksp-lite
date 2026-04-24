use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::{AppState, CelestialBody, Rocket, OrbitCamera, StageMarker, ExhaustFlame, FuelTank, Engine, FloatingOrigin, OrbitAngle, OrbitParent, OrbitInclination, SasMode};
use crate::vab::RocketConfig;
use crate::constants::*;

// ===== Components =====

#[derive(Component)]
pub struct FlightEntity;

#[derive(Component)]
pub struct MapIcon {
    pub target: MapIconTarget,
}

#[derive(Clone)]
pub enum MapIconTarget {
    Body(Entity),
    Rocket,
}

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
    offset: Res<crate::sim::LocalOffset>,
    config: Res<RocketConfig>,
    mut camera_q: Query<&mut OrbitCamera>,
    rocket_q: Query<(), With<Rocket>>,
    planet_q: Query<(Entity, &CelestialBody)>,
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
        let local_pos = Vec3::new(y, 0.0, 0.0);

        let entity = commands.spawn((
            Mesh3d(meshes.add(Cylinder::new(stage.radius, stage.height))),
            MeshMaterial3d(stage_mat),
            Transform::from_translation(local_pos).with_rotation(equator_rot),
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
        )).insert(crate::sim::SimState {
            position: offset.ssb_from_local(local_pos),
            velocity: crate::sim::SsbVelocity(crate::sim::DVec3::ZERO),
        }).id();

        let mut entity_cmds = commands.entity(entity);

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

    // Map icons (hidden by default, shown in map view)
    // Spawned as FlightEntity so they get cleaned up
    let icon_size = 16.0;
    for (entity, body) in planet_q.iter() {
        let color = match body.name.as_str() {
            "Kerbin" => Color::srgb(0.2, 0.6, 0.9),
            "Mun" => Color::srgb(0.6, 0.6, 0.6),
            "Minmus" => Color::srgb(0.4, 0.8, 0.5),
            "Sun" => Color::srgb(1.0, 0.9, 0.3),
            _ => Color::srgb(0.7, 0.7, 0.7),
        };
        commands.spawn((
            Node {
                width: Val::Px(icon_size),
                height: Val::Px(icon_size),
                position_type: PositionType::Absolute,
                ..default()
            },
            BackgroundColor(color.with_alpha(0.8)),
            BorderRadius::all(Val::Percent(50.0)),
            Visibility::Hidden,
            MapIcon { target: MapIconTarget::Body(entity) },
            FlightEntity,
        ));
    }
    // Rocket icon
    commands.spawn((
        Node {
            width: Val::Px(icon_size),
            height: Val::Px(icon_size),
            position_type: PositionType::Absolute,
            ..default()
        },
        BackgroundColor(Color::srgb(0.0, 1.0, 0.0).with_alpha(0.8)),
        BorderRadius::all(Val::Percent(50.0)),
        Visibility::Hidden,
        MapIcon { target: MapIconTarget::Rocket },
        FlightEntity,
    ));
}

pub fn cleanup_game(
    mut commands: Commands,
    game_entities: Query<Entity, With<FlightEntity>>,
    mut time_warp: ResMut<crate::TimeWarp>,
    mut time: ResMut<Time<Virtual>>,
    mut maneuver: ResMut<crate::orbit::ManeuverNode>,
    mut offset: ResMut<crate::sim::LocalOffset>,
    mut transform_q: Query<&mut Transform>,
) {
    // Reset floating origin: shift everything back to true world coordinates
    if offset.0.length() > 0.0 {
        let shift = offset.0.as_vec3();
        for mut tf in transform_q.iter_mut() {
            tf.translation += shift;
        }
        offset.0 = crate::sim::DVec3::ZERO;
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
    offset: Res<crate::sim::LocalOffset>,
    mut commands: Commands,
    planet_q: Query<(&CelestialBody, &crate::sim::SimState), Without<Rocket>>,
    mut rocket_q: Query<(Entity, &mut Rocket, &Transform, &mut ExternalForce, &Velocity, &mut FuelTank, &Engine, &mut ColliderMassProperties, &crate::sim::SimState)>,
    mut part_q: Query<(Entity, &mut FuelTank, &Engine, &mut ExternalForce, &mut ColliderMassProperties, &Transform, &Velocity), Without<Rocket>>,
    mut flame_q: Query<(&mut Visibility, &ExhaustFlame)>,
    joint_q: Query<(Entity, &ImpulseJoint)>,
) {
    let real_dt = real_time.delta_secs();
    let warp_rate = time_warp.rate();

    let Ok((rocket_entity, mut rocket, rocket_tf, mut rocket_ext_force, rocket_vel, mut rocket_fuel, rocket_engine, mut rocket_mass_props, rocket_sim)) = rocket_q.get_single_mut() else { return; };

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
    let (soi_body, soi_sim) = crate::orbit::find_soi_body(rocket_sim.position, planet_q.iter(), false);

    let world_torque = if manual_input {
        rocket_tf.rotation * target_torque
    } else {
        match rocket.sas_mode {
            SasMode::Stability => -rocket_vel.angvel * SAS_DAMPING,
            SasMode::Prograde | SasMode::Retrograde => {
                let orbital_vel = rocket_sim.velocity.0 - soi_sim.velocity.0;
                let orbital_vel_f32 = orbital_vel.as_vec3();
                let speed = orbital_vel_f32.length();
                if speed < 1.0 {
                    -rocket_vel.angvel * SAS_DAMPING
                } else {
                    let desired_forward = if rocket.sas_mode == SasMode::Prograde {
                        orbital_vel_f32.normalize()
                    } else {
                        -orbital_vel_f32.normalize()
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

    // N-body gravity: collect all planet positions (local frame) so the closure can iterate them
    // SSB-to-local conversion for each body position.
    let planet_gravity: Vec<(f32, Vec3)> = planet_q.iter()
        .map(|(b, sim)| (b.mu, (sim.position.0 - offset.0).as_vec3()))
        .collect();
    let soi_local = (soi_sim.position.0 - offset.0).as_vec3();

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
        let to_soi = soi_local - pos;
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
    _offset: Res<crate::sim::LocalOffset>,
    planet_q: Query<(&CelestialBody, &crate::sim::SimState), Without<Rocket>>,
    rocket_q: Query<(&Rocket, &Transform, &Velocity, &crate::sim::SimState)>,
    part_q: Query<(&FuelTank, &Engine)>,
    mut text_q: Query<&mut Text, With<TelemetryUI>>,
    time_warp: Res<crate::TimeWarp>,
    time: Res<Time<Virtual>>,
    maneuver: Res<crate::orbit::ManeuverNode>,
) {
    let Ok((rocket, transform, velocity, rocket_sim)) = rocket_q.get_single() else { return; };
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

    let (planet, planet_sim) = crate::orbit::find_soi_body(rocket_sim.position, planet_q.iter(), false);

    let rel_pos_d = rocket_sim.position.0 - planet_sim.position.0;
    let rel_pos = rel_pos_d.as_vec3();
    let dist = rel_pos.length();
    let local_up = if dist > 0.0 { rel_pos / dist } else { Vec3::Y };
    let altitude = dist - planet.radius;

    // Orbital velocity = rocket SSB vel - body SSB vel (inertial frame)
    let orbital_vel_d = rocket_sim.velocity.0 - planet_sim.velocity.0;
    let orbital_vel = orbital_vel_d.as_vec3();
    let orbital_vel_mag = orbital_vel.length();

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
        let mu_d = planet.mu as f64;
        match crate::orbit::OrbitalElements::from_state(rel_pos_d, orbital_vel_d, mu_d) {
            Some(elems) if elems.eccentricity > 0.001 && elems.eccentricity < 1.0 && elems.semi_major_axis > 0.0 => {
                let r_ap = elems.apoapsis_radius.unwrap();
                let r_pe = elems.periapsis_radius.unwrap();
                let ap_alt = (r_ap - planet.radius as f64) as f32;
                let pe_alt = (r_pe - planet.radius as f64) as f32;
                let period = elems.period.unwrap();
                let p_min = (period / 60.0).floor() as i32;
                let p_sec = (period % 60.0).round() as i32;
                format!("\nAp: {:.0} m\nPe: {:.0} m\nT: {}m {}s\nInc: {:.1}°", ap_alt, pe_alt, p_min, p_sec, elems.inclination_deg)
            }
            _ => "".to_string()
        }
    };

    // Landing detection
    let landed_str = if altitude < LANDED_ALTITUDE_THRESHOLD && vertical_vel.abs() < 2.0 {
        let speed_label = if vertical_vel.abs() < LANDING_SAFE_SPEED {
            "SOFT"
        } else {
            "HARD"
        };
        format!("\n=== LANDED ({}) ===\nV-Speed: {:.1} m/s", speed_label, vertical_vel)
    } else {
        "".to_string()
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

    // Mission elapsed time
    let t_secs = time.elapsed_secs();
    let t_min = (t_secs / 60.0).floor() as i32;
    let t_sec = (t_secs % 60.0).round() as i32;
    let met_str = format!("T+ {}:{:02}", t_min, t_sec);

    text.0 = format!(
        "SOI: {} | {}\nStage: {}\nAltitude: {:.1} m\nPitch (vs Horizon): {:.1} deg\nOrbital Vel: {:.1} m/s\nSurface Vel: {:.1} m/s\nV.Speed: {:.1} m/s\nThrottle: {:.0}%{}\nThrust: {:.0} N\nFuel: {:.0} kg\nAir Density: {:.3}{}\n{}{}{}",
        planet.name,
        met_str,
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
        landed_str,
        maneuver_str,
    );
}

pub fn time_warp_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut time_warp: ResMut<crate::TimeWarp>,
    mut time: ResMut<Time<Virtual>>,
    rocket_q: Query<&crate::sim::SimState, With<Rocket>>,
    planet_q: Query<(&CelestialBody, &crate::sim::SimState), Without<Rocket>>,
) {
    if let Ok(rocket_sim) = rocket_q.get_single() {
        let (body, body_sim) = crate::orbit::find_soi_body(rocket_sim.position, planet_q.iter(), false);
        let altitude = (rocket_sim.position.0 - body_sim.position.0).length() as f32 - body.radius;
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
    offset: Res<crate::sim::LocalOffset>,
    mut orbit_q: Query<(&mut OrbitAngle, &OrbitParent, &CelestialBody, &mut crate::sim::SimState, &mut Transform, Option<&OrbitInclination>)>,
    parent_q: Query<&crate::sim::SimState, (With<CelestialBody>, Without<OrbitAngle>)>,
) {
    let dt = time.delta_secs();
    for (mut angle, parent, body, mut sim, mut transform, inclination) in orbit_q.iter_mut() {
        if body.orbit_radius > 0.0 && body.orbit_speed > 0.0 {
            angle.0 += body.orbit_speed * dt;

            // Get parent SSB state
            let parent_ssb_pos = parent_q.get(parent.0)
                .map(|s| s.position.0)
                .unwrap_or(crate::sim::DVec3::ZERO);
            let parent_ssb_vel = parent_q.get(parent.0)
                .map(|s| s.velocity.0)
                .unwrap_or(crate::sim::DVec3::ZERO);

            let orbit_r = body.orbit_radius as f64;
            let a = angle.0 as f64;
            let w = body.orbit_speed as f64;
            let inc = inclination.map(|i| i.0).unwrap_or(0.0) as f64;

            // Position in orbital plane
            let px = orbit_r * a.cos();
            let pz = orbit_r * a.sin();

            // Velocity in orbital plane (d/dt)
            let vx = -orbit_r * a.sin() * w;
            let vz = orbit_r * a.cos() * w;

            // Apply inclination (rotation around X axis)
            let (pos, vel) = if inc != 0.0 {
                let ci = inc.cos();
                let si = inc.sin();
                (
                    crate::sim::DVec3::new(px, pz * si, pz * ci),
                    crate::sim::DVec3::new(vx, vz * si, vz * ci),
                )
            } else {
                (crate::sim::DVec3::new(px, 0.0, pz), crate::sim::DVec3::new(vx, 0.0, vz))
            };

            // Update SimState (truth)
            sim.position = crate::sim::SsbPosition(parent_ssb_pos + pos);
            sim.velocity = crate::sim::SsbVelocity(parent_ssb_vel + vel);

            // Derive Transform from SSB + LocalOffset
            transform.translation = sim.position.to_local(&offset);
        }
    }
}

/// Read back Rapier physics results into SimState for dynamic bodies (rocket + parts).
/// Must run after Rapier writeback.
pub fn sim_state_readback_system(
    offset: Res<crate::sim::LocalOffset>,
    mut q: Query<(&Transform, &Velocity, &mut crate::sim::SimState), With<RigidBody>>,
) {
    for (tf, vel, mut sim) in q.iter_mut() {
        sim.position = offset.ssb_from_local(tf.translation);
        sim.velocity = offset.ssb_vel_from_local(vel.linvel);
    }
}

/// Debug: verify SimState ↔ Transform consistency every frame.
/// Catches conversion bugs, double-offset, missing offset, etc.
pub fn sim_invariant_check_system(
    offset: Res<crate::sim::LocalOffset>,
    body_q: Query<(&Transform, &crate::sim::SimState, &CelestialBody, Option<&OrbitAngle>), With<CelestialBody>>,
    rocket_q: Query<(&crate::sim::SimState, &Rocket), With<RigidBody>>,
    mut prev_energy: Local<Option<f64>>,
    mut warn_timer: Local<Timer>,
    time: Res<Time<Virtual>>,
) {
    if warn_timer.duration().as_secs_f32() == 0.0 {
        *warn_timer = Timer::from_seconds(2.0, TimerMode::Repeating);
    }
    let should_warn = warn_timer.tick(time.delta()).just_finished();

    // Check 1: For all entities, SimState.position → local should match Transform.translation
    for (tf, sim, ..) in body_q.iter() {
        let expected_local = sim.position.to_local(&offset);
        let err = (tf.translation - expected_local).length();
        if err > 0.1 && should_warn {
            warn!(
                "Sim invariant FAIL: body transform ({:.1},{:.1},{:.1}) != SSB→local ({:.1},{:.1},{:.1}) | err={:.2}",
                tf.translation.x, tf.translation.y, tf.translation.z,
                expected_local.x, expected_local.y, expected_local.z,
                err,
            );
        }
    }

    // Check 2: For orbiting bodies, velocity should be consistent with orbit parameters
    for (_, sim, body, angle) in body_q.iter() {
        let Some(_angle) = angle else { continue };
        if body.orbit_radius > 0.0 && body.orbit_speed > 0.0 {
            let expected_speed = body.orbit_speed * body.orbit_radius;
            let actual_speed = sim.velocity.0.length() as f32;
            let rel_err = (actual_speed - expected_speed).abs() / expected_speed;
            if rel_err > 0.01 && should_warn {
                warn!(
                    "Sim invariant FAIL: {} orbital speed {:.2} != expected {:.2} (orbit_speed×radius) | rel_err={:.4}",
                    body.name, actual_speed, expected_speed, rel_err,
                );
            }
        }
    }

    // Check 3: Kerbin should be at origin with zero velocity
    for (_, sim, body, _) in body_q.iter() {
        if body.name == "Kerbin" {
            if sim.position.0.length() > 0.01 && should_warn {
                warn!("Sim invariant FAIL: Kerbin SSB position != zero: ({:.1},{:.1},{:.1})",
                    sim.position.0.x, sim.position.0.y, sim.position.0.z);
            }
            if sim.velocity.0.length() > 0.01 && should_warn {
                warn!("Sim invariant FAIL: Kerbin SSB velocity != zero: ({:.4},{:.4},{:.4})",
                    sim.velocity.0.x, sim.velocity.0.y, sim.velocity.0.z);
            }
        }
    }

    // Check 4: Energy conservation for rocket (when coasting, no atmosphere)
    // Specific orbital energy: E = v²/2 - μ/r  (relative to SOI body)
    // Should be constant when no thrust and no drag.
    if let Ok((rocket_sim, rocket)) = rocket_q.get_single() {
        let (planet, _) = crate::orbit::find_soi_body(
            rocket_sim.position, body_q.iter().map(|(_, s, b, _)| (b, s)), false,
        );
        // Re-derive planet_sim from the query
        let planet_sim = body_q.iter()
            .find(|(_, _, b, _)| b.name == planet.name)
            .map(|(_, s, _, _)| s);

        if let Some(planet_sim) = planet_sim {
            let rel_pos = rocket_sim.position.0 - planet_sim.position.0;
            let rel_vel = rocket_sim.velocity.0 - planet_sim.velocity.0;
            let r = rel_pos.length();
            let v_sq = rel_vel.length_squared();

            if r > 1.0 && planet.mu > 0.0 {
                let energy = v_sq / 2.0 - planet.mu as f64 / r;

                // Only check when coasting (no thrust) and above atmosphere
                let altitude = r as f32 - planet.radius;
                let in_atm = altitude < planet.atmosphere_height && planet.atmosphere_height > 0.0;
                let coasting = rocket.throttle < 0.01 && !in_atm;

                if coasting && energy.abs() > 1.0 {
                    if let Some(prev_e) = *prev_energy {
                        let rel_err = (energy - prev_e).abs() / prev_e.abs().max(1.0);
                        // 5% tolerance — accounts for Rapier integration error + time warp
                        if rel_err > 0.05 && should_warn {
                            warn!(
                                "Sim invariant FAIL: energy drift | prev={:.2} now={:.2} | Δ={:.2} ({:.1}%) | alt={:.0} throttle={:.0}%",
                                prev_e, energy, energy - prev_e, rel_err * 100.0,
                                altitude, rocket.throttle * 100.0,
                            );
                        }
                    }
                    *prev_energy = Some(energy);
                } else {
                    // Reset tracking when thrusting or in atmosphere
                    *prev_energy = None;
                }
            }
        } else {
            *prev_energy = None;
        }
    } else {
        *prev_energy = None;
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

pub fn enter_map_view_system(
    mut camera_q: Query<&mut OrbitCamera>,
) {
    for mut orbit in camera_q.iter_mut() {
        orbit.distance = 8000.0;
    }
}

// ===== Map Icon Positioning =====

pub fn map_icon_system(
    state: Res<State<AppState>>,
    camera_q: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    windows: Query<&Window>,
    planet_q: Query<&Transform, With<CelestialBody>>,
    rocket_q: Query<&Transform, With<Rocket>>,
    mut icon_q: Query<(&MapIcon, &mut Node, &mut Visibility)>,
) {
    let is_map = *state.get() == AppState::MapView;
    let Ok(window) = windows.get_single() else { return };
    let Ok((camera, camera_gt)) = camera_q.get_single() else { return };
    let win_w = window.width();
    let win_h = window.height();

    for (icon, mut node, mut vis) in icon_q.iter_mut() {
        if !is_map {
            *vis = Visibility::Hidden;
            continue;
        }

        let world_pos = match &icon.target {
            MapIconTarget::Body(entity) => {
                match planet_q.get(*entity) {
                    Ok(tf) => tf.translation,
                    Err(_) => { *vis = Visibility::Hidden; continue; }
                }
            }
            MapIconTarget::Rocket => {
                match rocket_q.get_single() {
                    Ok(tf) => tf.translation,
                    Err(_) => { *vis = Visibility::Hidden; continue; }
                }
            }
        };

        let Ok(screen_pos) = camera.world_to_viewport(camera_gt, world_pos) else {
            *vis = Visibility::Hidden;
            continue;
        };

        // Check if on screen
        if screen_pos.x < -50.0 || screen_pos.x > win_w + 50.0
            || screen_pos.y < -50.0 || screen_pos.y > win_h + 50.0
        {
            *vis = Visibility::Hidden;
            continue;
        }

        *vis = Visibility::Visible;
        node.left = Val::Px(screen_pos.x - 8.0);
        node.top = Val::Px(screen_pos.y - 8.0);
    }
}

// ===== Debug Orbit Spawn =====

pub fn debug_orbit_apply_system(
    mut commands: Commands,
    launched: Option<ResMut<crate::DebugLaunched>>,
    local_offset: Res<crate::sim::LocalOffset>,
    planet_q: Query<(&CelestialBody, &crate::sim::SimState), Without<FlightEntity>>,
    mut transforms: ParamSet<(
        Query<&Transform, (With<Rocket>, With<FlightEntity>)>,
        Query<&mut Transform, With<FlightEntity>>,
    )>,
    mut sim_q: Query<&mut crate::sim::SimState, With<FlightEntity>>,
    mut rocket_mut: Query<&mut Rocket>,
    mut vel_q: Query<&mut Velocity, With<FlightEntity>>,
) {
    let Some(launched) = launched else { return };

    let rocket_pos = match transforms.p0().get_single() {
        Ok(tf) => tf.translation,
        Err(_) => return,
    };

    let (kerbin_body, kerbin_sim) = match planet_q.iter().find(|(b, _)| b.name == "Kerbin") {
        Some(pair) => pair,
        None => return,
    };

    let mu = kerbin_body.mu as f64;
    let r_kerbin = kerbin_body.radius as f64;

    // Orbit presets: (pe_alt, ap_alt) in meters above surface
    let (pe_alt, ap_alt) = match launched.0 {
        crate::DebugOrbitPreset::KerbinElliptical1 => {
            // Moderate ellipse: 200m × 2000m, T≈162s
            (200.0, 2000.0)
        }
        crate::DebugOrbitPreset::KerbinElliptical2 => {
            // Highly elliptical: 100m × 4000m, T≈246s
            (100.0, 4000.0)
        }
        crate::DebugOrbitPreset::KerbinElliptical3 => {
            // Near-circular: 500m × 800m, T≈131s
            (500.0, 800.0)
        }
    };

    let r_pe = r_kerbin + pe_alt as f64;
    let r_ap = r_kerbin + ap_alt as f64;
    let a = (r_pe + r_ap) / 2.0;

    // Vis-viva at periapsis: v = sqrt(mu * (2/r - 1/a))
    let v_pe = (mu * (2.0 / r_pe - 1.0 / a)).sqrt();

    // Place rocket at periapsis: radial direction from Kerbin center along +X
    let kerbin_pos = kerbin_sim.position.0;
    let pe_dir = crate::sim::DVec3::X; // periapsis along +X
    let orbit_ssb_pos = kerbin_pos + pe_dir * r_pe;

    // Velocity at periapsis: tangent (prograde = +Z for +X radial, CCW)
    let prograde = crate::sim::DVec3::Z;
    let orbit_ssb_vel = kerbin_sim.velocity.0 + prograde * v_pe;

    debug!(
        "debug_orbit: preset={:?} | pe_alt={:.0} ap_alt={:.0} | r_pe={:.1} r_ap={:.1} a={:.1} | v_pe={:.2} | mu={:.0}",
        launched.0, pe_alt, ap_alt, r_pe, r_ap, a, v_pe, mu,
    );

    // Teleport all flight entities
    let orbit_local_pos = (orbit_ssb_pos - local_offset.0).as_vec3();
    let offset_vec = orbit_local_pos - rocket_pos;

    for mut tf in transforms.p1().iter_mut() {
        tf.translation += offset_vec;
    }

    for mut vel in vel_q.iter_mut() {
        vel.linvel = orbit_ssb_vel.as_vec3();
    }

    for mut sim in sim_q.iter_mut() {
        sim.position = crate::sim::SsbPosition(orbit_ssb_pos);
        sim.velocity = crate::sim::SsbVelocity(orbit_ssb_vel);
    }

    for mut rocket in rocket_mut.iter_mut() {
        rocket.is_launched = true;
    }

    commands.remove_resource::<crate::DebugLaunched>();
}
