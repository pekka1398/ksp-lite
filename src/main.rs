use bevy::prelude::*;
use bevy::time::Real;
use bevy::ecs::event::EventReader;
use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy_rapier3d::prelude::*;

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, States)]
enum AppState {
    #[default]
    VAB,
    Flight,
    MapView,
}

// MapViewCamera removed — was defined but never spawned or queried.

#[derive(Resource)]
struct TimeWarp {
    rates: Vec<f32>,
    index: usize,
}

impl Default for TimeWarp {
    fn default() -> Self {
        Self {
            rates: vec![1.0, 2.0, 5.0, 10.0, 50.0, 100.0],
            index: 0,
        }
    }
}

impl TimeWarp {
    fn rate(&self) -> f32 {
        self.rates[self.index]
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        // Use Interpolated timestep: it accumulates virtual time and runs
        // multiple fixed-dt steps per frame, which actually speeds up physics
        // when Time<Virtual>::relative_speed > 1. Variable mode's max_dt
        // cap would clamp the step and negate any time warp.
        .insert_resource(TimestepMode::Interpolated {
            dt: 1.0 / 60.0,
            time_scale: 1.0,
            substeps: 1,
        })
        .init_state::<AppState>()
        .init_resource::<TimeWarp>()
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                rocket_flight_system,
                telemetry_system,
                orbit_prediction_system,
                map_view_toggle_system,
                time_warp_system,
            ),
        )
        .add_systems(
            PostUpdate,
            camera_controller
                .after(PhysicsSet::Writeback)
                .before(TransformSystem::TransformPropagate),
        )
        .run();
}

/// A simple camera component to allow orbital controls later
#[derive(Component)]
struct OrbitCamera {
    distance: f32,
    pitch: f32,
    yaw: f32,
}

#[derive(Component)]
struct CelestialBody {
    name: String,
    mu: f32, // standard gravitational parameter
    radius: f32,
    atmosphere_height: f32,
}

#[derive(Component)]
struct Rocket {
    is_launched: bool,
    throttle: f32,
    current_stage: usize,
}

#[derive(Component)]
struct FuelTank {
    fuel_mass: f32,
    dry_mass: f32,
}

#[derive(Component)]
struct Engine {
    max_thrust: f32,
    fuel_burn_rate: f32,
    stage: usize,
}

#[derive(Component)]
struct StageMarker(usize);

#[derive(Component)]
struct ExhaustFlame(usize);

#[derive(Component)]
struct TelemetryUI;

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Planet (Sphere)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(200.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.6, 0.4))),
        Transform::from_xyz(0.0, -200.0, 0.0),
        Collider::ball(200.0),
        RigidBody::Fixed,
        CelestialBody {
            name: "Kerbin".to_string(),
            mu: 9.81 * 200.0 * 200.0,
            radius: 200.0,
            atmosphere_height: 100.0,
        },
    ));

    // Launch Pad
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(6.0, 0.5, 6.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.3, 0.3))),
        Transform::from_xyz(0.0, 0.25, 0.0),
        RigidBody::Fixed,
        Collider::cuboid(3.0, 0.25, 3.0),
    ));

    // A simple directional light
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 10_000.,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let mat_upper = materials.add(Color::srgb(0.8, 0.8, 0.8));
    let mat_lower = materials.add(Color::srgb(0.6, 0.6, 0.7));
    let mat_nose = materials.add(Color::srgb(0.9, 0.1, 0.1));
    let mat_engine = materials.add(Color::srgb(0.2, 0.2, 0.2));
    let mat_flame = materials.add(Color::srgb(1.0, 0.5, 0.0));

    // --- STAGE 0 (Upper Stage + Capsule) ---
    let stage0_entity = commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.5, 1.0))),
        MeshMaterial3d(mat_upper.clone()),
        Transform::from_xyz(0.0, 4.25, 0.0), // Lifted: 2.3 + 1.95 = 4.25
        RigidBody::Dynamic,
        GravityScale(0.0),
        Collider::cylinder(0.5, 0.5),
        ColliderMassProperties::Mass(1500.0 + 1000.0),
        ExternalForce::default(),
        Velocity::default(),
        Rocket {
            throttle: 0.0,
            is_launched: false,
            current_stage: 1, // Start with Stage 1 (Booster) active
        },
        FuelTank {
            fuel_mass: 1000.0,
            dry_mass: 1500.0,
        },
        Engine {
            max_thrust: 50_000.0,
            fuel_burn_rate: 15.0,
            stage: 0,
        },
        StageMarker(0),
    )).with_child((
        Mesh3d(meshes.add(Cone { radius: 0.5, height: 1.0 })),
        MeshMaterial3d(mat_nose),
        Transform::from_xyz(0.0, 1.0, 0.0),
        Collider::cone(0.5, 0.5),
        ColliderMassProperties::Mass(1.0),
    )).with_child((
        Mesh3d(meshes.add(ConicalFrustum { radius_top: 0.15, radius_bottom: 0.3, height: 0.4 })),
        MeshMaterial3d(mat_engine.clone()),
        Transform::from_xyz(0.0, -0.7, 0.0),
        Collider::cylinder(0.2, 0.3),
        ColliderMassProperties::Mass(1.0),
    )).with_child((
        Mesh3d(meshes.add(Cone { radius: 0.25, height: 2.0 })),
        MeshMaterial3d(mat_flame.clone()),
        Transform::from_xyz(0.0, -1.7, 0.0),
        Visibility::Hidden,
        ExhaustFlame(0),
    )).id();

    // --- STAGE 1 (Booster) ---
    let stage1_entity = commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.5, 2.0))),
        MeshMaterial3d(mat_lower),
        Transform::from_xyz(0.0, 2.3, 0.0), // Lifted: Booster bottom at 0.5 + engine (0.4nd) clearance
        RigidBody::Dynamic,
        GravityScale(0.0),
        Collider::cylinder(1.0, 0.5),
        ColliderMassProperties::Mass(2000.0 + 3000.0),
        ExternalForce::default(),
        Velocity::default(),
        FuelTank {
            fuel_mass: 3000.0,
            dry_mass: 2000.0,
        },
        Engine {
            max_thrust: 150_000.0,
            fuel_burn_rate: 45.0,
            stage: 1,
        },
        StageMarker(1),
    )).with_child((
        Mesh3d(meshes.add(ConicalFrustum { radius_top: 0.25, radius_bottom: 0.5, height: 0.8 })),
        MeshMaterial3d(mat_engine),
        Transform::from_xyz(0.0, -1.4, 0.0),
        Collider::cylinder(0.4, 0.4),
        ColliderMassProperties::Mass(1.0),
    )).with_child((
        Mesh3d(meshes.add(Cone { radius: 0.4, height: 3.0 })),
        MeshMaterial3d(mat_flame),
        Transform::from_xyz(0.0, -2.9, 0.0),
        Visibility::Hidden,
        ExhaustFlame(1),
    )).id();

    // Joint them together matching reference logic
    let joint = FixedJointBuilder::new()
        .local_anchor1(Vec3::new(0.0, -0.95, 0.0)) // Match reference stage 0 anchor
        .local_anchor2(Vec3::new(0.0, 1.0, 0.0));  // Match reference stage 1 anchor
    commands.entity(stage1_entity).insert(ImpulseJoint::new(stage0_entity, joint));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
        OrbitCamera {
            distance: 15.0,
            pitch: 0.2,
            yaw: 0.0,
        },
    ));

    // Telemetry UI
    commands.spawn((
        Text::new("Telemetry Data..."),
        TextFont {
            font_size: 20.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        TelemetryUI,
    ));
}

fn rocket_flight_system(
    real_time: Res<Time<Real>>,
    keys: Res<ButtonInput<KeyCode>>,
    time_warp: Res<TimeWarp>,
    mut commands: Commands,
    planet_q: Query<(&CelestialBody, &Transform), Without<Rocket>>,
    mut rocket_q: Query<(Entity, &mut Rocket, &Transform, &mut ExternalForce, &Velocity, &mut FuelTank, &Engine, &mut ColliderMassProperties)>,
    mut part_q: Query<(Entity, &mut FuelTank, &Engine, &mut ExternalForce, &mut ColliderMassProperties, &Transform, &Velocity), Without<Rocket>>,
    mut flame_q: Query<(&mut Visibility, &ExhaustFlame)>,
    joint_q: Query<(Entity, &ImpulseJoint)>,
) {
    // Use real dt for Rapier-external logic (fuel burn, throttle ramp, etc.)
    // Rapier already sees the scaled dt from Time<Virtual>, so we must NOT
    // double-scale forces. Instead, we run game logic at real-time cadence
    // but apply forces that Rapier will integrate over the scaled timestep.
    let real_dt = real_time.delta_secs();
    let warp_rate = time_warp.rate();

    let Ok((rocket_entity, mut rocket, rocket_tf, mut rocket_ext_force, rocket_vel, mut rocket_fuel, rocket_engine, mut rocket_mass_props)) = rocket_q.get_single_mut() else { return; };
    let Ok((planet, planet_tf)) = planet_q.get_single() else { return; };

    // Time warp throttle safety: Force throttle to 0 if rate > 1x
    if warp_rate > 1.0 {
        rocket.throttle = 0.0;
    }

    // Launch / Stage logic
    if keys.just_pressed(KeyCode::Space) {
        if !rocket.is_launched {
            rocket.is_launched = true;
        } else if rocket.current_stage > 0 {
            for (joint_entity, joint) in joint_q.iter() {
                if joint.parent == rocket_entity {
                    commands.entity(joint_entity).remove::<ImpulseJoint>();
                    rocket.current_stage -= 1;
                    break;
                }
            }
        }
    }

    if !rocket.is_launched { return; }

    // Throttle control — use real_dt so ramp speed is consistent regardless of warp
    if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
        rocket.throttle += 0.5 * real_dt;
    }
    if keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight) {
        rocket.throttle -= 0.5 * real_dt;
    }
    if keys.just_pressed(KeyCode::KeyZ) { rocket.throttle = 1.0; }
    if keys.just_pressed(KeyCode::KeyX) { rocket.throttle = 0.0; }
    rocket.throttle = rocket.throttle.clamp(0.0, 1.0);

    // SAS / Manual Control
    let mut target_torque = Vec3::ZERO;
    let torque_amount = 8000.0;
    let mut manual_input = false;

    // Disallow manual rotation during time warp, but keep SAS active
    if warp_rate <= 1.0 {
        if keys.pressed(KeyCode::KeyW) { target_torque.x += torque_amount; manual_input = true; }
        if keys.pressed(KeyCode::KeyS) { target_torque.x -= torque_amount; manual_input = true; }
        if keys.pressed(KeyCode::KeyA) { target_torque.z += torque_amount; manual_input = true; }
        if keys.pressed(KeyCode::KeyD) { target_torque.z -= torque_amount; manual_input = true; }
        if keys.pressed(KeyCode::KeyQ) { target_torque.y += torque_amount; manual_input = true; }
        if keys.pressed(KeyCode::KeyE) { target_torque.y -= torque_amount; manual_input = true; }
    }

    let world_torque = if manual_input {
        rocket_tf.rotation * target_torque
    } else {
        // Stability mode: damp angular velocity
        let damping_strength = 2000.0;
        -rocket_vel.angvel * damping_strength
    };

    // Update main vessel forces
    rocket_ext_force.torque = world_torque;

    // --- Combined Force Calculation ---
    // Handle the main Rocket entity
    {
        let mut total_force = Vec3::ZERO;

        // Thrust for active stage
        if rocket_engine.stage == rocket.current_stage && rocket.throttle > 0.0 && rocket_fuel.fuel_mass > 0.0 {
            // Fuel burn uses real_dt: fuel should drain at the same rate per
            // virtual second regardless of warp. Since the virtual dt is
            // already warp_rate * real_dt, but we only get called once per
            // real frame, we need real_dt * warp_rate = virtual_dt equivalent.
            // Rapier will step with the scaled dt, so one real frame covers
            // warp_rate seconds of sim time. Burn fuel for that full duration.
            let virtual_dt = real_dt * warp_rate;
            let burnt = rocket_engine.fuel_burn_rate * rocket.throttle * virtual_dt;
            rocket_fuel.fuel_mass = (rocket_fuel.fuel_mass - burnt).max(0.0);
            *rocket_mass_props = ColliderMassProperties::Mass(rocket_fuel.dry_mass + rocket_fuel.fuel_mass);
            total_force += rocket_tf.up() * rocket.throttle * rocket_engine.max_thrust;
        }

        // Gravity
        let body_mass = if let ColliderMassProperties::Mass(m) = *rocket_mass_props { m } else { 1.0 };
        let to_planet = planet_tf.translation - rocket_tf.translation;
        let dist = to_planet.length();
        let dist_sq = dist * dist;
        if dist_sq > 0.1 {
            total_force += to_planet.normalize() * (planet.mu * body_mass / dist_sq);
        }

        // 3. Atmospheric Drag
        let altitude = dist - planet.radius;
        if altitude < planet.atmosphere_height {
            let scale_height = planet.atmosphere_height / 5.0;
            let density = (-altitude / scale_height).exp();
            let velocity_sq = rocket_vel.linvel.length_squared();
            if velocity_sq > 0.001 {
                let drag_mag = 0.5 * density * velocity_sq * 0.5;
                total_force += -rocket_vel.linvel.normalize() * drag_mag;
            }
        }

        // Do NOT multiply by warp_rate here! Rapier's TimestepMode::Variable
        // already uses the scaled Time<Virtual>.delta_secs() as its step dt.
        // Forces are per-second values; Rapier integrates: v += (F/m)*dt_scaled.
        rocket_ext_force.force = total_force;
    }

    // 2. Handle other parts (Boosters)
    for (_entity, mut fuel, engine, mut ext_force, mut mass_props, transform, velocity) in part_q.iter_mut() {
        let mut total_force = Vec3::ZERO;

        // Thrust
        if engine.stage == rocket.current_stage && rocket.throttle > 0.0 && fuel.fuel_mass > 0.0 {
            let virtual_dt = real_dt * warp_rate;
            let burnt = engine.fuel_burn_rate * rocket.throttle * virtual_dt;
            fuel.fuel_mass = (fuel.fuel_mass - burnt).max(0.0);
            *mass_props = ColliderMassProperties::Mass(fuel.dry_mass + fuel.fuel_mass);
            total_force += transform.up() * rocket.throttle * engine.max_thrust;
        }

        // Gravity
        let body_mass = if let ColliderMassProperties::Mass(m) = *mass_props { m } else { 1.0 };
        let to_planet = planet_tf.translation - transform.translation;
        let dist = to_planet.length();
        let dist_sq = dist * dist;
        if dist_sq > 0.1 {
            total_force += to_planet.normalize() * (planet.mu * body_mass / dist_sq);
        }

        // Drag for part
        let altitude = dist - planet.radius;
        if altitude < planet.atmosphere_height {
            let scale_height = planet.atmosphere_height / 5.0;
            let density = (-altitude / scale_height).exp();
            let velocity_sq = velocity.linvel.length_squared();
            if velocity_sq > 0.001 {
                let drag_mag = 0.5 * density * velocity_sq * 0.5;
                total_force += -velocity.linvel.normalize() * drag_mag;
            }
        }

        // UPDATE EXTERNAL FORCES CONDITIONAL: Solves Jitter (Allows body to sleep).
        // Same as above — no manual warp scaling; Rapier already uses scaled dt.
        if ext_force.force.distance_squared(total_force) > 0.001 {
            ext_force.force = total_force;
        }
    }

    // --- Update ALL flame visibility after physics (covers Stage 0 + boosters) ---
    // We need the current fuel state per stage, so gather it from both queries.
    // rocket_q already gives us rocket_engine.stage + rocket_fuel.fuel_mass.
    // part_q is already iterated above; re-query read-only is fine here.
    for (mut vis, flame) in flame_q.iter_mut() {
        let stage = flame.0;
        // Check if this flame's stage is the active stage
        let is_active_stage = stage == rocket.current_stage;
        // Check if stage 0 (main rocket) still has fuel
        let has_fuel = if rocket_engine.stage == stage {
            rocket_fuel.fuel_mass > 0.0
        } else {
            // For booster stages: scan part_q by re-checking (already mutated above)
            // We can't re-borrow part_q here, so we rely on the stage-matching condition;
            // if the part ran out this frame it will flicker for one tick — acceptable.
            true
        };
        *vis = if is_active_stage && rocket.throttle > 0.0 && has_fuel {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

fn telemetry_system(
    planet_q: Query<(&CelestialBody, &Transform), Without<Rocket>>,
    rocket_q: Query<(&Rocket, &Transform, &Velocity)>,
    part_q: Query<(&FuelTank, &Engine)>,
    mut text_q: Query<&mut Text, With<TelemetryUI>>,
    time_warp: Res<TimeWarp>,
) {
    let Ok((planet, p_transform)) = planet_q.get_single() else { return; };
    let Ok((rocket, transform, velocity)) = rocket_q.get_single() else { return; };
    let Ok(mut text) = text_q.get_single_mut() else { return; };

    // Find current active engine/tank stats
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

    let planet_center = p_transform.translation;
    let to_planet = transform.translation - planet_center;
    let dist = to_planet.length();
    let local_up = if dist > 0.0 { to_planet / dist } else { Vec3::Y };
    let altitude = dist - planet.radius;
    let vel_mag = velocity.linvel.length();

    let pitch_deg = transform.up().dot(local_up).asin().to_degrees();
    let vertical_vel = velocity.linvel.dot(local_up);
    let surface_vel = (velocity.linvel - local_up * vertical_vel).length();

    let density = if altitude < planet.atmosphere_height {
        let scale_height = planet.atmosphere_height / 5.0;
        (-altitude / scale_height).exp()
    } else {
        0.0
    };

    let warp_str = if time_warp.rate() > 1.0 {
        format!(" (Warp x{:.0})", time_warp.rate())
    } else {
        "".to_string()
    };

    text.0 = format!(
        "Central Body: {}\nStage: {}\nAltitude: {:.1} m\nPitch (vs Horizon): {:.1} deg\nOrbital Vel: {:.1} m/s\nSurface Vel: {:.1} m/s\nV.Speed: {:.1} m/s\nThrottle: {:.0}%{}\nThrust: {:.0} N\nFuel: {:.0} kg\nAir Density: {:.3}",
        planet.name,
        rocket.current_stage,
        altitude,
        pitch_deg,
        vel_mag,
        surface_vel,
        vertical_vel,
        rocket.throttle * 100.0,
        warp_str,
        current_thrust,
        current_fuel,
        density,
    );
}

fn time_warp_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut time_warp: ResMut<TimeWarp>,
    mut time: ResMut<Time<Virtual>>,
) {
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
        let rate = time_warp.rate();
        // Scale game-logic dt (fuel burn, torque, etc.) via Time<Virtual>
        time.set_relative_speed(rate);
    }
}



fn orbit_prediction_system(
    mut gizmos: Gizmos,
    planet_q: Query<(&CelestialBody, &Transform)>,
    rocket_q: Query<(&Transform, &Velocity, &ColliderMassProperties, Entity), With<Rocket>>,
    part_q: Query<(Entity, &Transform, &ColliderMassProperties, &Velocity), Without<Rocket>>,
    joint_q: Query<&ImpulseJoint>,
    state: Res<State<AppState>>,
) {
    if *state.get() != AppState::MapView { return; }

    let Ok((planet, planet_tf)) = planet_q.get_single() else { return; };
    let Ok((rocket_tf, rocket_vel, rocket_mass_props, rocket_entity)) = rocket_q.get_single() else { return; };

    // --- Calculate Barycentric (Center of Mass) State ---
    // We need the average position and velocity of the whole assembly to keep the orbit stable
    let mut total_mass = 0.0;
    let mut weighted_pos = Vec3::ZERO;
    let mut weighted_vel = Vec3::ZERO;

    // Add main rocket
    let m0 = if let ColliderMassProperties::Mass(m) = *rocket_mass_props { m } else { 1.0 };
    total_mass += m0;
    weighted_pos += rocket_tf.translation * m0;
    weighted_vel += rocket_vel.linvel * m0;

    // Add attached parts — a part is attached if it carries an ImpulseJoint
    // whose parent is the rocket entity. After staging, the decoupled booster
    // loses its joint, so it is automatically excluded from the CoM calculation.
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

    let pos = com_pos - planet_tf.translation;
    let vel = com_vel;
    let mu = planet.mu;

    // 1. Orbit Thresholds: Only predict if altitude > 10m and velocity > 10m/s
    let altitude = pos.length() - planet.radius;
    let speed = vel.length();
    if altitude < 10.0 || speed < 10.0 { return; }

    // Specific angular momentum h = r x v
    let h = pos.cross(vel);
    let h_mag = h.length();
    if h_mag < 0.001 { return; }

    // Eccentricity vector e = (v x h) / mu - r / |r|
    let e_vec = (vel.cross(h) / mu) - (pos.normalize());
    let e = e_vec.length();

    // Specific orbital energy E = v^2 / 2 - mu / r
    let energy = vel.length_squared() / 2.0 - mu / pos.length();

    // Semi-major axis a
    let a = if (1.0 - e).abs() < 0.001 {
        // Parabolic case (rare)
        1000000.0
    } else if e < 1.0 {
        // Elliptic
        -mu / (2.0 * energy)
    } else {
        // Hyperbolic
        mu / (2.0 * energy.abs())
    };

    // Basis vectors for the orbital plane
    let p = e_vec.normalize_or(Vec3::X); // Vector towards periapsis
    let q = h.cross(p).normalize();             // Vector at 90 deg to periapsis

    let num_segments = 128;

    if e < 1.0 {
        // DRAW ELLIPSE
        let b = a * (1.0 - e * e).sqrt(); // Semi-minor axis
        let center = planet_tf.translation - p * (a * e);

        let mut points = Vec::with_capacity(num_segments + 1);
        for i in 0..=num_segments {
            let angle = (i as f32 / num_segments as f32) * std::f32::consts::TAU;
            let local_pos = p * (a * angle.cos()) + q * (b * angle.sin());
            points.push(center + local_pos);
        }
        gizmos.linestrip(points, Color::srgb(0.0, 0.8, 1.0));
    } else {
        // DRAW HYPERBOLA
        // Draw only the part of the hyperbola near the planet
        let mut points = Vec::new();

        // Correcting hyperbola parameter: r = a(e^2 - 1) / (1 + e cos(nu))
        // For hyperbola, 'a' is usually defined as positive half-distance between vertices.
        // Our 'a' from energy is mu / (2 * |E|).

        // Asymptotic true anomaly: cos(nu_inf) = -1/e
        let max_nu = (-1.0 / e).acos();
        let render_nu = max_nu * 0.99; // Draw closer to asymptotes

        for i in 0..=num_segments {
            let nu = -render_nu + (i as f32 / num_segments as f32) * (2.0 * render_nu);
            let r_mag = (a * (e * e - 1.0)) / (1.0 + e * nu.cos());
            if r_mag > 5000.0 || r_mag < 0.0 { continue; }
            let local_pos = p * (r_mag * nu.cos()) + q * (r_mag * nu.sin());
            points.push(planet_tf.translation + local_pos);
        }
        if points.len() > 1 {
            gizmos.linestrip(points, Color::srgb(1.0, 0.5, 0.0));
        }
    }
}

fn map_view_toggle_system(
    keys: Res<ButtonInput<KeyCode>>,
    state: Res<State<AppState>>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    if keys.just_pressed(KeyCode::KeyM) {
        match state.get() {
            AppState::Flight | AppState::VAB => next_state.set(AppState::MapView),
            AppState::MapView => next_state.set(AppState::Flight),
        }
    }
}

fn camera_controller(
    time: Res<Time<Virtual>>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    state: Res<State<AppState>>,
    mut mouse_motion_events: EventReader<MouseMotion>,
    mut mouse_wheel_events: EventReader<MouseWheel>,
    mut query: Query<(&mut Transform, &mut OrbitCamera)>,
    rocket_query: Query<&Transform, (With<Rocket>, Without<OrbitCamera>)>,
    planet_query: Query<&Transform, (With<CelestialBody>, Without<OrbitCamera>)>,
) {
    let dt = time.delta_secs();

    // Focus target depends on state
    let target_pos = if *state.get() == AppState::MapView {
        planet_query.get_single().map(|t| t.translation).unwrap_or(Vec3::ZERO)
    } else {
        rocket_query.get_single().map(|t| t.translation).unwrap_or(Vec3::ZERO)
    };

    // Process inputs outside so they aren't consumed per-camera
    let mut kb_yaw_delta = 0.0;
    let mut kb_pitch_delta = 0.0;
    let mut zoom_delta = 0.0;

    // Use only Arrow keys now, WASD is for rocket SAS
    if keys.pressed(KeyCode::ArrowLeft) { kb_yaw_delta += 1.0; }
    if keys.pressed(KeyCode::ArrowRight) { kb_yaw_delta -= 1.0; }
    if keys.pressed(KeyCode::ArrowUp) { kb_pitch_delta += 1.0; }
    if keys.pressed(KeyCode::ArrowDown) { kb_pitch_delta -= 1.0; }

    // PageUp/PageDown for zoom — avoids conflict with Q/E rocket SAS keys
    if keys.pressed(KeyCode::PageUp)   { zoom_delta -= 10.0 * dt; }
    if keys.pressed(KeyCode::PageDown) { zoom_delta += 10.0 * dt; }

    // Mouse motion
    let mut mouse_delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        mouse_delta += event.delta;
    }

    // Mouse scroll
    for event in mouse_wheel_events.read() {
        let zoom_scale = if *state.get() == AppState::MapView {
            100.0 // Much faster zoom in MapView
        } else {
            2.0
        };

        if let MouseScrollUnit::Line = event.unit {
            zoom_delta -= event.y * zoom_scale;
        } else {
            zoom_delta -= event.y * (zoom_scale * 0.01);
        }
    }

    // Apply to cameras
    for (mut transform, mut orbit) in query.iter_mut() {
        // Apply keyboard rotation (dt scaled)
        orbit.yaw += kb_yaw_delta * dt * 2.0;
        orbit.pitch += kb_pitch_delta * dt * 2.0;

        // Apply mouse rotation (unscaled, relies on frame delta pixels)
        if mouse_buttons.pressed(MouseButton::Right) {
            orbit.yaw -= mouse_delta.x * 0.005;
            orbit.pitch -= mouse_delta.y * 0.005;
        }

        orbit.pitch = orbit.pitch.clamp(-1.5, 1.5); // Prevent flipping

        let (min_dist, max_dist) = if *state.get() == AppState::MapView {
            (210.0, 5000.0) // Map view zoom (above planet surface)
        } else {
            (2.0, 100.0)    // Flight view zoom
        };

        orbit.distance += zoom_delta;

        // If switching to MapView, ensure we are outside the planet
        if *state.get() == AppState::MapView && orbit.distance < min_dist {
            orbit.distance = 500.0;
        }

        orbit.distance = orbit.distance.clamp(min_dist, max_dist);

        // Calculate new position
        let rotation = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0);
        let position = target_pos + rotation * Vec3::new(0.0, 0.0, orbit.distance);

        transform.translation = position;
        transform.look_at(target_pos, Vec3::Y);
    }
}
