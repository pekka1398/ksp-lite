use bevy::prelude::*;
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

#[derive(Component)]
struct MapViewCamera;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .init_state::<AppState>()
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                rocket_flight_system,
                telemetry_system,
                orbit_prediction_system,
                map_view_toggle_system,
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
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    planet_q: Query<(&CelestialBody, &Transform), Without<Rocket>>,
    mut rocket_q: Query<(Entity, &mut Rocket, &Transform, &mut ExternalForce, &Velocity, &mut FuelTank, &Engine, &mut ColliderMassProperties)>,
    mut part_q: Query<(Entity, &mut FuelTank, &Engine, &mut ExternalForce, &mut ColliderMassProperties, &Transform), Without<Rocket>>,
    mut flame_q: Query<(&mut Visibility, &ExhaustFlame)>,
    joint_q: Query<(Entity, &ImpulseJoint)>,
) {
    let dt = time.delta_secs();

    let Ok((rocket_entity, mut rocket, rocket_tf, mut rocket_ext_force, rocket_vel, mut rocket_fuel, rocket_engine, mut rocket_mass_props)) = rocket_q.get_single_mut() else { return; };
    let Ok((planet, planet_tf)) = planet_q.get_single() else { return; };

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

    // Throttle control
    if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
        rocket.throttle += 0.5 * dt;
    }
    if keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight) {
        rocket.throttle -= 0.5 * dt;
    }
    if keys.just_pressed(KeyCode::KeyZ) { rocket.throttle = 1.0; }
    if keys.just_pressed(KeyCode::KeyX) { rocket.throttle = 0.0; }
    rocket.throttle = rocket.throttle.clamp(0.0, 1.0);

    // SAS / Manual Control
    let mut target_torque = Vec3::ZERO;
    let torque_amount = 8000.0;
    let mut manual_input = false;

    if keys.pressed(KeyCode::KeyW) { target_torque.x += torque_amount; manual_input = true; }
    if keys.pressed(KeyCode::KeyS) { target_torque.x -= torque_amount; manual_input = true; }
    if keys.pressed(KeyCode::KeyA) { target_torque.z += torque_amount; manual_input = true; }
    if keys.pressed(KeyCode::KeyD) { target_torque.z -= torque_amount; manual_input = true; }
    if keys.pressed(KeyCode::KeyQ) { target_torque.y += torque_amount; manual_input = true; }
    if keys.pressed(KeyCode::KeyE) { target_torque.y -= torque_amount; manual_input = true; }

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
            let burnt = rocket_engine.fuel_burn_rate * rocket.throttle * dt;
            rocket_fuel.fuel_mass = (rocket_fuel.fuel_mass - burnt).max(0.0);
            *rocket_mass_props = ColliderMassProperties::Mass(rocket_fuel.dry_mass + rocket_fuel.fuel_mass);
            total_force += rocket_tf.up() * rocket.throttle * rocket_engine.max_thrust;
        }

        // Gravity
        let body_mass = if let ColliderMassProperties::Mass(m) = *rocket_mass_props { m } else { 1.0 };
        let to_planet = planet_tf.translation - rocket_tf.translation;
        let dist_sq = to_planet.length_squared();
        if dist_sq > 0.1 {
            total_force += to_planet.normalize() * (planet.mu * body_mass / dist_sq);
        }
        rocket_ext_force.force = total_force;
    }

    // 2. Handle other parts (Boosters)
    for (_entity, mut fuel, engine, mut ext_force, mut mass_props, transform) in part_q.iter_mut() {
        let mut total_force = Vec3::ZERO;

        // Thrust (if this part is the active stage - usually only the main body is active,
        // but this allows for boosters to have their own "Rocket" logic if we expanded)
        if engine.stage == rocket.current_stage && rocket.throttle > 0.0 && fuel.fuel_mass > 0.0 {
            let burnt = engine.fuel_burn_rate * rocket.throttle * dt;
            fuel.fuel_mass = (fuel.fuel_mass - burnt).max(0.0);
            *mass_props = ColliderMassProperties::Mass(fuel.dry_mass + fuel.fuel_mass);
            total_force += transform.up() * rocket.throttle * engine.max_thrust;
        }

        // Gravity
        let body_mass = match *mass_props {
            ColliderMassProperties::Mass(m) => m,
            _ => 1.0,
        };
        let to_planet = planet_tf.translation - transform.translation;
        let dist_sq = to_planet.length_squared();
        if dist_sq > 0.1 {
            let force_mag = (planet.mu * body_mass) / dist_sq;
            total_force += to_planet.normalize() * force_mag;
        }

        // UPDATE EXTERNAL FORCES CONDITIONAL: Solves Jitter (Allows body to sleep).
        if ext_force.force.distance_squared(total_force) > 0.001 {
            ext_force.force = total_force;
        }

        // Update Flame visibility
        for (mut vis, flame) in flame_q.iter_mut() {
            if flame.0 == engine.stage {
                *vis = if engine.stage == rocket.current_stage && rocket.throttle > 0.0 && fuel.fuel_mass > 0.0 {
                    Visibility::Visible
                } else {
                    Visibility::Hidden
                };
            }
        }
    }
}

fn telemetry_system(
    planet_q: Query<(&CelestialBody, &Transform), Without<Rocket>>,
    rocket_q: Query<(&Rocket, &Transform, &Velocity)>,
    part_q: Query<(&FuelTank, &Engine)>,
    mut text_q: Query<&mut Text, With<TelemetryUI>>,
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

    text.0 = format!(
        "Central Body: {}\nStage: {}\nAltitude: {:.1} m\nPitch (vs Horizon): {:.1} deg\nOrbital Vel: {:.1} m/s\nSurface Vel: {:.1} m/s\nV.Speed: {:.1} m/s\nThrottle: {:.0}%\nThrust: {:.0} N\nFuel: {:.0} kg",
        planet.name,
        rocket.current_stage,
        altitude,
        pitch_deg,
        vel_mag,
        surface_vel,
        vertical_vel,
        rocket.throttle * 100.0,
        current_thrust,
        current_fuel,
    );
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

    // Add attached parts
    for (part_entity, part_tf, part_mass_props, part_vel) in part_q.iter() {
        // Only count parts that are actually joined to the rocket
        let is_attached = joint_q.iter().any(|j|
            (j.parent == rocket_entity && part_entity == part_entity) || // Simpler check: is it in the scene
            true // For now, we assume all parts in part_q are either boosters or the active ship
        );

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
        let max_nu = (1.0 / e).acos().max(0.1); // Angle of asymptote
        let render_nu = max_nu * 0.95; // Don't draw to infinity

        for i in 0..=num_segments {
            let nu = -render_nu + (i as f32 / num_segments as f32) * (2.0 * render_nu);
            let r_mag = (a * (e * e - 1.0)) / (1.0 + e * nu.cos());
            if r_mag > 5000.0 { continue; } // Clipping distance
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
    time: Res<Time>,
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

    if keys.pressed(KeyCode::KeyE) { zoom_delta -= 10.0 * dt; }
    if keys.pressed(KeyCode::KeyQ) { zoom_delta += 10.0 * dt; }

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
