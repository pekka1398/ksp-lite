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

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .init_state::<AppState>()
        .add_systems(Startup, setup_scene)
        .add_systems(Update, (rocket_flight_system, telemetry_system))
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
    mut rocket_q: Query<(Entity, &mut Rocket, &Transform)>,
    mut part_q: Query<(Entity, &mut FuelTank, &Engine, &mut ExternalForce, &mut ColliderMassProperties, &Transform)>,
    mut flame_q: Query<(&mut Visibility, &ExhaustFlame)>,
    joint_q: Query<(Entity, &ImpulseJoint)>,
) {
    let dt = time.delta_secs();

    let Ok((rocket_entity, mut rocket, _)) = rocket_q.get_single_mut() else { return; };

    // Launch / Stage logic
    if keys.just_pressed(KeyCode::Space) {
        if !rocket.is_launched {
            rocket.is_launched = true;
        } else if rocket.current_stage > 0 {
            // DECOUPLE
            // Find the joint connecting TO our rocket_entity or FROM it.
            // In our setup, stage1_entity had the ImpulseJoint pointing to stage0_entity (rocket_entity).
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

    // Apply SAS Torque to the main rocket entity (the command pod/vessel)
    if let Ok((_, _, _, mut ext_force, _, transform)) = part_q.get_mut(rocket_entity) {
        let mut local_torque = Vec3::ZERO;
        let torque_amount = 8000.0;
        if keys.pressed(KeyCode::KeyW) { local_torque.x += torque_amount; }
        if keys.pressed(KeyCode::KeyS) { local_torque.x -= torque_amount; }
        if keys.pressed(KeyCode::KeyA) { local_torque.z += torque_amount; }
        if keys.pressed(KeyCode::KeyD) { local_torque.z -= torque_amount; }
        ext_force.torque = transform.rotation * local_torque;
    }

    // Process all rocket parts (Engines and Fuel Tanks)
    for (_entity, mut fuel, engine, mut ext_force, mut mass_props, transform) in part_q.iter_mut() {
        // Only active stage engine burns
        if engine.stage == rocket.current_stage && rocket.throttle > 0.0 && fuel.fuel_mass > 0.0 {
            let burnt = engine.fuel_burn_rate * rocket.throttle * dt;
            fuel.fuel_mass = (fuel.fuel_mass - burnt).max(0.0);

            // Adjust physics mass
            *mass_props = ColliderMassProperties::Mass(fuel.dry_mass + fuel.fuel_mass);

            let thrust = transform.up() * rocket.throttle * engine.max_thrust;
            ext_force.force = thrust;
        } else {
            ext_force.force = Vec3::ZERO;
            // Spent stages still have mass (already handled by mass_props being on the entity)
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
    rocket_q: Query<(&Rocket, &Transform, &Velocity)>,
    part_q: Query<(&FuelTank, &Engine)>,
    mut text_q: Query<&mut Text, With<TelemetryUI>>,
) {
    let Ok((rocket, transform, velocity)) = rocket_q.get_single() else { return; };
    let Ok(mut text) = text_q.get_single_mut() else { return; };

    // Find current active engine/tank stats
    let mut current_fuel = 0.0;
    let mut current_thrust = 0.0;
    for (tank, engine) in part_q.iter() {
        if engine.stage == rocket.current_stage {
            current_fuel = tank.fuel_mass;
            current_thrust = rocket.throttle * engine.max_thrust;
        }
    }

    let planet_center = Vec3::new(0.0, -200.0, 0.0);
    let altitude = transform.translation.distance(planet_center) - 200.0;
    let vel_mag = velocity.linvel.length();
    let (yaw, pitch, roll) = transform.rotation.to_euler(EulerRot::YXZ);

    text.0 = format!(
        "Stage: {}\nAltitude: {:.1} m\nVelocity: {:.1} m/s\nThrottle: {:.0}%\nThrust: {:.0} N\nFuel: {:.0} kg\nPitch: {:.1} deg Yaw: {:.1} deg Roll: {:.1} deg",
        rocket.current_stage,
        altitude,
        vel_mag,
        rocket.throttle * 100.0,
        if current_fuel > 0.0 { current_thrust } else { 0.0 },
        current_fuel,
        pitch.to_degrees(),
        yaw.to_degrees(),
        roll.to_degrees()
    );
}

fn camera_controller(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut mouse_motion_events: EventReader<MouseMotion>,
    mut mouse_wheel_events: EventReader<MouseWheel>,
    mut query: Query<(&mut Transform, &mut OrbitCamera)>,
    rocket_query: Query<&Transform, (With<Rocket>, Without<OrbitCamera>)>,
) {
    let dt = time.delta_secs();
    let rocket_transform = rocket_query.get_single().ok();
    let target_pos = rocket_transform.map(|t| t.translation).unwrap_or(Vec3::ZERO);

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
        if let MouseScrollUnit::Line = event.unit {
            zoom_delta -= event.y * 2.0;
        } else {
            zoom_delta -= event.y * 0.02;
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

        orbit.distance += zoom_delta;
        orbit.distance = orbit.distance.clamp(2.0, 100.0);

        // Calculate new position
        let rotation = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0);
        let position = target_pos + rotation * Vec3::new(0.0, 0.0, orbit.distance);

        transform.translation = position;
        transform.look_at(target_pos, Vec3::Y);
    }
}
