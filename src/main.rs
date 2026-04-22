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
        .add_systems(Update, (camera_controller, rocket_flight_system, telemetry_system))
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
struct ExhaustFlame;

#[derive(Component)]
struct Rocket {
    throttle: f32,
    is_launched: bool,
    max_thrust: f32,
    fuel_burn_rate: f32,
    fuel_mass: f32,
    dry_mass: f32,
}

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
    let mat_nose = materials.add(Color::srgb(0.9, 0.1, 0.1));
    let mat_engine = materials.add(Color::srgb(0.2, 0.2, 0.2));
    let mat_flame = materials.add(Color::srgb(1.0, 0.5, 0.0));

    // The root vessel entity
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.5, 1.0))),
        MeshMaterial3d(mat_upper),
        Transform::from_xyz(0.0, 1.2, 0.0), 
        RigidBody::Dynamic,
        Collider::cylinder(0.5, 0.5),
        ColliderMassProperties::Mass(1500.0 + 4000.0), // Dry mass + initial fuel
        ExternalForce::default(),
        Velocity::default(),
        Rocket { 
            throttle: 0.0, 
            is_launched: false,
            max_thrust: 150_000.0, // 150 kN
            fuel_burn_rate: 45.0, // 45 kg per second at full throttle
            fuel_mass: 4000.0, // 4 tons of fuel
            dry_mass: 1500.0, // 1.5 tons of metal 
        },
    )).with_child((
        Mesh3d(meshes.add(Cone { radius: 0.5, height: 1.0 })),
        MeshMaterial3d(mat_nose),
        Transform::from_xyz(0.0, 1.0, 0.0),
        Collider::cone(0.5, 0.5),
        ColliderMassProperties::Mass(1.0), // negligible
    )).with_child((
        Mesh3d(meshes.add(ConicalFrustum { radius_top: 0.15, radius_bottom: 0.3, height: 0.4 })),
        MeshMaterial3d(mat_engine),
        Transform::from_xyz(0.0, -0.7, 0.0),
        Collider::cylinder(0.2, 0.3), 
        ColliderMassProperties::Mass(1.0), // negligible
    )).with_child((
        Mesh3d(meshes.add(Cone { radius: 0.25, height: 2.0 })),
        MeshMaterial3d(mat_flame),
        Transform::from_xyz(0.0, -1.7, 0.0),
        Visibility::Hidden,
        ExhaustFlame,
    ));

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
    mut query: Query<(&mut Rocket, &mut ExternalForce, &mut ColliderMassProperties, &Transform)>,
    mut flame_query: Query<&mut Visibility, With<ExhaustFlame>>,
) {
    let dt = time.delta_secs();

    for (mut rocket, mut ext_force, mut mass_props, transform) in query.iter_mut() {
        if keys.just_pressed(KeyCode::Space) {
            rocket.is_launched = true;
        }

        if !rocket.is_launched {
            continue;
        }

        // Throttle control
        if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
            rocket.throttle += 0.5 * dt;
        }
        if keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight) {
            rocket.throttle -= 0.5 * dt;
        }
        if keys.just_pressed(KeyCode::KeyZ) {
            rocket.throttle = 1.0;
        }
        if keys.just_pressed(KeyCode::KeyX) {
            rocket.throttle = 0.0;
        }
        rocket.throttle = rocket.throttle.clamp(0.0, 1.0);

        // Burn Fuel and Apply Thrust
        if rocket.throttle > 0.0 && rocket.fuel_mass > 0.0 {
            let burnt = rocket.fuel_burn_rate * rocket.throttle * dt;
            rocket.fuel_mass = (rocket.fuel_mass - burnt).max(0.0);
            
            // Adjust physics mass dynamically!
            *mass_props = ColliderMassProperties::Mass(rocket.dry_mass + rocket.fuel_mass);

            let thrust = transform.up() * rocket.throttle * rocket.max_thrust;
            ext_force.force = thrust;
        } else {
            // Cut thrust if no fuel
            ext_force.force = Vec3::ZERO;
        }

        // Apply WASD Torque for SAS
        let mut local_torque = Vec3::ZERO;
        let torque_amount = 8000.0;
        
        if keys.pressed(KeyCode::KeyW) { local_torque.x += torque_amount; }
        if keys.pressed(KeyCode::KeyS) { local_torque.x -= torque_amount; }
        if keys.pressed(KeyCode::KeyA) { local_torque.z += torque_amount; }
        if keys.pressed(KeyCode::KeyD) { local_torque.z -= torque_amount; }

        ext_force.torque = transform.rotation * local_torque;

        // Flame visibility based on actual propulsion
        for mut vis in flame_query.iter_mut() {
            *vis = if rocket.throttle > 0.0 && rocket.fuel_mass > 0.0 { Visibility::Visible } else { Visibility::Hidden };
        }
    }
}

fn telemetry_system(
    rocket_q: Query<(&Rocket, &Transform, &Velocity)>,
    mut text_q: Query<&mut Text, With<TelemetryUI>>,
) {
    let Ok((rocket, transform, velocity)) = rocket_q.get_single() else { return; };
    let Ok(mut text) = text_q.get_single_mut() else { return; };

    let planet_center = Vec3::new(0.0, -200.0, 0.0);
    // Measured altitude = distance from planet center - planet radius
    let altitude = transform.translation.distance(planet_center) - 200.0;
    
    let vel_mag = velocity.linvel.length();
    let thrust = rocket.throttle * rocket.max_thrust;

    let (yaw, pitch, roll) = transform.rotation.to_euler(EulerRot::YXZ);

    text.0 = format!(
        "Altitude: {:.1} m\nVelocity: {:.1} m/s\nThrottle: {:.0}%\nThrust: {:.0} N\nFuel: {:.0} kg\nPitch: {:.1} deg Yaw: {:.1} deg Roll: {:.1} deg",
        altitude,
        vel_mag,
        rocket.throttle * 100.0,
        if rocket.fuel_mass > 0.0 { thrust } else { 0.0 },
        rocket.fuel_mass,
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
