use bevy::prelude::*;
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
        .add_systems(Update, camera_controller)
        .run();
}

/// A simple camera component to allow orbital controls later
#[derive(Component)]
struct OrbitCamera {
    distance: f32,
    pitch: f32,
    yaw: f32,
}

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

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
        OrbitCamera {
            distance: 15.0,
            pitch: 0.0,
            yaw: 0.0,
        },
    ));
}

fn camera_controller(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut mouse_motion_events: EventReader<MouseMotion>,
    mut mouse_wheel_events: EventReader<MouseWheel>,
    mut query: Query<(&mut Transform, &mut OrbitCamera)>,
) {
    let dt = time.delta_secs();

    // Process inputs outside so they aren't consumed per-camera
    let mut kb_yaw_delta = 0.0;
    let mut kb_pitch_delta = 0.0;
    let mut zoom_delta = 0.0;

    // Keyboard rotation
    if keys.pressed(KeyCode::ArrowLeft) || keys.pressed(KeyCode::KeyA) {
        kb_yaw_delta += 1.0;
    }
    if keys.pressed(KeyCode::ArrowRight) || keys.pressed(KeyCode::KeyD) {
        kb_yaw_delta -= 1.0;
    }
    if keys.pressed(KeyCode::ArrowUp) || keys.pressed(KeyCode::KeyW) {
        kb_pitch_delta += 1.0;
    }
    if keys.pressed(KeyCode::ArrowDown) || keys.pressed(KeyCode::KeyS) {
        kb_pitch_delta -= 1.0;
    }

    // Keyboard zoom
    if keys.pressed(KeyCode::KeyE) {
        zoom_delta -= 10.0 * dt;
    }
    if keys.pressed(KeyCode::KeyQ) {
        zoom_delta += 10.0 * dt;
    }

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
        let position = rotation * Vec3::new(0.0, 0.0, orbit.distance);

        transform.translation = position;
        transform.look_at(Vec3::ZERO, Vec3::Y);
    }
}
