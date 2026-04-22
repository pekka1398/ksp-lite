use bevy::prelude::*;
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
        Mesh3d(meshes.add(Sphere::new(10.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.6, 0.4))),
        Transform::from_xyz(0.0, -10.0, 0.0),
        Collider::ball(10.0),
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
    mut query: Query<(&mut Transform, &mut OrbitCamera)>,
) {
    for (mut transform, mut orbit) in query.iter_mut() {
        let mut yaw_delta = 0.0;
        let mut pitch_delta = 0.0;
        let mut dist_delta = 0.0;

        if keys.pressed(KeyCode::ArrowLeft) || keys.pressed(KeyCode::KeyA) {
            yaw_delta += 1.0;
        }
        if keys.pressed(KeyCode::ArrowRight) || keys.pressed(KeyCode::KeyD) {
            yaw_delta -= 1.0;
        }
        if keys.pressed(KeyCode::ArrowUp) || keys.pressed(KeyCode::KeyW) {
            pitch_delta += 1.0;
        }
        if keys.pressed(KeyCode::ArrowDown) || keys.pressed(KeyCode::KeyS) {
            pitch_delta -= 1.0;
        }
        if keys.pressed(KeyCode::KeyE) {
            dist_delta -= 1.0;
        }
        if keys.pressed(KeyCode::KeyQ) {
            dist_delta += 1.0;
        }

        let dt = time.delta_secs();
        orbit.yaw += yaw_delta * dt * 2.0;
        orbit.pitch += pitch_delta * dt * 2.0;
        orbit.pitch = orbit.pitch.clamp(-1.5, 1.5); // Prevent flipping
        orbit.distance += dist_delta * dt * 10.0;
        orbit.distance = orbit.distance.max(2.0);

        // Calculate new position
        let rotation = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0);
        let position = rotation * Vec3::new(0.0, 0.0, orbit.distance);

        transform.translation = position;
        transform.look_at(Vec3::ZERO, Vec3::Y);
    }
}
