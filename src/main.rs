use bevy::prelude::*;
use bevy::time::Real;
use bevy::ecs::event::EventReader;
use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy_rapier3d::prelude::*;

mod vab;
mod flight;
mod orbit;

use vab::RocketConfig;
use flight::PrePauseView;
use orbit::ManeuverNode;

// ===== States =====

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, States)]
pub enum AppState {
    #[default]
    MainMenu,
    VAB,
    Flight,
    MapView,
    Paused,
}

// ===== Component Markers =====

#[derive(Component)]
struct MainMenuUI;

#[derive(Component)]
enum MenuButton {
    Start,
    Quit,
}

// ===== Shared Resources =====

#[derive(Resource)]
pub struct TimeWarp {
    pub rates: Vec<f32>,
    pub index: usize,
}

impl Default for TimeWarp {
    fn default() -> Self {
        Self {
            rates: vec![1.0, 2.0, 5.0, 10.0, 20.0],
            index: 0,
        }
    }
}

impl TimeWarp {
    pub fn rate(&self) -> f32 {
        self.rates[self.index]
    }
}

// ===== Shared Game Components =====

#[derive(Component)]
pub struct OrbitCamera {
    pub distance: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Component)]
pub struct CelestialBody {
    pub name: String,
    pub mu: f32,
    pub radius: f32,
    pub atmosphere_height: f32,
    pub soi_radius: f32,
    pub orbit_radius: f32,
    pub orbit_speed: f32,
    pub rotation_speed: f32,
}

#[derive(Component)]
pub struct Rocket {
    pub is_launched: bool,
    pub throttle: f32,
    pub current_stage: usize,
}

#[derive(Component)]
pub struct FuelTank {
    pub fuel_mass: f32,
    pub dry_mass: f32,
}

#[derive(Component)]
pub struct Engine {
    pub max_thrust: f32,
    pub fuel_burn_rate: f32,
    pub stage: usize,
}

#[derive(Component)]
pub struct StageMarker(pub usize);

#[derive(Component)]
pub struct ExhaustFlame(pub usize);

// ===== Main =====

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .insert_resource(TimestepMode::Interpolated {
            dt: 1.0 / 60.0,
            time_scale: 1.0,
            substeps: 1,
        })
        .init_state::<AppState>()
        .init_resource::<TimeWarp>()
        .init_resource::<ManeuverNode>()
        .init_resource::<PrePauseView>()
        .init_resource::<RocketConfig>()
        .add_systems(Startup, setup_scene)
        // State transitions
        .add_systems(OnEnter(AppState::MainMenu), spawn_main_menu)
        .add_systems(OnExit(AppState::MainMenu), despawn_main_menu)
        .add_systems(OnEnter(AppState::VAB), vab::spawn_vab)
        .add_systems(OnExit(AppState::VAB), vab::despawn_vab)
        .add_systems(OnEnter(AppState::Flight), flight::spawn_flight)
        .add_systems(OnEnter(AppState::MainMenu), flight::cleanup_game)
        .add_systems(OnEnter(AppState::VAB), flight::cleanup_game)
        // Update systems
        .add_systems(
            Update,
            (
                main_menu_button_system.run_if(in_state(AppState::MainMenu)),
                vab::vab_button_system.run_if(in_state(AppState::VAB)),
                flight::celestial_orbit_system.run_if(in_state(AppState::Flight).or(in_state(AppState::MapView))),
                flight::rocket_flight_system.run_if(in_state(AppState::Flight).or(in_state(AppState::MapView))),
                flight::telemetry_system.run_if(in_state(AppState::Flight).or(in_state(AppState::MapView))),
                orbit::orbit_prediction_system.run_if(in_state(AppState::MapView)),
                flight::map_view_toggle_system.run_if(in_state(AppState::Flight).or(in_state(AppState::MapView))),
                flight::time_warp_system.run_if(in_state(AppState::Flight).or(in_state(AppState::MapView))),
                orbit::maneuver_node_system.run_if(in_state(AppState::MapView)),
                flight::pause_menu_system,
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

// ===== Scene Setup =====

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let kerbin_radius = 2000.0;
    let kerbin_g = 5.0;
    let kerbin_mu = kerbin_g * kerbin_radius * kerbin_radius;

    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(kerbin_radius))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.6, 0.4))),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Collider::ball(kerbin_radius),
        RigidBody::Fixed,
        CelestialBody {
            name: "Kerbin".to_string(),
            mu: kerbin_mu,
            radius: kerbin_radius,
            atmosphere_height: 150.0,
            soi_radius: 16000.0,
            orbit_radius: 0.0,
            orbit_speed: 0.0,
            rotation_speed: 0.025,
        },
    ));

    let mun_orbit_radius = 14000.0;
    let mun_radius = 200.0;
    let mun_g = 1.0;
    let mun_mu = mun_g * mun_radius * mun_radius;
    let mun_orbital_speed = f32::sqrt(kerbin_mu / mun_orbit_radius);
    let mun_angular_speed = mun_orbital_speed / mun_orbit_radius;
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(mun_radius))),
        MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.5))),
        Transform::from_xyz(mun_orbit_radius, 0.0, 0.0),
        Collider::ball(mun_radius),
        RigidBody::Fixed,
        CelestialBody {
            name: "Mun".to_string(),
            mu: mun_mu,
            radius: mun_radius,
            atmosphere_height: 0.0,
            soi_radius: 2500.0,
            orbit_radius: mun_orbit_radius,
            orbit_speed: mun_angular_speed,
            rotation_speed: 0.0,
        },
    ));

    let equator_rot = Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(6.0, 0.5, 6.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.3, 0.3))),
        Transform::from_xyz(kerbin_radius + 0.25, 0.0, 0.0).with_rotation(equator_rot),
        RigidBody::Fixed,
        Collider::cuboid(3.0, 0.25, 3.0),
    ));

    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 10_000.,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(5000.0, 2000.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
        OrbitCamera {
            distance: 5000.0,
            pitch: 0.3,
            yaw: 0.0,
        },
    ));
}

// ===== Main Menu =====

fn spawn_main_menu(mut commands: Commands, mut camera_q: Query<&mut OrbitCamera>) {
    for mut orbit in camera_q.iter_mut() {
        orbit.distance = 5000.0;
        orbit.pitch = 0.3;
        orbit.yaw = 0.0;
    }

    commands.spawn((
        Node {
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            flex_direction: FlexDirection::Column,
            align_items: AlignItems::Center,
            justify_content: JustifyContent::Center,
            row_gap: Val::Px(24.0),
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.05, 0.85)),
        MainMenuUI,
    )).with_children(|parent| {
        parent.spawn((
            Text::new("KSP-LITE"),
            TextFont { font_size: 72.0, ..default() },
            TextColor(Color::WHITE),
            Node { margin: UiRect::bottom(Val::Px(40.0)), ..default() },
        ));

        parent.spawn((
            Button,
            Node {
                width: Val::Px(220.0),
                height: Val::Px(64.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                border: UiRect::all(Val::Px(2.0)),
                ..default()
            },
            BorderColor(Color::srgb(0.3, 0.8, 0.4)),
            BackgroundColor(Color::srgb(0.15, 0.45, 0.2)),
            MenuButton::Start,
        )).with_children(|button| {
            button.spawn((
                Text::new("START"),
                TextFont { font_size: 32.0, ..default() },
                TextColor(Color::WHITE),
            ));
        });

        parent.spawn((
            Button,
            Node {
                width: Val::Px(220.0),
                height: Val::Px(64.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                border: UiRect::all(Val::Px(2.0)),
                ..default()
            },
            BorderColor(Color::srgb(0.8, 0.3, 0.3)),
            BackgroundColor(Color::srgb(0.45, 0.15, 0.15)),
            MenuButton::Quit,
        )).with_children(|button| {
            button.spawn((
                Text::new("QUIT"),
                TextFont { font_size: 32.0, ..default() },
                TextColor(Color::WHITE),
            ));
        });
    });
}

fn despawn_main_menu(mut commands: Commands, q: Query<Entity, With<MainMenuUI>>) {
    if let Ok(entity) = q.get_single() {
        commands.entity(entity).despawn_recursive();
    }
}

fn main_menu_button_system(
    q: Query<(&Interaction, &MenuButton), Changed<Interaction>>,
    mut next_state: ResMut<NextState<AppState>>,
    mut exit_events: EventWriter<AppExit>,
) {
    for (interaction, button) in q.iter() {
        if *interaction == Interaction::Pressed {
            match button {
                MenuButton::Start => next_state.set(AppState::VAB),
                MenuButton::Quit => { exit_events.send(AppExit::Success); }
            }
        }
    }
}

// ===== Camera =====

fn camera_controller(
    real_time: Res<Time<Real>>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    state: Res<State<AppState>>,
    mut mouse_motion_events: EventReader<MouseMotion>,
    mut mouse_wheel_events: EventReader<MouseWheel>,
    mut query: Query<(&mut Transform, &mut OrbitCamera)>,
    rocket_query: Query<&Transform, (With<Rocket>, Without<OrbitCamera>)>,
    planet_query: Query<(&CelestialBody, &Transform), (With<CelestialBody>, Without<OrbitCamera>)>,
) {
    if *state.get() == AppState::MainMenu {
        for (mut transform, _) in query.iter_mut() {
            let time = real_time.elapsed_secs();
            let angle = time * 0.1;
            let dist = 5000.0;
            let x = dist * angle.cos();
            let z = dist * angle.sin();
            transform.translation = Vec3::new(x, 2000.0, z);
            transform.look_at(Vec3::ZERO, Vec3::Y);
        }
        return;
    }

    let dt = real_time.delta_secs();

    let (target_pos, up_dir) = if *state.get() == AppState::VAB {
        (Vec3::new(2005.0, 0.0, 0.0), Vec3::X)
    } else if *state.get() == AppState::MapView {
        let rocket_pos = rocket_query.get_single().map(|t| t.translation).unwrap_or(Vec3::ZERO);
        let (_, body_tf) = orbit::find_soi_body(rocket_pos, planet_query.iter());
        (body_tf.translation, Vec3::Y)
    } else {
        let rocket_pos = rocket_query.get_single().map(|t| t.translation).unwrap_or(Vec3::ZERO);
        let (_, body_tf) = orbit::find_soi_body(rocket_pos, planet_query.iter());
        let up = (rocket_pos - body_tf.translation).try_normalize().unwrap_or(Vec3::Y);
        (rocket_pos, up)
    };

    let mut kb_yaw_delta = 0.0;
    let mut kb_pitch_delta = 0.0;
    let mut zoom_delta = 0.0;

    if keys.pressed(KeyCode::ArrowLeft) { kb_yaw_delta += 1.0; }
    if keys.pressed(KeyCode::ArrowRight) { kb_yaw_delta -= 1.0; }
    if keys.pressed(KeyCode::ArrowUp) { kb_pitch_delta += 1.0; }
    if keys.pressed(KeyCode::ArrowDown) { kb_pitch_delta -= 1.0; }

    if keys.pressed(KeyCode::PageUp)   { zoom_delta -= 10.0 * dt; }
    if keys.pressed(KeyCode::PageDown) { zoom_delta += 10.0 * dt; }

    let mut mouse_delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        mouse_delta += event.delta;
    }

    for event in mouse_wheel_events.read() {
        let zoom_scale = if *state.get() == AppState::MapView {
            100.0
        } else if *state.get() == AppState::VAB {
            5.0
        } else {
            2.0
        };

        if let MouseScrollUnit::Line = event.unit {
            zoom_delta -= event.y * zoom_scale;
        } else {
            zoom_delta -= event.y * (zoom_scale * 0.01);
        }
    }

    for (mut transform, mut orbit) in query.iter_mut() {
        orbit.yaw += kb_yaw_delta * dt * 2.0;
        orbit.pitch += kb_pitch_delta * dt * 2.0;

        if mouse_buttons.pressed(MouseButton::Right) {
            orbit.yaw -= mouse_delta.x * 0.005;
            orbit.pitch -= mouse_delta.y * 0.005;
        }

        orbit.pitch = orbit.pitch.clamp(-1.5, 1.5);

        let (min_dist, max_dist) = if *state.get() == AppState::MapView {
            (2100.0, 30000.0)
        } else if *state.get() == AppState::VAB {
            (3.0, 50.0)
        } else {
            (2.0, 200.0)
        };

        orbit.distance += zoom_delta;

        if *state.get() == AppState::MapView && orbit.distance < min_dist {
            orbit.distance = 5000.0;
        }

        orbit.distance = orbit.distance.clamp(min_dist, max_dist);

        let align = Quat::from_rotation_arc(Vec3::Y, up_dir);
        let local_rot = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0);
        let rotation = align * local_rot;
        let position = target_pos + rotation * Vec3::new(0.0, 0.0, orbit.distance);

        transform.translation = position;
        transform.look_at(target_pos, up_dir);
    }
}
