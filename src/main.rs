use bevy::prelude::*;
use bevy::time::Real;
use bevy::ecs::event::EventReader;
use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy_rapier3d::prelude::*;

// ===== States =====

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, States)]
enum AppState {
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
struct VabUI;

#[derive(Component)]
struct VabConfigText;

#[derive(Component)]
struct VabPreviewEntity;

#[derive(Component)]
struct FlightEntity;

#[derive(Component)]
struct PauseMenuUI;

#[derive(Component)]
struct TelemetryUI;

#[derive(Component)]
enum MenuButton {
    Start,
    Quit,
}

#[derive(Component)]
enum VabButton {
    AddStage,
    RemoveStage,
    CycleStageType(usize, i32), // stage index, direction (+1/-1)
    Launch,
    Back,
}

// ===== Resources =====

/// Remembers which view was active before pausing, so ESC returns to the same one.
#[derive(Resource, Default)]
struct PrePauseView(AppState);

#[derive(Resource)]
struct TimeWarp {
    rates: Vec<f32>,
    index: usize,
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
    fn rate(&self) -> f32 {
        self.rates[self.index]
    }
}

#[derive(Resource, Clone, Debug)]
struct RocketConfig {
    stages: Vec<StageConfig>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StageType {
    UpperStage,
    Booster,
    HeavyBooster,
}

impl StageType {
    const ALL: [StageType; 3] = [
        StageType::UpperStage,
        StageType::Booster,
        StageType::HeavyBooster,
    ];

    fn name(self) -> &'static str {
        match self {
            StageType::UpperStage => "Upper Stage",
            StageType::Booster => "Booster",
            StageType::HeavyBooster => "Heavy Booster",
        }
    }

    fn cycle(self, dir: i32) -> Self {
        let idx = Self::ALL.iter().position(|&t| t == self).unwrap() as i32;
        let len = Self::ALL.len() as i32;
        let new_idx = ((idx + dir).rem_euclid(len)) as usize;
        Self::ALL[new_idx]
    }
}

#[derive(Clone, Debug)]
struct StageConfig {
    stage_type: StageType,
    dry_mass: f32,
    fuel_mass: f32,
    max_thrust: f32,
    fuel_burn_rate: f32,
    radius: f32,
    height: f32,
}

impl StageConfig {
    fn from_type(stage_type: StageType) -> Self {
        match stage_type {
            StageType::UpperStage => Self {
                stage_type,
                dry_mass: 600.0,
                fuel_mass: 400.0,
                max_thrust: 12000.0,
                fuel_burn_rate: 6.0,
                radius: 0.5,
                height: 1.0,
            },
            StageType::Booster => Self {
                stage_type,
                dry_mass: 600.0,
                fuel_mass: 1000.0,
                max_thrust: 40000.0,
                fuel_burn_rate: 18.0,
                radius: 0.5,
                height: 2.0,
            },
            StageType::HeavyBooster => Self {
                stage_type,
                dry_mass: 1200.0,
                fuel_mass: 2500.0,
                max_thrust: 80000.0,
                fuel_burn_rate: 36.0,
                radius: 0.75,
                height: 3.0,
            },
        }
    }

    fn default_upper() -> Self {
        Self::from_type(StageType::UpperStage)
    }

    fn default_booster() -> Self {
        Self::from_type(StageType::Booster)
    }
}

impl Default for RocketConfig {
    fn default() -> Self {
        Self {
            stages: vec![
                StageConfig::default_upper(),
                StageConfig::default_booster(),
            ],
        }
    }
}

/// A maneuver node: planned delta-v burn at a future point on the orbit.
/// Components are in orbital frame: prograde (along velocity),
/// normal (along angular momentum), radial (in-plane toward body).
#[derive(Resource)]
struct ManeuverNode {
    prograde: f32,
    normal: f32,
    radial: f32,
    ut: f64, // absolute game-time (seconds) when the burn happens; < 0 = no node
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
    fn is_active(&self) -> bool {
        self.ut >= 0.0
    }
}

// ===== Game Components =====

#[derive(Component)]
struct OrbitCamera {
    distance: f32,
    pitch: f32,
    yaw: f32,
}

#[derive(Component)]
struct CelestialBody {
    name: String,
    mu: f32,
    radius: f32,
    atmosphere_height: f32,
    soi_radius: f32,
    orbit_radius: f32,
    orbit_speed: f32,
    rotation_speed: f32,
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

// ===== Helper Functions =====

/// Propagate a Keplerian orbit forward by `dt` seconds.
/// Returns (new_position, new_velocity) relative to the central body.
fn propagate_kepler(pos: Vec3, vel: Vec3, mu: f32, dt: f32) -> (Vec3, Vec3) {
    // Use universal variable / Stumpff function approach.
    // For simplicity, we use a high-quality RK4 integrator for the two-body problem.
    // This avoids the complexity of solving Kepler's equation.
    let steps = ((dt / 0.5).ceil() as usize).max(1);
    let h = dt / steps as f32;

    let mut r = pos;
    let mut v = vel;

    for _ in 0..steps {
        // RK4 for dr/dt = v, dv/dt = -mu * r / |r|^3
        let accel = |r: Vec3| -> Vec3 {
            let r_mag = r.length();
            if r_mag < 0.001 { return Vec3::ZERO; }
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
    }

    (r, v)
}

/// Find the dominant SOI body for a given world position from an iterator of (CelestialBody, Transform).
/// Returns the body whose SOI contains the position and is deepest (smallest soi_ratio).
/// Falls back to the nearest body if none contain the position.
fn find_soi_body<'a>(
    pos: Vec3,
    bodies: impl Iterator<Item = (&'a CelestialBody, &'a Transform)>,
) -> (&'a CelestialBody, &'a Transform) {
    let mut soi: Option<(&CelestialBody, &Transform)> = None;
    let mut soi_ratio = f32::MAX;
    let mut nearest: Option<(&CelestialBody, &Transform)> = None;
    let mut nearest_dist = f32::MAX;

    for (body, body_tf) in bodies {
        let dist = (pos - body_tf.translation).length();
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

    soi.or(nearest).unwrap()
}

/// Compute the world-space velocity of a celestial body (orbital motion).
fn soi_body_velocity(body: &CelestialBody, body_tf: &Transform) -> Vec3 {
    if body.orbit_radius > 0.0 && body.orbit_speed > 0.0 {
        body.orbit_speed * Vec3::new(-body_tf.translation.z, 0.0, body_tf.translation.x)
    } else {
        Vec3::ZERO
    }
}

/// Compute the surface rotation velocity at a world position on a rotating body.
/// The rotation axis is +Y (poles at ±Y), so equatorial velocity is tangential in XZ.
fn surface_rotation_velocity(body: &CelestialBody, pos: Vec3) -> Vec3 {
    if body.rotation_speed == 0.0 { return Vec3::ZERO; }
    // ω along +Y, position relative to body center
    // v = ω × r = ω * (-z, 0, x)
    body.rotation_speed * Vec3::new(-pos.z, 0.0, pos.x)
}

fn compute_total_dv(config: &RocketConfig) -> f32 {
    let mut dv = 0.0;
    let mut remaining_mass: f32 = config.stages.iter().map(|s| s.dry_mass + s.fuel_mass).sum();
    for i in (0..config.stages.len()).rev() {
        let stage = &config.stages[i];
        let ve = stage.max_thrust / stage.fuel_burn_rate;
        let m0 = remaining_mass;
        let m1 = remaining_mass - stage.fuel_mass;
        if m1 > 0.0 && m0 > m1 {
            dv += ve * (m0 / m1).ln();
        }
        remaining_mass -= stage.dry_mass + stage.fuel_mass;
    }
    dv
}

fn vab_config_text(config: &RocketConfig) -> String {
    let mut s = String::new();
    let total_mass: f32 = config.stages.iter().map(|s| s.dry_mass + s.fuel_mass).sum();
    let dv = compute_total_dv(config);

    for stage in config.stages.iter() {
        s.push_str(&format!(
            "[{}] {}kN | Fuel: {}kg | TWR: {:.1}\n",
            stage.stage_type.name(),
            stage.max_thrust as i32 / 1000,
            stage.fuel_mass as i32,
            stage.max_thrust / (total_mass * 5.0), // Kerbin g=5.0
        ));
    }

    s.push_str(&format!("\nMass: {}kg | dV: {:.0} m/s", total_mass as i32, dv));
    s
}

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
        // Startup: world only (planet, mun, pad, light, camera)
        .add_systems(Startup, setup_scene)
        // State transitions
        .add_systems(OnEnter(AppState::MainMenu), spawn_main_menu)
        .add_systems(OnExit(AppState::MainMenu), despawn_main_menu)
        .add_systems(OnEnter(AppState::VAB), spawn_vab)
        .add_systems(OnExit(AppState::VAB), despawn_vab)
        .add_systems(OnEnter(AppState::Flight), spawn_flight)
        .add_systems(OnEnter(AppState::MainMenu), cleanup_game)
        .add_systems(OnEnter(AppState::VAB), cleanup_game)
        // Update systems
        .add_systems(
            Update,
            (
                main_menu_button_system.run_if(in_state(AppState::MainMenu)),
                vab_button_system.run_if(in_state(AppState::VAB)),
                celestial_orbit_system.run_if(in_state(AppState::Flight).or(in_state(AppState::MapView))),
                rocket_flight_system.run_if(in_state(AppState::Flight).or(in_state(AppState::MapView))),
                telemetry_system.run_if(in_state(AppState::Flight).or(in_state(AppState::MapView))),
                orbit_prediction_system.run_if(in_state(AppState::MapView)),
                map_view_toggle_system.run_if(in_state(AppState::Flight).or(in_state(AppState::MapView))),
                time_warp_system.run_if(in_state(AppState::Flight).or(in_state(AppState::MapView))),
                maneuver_node_system.run_if(in_state(AppState::MapView)),
                pause_menu_system,
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

// ===== Scene Setup (world only) =====

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let kerbin_radius = 2000.0;
    let kerbin_g = 5.0;
    let kerbin_mu = kerbin_g * kerbin_radius * kerbin_radius;

    // Planet (Sphere) — Kerbin
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

    // Moon — The Mun
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

    // Launch Pad — on the equator at +X
    let equator_rot = Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(6.0, 0.5, 6.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.3, 0.3))),
        Transform::from_xyz(kerbin_radius + 0.25, 0.0, 0.0).with_rotation(equator_rot),
        RigidBody::Fixed,
        Collider::cuboid(3.0, 0.25, 3.0),
    ));

    // Directional light
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 10_000.,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Camera (persistent across states, controller adjusts behavior per state)
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

        // Start button
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

        // Quit button
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

// ===== VAB =====

fn spawn_vab(
    mut commands: Commands,
    config: Res<RocketConfig>,
    mut camera_q: Query<&mut OrbitCamera>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for mut orbit in camera_q.iter_mut() {
        orbit.distance = 20.0;
        orbit.pitch = 0.2;
        orbit.yaw = 0.0;
    }

    // Spawn 3D preview
    spawn_vab_preview(&mut commands, &config, &mut meshes, &mut materials);

    let config_display = vab_config_text(&config);

    // Left panel for controls
    commands.spawn((
        Node {
            width: Val::Px(320.0),
            height: Val::Percent(100.0),
            position_type: PositionType::Absolute,
            top: Val::Px(0.0),
            left: Val::Px(0.0),
            flex_direction: FlexDirection::Column,
            padding: UiRect::all(Val::Px(16.0)),
            row_gap: Val::Px(12.0),
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.05, 0.92)),
        VabUI,
    )).with_children(|parent| {
        parent.spawn((
            Text::new("VEHICLE ASSEMBLY"),
            TextFont { font_size: 36.0, ..default() },
            TextColor(Color::WHITE),
            Node { margin: UiRect::bottom(Val::Px(8.0)), ..default() },
        ));

        // Add/Remove stage buttons
        parent.spawn((
            Node {
                flex_direction: FlexDirection::Row,
                align_items: AlignItems::Center,
                column_gap: Val::Px(8.0),
                ..default()
            },
        )).with_children(|row| {
            row.spawn((
                Button,
                Node {
                    width: Val::Px(44.0),
                    height: Val::Px(36.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                BackgroundColor(Color::srgb(0.6, 0.3, 0.1)),
                VabButton::RemoveStage,
            )).with_children(|btn| {
                btn.spawn((
                    Text::new("- Stage"),
                    TextFont { font_size: 16.0, ..default() },
                    TextColor(Color::WHITE),
                ));
            });

            row.spawn((
                Button,
                Node {
                    width: Val::Px(44.0),
                    height: Val::Px(36.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                BackgroundColor(Color::srgb(0.1, 0.4, 0.6)),
                VabButton::AddStage,
            )).with_children(|btn| {
                btn.spawn((
                    Text::new("+ Stage"),
                    TextFont { font_size: 16.0, ..default() },
                    TextColor(Color::WHITE),
                ));
            });
        });

        // Per-stage type selectors
        for (i, stage) in config.stages.iter().enumerate() {
            parent.spawn((
                Node {
                    flex_direction: FlexDirection::Row,
                    align_items: AlignItems::Center,
                    column_gap: Val::Px(6.0),
                    ..default()
                },
            )).with_children(|row| {
                row.spawn((
                    Text::new(format!("S{}:", i)),
                    TextFont { font_size: 18.0, ..default() },
                    TextColor(Color::srgb(0.6, 0.6, 0.6)),
                    Node { width: Val::Px(30.0), ..default() },
                ));

                // Left arrow
                row.spawn((
                    Button,
                    Node {
                        width: Val::Px(28.0),
                        height: Val::Px(28.0),
                        justify_content: JustifyContent::Center,
                        align_items: AlignItems::Center,
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.3, 0.3, 0.3)),
                    VabButton::CycleStageType(i, -1),
                )).with_children(|btn| {
                    btn.spawn((
                        Text::new("<"),
                        TextFont { font_size: 18.0, ..default() },
                        TextColor(Color::WHITE),
                    ));
                });

                // Type name
                row.spawn((
                    Text::new(stage.stage_type.name()),
                    TextFont { font_size: 16.0, ..default() },
                    TextColor(Color::srgb(0.8, 0.9, 0.8)),
                    Node { width: Val::Px(110.0), ..default() },
                ));

                // Right arrow
                row.spawn((
                    Button,
                    Node {
                        width: Val::Px(28.0),
                        height: Val::Px(28.0),
                        justify_content: JustifyContent::Center,
                        align_items: AlignItems::Center,
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.3, 0.3, 0.3)),
                    VabButton::CycleStageType(i, 1),
                )).with_children(|btn| {
                    btn.spawn((
                        Text::new(">"),
                        TextFont { font_size: 18.0, ..default() },
                        TextColor(Color::WHITE),
                    ));
                });
            });
        }

        // Config summary
        parent.spawn((
            Text::new(config_display),
            TextFont { font_size: 16.0, ..default() },
            TextColor(Color::srgb(0.7, 0.9, 0.7)),
            VabConfigText,
            Node { margin: UiRect::top(Val::Px(8.0)), ..default() },
        ));

        // Spacer
        parent.spawn(Node { flex_grow: 1.0, ..default() });

        // Launch button
        parent.spawn((
            Button,
            Node {
                width: Val::Percent(100.0),
                height: Val::Px(52.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                border: UiRect::all(Val::Px(2.0)),
                ..default()
            },
            BorderColor(Color::srgb(0.3, 0.8, 0.4)),
            BackgroundColor(Color::srgb(0.15, 0.45, 0.2)),
            VabButton::Launch,
        )).with_children(|btn| {
            btn.spawn((
                Text::new("LAUNCH"),
                TextFont { font_size: 28.0, ..default() },
                TextColor(Color::WHITE),
            ));
        });

        // Back button
        parent.spawn((
            Button,
            Node {
                width: Val::Percent(100.0),
                height: Val::Px(40.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                border: UiRect::all(Val::Px(2.0)),
                ..default()
            },
            BorderColor(Color::srgb(0.4, 0.4, 0.4)),
            BackgroundColor(Color::srgb(0.25, 0.25, 0.25)),
            VabButton::Back,
        )).with_children(|btn| {
            btn.spawn((
                Text::new("BACK"),
                TextFont { font_size: 20.0, ..default() },
                TextColor(Color::srgb(0.7, 0.7, 0.7)),
            ));
        });
    });
}

fn despawn_vab(mut commands: Commands, ui_q: Query<Entity, With<VabUI>>, preview_q: Query<Entity, With<VabPreviewEntity>>) {
    if let Ok(entity) = ui_q.get_single() {
        commands.entity(entity).despawn_recursive();
    }
    for entity in preview_q.iter() {
        commands.entity(entity).despawn_recursive();
    }
}

fn spawn_vab_preview(
    commands: &mut Commands,
    config: &RocketConfig,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let mat_upper = materials.add(Color::srgb(0.8, 0.8, 0.8));
    let mat_booster = materials.add(Color::srgb(0.6, 0.6, 0.7));
    let mat_heavy = materials.add(Color::srgb(0.7, 0.5, 0.3));
    let mat_nose = materials.add(Color::srgb(0.9, 0.1, 0.1));
    let mat_engine = materials.add(Color::srgb(0.2, 0.2, 0.2));

    let n_stages = config.stages.len();
    if n_stages == 0 { return; }

    // Compute positions (same math as flight, but centered at origin for clean preview)
    let stage_gap = 0.45;
    let mut y_positions = Vec::with_capacity(n_stages);
    let mut y = 0.0;
    for i in (0..n_stages).rev() {
        let h = config.stages[i].height;
        y += h / 2.0;
        y_positions.insert(0, y);
        y += h / 2.0;
        y += stage_gap;
    }

    // Center the rocket vertically, position at the launch pad
    let center_offset = (y_positions[0] + y_positions[n_stages - 1]) / 2.0;
    let vab_origin = Vec3::new(2005.0, 0.0, 0.0); // above launch pad on equator

    for i in 0..n_stages {
        let stage = &config.stages[i];
        let local_y = y_positions[i] - center_offset;
        let is_top = i == 0;

        let stage_mat = match stage.stage_type {
            StageType::UpperStage => mat_upper.clone(),
            StageType::Booster => mat_booster.clone(),
            StageType::HeavyBooster => mat_heavy.clone(),
        };

        // Rotate so local +Y points along world +X (radially outward at equator)
        let equator_rot = Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2);
        let mut entity_cmds = commands.spawn((
            Mesh3d(meshes.add(Cylinder::new(stage.radius, stage.height))),
            MeshMaterial3d(stage_mat),
            Transform::from_translation(vab_origin + equator_rot * Vec3::new(0.0, local_y, 0.0))
                .with_rotation(equator_rot),
            VabPreviewEntity,
        ));

        if is_top {
            entity_cmds.with_child((
                Mesh3d(meshes.add(Cone { radius: stage.radius, height: stage.radius * 2.0 })),
                MeshMaterial3d(mat_nose.clone()),
                Transform::from_xyz(0.0, stage.height / 2.0 + stage.radius, 0.0),
            ));
        }

        let nozzle_height = stage.radius * 0.8;
        entity_cmds.with_child((
            Mesh3d(meshes.add(ConicalFrustum {
                radius_top: stage.radius * 0.3,
                radius_bottom: stage.radius * 0.6,
                height: nozzle_height,
            })),
            MeshMaterial3d(mat_engine.clone()),
            Transform::from_xyz(0.0, -stage.height / 2.0 - nozzle_height / 2.0, 0.0),
        ));
    }
}

fn rebuild_vab_preview(
    commands: &mut Commands,
    config: &RocketConfig,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    preview_q: &Query<Entity, With<VabPreviewEntity>>,
) {
    for entity in preview_q.iter() {
        commands.entity(entity).despawn_recursive();
    }
    spawn_vab_preview(commands, config, meshes, materials);
}

fn vab_button_system(
    interaction_q: Query<(&Interaction, &VabButton), Changed<Interaction>>,
    mut config: ResMut<RocketConfig>,
    mut next_state: ResMut<NextState<AppState>>,
    mut text_q: Query<&mut Text, With<VabConfigText>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    preview_q: Query<Entity, With<VabPreviewEntity>>,
    ui_q: Query<Entity, With<VabUI>>,
    keys: Res<ButtonInput<KeyCode>>,
) {
    // ESC goes back to main menu
    if keys.just_pressed(KeyCode::Escape) {
        next_state.set(AppState::MainMenu);
        return;
    }

    let mut need_ui_rebuild = false;
    let mut need_preview_rebuild = false;

    for (interaction, button) in interaction_q.iter() {
        if *interaction != Interaction::Pressed { continue; }
        match button {
            VabButton::AddStage => {
                config.stages.push(StageConfig::default_booster());
                need_ui_rebuild = true;
                need_preview_rebuild = true;
            }
            VabButton::RemoveStage => {
                if config.stages.len() > 1 {
                    config.stages.pop();
                    need_ui_rebuild = true;
                    need_preview_rebuild = true;
                }
            }
            VabButton::CycleStageType(idx, dir) => {
                if let Some(stage) = config.stages.get_mut(*idx) {
                    let new_type = stage.stage_type.cycle(*dir);
                    *stage = StageConfig::from_type(new_type);
                    need_ui_rebuild = true;
                    need_preview_rebuild = true;
                }
            }
            VabButton::Launch => {
                next_state.set(AppState::Flight);
            }
            VabButton::Back => {
                next_state.set(AppState::MainMenu);
            }
        }
    }

    if need_preview_rebuild {
        rebuild_vab_preview(&mut commands, &config, &mut meshes, &mut materials, &preview_q);
    }

    if need_ui_rebuild {
        // Update config text
        if let Ok(mut text) = text_q.get_single_mut() {
            text.0 = vab_config_text(&config);
        }
        // Rebuild the whole VAB UI so per-stage buttons match the new config
        if let Ok(entity) = ui_q.get_single() {
            commands.entity(entity).despawn_recursive();
        }
        spawn_vab_ui_only(&mut commands, &config);
    }
}

/// Spawn just the UI panel (no 3D preview, no camera reset).
/// Used when config changes and we need to rebuild per-stage buttons.
fn spawn_vab_ui_only(commands: &mut Commands, config: &RocketConfig) {
    let config_display = vab_config_text(&config);

    commands.spawn((
        Node {
            width: Val::Px(320.0),
            height: Val::Percent(100.0),
            position_type: PositionType::Absolute,
            top: Val::Px(0.0),
            left: Val::Px(0.0),
            flex_direction: FlexDirection::Column,
            padding: UiRect::all(Val::Px(16.0)),
            row_gap: Val::Px(12.0),
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.05, 0.92)),
        VabUI,
    )).with_children(|parent| {
        parent.spawn((
            Text::new("VEHICLE ASSEMBLY"),
            TextFont { font_size: 36.0, ..default() },
            TextColor(Color::WHITE),
            Node { margin: UiRect::bottom(Val::Px(8.0)), ..default() },
        ));

        // Add/Remove stage buttons
        parent.spawn((
            Node {
                flex_direction: FlexDirection::Row,
                align_items: AlignItems::Center,
                column_gap: Val::Px(8.0),
                ..default()
            },
        )).with_children(|row| {
            row.spawn((
                Button,
                Node {
                    width: Val::Px(44.0),
                    height: Val::Px(36.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                BackgroundColor(Color::srgb(0.6, 0.3, 0.1)),
                VabButton::RemoveStage,
            )).with_children(|btn| {
                btn.spawn((
                    Text::new("- Stage"),
                    TextFont { font_size: 16.0, ..default() },
                    TextColor(Color::WHITE),
                ));
            });

            row.spawn((
                Button,
                Node {
                    width: Val::Px(44.0),
                    height: Val::Px(36.0),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                },
                BackgroundColor(Color::srgb(0.1, 0.4, 0.6)),
                VabButton::AddStage,
            )).with_children(|btn| {
                btn.spawn((
                    Text::new("+ Stage"),
                    TextFont { font_size: 16.0, ..default() },
                    TextColor(Color::WHITE),
                ));
            });
        });

        // Per-stage type selectors
        for (i, stage) in config.stages.iter().enumerate() {
            parent.spawn((
                Node {
                    flex_direction: FlexDirection::Row,
                    align_items: AlignItems::Center,
                    column_gap: Val::Px(6.0),
                    ..default()
                },
            )).with_children(|row| {
                row.spawn((
                    Text::new(format!("S{}:", i)),
                    TextFont { font_size: 18.0, ..default() },
                    TextColor(Color::srgb(0.6, 0.6, 0.6)),
                    Node { width: Val::Px(30.0), ..default() },
                ));

                row.spawn((
                    Button,
                    Node {
                        width: Val::Px(28.0),
                        height: Val::Px(28.0),
                        justify_content: JustifyContent::Center,
                        align_items: AlignItems::Center,
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.3, 0.3, 0.3)),
                    VabButton::CycleStageType(i, -1),
                )).with_children(|btn| {
                    btn.spawn((
                        Text::new("<"),
                        TextFont { font_size: 18.0, ..default() },
                        TextColor(Color::WHITE),
                    ));
                });

                row.spawn((
                    Text::new(stage.stage_type.name()),
                    TextFont { font_size: 16.0, ..default() },
                    TextColor(Color::srgb(0.8, 0.9, 0.8)),
                    Node { width: Val::Px(110.0), ..default() },
                ));

                row.spawn((
                    Button,
                    Node {
                        width: Val::Px(28.0),
                        height: Val::Px(28.0),
                        justify_content: JustifyContent::Center,
                        align_items: AlignItems::Center,
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.3, 0.3, 0.3)),
                    VabButton::CycleStageType(i, 1),
                )).with_children(|btn| {
                    btn.spawn((
                        Text::new(">"),
                        TextFont { font_size: 18.0, ..default() },
                        TextColor(Color::WHITE),
                    ));
                });
            });
        }

        // Config summary
        parent.spawn((
            Text::new(config_display),
            TextFont { font_size: 16.0, ..default() },
            TextColor(Color::srgb(0.7, 0.9, 0.7)),
            VabConfigText,
            Node { margin: UiRect::top(Val::Px(8.0)), ..default() },
        ));

        // Spacer
        parent.spawn(Node { flex_grow: 1.0, ..default() });

        // Launch button
        parent.spawn((
            Button,
            Node {
                width: Val::Percent(100.0),
                height: Val::Px(52.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                border: UiRect::all(Val::Px(2.0)),
                ..default()
            },
            BorderColor(Color::srgb(0.3, 0.8, 0.4)),
            BackgroundColor(Color::srgb(0.15, 0.45, 0.2)),
            VabButton::Launch,
        )).with_children(|btn| {
            btn.spawn((
                Text::new("LAUNCH"),
                TextFont { font_size: 28.0, ..default() },
                TextColor(Color::WHITE),
            ));
        });

        // Back button
        parent.spawn((
            Button,
            Node {
                width: Val::Percent(100.0),
                height: Val::Px(40.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                border: UiRect::all(Val::Px(2.0)),
                ..default()
            },
            BorderColor(Color::srgb(0.4, 0.4, 0.4)),
            BackgroundColor(Color::srgb(0.25, 0.25, 0.25)),
            VabButton::Back,
        )).with_children(|btn| {
            btn.spawn((
                Text::new("BACK"),
                TextFont { font_size: 20.0, ..default() },
                TextColor(Color::srgb(0.7, 0.7, 0.7)),
            ));
        });
    });
}

// ===== Flight =====

fn spawn_flight(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<RocketConfig>,
    mut camera_q: Query<&mut OrbitCamera>,
    rocket_q: Query<(), With<Rocket>>,
) {
    // Skip if rocket already exists (e.g. resuming from pause)
    if !rocket_q.is_empty() {
        return;
    }

    for mut orbit in camera_q.iter_mut() {
        orbit.distance = 20.0;
        orbit.pitch = 0.2;
        orbit.yaw = 0.0;
    }

    let kerbin_radius = 2000.0;
    let equator_rot = Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2);
    let n_stages = config.stages.len();
    if n_stages == 0 { return; }

    // Compute Y positions for each stage (local frame, stacking along radial axis)
    // Stage 0 = top, Stage N-1 = bottom
    // Must leave gaps between colliders to prevent Rapier jitter.
    // Original hardcoded values: pad top at +0.5, stage 1 center at +2.3, stage 0 center at +4.25
    // giving ~0.8 gap above pad, ~0.45 gap between stages.
    let stage_gap = 0.45;
    let pad_clearance = 0.8;
    let pad_top = 0.5; // pad center +0.25, half-height 0.25
    let mut y_positions = Vec::with_capacity(n_stages);
    let mut y = pad_top + pad_clearance; // start above pad
    for i in (0..n_stages).rev() {
        let h = config.stages[i].height;
        y += h / 2.0;  // move to center
        y_positions.insert(0, y);
        y += h / 2.0;  // move past this stage
        y += stage_gap; // gap before next stage
    }

    let mat_upper = materials.add(Color::srgb(0.8, 0.8, 0.8));
    let mat_lower = materials.add(Color::srgb(0.6, 0.6, 0.7));
    let mat_nose = materials.add(Color::srgb(0.9, 0.1, 0.1));
    let mat_engine = materials.add(Color::srgb(0.2, 0.2, 0.2));
    let mat_flame = materials.add(Color::srgb(1.0, 0.5, 0.0));

    let mut stage_entities = Vec::with_capacity(n_stages);

    for i in 0..n_stages {
        let stage = &config.stages[i];
        let is_top = i == 0;
        let stage_mat = if is_top { mat_upper.clone() } else { mat_lower.clone() };
        let y = y_positions[i];

        let mut entity_cmds = commands.spawn((
            Mesh3d(meshes.add(Cylinder::new(stage.radius, stage.height))),
            MeshMaterial3d(stage_mat),
            Transform::from_xyz(kerbin_radius + y, 0.0, 0.0).with_rotation(equator_rot),
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

        // Top stage gets the Rocket component + nose cone
        if is_top {
            entity_cmds.insert(Rocket {
                throttle: 0.0,
                is_launched: false,
                current_stage: n_stages - 1,
            });
            entity_cmds.with_child((
                Mesh3d(meshes.add(Cone { radius: stage.radius, height: stage.radius * 2.0 })),
                MeshMaterial3d(mat_nose.clone()),
                Transform::from_xyz(0.0, stage.height / 2.0 + stage.radius, 0.0),
                Collider::cone(stage.radius, stage.radius),
                ColliderMassProperties::Mass(1.0),
            ));
        }

        // Every stage gets engine nozzle + flame
        let nozzle_height = stage.radius * 0.8;
        entity_cmds.with_child((
            Mesh3d(meshes.add(ConicalFrustum {
                radius_top: stage.radius * 0.3,
                radius_bottom: stage.radius * 0.6,
                height: nozzle_height,
            })),
            MeshMaterial3d(mat_engine.clone()),
            Transform::from_xyz(0.0, -stage.height / 2.0 - nozzle_height / 2.0, 0.0),
            Collider::cylinder(nozzle_height / 2.0, stage.radius * 0.5),
            ColliderMassProperties::Mass(1.0),
        ));

        let flame_height = stage.height;
        entity_cmds.with_child((
            Mesh3d(meshes.add(Cone { radius: stage.radius * 0.5, height: flame_height })),
            MeshMaterial3d(mat_flame.clone()),
            Transform::from_xyz(0.0, -stage.height / 2.0 - nozzle_height - flame_height / 2.0, 0.0),
            Visibility::Hidden,
            ExhaustFlame(i),
        ));

        stage_entities.push(entity_cmds.id());
    }

    // Create joints between adjacent stages (lower entity holds the joint)
    // Joint connection point = top of the lower stage's collider.
    // This matches the original pattern where anchor1=(0, -0.95, 0), anchor2=(0, 1.0, 0)
    // for a 0.45 gap between the upper bottom and lower top.
    for i in 0..n_stages - 1 {
        let upper = stage_entities[i];
        let lower = stage_entities[i + 1];
        let lower_height = config.stages[i + 1].height;

        // Lower anchor: top of lower collider
        let lower_anchor_y = lower_height / 2.0;
        // Upper anchor: must map to same world point as lower anchor
        // world_meeting = y_positions[i+1] + lower_anchor_y
        // upper_anchor_y = world_meeting - y_positions[i]
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
}

fn cleanup_game(
    mut commands: Commands,
    game_entities: Query<Entity, With<FlightEntity>>,
    mut time_warp: ResMut<TimeWarp>,
    mut time: ResMut<Time<Virtual>>,
    mut maneuver: ResMut<ManeuverNode>,
) {
    for entity in game_entities.iter() {
        commands.entity(entity).despawn_recursive();
    }
    time_warp.index = 0;
    time.set_relative_speed(1.0);
    time.unpause();
    *maneuver = ManeuverNode::default();
}

// ===== Flight Systems =====

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
    let real_dt = real_time.delta_secs();
    let warp_rate = time_warp.rate();

    let Ok((rocket_entity, mut rocket, rocket_tf, mut rocket_ext_force, rocket_vel, mut rocket_fuel, rocket_engine, mut rocket_mass_props)) = rocket_q.get_single_mut() else { return; };

    // Time warp throttle safety: Force throttle to 0 if rate > 1x
    if warp_rate > 1.0 {
        rocket.throttle = 0.0;
    }

    // Launch / Stage logic
    if keys.just_pressed(KeyCode::Space) {
        if !rocket.is_launched {
            rocket.is_launched = true;
        } else if rocket.current_stage > 0 {
            // Collect all entities still attached to the rocket via joints.
            // We need to find the bottommost attached stage and detach it.
            // Joint structure: lower_stage holds ImpulseJoint with parent = upper entity.
            // Walk the chain: rocket → first attached → next attached → ...
            let mut attached_entities: Vec<Entity> = Vec::new();
            let mut current = rocket_entity;
            loop {
                let mut found_next = None;
                for (joint_entity, joint) in joint_q.iter() {
                    // joint_entity holds the joint; joint.parent is who it's attached to
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
            // The bottommost attached entity is the one to detach
            if let Some(bottom) = attached_entities.last() {
                commands.entity(*bottom).remove::<ImpulseJoint>();
                rocket.current_stage -= 1;
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

    // --- Patched Conic: only apply gravity from the SOI body ---
    // This matches the orbit prediction system (Kepler two-body) so that
    // the predicted trajectory matches the actual flight path.
    let (soi_body, soi_tf) = find_soi_body(rocket_tf.translation, planet_q.iter());

    // Helper: compute gravity + drag from the SOI body only
    let compute_gravity_and_drag = |pos: Vec3, mass: f32, vel: &Velocity| -> Vec3 {
        let mut force = Vec3::ZERO;
        let to_body = soi_tf.translation - pos;
        let dist = to_body.length();
        let dist_sq = dist * dist;
        if dist_sq > 0.1 {
            force += to_body.normalize() * (soi_body.mu * mass / dist_sq);
        }
        let altitude = dist - soi_body.radius;
        if altitude < soi_body.atmosphere_height && soi_body.atmosphere_height > 0.0 {
            let scale_height = soi_body.atmosphere_height / 5.0;
            let density = (-altitude / scale_height).exp();
            let velocity_sq = vel.linvel.length_squared();
            if velocity_sq > 0.001 {
                let drag_mag = 0.5 * density * velocity_sq * 0.5;
                force += -vel.linvel.normalize() * drag_mag;
            }
        }
        force
    };

    // --- Combined Force Calculation ---
    // Handle the main Rocket entity
    {
        let mut total_force = Vec3::ZERO;

        // Thrust for active stage
        if rocket_engine.stage == rocket.current_stage && rocket.throttle > 0.0 && rocket_fuel.fuel_mass > 0.0 {
            let virtual_dt = real_dt * warp_rate;
            let burnt = rocket_engine.fuel_burn_rate * rocket.throttle * virtual_dt;
            rocket_fuel.fuel_mass = (rocket_fuel.fuel_mass - burnt).max(0.0);
            *rocket_mass_props = ColliderMassProperties::Mass(rocket_fuel.dry_mass + rocket_fuel.fuel_mass);
            total_force += rocket_tf.up() * rocket.throttle * rocket_engine.max_thrust;
        }

        // Gravity from ALL bodies + drag
        let body_mass = if let ColliderMassProperties::Mass(m) = *rocket_mass_props { m } else { 1.0 };
        total_force += compute_gravity_and_drag(rocket_tf.translation, body_mass, rocket_vel);

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

        // Gravity from ALL bodies + drag
        let body_mass = if let ColliderMassProperties::Mass(m) = *mass_props { m } else { 1.0 };
        total_force += compute_gravity_and_drag(transform.translation, body_mass, &velocity);

        // UPDATE EXTERNAL FORCES CONDITIONAL: Solves Jitter (Allows body to sleep).
        if ext_force.force.distance_squared(total_force) > 0.001 {
            ext_force.force = total_force;
        }
    }

    // --- Update ALL flame visibility ---
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

fn telemetry_system(
    planet_q: Query<(&CelestialBody, &Transform), Without<Rocket>>,
    rocket_q: Query<(&Rocket, &Transform, &Velocity)>,
    part_q: Query<(&FuelTank, &Engine)>,
    mut text_q: Query<&mut Text, With<TelemetryUI>>,
    time_warp: Res<TimeWarp>,
    time: Res<Time<Virtual>>,
    maneuver: Res<ManeuverNode>,
) {
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

    // Determine SOI body
    let (planet, p_transform) = find_soi_body(transform.translation, planet_q.iter());

    let planet_center = p_transform.translation;
    let to_planet = transform.translation - planet_center;
    let dist = to_planet.length();
    let local_up = if dist > 0.0 { to_planet / dist } else { Vec3::Y };
    let altitude = dist - planet.radius;

    // Inertial velocity (for orbital calculations) = physical vel + surface rotation
    let rel_pos = transform.translation - planet_center;
    let inertial_vel = velocity.linvel + surface_rotation_velocity(planet, rel_pos);
    let orbital_vel_mag = inertial_vel.length();

    let pitch_deg = transform.up().dot(local_up).asin().to_degrees();
    let vertical_vel = velocity.linvel.dot(local_up);
    let surface_vel = (velocity.linvel - local_up * vertical_vel).length();

    let density = if altitude < planet.atmosphere_height && planet.atmosphere_height > 0.0 {
        let scale_height = planet.atmosphere_height / 5.0;
        (-altitude / scale_height).exp()
    } else {
        0.0
    };

    // Compute Ap/Pe altitudes from orbital elements (use inertial velocity)
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

    text.0 = format!(
        "SOI: {}\nStage: {}\nAltitude: {:.1} m\nPitch (vs Horizon): {:.1} deg\nOrbital Vel: {:.1} m/s\nSurface Vel: {:.1} m/s\nV.Speed: {:.1} m/s\nThrottle: {:.0}%{}\nThrust: {:.0} N\nFuel: {:.0} kg\nAir Density: {:.3}{}{}",
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
        ap_pe_str,
        maneuver_str,
    );
}

fn time_warp_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut time_warp: ResMut<TimeWarp>,
    mut time: ResMut<Time<Virtual>>,
    rocket_q: Query<&Transform, With<Rocket>>,
    planet_q: Query<(&CelestialBody, &Transform), Without<Rocket>>,
) {
    // Safety: prevent warp if in atmosphere
    if let Ok(rocket_tf) = rocket_q.get_single() {
        let (body, body_tf) = find_soi_body(rocket_tf.translation, planet_q.iter());
        let altitude = (rocket_tf.translation - body_tf.translation).length() - body.radius;
        if altitude < body.atmosphere_height && body.atmosphere_height > 0.0 {
            if time_warp.index > 0 {
                time_warp.index = 0;
                time.set_relative_speed(1.0);
            }
            return; // Can't warp in atmosphere, skip key handling too
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

/// Move celestial bodies along their orbits.
/// Bodies with orbit_radius > 0 orbit around the origin (Kerbin).
fn celestial_orbit_system(
    time: Res<Time<Virtual>>,
    mut query: Query<(&CelestialBody, &mut Transform)>,
) {
    let dt = time.delta_secs();
    for (body, mut transform) in query.iter_mut() {
        if body.orbit_radius > 0.0 && body.orbit_speed > 0.0 {
            // Rotate around the Y axis at the given angular speed
            let angle = body.orbit_speed * dt;
            let rotation = Quat::from_rotation_y(angle);
            transform.translation = rotation * transform.translation;
        }
    }
}

/// Maneuver node management: create, adjust, delete nodes.
/// N = create/toggle node, Delete/Backspace = remove node
/// I/K = adjust prograde +/-, J/L = adjust radial +/-, U/O = adjust normal +/-
/// T/G = adjust UT +/- (move node earlier/later on orbit)
fn maneuver_node_system(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time<Virtual>>,
    mut node: ResMut<ManeuverNode>,
) {
    let now = time.elapsed_secs_f64();

    // Toggle/create node
    if keys.just_pressed(KeyCode::KeyN) {
        if node.is_active() {
            *node = ManeuverNode::default();
        } else {
            *node = ManeuverNode {
                prograde: 100.0,
                normal: 0.0,
                radial: 0.0,
                ut: now + 300.0, // Default 5 minutes ahead
            };
        }
    }

    // Delete node
    if keys.just_pressed(KeyCode::Delete) || keys.just_pressed(KeyCode::Backspace) {
        *node = ManeuverNode::default();
    }

    // Adjust dV and UT only when a node exists
    if node.is_active() {
        // Adjust prograde (I = increase, K = decrease)
        let dv_step = 50.0;
        if keys.just_pressed(KeyCode::KeyI) { node.prograde += dv_step; }
        if keys.just_pressed(KeyCode::KeyK) { node.prograde -= dv_step; }

        // Adjust radial (J = inward, L = outward)
        if keys.just_pressed(KeyCode::KeyJ) { node.radial -= dv_step; }
        if keys.just_pressed(KeyCode::KeyL) { node.radial += dv_step; }

        // Adjust normal (U = -, O = +)
        if keys.just_pressed(KeyCode::KeyU) { node.normal -= dv_step; }
        if keys.just_pressed(KeyCode::KeyO) { node.normal += dv_step; }

        // Adjust UT (T = later, G = earlier) by 30 seconds
        let time_step = 30.0_f64;
        if keys.just_pressed(KeyCode::KeyT) { node.ut += time_step; }
        if keys.just_pressed(KeyCode::KeyG) { node.ut = (node.ut - time_step).max(now); }
    }
}

/// Ap/Pe world positions returned by draw_orbit_gizmo.
struct ApPe {
    apoapsis: Option<Vec3>,
    periapsis: Option<Vec3>,
}

/// Draw a Keplerian orbit from position/velocity around a body at planet_center.
fn draw_orbit_gizmo(
    gizmos: &mut Gizmos,
    pos: Vec3,
    vel: Vec3,
    mu: f32,
    planet_center: Vec3,
    planet_radius: f32,
    soi_radius: f32,
    color: Color,
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
        -mu / (2.0 * energy)   // Elliptic: a > 0
    } else {
        mu / (2.0 * energy.abs()) // Hyperbolic: treat a as positive for drawing
    };

    let p = if e > 0.0001 {
        e_vec.normalize()
    } else {
        // Near-circular: pos is always in the orbital plane (h ⊥ pos),
        // use it as an arbitrary in-plane reference direction.
        pos.normalize_or(Vec3::X)
    };
    let q = h.cross(p).normalize();

    // Compute Ap/Pe positions along the eccentricity vector direction
    let ap_pe = if e < 1.0 && e > 0.001 {
        let r_ap = a * (1.0 + e);
        let r_pe = a * (1.0 - e);
        ApPe {
            apoapsis: Some(planet_center - p * r_ap),
            periapsis: Some(planet_center + p * r_pe),
        }
    } else {
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

fn orbit_prediction_system(
    mut gizmos: Gizmos,
    planet_q: Query<(&CelestialBody, &Transform)>,
    rocket_q: Query<(&Transform, &Velocity, &ColliderMassProperties, Entity), With<Rocket>>,
    part_q: Query<(Entity, &Transform, &ColliderMassProperties, &Velocity), Without<Rocket>>,
    joint_q: Query<&ImpulseJoint>,
    state: Res<State<AppState>>,
    time: Res<Time<Virtual>>,
    mut maneuver: ResMut<ManeuverNode>,
) {
    if *state.get() != AppState::MapView { return; }

    let now = time.elapsed_secs_f64();

    let Ok((rocket_tf, rocket_vel, rocket_mass_props, rocket_entity)) = rocket_q.get_single() else { return; };

    // --- Calculate Barycentric (Center of Mass) State ---
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

    // --- Determine SOI body for orbit prediction ---
    let (planet, planet_tf) = find_soi_body(com_pos, planet_q.iter());

    let pos = com_pos - planet_tf.translation;
    // Inertial velocity = physical velocity - orbital velocity of body + surface rotation
    let vel = com_vel - soi_body_velocity(planet, planet_tf) + surface_rotation_velocity(planet, pos);
    let mu = planet.mu;

    // Draw current orbit (cyan) and get Ap/Pe
    let ap_pe = draw_orbit_gizmo(&mut gizmos, pos, vel, mu, planet_tf.translation, planet.radius, planet.soi_radius, Color::srgb(0.0, 0.8, 1.0));

    // Ship marker (green diamond)
    gizmos.sphere(com_pos, 10.0, Color::srgb(0.0, 1.0, 0.0));

    // Ap/Pe markers on current orbit
    let marker_size = 12.0;
    if let Some(ap_pos) = ap_pe.apoapsis {
        gizmos.sphere(ap_pos, marker_size, Color::srgb(1.0, 0.3, 0.3));
    }
    if let Some(pe_pos) = ap_pe.periapsis {
        gizmos.sphere(pe_pos, marker_size, Color::srgb(0.3, 0.6, 1.0));
    }

    // Draw maneuver orbit (yellow) if node exists and is in the future
    let time_to_node = (maneuver.ut - now) as f32;
    if maneuver.is_active() && time_to_node <= 0.0 {
        // Node is in the past — auto-remove
        warn!("Maneuver node expired (time_to_node={time_to_node:.1}s), auto-removing");
        *maneuver = ManeuverNode::default();
    }
    if maneuver.is_active() {
        let speed = vel.length();
        if speed > 0.1 {
            // Propagate current orbit to the maneuver time
            let (future_pos, future_vel) = propagate_kepler(pos, vel, mu, time_to_node);

            // Draw the trajectory segment from current position to the node (gray)
            let mut trail_points = vec![planet_tf.translation + pos];
            let trail_steps = 50;
            for i in 1..=trail_steps {
                let t = time_to_node * (i as f32 / trail_steps as f32);
                let (rp, _) = propagate_kepler(pos, vel, mu, t);
                trail_points.push(planet_tf.translation + rp);
            }
            gizmos.linestrip(trail_points, Color::srgb(0.5, 0.5, 0.5));

            // Compute orbital frame at the future position
            let future_speed = future_vel.length();
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
                    let mnv_ap_pe = draw_orbit_gizmo(
                        &mut gizmos,
                        future_pos,
                        post_maneuver_vel,
                        mu,
                        planet_tf.translation,
                        planet.radius,
                        planet.soi_radius,
                        Color::srgb(1.0, 0.9, 0.0),
                    );

                    // Maneuver node marker (orange)
                    let node_world = planet_tf.translation + future_pos;
                    gizmos.sphere(node_world, 12.0, Color::srgb(1.0, 0.3, 0.0));

                    // Ap/Pe on maneuver orbit
                    if let Some(ap_pos) = mnv_ap_pe.apoapsis {
                        gizmos.sphere(ap_pos, marker_size, Color::srgb(1.0, 0.5, 0.3));
                    }
                    if let Some(pe_pos) = mnv_ap_pe.periapsis {
                        gizmos.sphere(pe_pos, marker_size, Color::srgb(0.3, 0.5, 1.0));
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

fn map_view_toggle_system(
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

fn pause_menu_system(
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
            // Resume — return to whichever view was active before pausing
            if keys.just_pressed(KeyCode::Escape) {
                time.unpause();
                *menu_vis = Visibility::Hidden;
                next_state.set(pre_pause.0);
            }
            // Flight View (also resumes)
            if keys.just_pressed(KeyCode::Digit1) {
                time.unpause();
                *menu_vis = Visibility::Hidden;
                next_state.set(AppState::Flight);
            }
            // Main Menu
            if keys.just_pressed(KeyCode::Digit2) {
                time.unpause();
                *menu_vis = Visibility::Hidden;
                next_state.set(AppState::MainMenu);
            }
            // Quit
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
    // Handle non-game states separately
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

    // Focus target and up direction depend on state
    let (target_pos, up_dir) = if *state.get() == AppState::VAB {
        (Vec3::new(2005.0, 0.0, 0.0), Vec3::X)
    } else if *state.get() == AppState::MapView {
        // In MapView, focus on the SOI body, up = Y (looking down at orbital plane)
        let rocket_pos = rocket_query.get_single().map(|t| t.translation).unwrap_or(Vec3::ZERO);
        let (_, body_tf) = find_soi_body(rocket_pos, planet_query.iter());
        (body_tf.translation, Vec3::Y)
    } else {
        // In Flight, focus on rocket, up = radially outward from planet
        let rocket_pos = rocket_query.get_single().map(|t| t.translation).unwrap_or(Vec3::ZERO);
        let (_, body_tf) = find_soi_body(rocket_pos, planet_query.iter());
        let up = (rocket_pos - body_tf.translation).try_normalize().unwrap_or(Vec3::Y);
        (rocket_pos, up)
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
            (2100.0, 30000.0) // Map view: above Kerbin surface to beyond Mun orbit
        } else if *state.get() == AppState::VAB {
            (3.0, 50.0)     // VAB preview zoom
        } else {
            (2.0, 200.0)    // Flight view zoom
        };

        orbit.distance += zoom_delta;

        // If switching to MapView, ensure we are outside the planet
        if *state.get() == AppState::MapView && orbit.distance < min_dist {
            orbit.distance = 5000.0;
        }

        orbit.distance = orbit.distance.clamp(min_dist, max_dist);

        // Calculate new position
        // Align the Y-up orbit rotation to the local up direction
        let align = Quat::from_rotation_arc(Vec3::Y, up_dir);
        let local_rot = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0);
        let rotation = align * local_rot;
        let position = target_pos + rotation * Vec3::new(0.0, 0.0, orbit.distance);

        transform.translation = position;
        transform.look_at(target_pos, up_dir);
    }
}
