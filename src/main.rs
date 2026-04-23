use bevy::prelude::*;
use bevy::ecs::system::ParamSet;
use bevy::time::Real;
use bevy::ecs::event::EventReader;
use bevy::input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel};
use bevy::core_pipeline::Skybox;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy_rapier3d::prelude::*;

mod vab;
mod flight;
mod orbit;
mod constants;
mod starfield;
mod navball;

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
    Orbit,
    Quit,
}

#[derive(Resource)]
pub struct DebugLaunched;

// ===== Shared Resources =====

#[derive(Resource)]
pub struct TimeWarp {
    pub rates: Vec<f32>,
    pub index: usize,
}

impl Default for TimeWarp {
    fn default() -> Self {
        Self {
            rates: constants::TIME_WARP_RATES.to_vec(),
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
    pub sas_mode: SasMode,
}

#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub enum SasMode {
    #[default]
    Stability,
    Prograde,
    Retrograde,
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

#[derive(Component)]
pub struct FloatingOrigin;

#[derive(Resource, Default)]
pub struct FloatingOriginOffset(pub Vec3);

#[derive(Resource)]
pub struct FloatingOriginSettings {
    pub threshold: f32,
}

impl Default for FloatingOriginSettings {
    fn default() -> Self {
        Self { threshold: constants::FLOATING_ORIGIN_THRESHOLD }
    }
}

#[derive(Component)]
pub struct OrbitAngle(pub f32);

#[derive(Component)]
pub struct OrbitParent(pub Entity);

#[derive(Component)]
pub struct OrbitInclination(pub f32);

#[derive(Component)]
struct SunLight;

#[derive(Component)]
struct SunMarker;

// ===== Main =====

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .insert_resource(TimestepMode::Interpolated {
            dt: 1.0 / 60.0, // physics timestep
            time_scale: 1.0,
            substeps: 1,
        })
        .init_state::<AppState>()
        .init_resource::<TimeWarp>()
        .init_resource::<ManeuverNode>()
        .init_resource::<PrePauseView>()
        .init_resource::<RocketConfig>()
        .init_resource::<FloatingOriginOffset>()
        .init_resource::<FloatingOriginSettings>()
        .add_systems(Startup, setup_scene)
        // State transitions — cleanup before spawn so floating origin resets first
        .add_systems(OnEnter(AppState::MainMenu), flight::cleanup_game)
        .add_systems(OnEnter(AppState::MainMenu), spawn_main_menu)
        .add_systems(OnExit(AppState::MainMenu), despawn_main_menu)
        .add_systems(OnEnter(AppState::VAB), flight::cleanup_game)
        .add_systems(OnEnter(AppState::VAB), vab::spawn_vab)
        .add_systems(OnExit(AppState::VAB), vab::despawn_vab)
        .add_systems(OnEnter(AppState::Flight), flight::spawn_flight)
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
                flight::debug_orbit_apply_system,
                navball::navball_system,
            ),
        )
        .add_systems(
            PostUpdate,
            (
                floating_origin_system
                    .before(PhysicsSet::SyncBackend),
                sun_light_system
                    .after(PhysicsSet::Writeback)
                    .before(TransformSystem::TransformPropagate),
                camera_controller
                    .after(PhysicsSet::Writeback)
                    .before(TransformSystem::TransformPropagate),
            ),
        )
        .run();
}

// ===== Scene Setup =====

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    let kerbin_mu = constants::KERBIN_SURFACE_GRAVITY * constants::KERBIN_RADIUS * constants::KERBIN_RADIUS;

    // Rotate mesh so Bevy's UV sphere +Z pole aligns with world +Y (our north)
    let kerbin_pole_rot = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
    let kerbin_entity = commands.spawn((
        Mesh3d(meshes.add(Sphere::new(constants::KERBIN_RADIUS).mesh().uv(32, 18))),
        MeshMaterial3d(materials.add(generate_kerbin_material(&mut images))),
        Transform::from_xyz(0.0, 0.0, 0.0).with_rotation(kerbin_pole_rot),
        Collider::ball(constants::KERBIN_RADIUS),
        RigidBody::Fixed,
        CelestialBody {
            name: "Kerbin".to_string(),
            mu: kerbin_mu,
            radius: constants::KERBIN_RADIUS,
            atmosphere_height: constants::KERBIN_ATMOSPHERE_HEIGHT,
            soi_radius: constants::KERBIN_SOI_RADIUS,
            orbit_radius: 0.0,
            orbit_speed: 0.0,
            rotation_speed: constants::KERBIN_ROTATION_SPEED,
        },
    )).id();

    let mun_mu = constants::MUN_SURFACE_GRAVITY * constants::MUN_RADIUS * constants::MUN_RADIUS;
    let mun_orbital_speed = f32::sqrt(kerbin_mu / constants::MUN_ORBIT_RADIUS);
    let mun_angular_speed = mun_orbital_speed / constants::MUN_ORBIT_RADIUS;
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(constants::MUN_RADIUS).mesh().uv(24, 12))),
        MeshMaterial3d(materials.add(generate_mun_material(&mut images))),
        Transform::from_xyz(constants::MUN_ORBIT_RADIUS, 0.0, 0.0).with_rotation(kerbin_pole_rot),
        Collider::ball(constants::MUN_RADIUS),
        RigidBody::KinematicPositionBased,
        CelestialBody {
            name: "Mun".to_string(),
            mu: mun_mu,
            radius: constants::MUN_RADIUS,
            atmosphere_height: 0.0,
            soi_radius: constants::MUN_SOI_RADIUS,
            orbit_radius: constants::MUN_ORBIT_RADIUS,
            orbit_speed: mun_angular_speed,
            rotation_speed: 0.0,
        },
        OrbitAngle(0.0),
        OrbitParent(kerbin_entity),
    ));

    // Minmus — small, distant, inclined orbit
    let minmus_mu = constants::MINMUS_SURFACE_GRAVITY * constants::MINMUS_RADIUS * constants::MINMUS_RADIUS;
    let minmus_orbital_speed = f32::sqrt(kerbin_mu / constants::MINMUS_ORBIT_RADIUS);
    let minmus_angular_speed = minmus_orbital_speed / constants::MINMUS_ORBIT_RADIUS;
    let minmus_inclination = constants::MINMUS_INCLINATION_DEG.to_radians();
    // Start at ascending node (inclined orbit, initial angle = 0)
    let minmus_start = Quat::from_rotation_z(minmus_inclination) * Vec3::new(constants::MINMUS_ORBIT_RADIUS, 0.0, 0.0);
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(constants::MINMUS_RADIUS).mesh().ico(5).unwrap())),
        MeshMaterial3d(materials.add(Color::srgb(0.55, 0.7, 0.5))),
        Transform::from_translation(minmus_start),
        Collider::ball(constants::MINMUS_RADIUS),
        RigidBody::KinematicPositionBased,
        CelestialBody {
            name: "Minmus".to_string(),
            mu: minmus_mu,
            radius: constants::MINMUS_RADIUS,
            atmosphere_height: 0.0,
            soi_radius: constants::MINMUS_SOI_RADIUS,
            orbit_radius: constants::MINMUS_ORBIT_RADIUS,
            orbit_speed: minmus_angular_speed,
            rotation_speed: 0.0,
        },
        OrbitAngle(0.0),
        OrbitParent(kerbin_entity),
        OrbitInclination(minmus_inclination),
    ));

    // Sun — orbits Kerbin in ECI frame, angular speed matches Kerbin rotation for day/night
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(constants::SUN_RADIUS).mesh().ico(6).unwrap())),
        MeshMaterial3d(materials.add(StandardMaterial {
            emissive: LinearRgba::new(1.0, 0.95, 0.7, 1.0),
            emissive_exposure_weight: 0.0,
            ..default()
        })),
        Transform::from_xyz(constants::SUN_ORBIT_RADIUS, 0.0, 0.0),
        CelestialBody {
            name: "Sun".to_string(),
            mu: 0.0,
            radius: constants::SUN_RADIUS,
            atmosphere_height: 0.0,
            soi_radius: 0.0,
            orbit_radius: constants::SUN_ORBIT_RADIUS,
            orbit_speed: constants::KERBIN_ROTATION_SPEED,
            rotation_speed: 0.0,
        },
        OrbitAngle(0.0),
        OrbitParent(kerbin_entity),
        SunMarker,
    ));

    let equator_rot = Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2);
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(constants::LAUNCH_PAD_WIDTH, constants::LAUNCH_PAD_THICKNESS, constants::LAUNCH_PAD_WIDTH))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.3, 0.3))),
        Transform::from_xyz(constants::LAUNCH_PAD_SURFACE_Y, 0.0, 0.0).with_rotation(equator_rot),
        RigidBody::Fixed,
        Collider::cuboid(constants::LAUNCH_PAD_WIDTH / 2.0, constants::LAUNCH_PAD_THICKNESS / 2.0, constants::LAUNCH_PAD_WIDTH / 2.0),
    ));

    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 10_000.,
            ..default()
        },
        Transform::from_xyz(constants::SUN_ORBIT_RADIUS, 0.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
        SunLight,
    ));

    // Starfield — procedural cubemap skybox (must be on camera entity)
    let skybox_handle = starfield::create_starfield_cubemap(&mut images);

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(5000.0, 2000.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
        OrbitCamera {
            distance: 5000.0,
            pitch: 0.3,
            yaw: 0.0,
        },
        Skybox {
            image: skybox_handle,
            brightness: 500.0,
            rotation: Quat::IDENTITY,
        },
    ));
}

// ===== Procedural Kerbin Texture =====

fn generate_kerbin_material(images: &mut Assets<Image>) -> StandardMaterial {
    let w = 512u32;
    let h = 256u32;
    let mut data = vec![0u8; (w * h * 4) as usize];

    for py in 0..h {
        for px in 0..w {
            let u = px as f32 / w as f32;
            let v = py as f32 / h as f32;

            // Bevy UV sphere: v=0 → +Z pole, u → angle in XY
            let stack_angle = std::f32::consts::FRAC_PI_2 - v * std::f32::consts::PI;
            let sector_angle = u * std::f32::consts::TAU;
            let dir = Vec3::new(
                stack_angle.cos() * sector_angle.cos(),
                stack_angle.cos() * sector_angle.sin(),
                stack_angle.sin(),
            );

            // Latitude from mesh Z-axis (pole), now rotated to match world +Y
            let lat = dir.z.asin();

            // Multi-octave value noise on 3D position
            let n = fbm(dir * 2.5, 5);

            // Polar ice
            let polar = (lat.abs() - 1.1).max(0.0) * 5.0;

            let (r, g, b) = if polar > 0.5 || n > 0.62 {
                // Land / ice (~30% of surface)
                if polar > 0.5 {
                    // Ice caps
                    let t = polar.min(1.0);
                    (200 + (55.0 * t) as u8, 210 + (45.0 * t) as u8, 220 + (35.0 * t) as u8)
                } else if n > 0.78 {
                    // Mountains / highlands
                    let t = ((n - 0.78) / 0.22).min(1.0);
                    (100 + (50.0 * t) as u8, 85 + (40.0 * t) as u8, 55 + (30.0 * t) as u8)
                } else {
                    // Lowland green
                    let t = (n - 0.62) / 0.16;
                    (35 + (40.0 * t) as u8, 110 + (50.0 * t) as u8, 45 + (20.0 * t) as u8)
                }
            } else if n > 0.55 {
                // Coast / shallow water
                let t = (n - 0.55) / 0.07;
                (30 + (25.0 * t) as u8, 80 + (60.0 * t) as u8, 140 + (20.0 * t) as u8)
            } else {
                // Deep ocean (~70% of surface)
                let depth = n / 0.55;
                (15 + (15.0 * depth) as u8, 40 + (40.0 * depth) as u8, 100 + (50.0 * depth) as u8)
            };

            let idx = (py * w + px) as usize * 4;
            data[idx] = r;
            data[idx + 1] = g;
            data[idx + 2] = b;
            data[idx + 3] = 255;
        }
    }

    let image = Image::new(
        Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        bevy::asset::RenderAssetUsages::RENDER_WORLD | bevy::asset::RenderAssetUsages::MAIN_WORLD,
    );

    StandardMaterial {
        base_color_texture: Some(images.add(image)),
        perceptual_roughness: 0.85,
        ..default()
    }
}

fn generate_mun_material(images: &mut Assets<Image>) -> StandardMaterial {
    let w = 512u32;
    let h = 256u32;
    let mut data = vec![0u8; (w * h * 4) as usize];

    // Pre-generate crater centers using hash-based positions
    // Each crater: (direction on unit sphere, radius)
    let crater_seeds: Vec<(Vec3, f32)> = (0..80).map(|i| {
        let theta = hash3(i * 7, i * 13, i * 3) * std::f32::consts::TAU;
        let phi = (hash3(i * 11, i * 17, i * 5) * 2.0 - 1.0).asin();
        let dir = Vec3::new(
            phi.cos() * theta.cos(),
            phi.cos() * theta.sin(),
            phi.sin(),
        );
        let radius = 0.05 + hash3(i * 19, i * 23, i * 29) * 0.2;
        (dir, radius)
    }).collect();

    for py in 0..h {
        for px in 0..w {
            let u = px as f32 / w as f32;
            let v = py as f32 / h as f32;

            let stack_angle = std::f32::consts::FRAC_PI_2 - v * std::f32::consts::PI;
            let sector_angle = u * std::f32::consts::TAU;
            let dir = Vec3::new(
                stack_angle.cos() * sector_angle.cos(),
                stack_angle.cos() * sector_angle.sin(),
                stack_angle.sin(),
            );

            // Base surface: subtle noise variation
            let base = 0.45 + fbm(dir * 4.0, 4) * 0.15;

            // Crater contribution
            let mut crater_val = 0.0;
            for (c_dir, c_r) in &crater_seeds {
                let dot = dir.dot(*c_dir);
                if dot < 0.0 { continue; }
                let ang = dot.acos();
                if ang < *c_r {
                    let t = ang / c_r;
                    // Bowl shape: dip in center, raised rim
                    if t < 0.7 {
                        crater_val -= 0.15 * (1.0 - t / 0.7);
                    } else {
                        crater_val += 0.08 * ((t - 0.7) / 0.3).min(1.0);
                    }
                }
            }

            let val = (base + crater_val).clamp(0.15, 0.75);
            let grey = (val * 255.0) as u8;
            // Slight warm tint for highlands, cool for low
            let r = (grey as f32 * 1.05).min(255.0) as u8;
            let g = grey;
            let b = (grey as f32 * 0.95).min(255.0) as u8;

            let idx = (py * w + px) as usize * 4;
            data[idx] = r;
            data[idx + 1] = g;
            data[idx + 2] = b;
            data[idx + 3] = 255;
        }
    }

    let image = Image::new(
        Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        bevy::asset::RenderAssetUsages::RENDER_WORLD | bevy::asset::RenderAssetUsages::MAIN_WORLD,
    );

    StandardMaterial {
        base_color_texture: Some(images.add(image)),
        perceptual_roughness: 0.95,
        ..default()
    }
}

/// Simple hash-based value noise on 3D integer grid.
fn hash3(x: i32, y: i32, z: i32) -> f32 {
    let mut h = (x as u32).wrapping_mul(374761393)
        .wrapping_add((y as u32).wrapping_mul(668265263))
        .wrapping_add((z as u32).wrapping_mul(1274126177));
    h = (h ^ (h >> 13)).wrapping_mul(1274126177);
    h = (h ^ (h >> 16)).wrapping_mul(668265263);
    h as f32 / u32::MAX as f32
}

/// Smooth value noise with trilinear interpolation.
fn vnoise(p: Vec3) -> f32 {
    let ix = p.x.floor() as i32;
    let iy = p.y.floor() as i32;
    let iz = p.z.floor() as i32;
    let fx = p.x - ix as f32;
    let fy = p.y - iy as f32;
    let fz = p.z - iz as f32;
    // Smoothstep
    let sx = fx * fx * (3.0 - 2.0 * fx);
    let sy = fy * fy * (3.0 - 2.0 * fy);
    let sz = fz * fz * (3.0 - 2.0 * fz);

    let mut acc = 0.0;
    for dz in 0..=1i32 {
        for dy in 0..=1i32 {
            for dx in 0..=1i32 {
                let v = hash3(ix + dx, iy + dy, iz + dz);
                let wx = if dx == 0 { 1.0 - sx } else { sx };
                let wy = if dy == 0 { 1.0 - sy } else { sy };
                let wz = if dz == 0 { 1.0 - sz } else { sz };
                acc += v * wx * wy * wz;
            }
        }
    }
    acc
}

/// Fractal Brownian Motion — layered noise.
fn fbm(p: Vec3, octaves: u32) -> f32 {
    let mut val = 0.0;
    let mut amp = 0.5;
    let mut freq = 1.0;
    let mut pos = p;
    for _ in 0..octaves {
        val += amp * vnoise(pos * freq);
        freq *= 2.0;
        amp *= 0.5;
        pos = Vec3::new(pos.z * 1.1, pos.x * 0.9, pos.y * 1.05);
    }
    val
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
            BorderColor(Color::srgb(0.3, 0.5, 0.8)),
            BackgroundColor(Color::srgb(0.15, 0.25, 0.45)),
            MenuButton::Orbit,
        )).with_children(|button| {
            button.spawn((
                Text::new("ORBIT"),
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
    mut commands: Commands,
    q: Query<(&Interaction, &MenuButton), Changed<Interaction>>,
    mut next_state: ResMut<NextState<AppState>>,
    mut exit_events: EventWriter<AppExit>,
) {
    for (interaction, button) in q.iter() {
        if *interaction == Interaction::Pressed {
            match button {
                MenuButton::Start => next_state.set(AppState::VAB),
                MenuButton::Orbit => {
                    commands.insert_resource(DebugLaunched);
                    next_state.set(AppState::Flight);
                }
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
        (Vec3::new(constants::VAB_ORIGIN_X, 0.0, 0.0), Vec3::X)
    } else if *state.get() == AppState::MapView {
        let rocket_pos = rocket_query.get_single().map(|t| t.translation).unwrap_or(Vec3::ZERO);
        let (_, body_tf) = orbit::find_soi_body(rocket_pos, planet_query.iter(), false);
        (body_tf.translation, Vec3::Y)
    } else {
        let rocket_pos = rocket_query.get_single().map(|t| t.translation).unwrap_or(Vec3::ZERO);
        let (_, body_tf) = orbit::find_soi_body(rocket_pos, planet_query.iter(), false);
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

// ===== Floating Origin =====

fn floating_origin_system(
    settings: Res<FloatingOriginSettings>,
    mut offset: ResMut<FloatingOriginOffset>,
    mut set: ParamSet<(
        Query<&Transform, With<FloatingOrigin>>,
        Query<&mut Transform>,
    )>,
) {
    let pos = match set.p0().get_single() {
        Ok(tf) => tf.translation,
        Err(_) => return,
    };
    if pos.length() < settings.threshold { return; }

    for mut tf in set.p1().iter_mut() {
        tf.translation -= pos;
    }

    offset.0 += pos;
    debug!("floating_origin: shifted by ({:.1},{:.1},{:.1}) | cumulative offset=({:.1},{:.1},{:.1})",
        pos.x, pos.y, pos.z,
        offset.0.x, offset.0.y, offset.0.z,
    );
}

// ===== Sun Light =====

fn sun_light_system(
    sun_q: Query<&Transform, (With<SunMarker>, Without<SunLight>)>,
    mut light_q: Query<&mut Transform, With<SunLight>>,
) {
    let Ok(sun_tf) = sun_q.get_single() else { return };
    for mut light_tf in light_q.iter_mut() {
        light_tf.translation = sun_tf.translation;
        light_tf.look_at(Vec3::ZERO, Vec3::Y);
    }
}
