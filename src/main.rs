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
    Paused,
}

#[derive(Component)]
struct PauseMenuUI;

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

/// A maneuver node: planned delta-v burn at a future point on the orbit.
/// Components are in orbital frame: prograde (along velocity),
/// normal (along angular momentum), radial (in-plane toward body).
#[derive(Resource, Default)]
struct ManeuverNode {
    prograde: f32,
    normal: f32,
    radial: f32,
    time_offset: f32, // seconds into the future where the node is placed
}

impl ManeuverNode {
    fn is_active(&self) -> bool {
        self.prograde.abs() > 0.01 || self.normal.abs() > 0.01 || self.radial.abs() > 0.01
    }
}

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
        .init_resource::<ManeuverNode>()
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
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
    soi_radius: f32, // sphere of influence radius
    orbit_radius: f32, // distance from parent body (0 if root)
    orbit_speed: f32, // angular velocity around parent (rad/s, 0 if root)
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
    // --- World Constants ---
    let kerbin_radius = 2000.0;
    let kerbin_g = 5.0;
    let kerbin_mu = kerbin_g * kerbin_radius * kerbin_radius; // 20,000,000

    // Planet (Sphere) — Kerbin
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(kerbin_radius))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.6, 0.4))),
        Transform::from_xyz(0.0, -kerbin_radius, 0.0),
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
        },
    ));

    // Moon — The Mun
    let mun_orbit_radius = 14000.0;
    let mun_radius = 200.0;
    let mun_g = 1.0;
    let mun_mu = mun_g * mun_radius * mun_radius; // 40,000
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

    // Rocket mass/thrust constants
    let s0_dry = 600.0;
    let s0_fuel = 400.0;
    let s0_thrust = 12000.0;
    let s0_burn_rate = 6.0;

    let s1_dry = 600.0;
    let s1_fuel = 1000.0;
    let s1_thrust = 40000.0;
    let s1_burn_rate = 18.0;

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
        Transform::from_xyz(0.0, 4.25, 0.0),
        RigidBody::Dynamic,
        GravityScale(0.0),
        Collider::cylinder(0.5, 0.5),
        ColliderMassProperties::Mass(s0_dry + s0_fuel),
        ExternalForce::default(),
        Velocity::default(),
        Rocket {
            throttle: 0.0,
            is_launched: false,
            current_stage: 1,
        },
        FuelTank {
            fuel_mass: s0_fuel,
            dry_mass: s0_dry,
        },
        Engine {
            max_thrust: s0_thrust,
            fuel_burn_rate: s0_burn_rate,
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
        Transform::from_xyz(0.0, 2.3, 0.0),
        RigidBody::Dynamic,
        GravityScale(0.0),
        Collider::cylinder(1.0, 0.5),
        ColliderMassProperties::Mass(s1_dry + s1_fuel),
        ExternalForce::default(),
        Velocity::default(),
        FuelTank {
            fuel_mass: s1_fuel,
            dry_mass: s1_dry,
        },
        Engine {
            max_thrust: s1_thrust,
            fuel_burn_rate: s1_burn_rate,
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
            distance: 20.0,
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
    )).with_children(|parent| {
        parent.spawn((
            Text::new("PAUSED"),
            TextFont { font_size: 48.0, ..default() },
            TextColor(Color::WHITE),
        ));
        parent.spawn((
            Text::new("[ESC] Resume\n[1] Flight View\n[2] Reset\n[3] Quit"),
            TextFont { font_size: 24.0, ..default() },
            TextColor(Color::srgb(0.8, 0.8, 0.8)),
        ));
    });
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
    let vel_mag = velocity.linvel.length();

    let pitch_deg = transform.up().dot(local_up).asin().to_degrees();
    let vertical_vel = velocity.linvel.dot(local_up);
    let surface_vel = (velocity.linvel - local_up * vertical_vel).length();

    let density = if altitude < planet.atmosphere_height && planet.atmosphere_height > 0.0 {
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

    let maneuver_str = if maneuver.is_active() {
        let dv = f32::sqrt(maneuver.prograde * maneuver.prograde + maneuver.normal * maneuver.normal + maneuver.radial * maneuver.radial);
        let t_min = maneuver.time_offset / 60.0;
        let t_sec = maneuver.time_offset % 60.0;
        format!(
            "\n--- Maneuver ---\nT-: {:.0}m {:.0}s\ndV: {:.0} m/s\n  Prograde: {:.0}\n  Normal: {:.0}\n  Radial: {:.0}",
            t_min, t_sec, dv, maneuver.prograde, maneuver.normal, maneuver.radial,
        )
    } else {
        "".to_string()
    };

    text.0 = format!(
        "SOI: {}\nStage: {}\nAltitude: {:.1} m\nPitch (vs Horizon): {:.1} deg\nOrbital Vel: {:.1} m/s\nSurface Vel: {:.1} m/s\nV.Speed: {:.1} m/s\nThrottle: {:.0}%{}\nThrust: {:.0} N\nFuel: {:.0} kg\nAir Density: {:.3}{}",
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
        maneuver_str,
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
/// T/G = adjust time offset +/- (move node earlier/later on orbit)
fn maneuver_node_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut node: ResMut<ManeuverNode>,
) {
    // Toggle/create node
    if keys.just_pressed(KeyCode::KeyN) {
        if node.is_active() {
            *node = ManeuverNode::default();
        } else {
            node.prograde = 100.0;
            node.time_offset = 300.0; // Default 5 minutes ahead
        }
    }

    // Delete node
    if keys.just_pressed(KeyCode::Delete) || keys.just_pressed(KeyCode::Backspace) {
        *node = ManeuverNode::default();
    }

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

    // Adjust time offset (T = later, G = earlier) by 30 seconds
    let time_step = 30.0;
    if keys.just_pressed(KeyCode::KeyT) { node.time_offset += time_step; }
    if keys.just_pressed(KeyCode::KeyG) { node.time_offset = (node.time_offset - time_step).max(0.0); }
}



/// Draw a Keplerian orbit from position/velocity around a body at planet_center.
fn draw_orbit_gizmo(
    gizmos: &mut Gizmos,
    pos: Vec3,
    vel: Vec3,
    mu: f32,
    planet_center: Vec3,
    planet_radius: f32,
    color: Color,
) {
    let altitude = pos.length() - planet_radius;
    let speed = vel.length();
    if altitude < 10.0 || speed < 10.0 { return; }

    let h = pos.cross(vel);
    let h_mag = h.length();
    if h_mag < 0.001 { return; }

    let e_vec = (vel.cross(h) / mu) - pos.normalize();
    let e = e_vec.length();
    let energy = vel.length_squared() / 2.0 - mu / pos.length();

    let a = if (1.0 - e).abs() < 0.001 {
        1000000.0
    } else if e < 1.0 {
        -mu / (2.0 * energy)
    } else {
        mu / (2.0 * energy.abs())
    };

    let p = e_vec.normalize_or(Vec3::X);
    let q = h.cross(p).normalize();

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
            if r_mag > 5000.0 || r_mag < 0.0 { continue; }
            let local_pos = p * (r_mag * nu.cos()) + q * (r_mag * nu.sin());
            points.push(planet_center + local_pos);
        }
        if points.len() > 1 {
            gizmos.linestrip(points, color);
        }
    }
}

fn orbit_prediction_system(
    mut gizmos: Gizmos,
    planet_q: Query<(&CelestialBody, &Transform)>,
    rocket_q: Query<(&Transform, &Velocity, &ColliderMassProperties, Entity), With<Rocket>>,
    part_q: Query<(Entity, &Transform, &ColliderMassProperties, &Velocity), Without<Rocket>>,
    joint_q: Query<&ImpulseJoint>,
    state: Res<State<AppState>>,
    maneuver: Res<ManeuverNode>,
) {
    if *state.get() != AppState::MapView { return; }

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
    let vel = com_vel;
    let mu = planet.mu;

    // Draw current orbit (cyan)
    draw_orbit_gizmo(&mut gizmos, pos, vel, mu, planet_tf.translation, planet.radius, Color::srgb(0.0, 0.8, 1.0));

    // Draw maneuver orbit (yellow) if node exists
    if maneuver.is_active() && maneuver.time_offset >= 0.0 {
        let speed = vel.length();
        if speed > 0.1 {
            // Propagate current orbit to the maneuver time
            let (future_pos, future_vel) = propagate_kepler(pos, vel, mu, maneuver.time_offset);

            // Draw the trajectory segment from current position to the node (gray)
            let mut trail_points = vec![planet_tf.translation + pos];
            let trail_steps = 50;
            for i in 1..=trail_steps {
                let t = maneuver.time_offset * (i as f32 / trail_steps as f32);
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
                    let radial_dir = normal_dir.cross(prograde_dir).normalize();

                    let maneuver_dv = prograde_dir * maneuver.prograde
                        + normal_dir * maneuver.normal
                        + radial_dir * maneuver.radial;

                    let post_maneuver_vel = future_vel + maneuver_dv;
                    draw_orbit_gizmo(
                        &mut gizmos,
                        future_pos,
                        post_maneuver_vel,
                        mu,
                        planet_tf.translation,
                        planet.radius,
                        Color::srgb(1.0, 0.9, 0.0),
                    );

                    // Draw maneuver node marker at future position
                    let node_world = planet_tf.translation + future_pos;
                    gizmos.sphere(node_world, 8.0, Color::srgb(1.0, 0.3, 0.0));
                }
            }
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
) {
    let Ok(mut menu_vis) = menu_q.get_single_mut() else { return };

    match state.get() {
        AppState::Paused => {
            // Resume
            if keys.just_pressed(KeyCode::Escape) {
                time.unpause();
                *menu_vis = Visibility::Hidden;
                next_state.set(AppState::Flight);
            }
            // Flight View (also resumes)
            if keys.just_pressed(KeyCode::Digit1) {
                time.unpause();
                *menu_vis = Visibility::Hidden;
                next_state.set(AppState::Flight);
            }
            // Reset: unpause and go to flight
            if keys.just_pressed(KeyCode::Digit2) {
                time.unpause();
                *menu_vis = Visibility::Hidden;
                next_state.set(AppState::Flight);
            }
            // Quit
            if keys.just_pressed(KeyCode::Digit3) {
                exit_events.send(AppExit::Success);
            }
        }
        AppState::Flight | AppState::MapView => {
            if keys.just_pressed(KeyCode::Escape) {
                time.pause();
                *menu_vis = Visibility::Visible;
                next_state.set(AppState::Paused);
            }
        }
        _ => {}
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
    planet_query: Query<(&CelestialBody, &Transform), (With<CelestialBody>, Without<OrbitCamera>)>,
) {
    let dt = time.delta_secs();

    // Focus target depends on state
    let target_pos = if *state.get() == AppState::MapView {
        // In MapView, focus on the SOI body
        let rocket_pos = rocket_query.get_single().map(|t| t.translation).unwrap_or(Vec3::ZERO);
        let (_, body_tf) = find_soi_body(rocket_pos, planet_query.iter());
        body_tf.translation
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
            (2100.0, 30000.0) // Map view: above Kerbin surface to beyond Mun orbit
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
        let rotation = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0);
        let position = target_pos + rotation * Vec3::new(0.0, 0.0, orbit.distance);

        transform.translation = position;
        transform.look_at(target_pos, Vec3::Y);
    }
}
