// ===== World / Celestial Bodies =====

pub const KERBIN_RADIUS: f32 = 2000.0;
pub const KERBIN_SURFACE_GRAVITY: f32 = 5.0;
pub const KERBIN_ATMOSPHERE_HEIGHT: f32 = 150.0;
pub const KERBIN_SOI_RADIUS: f32 = 16000.0;
pub const KERBIN_ROTATION_SPEED: f32 = 0.025;

pub const MUN_ORBIT_RADIUS: f32 = 14000.0;
pub const MUN_RADIUS: f32 = 200.0;
pub const MUN_SURFACE_GRAVITY: f32 = 1.0;
pub const MUN_SOI_RADIUS: f32 = 2500.0;

pub const MINMUS_ORBIT_RADIUS: f32 = 30000.0;
pub const MINMUS_RADIUS: f32 = 60.0;
pub const MINMUS_SURFACE_GRAVITY: f32 = 0.12;
pub const MINMUS_SOI_RADIUS: f32 = 1500.0;
/// Orbital inclination in degrees.
pub const MINMUS_INCLINATION_DEG: f32 = 6.0;

// ===== Sun =====

pub const SUN_ORBIT_RADIUS: f32 = 50000.0;
pub const SUN_RADIUS: f32 = 6000.0;

// ===== Launch Pad =====

pub const LAUNCH_PAD_WIDTH: f32 = 6.0;
pub const LAUNCH_PAD_THICKNESS: f32 = 0.5;
/// Y offset of the launch pad surface above the planet center (along radial axis).
/// = KERBIN_RADIUS + LAUNCH_PAD_THICKNESS / 2.0
pub const LAUNCH_PAD_SURFACE_Y: f32 = KERBIN_RADIUS + LAUNCH_PAD_THICKNESS / 2.0;

// ===== Rocket Stacking =====

pub const STAGE_GAP: f32 = 0.45;
pub const PAD_CLEARANCE: f32 = 0.8;

// ===== Part Geometry Multipliers =====

pub const NOSE_CONE_HEIGHT_FACTOR: f32 = 2.0;
pub const NOZZLE_HEIGHT_FACTOR: f32 = 0.8;
pub const NOZZLE_RADIUS_TOP_FACTOR: f32 = 0.3;
pub const NOZZLE_RADIUS_BOTTOM_FACTOR: f32 = 0.6;
pub const NOZZLE_COLLIDER_RADIUS_FACTOR: f32 = 0.5;
pub const EXHAUST_FLAME_RADIUS_FACTOR: f32 = 0.5;

// ===== Part Colors =====

use bevy::prelude::Color;

pub const UPPER_STAGE_COLOR: Color = Color::srgb(0.8, 0.8, 0.8);
pub const BOOSTER_COLOR: Color = Color::srgb(0.6, 0.6, 0.7);
pub const HEAVY_BOOSTER_COLOR: Color = Color::srgb(0.7, 0.5, 0.3);
pub const NOSE_CONE_COLOR: Color = Color::srgb(0.9, 0.1, 0.1);
pub const ENGINE_COLOR: Color = Color::srgb(0.2, 0.2, 0.2);
pub const EXHAUST_FLAME_COLOR: Color = Color::srgb(1.0, 0.5, 0.0);

// ===== Physics =====

pub const DRAG_COEFFICIENT: f32 = 0.5;
pub const ATMOSPHERE_SCALE_HEIGHT_DIVISOR: f32 = 5.0;
pub const MIN_DIST_SQ_FOR_GRAVITY: f32 = 0.1;
pub const MIN_VELOCITY_SQ_FOR_DRAG: f32 = 0.001;
pub const FORCE_UPDATE_THRESHOLD_SQ: f32 = 0.001;

// ===== Rocket Controls =====

pub const THROTTLE_CHANGE_RATE: f32 = 0.5;
pub const ROCKET_TORQUE: f32 = 8000.0;
pub const SAS_DAMPING: f32 = 2000.0;

// ===== Kepler / Orbit =====

pub const KEPLER_MAX_STEP_SIZE: f32 = 0.5;
pub const MIN_RADIUS_FOR_KEPLER: f32 = 0.001;

// ===== Maneuver Nodes =====

pub const DEFAULT_MANEUVER_PROGRADE: f32 = 100.0;
pub const DEFAULT_MANEUVER_TIME_OFFSET: f64 = 300.0;
pub const MANEUVER_DV_STEP: f32 = 50.0;
pub const MANEUVER_TIME_STEP: f64 = 30.0;

// ===== Time Warp =====

pub const TIME_WARP_RATES: [f32; 4] = [1.0, 2.0, 5.0, 10.0];

// ===== Floating Origin =====

pub const FLOATING_ORIGIN_THRESHOLD: f32 = 50000.0;

// ===== Starfield =====

pub const STARFIELD_COUNT: usize = 3000;

// ===== Debug =====

/// Altitude above Mun surface for the debug orbit spawn.
pub const DEBUG_ORBIT_ALTITUDE: f32 = 100.0;

// ===== VAB =====

/// VAB preview / camera target: just above the launch pad on the equator (+X).
pub const VAB_ORIGIN_X: f32 = KERBIN_RADIUS + 5.0;
