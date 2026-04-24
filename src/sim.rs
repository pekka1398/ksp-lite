use bevy::prelude::*;

pub use bevy::math::DVec3;

// ===== SSB Frame (Absolute Truth) =====
// Origin = solar system barycenter, Y = polar axis, XZ = orbital plane
// Units: SI (m, m/s, kg, N, s)
// Never offset by floating origin. This is the truth layer.

/// Position in the SSB inertial frame (f64).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SsbPosition(pub DVec3);

/// Velocity in the SSB inertial frame (f64).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SsbVelocity(pub DVec3);

/// Complete kinematic state of an entity in the SSB frame.
/// Attached as a single Component — the source of truth for position and velocity.
#[derive(Clone, Copy, Debug, Default, PartialEq, Component)]
pub struct SimState {
    pub position: SsbPosition,
    pub velocity: SsbVelocity,
}

// ===== Local Frame (Render / Physics) =====
// SSB translated so origin is near the active vessel.
// No rotation — velocities are identical to SSB.
// Units: SI, but stored as f32 for Bevy/Rapier.

/// Accumulated translation from SSB to local.
/// Local = SSB - offset, so SSB = local + offset.
#[derive(Resource, Clone, Copy, Debug, Default, PartialEq)]
pub struct LocalOffset(pub DVec3);

// ===== Conversions =====

impl SsbPosition {
    pub fn to_local(self, offset: &LocalOffset) -> Vec3 {
        (self.0 - offset.0).as_vec3()
    }
}
impl LocalOffset {
    pub fn ssb_from_local(&self, local_pos: Vec3) -> SsbPosition {
        SsbPosition(local_pos.as_dvec3() + self.0)
    }

    pub fn ssb_vel_from_local(&self, local_vel: Vec3) -> SsbVelocity {
        SsbVelocity(local_vel.as_dvec3())
    }
}
