use bevy::prelude::*;

use crate::{AppState, OrbitCamera};
use crate::constants::*;

// ===== Components =====

#[derive(Component)]
pub struct VabUI;

#[derive(Component)]
pub struct VabConfigText;

#[derive(Component)]
pub struct VabPreviewEntity;

#[derive(Component)]
pub enum VabButton {
    AddStage,
    RemoveStage,
    CycleStageType(usize, i32),
    Launch,
    Back,
}

// ===== Resources =====

#[derive(Resource, Clone, Debug)]
pub struct RocketConfig {
    pub stages: Vec<StageConfig>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StageType {
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

    pub fn name(self) -> &'static str {
        match self {
            StageType::UpperStage => "Upper Stage",
            StageType::Booster => "Booster",
            StageType::HeavyBooster => "Heavy Booster",
        }
    }

    pub fn cycle(self, dir: i32) -> Self {
        let idx = Self::ALL.iter().position(|&t| t == self).unwrap() as i32;
        let len = Self::ALL.len() as i32;
        let new_idx = ((idx + dir).rem_euclid(len)) as usize;
        Self::ALL[new_idx]
    }
}

#[derive(Clone, Debug)]
pub struct StageConfig {
    pub stage_type: StageType,
    pub dry_mass: f32,
    pub fuel_mass: f32,
    pub max_thrust: f32,
    pub fuel_burn_rate: f32,
    pub radius: f32,
    pub height: f32,
}

impl StageConfig {
    pub fn from_type(stage_type: StageType) -> Self {
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

    pub fn default_upper() -> Self {
        Self::from_type(StageType::UpperStage)
    }

    pub fn default_booster() -> Self {
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

// ===== Helpers =====

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
            stage.max_thrust / (total_mass * KERBIN_SURFACE_GRAVITY),
        ));
    }

    s.push_str(&format!("\nMass: {}kg | dV: {:.0} m/s", total_mass as i32, dv));
    s
}

// ===== 3D Preview =====

fn spawn_vab_preview(
    commands: &mut Commands,
    config: &RocketConfig,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let mat_upper = materials.add(UPPER_STAGE_COLOR);
    let mat_booster = materials.add(BOOSTER_COLOR);
    let mat_heavy = materials.add(HEAVY_BOOSTER_COLOR);
    let mat_nose = materials.add(NOSE_CONE_COLOR);
    let mat_engine = materials.add(ENGINE_COLOR);

    let n_stages = config.stages.len();
    if n_stages == 0 { return; }

    let stage_gap = STAGE_GAP;
    let mut y_positions = Vec::with_capacity(n_stages);
    let mut y = 0.0;
    for i in (0..n_stages).rev() {
        let h = config.stages[i].height;
        y += h / 2.0;
        y_positions.insert(0, y);
        y += h / 2.0;
        y += stage_gap;
    }

    let center_offset = (y_positions[0] + y_positions[n_stages - 1]) / 2.0;
    let vab_origin = Vec3::new(VAB_ORIGIN_X, 0.0, 0.0);

    for i in 0..n_stages {
        let stage = &config.stages[i];
        let local_y = y_positions[i] - center_offset;
        let is_top = i == 0;

        let stage_mat = match stage.stage_type {
            StageType::UpperStage => mat_upper.clone(),
            StageType::Booster => mat_booster.clone(),
            StageType::HeavyBooster => mat_heavy.clone(),
        };

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
                Mesh3d(meshes.add(Cone { radius: stage.radius, height: stage.radius * NOSE_CONE_HEIGHT_FACTOR })),
                MeshMaterial3d(mat_nose.clone()),
                Transform::from_xyz(0.0, stage.height / 2.0 + stage.radius, 0.0),
            ));
        }

        let nozzle_height = stage.radius * NOZZLE_HEIGHT_FACTOR;
        entity_cmds.with_child((
            Mesh3d(meshes.add(ConicalFrustum {
                radius_top: stage.radius * NOZZLE_RADIUS_TOP_FACTOR,
                radius_bottom: stage.radius * NOZZLE_RADIUS_BOTTOM_FACTOR,
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

// ===== Systems =====

pub fn spawn_vab(
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

    spawn_vab_preview(&mut commands, &config, &mut meshes, &mut materials);

    let config_display = vab_config_text(&config);
    spawn_vab_ui(&mut commands, &config, &config_display);
}

pub fn despawn_vab(
    mut commands: Commands,
    ui_q: Query<Entity, With<VabUI>>,
    preview_q: Query<Entity, With<VabPreviewEntity>>,
) {
    if let Ok(entity) = ui_q.get_single() {
        commands.entity(entity).despawn_recursive();
    }
    for entity in preview_q.iter() {
        commands.entity(entity).despawn_recursive();
    }
}

pub fn vab_button_system(
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
        if let Ok(mut text) = text_q.get_single_mut() {
            text.0 = vab_config_text(&config);
        }
        if let Ok(entity) = ui_q.get_single() {
            commands.entity(entity).despawn_recursive();
        }
        spawn_vab_ui(&mut commands, &config, &vab_config_text(&config));
    }
}

// ===== UI Builder =====

fn spawn_vab_ui(commands: &mut Commands, config: &RocketConfig, config_display: &str) {
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
