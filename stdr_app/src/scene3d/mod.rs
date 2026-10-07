//! The 3D view and camera sensors. Everything sits under one `SceneRoot` whose rotation maps the
//! ROS frame (Z up) to Bevy's (Y up), so robot mirrors take engine poses as they are: `(x, y, 0)`
//! turned `theta` about Z. Mirrors and sensor cameras exist in both view modes; only the orbit
//! camera depends on the toolbar's 2D/3D toggle.

pub mod camera_sensor;
pub mod mesh;

use std::collections::HashMap;
use std::f32::consts::FRAC_PI_2;

use bevy::camera::visibility::Visibility;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::EguiPrimaryContextPass;
use bevy_egui::input::EguiWantsInput;
use stdr_core::{Pose2D, RobotId};

use crate::sim::{SimWorld, sim_step};
use crate::ui::toolbar::toolbar;
use crate::view2d::{MainCamera, MapTexture, cursor_in_viewport, scroll_notches, sync_map_texture};
use camera_sensor::{CameraFrames, PendingCaptures};
pub use mesh::{extrude_footprint, extrude_grid};

/// Wall height of extruded occupied cells, metres.
pub const WALL_HEIGHT: f32 = 1.0;
/// Height of the extruded robot footprints, metres.
pub const ROBOT_HEIGHT: f32 = 0.2;

/// Orbit drag sensitivity, radians per pixel.
const ORBIT_RAD_PER_PX: f32 = 0.005;
/// Zoom factor per mouse-wheel notch.
const ZOOM_STEP: f32 = 1.1;
/// Lowest and highest orbit elevation, radians: never under the floor or straight down.
const PITCH_RANGE: (f32, f32) = (0.05, 1.5);

#[derive(Resource, Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum ViewMode {
    #[default]
    TwoD,
    ThreeD,
}

pub fn in_2d(mode: Res<ViewMode>) -> bool {
    *mode == ViewMode::TwoD
}

pub fn in_3d(mode: Res<ViewMode>) -> bool {
    *mode == ViewMode::ThreeD
}

/// ROS Z-up → Bevy Y-up: `(x, y, z)` lands at `(x, z, −y)`.
pub fn scene_root_rotation() -> Quat {
    Quat::from_rotation_x(-FRAC_PI_2)
}

/// A robot pose as a mirror transform in the scene root's (ROS) frame.
pub fn robot_transform(p: Pose2D) -> Transform {
    Transform::from_xyz(p.x as f32, p.y as f32, 0.0)
        .with_rotation(Quat::from_rotation_z(p.theta as f32))
}

#[derive(Component)]
pub struct SceneRoot;

/// The extruded walls and the textured floor of the current map.
#[derive(Component)]
struct MapGeometry;

/// A render mirror of one engine robot; never the source of truth.
#[derive(Component)]
pub struct RobotMarker {
    pub id: RobotId,
}

#[derive(Resource, Default)]
pub struct RobotEntities(pub HashMap<RobotId, Entity>);

#[derive(Resource)]
struct SceneMaterials {
    wall: Handle<StandardMaterial>,
    robot: Handle<StandardMaterial>,
}

/// Spherical coordinates of the 3D view camera around `focus` (Bevy frame).
#[derive(Component)]
pub struct OrbitCamera {
    pub focus: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

impl OrbitCamera {
    fn transform(&self) -> Transform {
        let dir = Vec3::new(
            self.pitch.cos() * self.yaw.sin(),
            self.pitch.sin(),
            self.pitch.cos() * self.yaw.cos(),
        );
        Transform::from_translation(self.focus + dir * self.distance)
            .looking_at(self.focus, Vec3::Y)
    }
}

pub struct Scene3dPlugin;

impl Plugin for Scene3dPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ViewMode>()
            .init_resource::<RobotEntities>()
            .init_resource::<CameraFrames>()
            .init_resource::<PendingCaptures>()
            .insert_resource(GlobalAmbientLight {
                brightness: 300.0,
                ..default()
            })
            .add_systems(Startup, spawn_scene)
            .add_systems(
                PreUpdate,
                sync_map_geometry
                    .after(sync_map_texture)
                    .run_if(resource_changed::<MapTexture>),
            )
            .add_systems(FixedUpdate, camera_sensor::collect_captures.after(sim_step))
            .add_systems(
                Update,
                (
                    sync_robot_mirrors,
                    camera_sensor::apply_captures,
                    apply_view_mode,
                    orbit_camera.run_if(in_3d),
                ),
            )
            .add_systems(
                EguiPrimaryContextPass,
                camera_sensor::cameras_tab.after(toolbar),
            );
    }
}

fn spawn_scene(mut commands: Commands, mut materials: ResMut<Assets<StandardMaterial>>) {
    commands.spawn((
        SceneRoot,
        Transform::from_rotation(scene_root_rotation()),
        Visibility::default(),
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(0.3, 1.0, 0.6).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Camera3d::default(),
        // Under the egui host camera, which draws the panes on top.
        Camera {
            is_active: false,
            order: -1,
            ..default()
        },
        Tonemapping::None,
        OrbitCamera {
            focus: Vec3::ZERO,
            yaw: 0.0,
            pitch: 1.0,
            distance: 20.0,
        },
    ));
    commands.insert_resource(SceneMaterials {
        wall: materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.4, 0.55),
            perceptual_roughness: 0.9,
            ..default()
        }),
        robot: materials.add(StandardMaterial {
            base_color: Color::srgb(0.0, 0.7, 0.0),
            perceptual_roughness: 0.6,
            ..default()
        }),
    });
}

/// On a new map texture: replace the walls and floor and refit the orbit camera.
#[allow(clippy::too_many_arguments)]
fn sync_map_geometry(
    sim: Res<SimWorld>,
    tex: Res<MapTexture>,
    root: Single<Entity, With<SceneRoot>>,
    old: Query<Entity, With<MapGeometry>>,
    mats: Res<SceneMaterials>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    orbit: Single<&mut OrbitCamera>,
    mut commands: Commands,
) {
    let Some(grid) = sim.map().filter(|_| tex.revision != 0) else {
        return;
    };
    for e in &old {
        commands.entity(e).despawn();
    }
    let rect = tex.rect;
    commands.spawn((
        MapGeometry,
        Mesh3d(meshes.add(extrude_grid(grid, WALL_HEIGHT))),
        MeshMaterial3d(mats.wall.clone()),
        ChildOf(*root),
    ));
    commands.spawn((
        MapGeometry,
        Mesh3d(meshes.add(Rectangle::from_size(rect.size()))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(tex.image.clone()),
            unlit: true,
            ..default()
        })),
        Transform::from_translation(rect.center().extend(0.0)),
        ChildOf(*root),
    ));
    let mut orbit = orbit.into_inner();
    orbit.focus = scene_root_rotation() * rect.center().extend(0.0);
    orbit.distance = rect.size().length() * 0.8;
}

/// Spawns, moves and despawns one mirror per engine robot (with its sensor cameras).
#[allow(clippy::too_many_arguments)]
fn sync_robot_mirrors(
    sim: Res<SimWorld>,
    mut entities: ResMut<RobotEntities>,
    mut frames: ResMut<CameraFrames>,
    mut mirrors: Query<&mut Transform, With<RobotMarker>>,
    root: Single<Entity, With<SceneRoot>>,
    mats: Res<SceneMaterials>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut commands: Commands,
) {
    entities.0.retain(|id, e| {
        let alive = sim.robot(*id).is_some();
        if !alive {
            commands.entity(*e).despawn();
        }
        alive
    });
    frames.0.retain(|(id, _), _| sim.robot(*id).is_some());
    for (id, r) in sim.robots() {
        let t = robot_transform(r.state.pose);
        if let Some(mut current) = entities.0.get(&id).and_then(|&e| mirrors.get_mut(e).ok()) {
            current.set_if_neq(t);
            continue;
        }
        let e = commands
            .spawn((
                RobotMarker { id },
                Mesh3d(meshes.add(extrude_footprint(&r.config.footprint, ROBOT_HEIGHT))),
                MeshMaterial3d(mats.robot.clone()),
                t,
                ChildOf(*root),
            ))
            .with_children(|c| {
                camera_sensor::spawn_camera_sensors(c, id, r, &mut images, &mut frames)
            })
            .id();
        entities.0.insert(id, e);
    }
}

/// The Map pane shows either the 2D camera or the orbit camera.
fn apply_view_mode(
    mode: Res<ViewMode>,
    mut orbit: Single<&mut Camera, (With<OrbitCamera>, Without<MainCamera>)>,
    mut main: Single<&mut Camera, With<MainCamera>>,
) {
    let three_d = *mode == ViewMode::ThreeD;
    if orbit.is_active != three_d {
        orbit.is_active = three_d;
    }
    if main.is_active == three_d {
        main.is_active = !three_d;
    }
}

/// Left-drag orbits, middle-drag pans along the floor, the wheel zooms; only over the Map pane.
fn orbit_camera(
    egui_input: Res<EguiWantsInput>,
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    window: Single<&Window, With<PrimaryWindow>>,
    cam: Single<(&Camera, &mut OrbitCamera, &mut Transform)>,
) {
    let (camera, mut orbit, mut t) = cam.into_inner();
    if !egui_input.wants_pointer_input() && cursor_in_viewport(&window, camera).is_some() {
        let d = motion.delta;
        if buttons.pressed(MouseButton::Left) {
            orbit.yaw -= d.x * ORBIT_RAD_PER_PX;
            orbit.pitch =
                (orbit.pitch + d.y * ORBIT_RAD_PER_PX).clamp(PITCH_RANGE.0, PITCH_RANGE.1);
        }
        if buttons.pressed(MouseButton::Middle) {
            // Screen right and screen up projected onto the floor; one pixel ≈ the same share
            // of the view at any zoom.
            let right = Vec3::new(orbit.yaw.cos(), 0.0, -orbit.yaw.sin());
            let away = Vec3::new(-orbit.yaw.sin(), 0.0, -orbit.yaw.cos());
            let scale = orbit.distance * ORBIT_RAD_PER_PX * 0.3;
            orbit.focus += (-right * d.x + away * d.y) * scale;
        }
        let notches = scroll_notches(&scroll);
        if notches != 0.0 {
            orbit.distance *= ZOOM_STEP.powf(-notches);
        }
    }
    t.set_if_neq(orbit.transform());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn root_maps_ros_z_up_to_bevy_y_up() {
        let p = scene_root_rotation() * Vec3::new(2.0, 3.0, 1.5);
        assert!((p - Vec3::new(2.0, 1.5, -3.0)).length() < 1e-6);
    }

    #[test]
    fn extruded_wall_top_is_bevy_up() {
        let grid = stdr_core::OccupancyGrid::new(1, 1, 1.0, Pose2D::default(), vec![100]).unwrap();
        let m = extrude_grid(&grid, WALL_HEIGHT);
        let ys: Vec<f32> = m
            .attribute(Mesh::ATTRIBUTE_POSITION)
            .unwrap()
            .as_float3()
            .unwrap()
            .iter()
            .map(|&p| (scene_root_rotation() * Vec3::from_array(p)).y)
            .collect();
        let max = ys.iter().copied().fold(f32::MIN, f32::max);
        let min = ys.iter().copied().fold(f32::MAX, f32::min);
        assert!((max - WALL_HEIGHT).abs() < 1e-6 && min.abs() < 1e-6);
    }

    #[test]
    fn mirror_heading_turns_about_ros_z() {
        let t = robot_transform(Pose2D {
            x: 1.0,
            y: 2.0,
            theta: FRAC_PI_2 as f64,
        });
        // Body +X (forward) points along ROS +Y; in Bevy that is −Z.
        let fwd = scene_root_rotation() * (t.rotation * Vec3::X);
        assert!((fwd - Vec3::NEG_Z).length() < 1e-6);
        assert_eq!(t.translation, Vec3::new(1.0, 2.0, 0.0));
    }

    #[test]
    fn orbit_camera_looks_at_focus_from_above() {
        let o = OrbitCamera {
            focus: Vec3::new(1.0, 0.0, -2.0),
            yaw: 0.3,
            pitch: 1.0,
            distance: 10.0,
        };
        let t = o.transform();
        assert!((t.translation - o.focus).length() - 10.0 < 1e-4);
        assert!(t.translation.y > 0.0);
        let to_focus = (o.focus - t.translation).normalize();
        assert!((t.forward().as_vec3() - to_focus).length() < 1e-5);
    }
}
