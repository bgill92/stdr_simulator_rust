//! Camera sensors: core schedules them (`RobotRuntime::fired`), the app renders each one into an
//! image from a `Camera3d` child of its robot mirror. Images are only shown in egui; nothing reads
//! pixels back from the GPU.

use std::collections::{HashMap, HashSet};

use bevy::camera::RenderTarget;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use bevy_egui::{EguiContexts, EguiTextureHandle, egui};
use stdr_core::{CameraSpec, Pose2D, RobotId, RobotRuntime, SensorConfig};

use super::ROBOT_HEIGHT;
use crate::sim::SimWorld;

/// Lens height above the floor: just over the robot body so its own top is out of view.
pub const CAMERA_Z: f32 = ROBOT_HEIGHT + 0.05;
/// Widest a camera image is drawn in the Cameras window, in logical pixels.
const MAX_DISPLAY_WIDTH: f32 = 320.0;

/// Each camera sensor's render target, by `(robot, sensor index)`. Entries live as long as the robot.
#[derive(Resource, Default)]
pub struct CameraFrames(pub HashMap<(RobotId, usize), Handle<Image>>);

#[derive(Component)]
pub struct CameraSensor {
    pub robot: RobotId,
    pub index: usize,
}

/// Cameras that fired in a tick since the last frame: each renders once this frame.
#[derive(Resource, Default)]
pub struct PendingCaptures(HashSet<(RobotId, usize)>);

/// Bevy's fov is vertical; the yaml's is horizontal.
pub fn vertical_fov(spec: &CameraSpec) -> f32 {
    (2.0 * ((spec.fov / 2.0).tan() * f64::from(spec.height) / f64::from(spec.width)).atan()) as f32
}

/// Mount pose in the robot frame (ROS, Z up): Bevy cameras look down local −Z with +Y up, so
/// aim along the sensor heading with ROS +Z as up.
pub fn camera_transform(mount: Pose2D) -> Transform {
    let heading = Vec3::new(mount.theta.cos() as f32, mount.theta.sin() as f32, 0.0);
    Transform::from_xyz(mount.x as f32, mount.y as f32, CAMERA_Z).looking_to(heading, Vec3::Z)
}

/// One inactive `Camera3d` child per camera sensor of a freshly spawned mirror.
pub fn spawn_camera_sensors(
    mirror: &mut ChildSpawnerCommands,
    id: RobotId,
    r: &RobotRuntime,
    images: &mut Assets<Image>,
    frames: &mut CameraFrames,
) {
    for (index, sensor) in r.config.sensors.iter().enumerate() {
        let SensorConfig::Camera(spec) = sensor.kind else {
            continue;
        };
        let image = images.add(Image::new_target_texture(
            spec.width,
            spec.height,
            TextureFormat::Rgba8UnormSrgb,
            None,
        ));
        frames.0.insert((id, index), image.clone());
        mirror.spawn((
            Camera3d::default(),
            Camera {
                is_active: false,
                ..default()
            },
            RenderTarget::Image(image.into()),
            Projection::Perspective(PerspectiveProjection {
                fov: vertical_fov(&spec),
                aspect_ratio: spec.width as f32 / spec.height as f32,
                near: spec.near as f32,
                far: spec.far as f32,
                ..default()
            }),
            Tonemapping::None,
            camera_transform(sensor.common.pose),
            CameraSensor { robot: id, index },
        ));
    }
}

/// FixedUpdate, after `sim_step`: remember which cameras this tick fired.
pub fn collect_captures(sim: Res<SimWorld>, mut pending: ResMut<PendingCaptures>) {
    for (id, r) in sim.robots() {
        for &i in &r.fired {
            if matches!(r.config.sensors[i].kind, SensorConfig::Camera(_)) {
                pending.0.insert((id, i));
            }
        }
    }
}

/// A camera renders on frames where at least one of its ticks fired; otherwise its image keeps
/// the last capture (e.g. while paused).
pub fn apply_captures(
    mut pending: ResMut<PendingCaptures>,
    mut cams: Query<(&mut Camera, &CameraSensor)>,
) {
    for (mut cam, s) in &mut cams {
        let fire = pending.0.contains(&(s.robot, s.index));
        if cam.is_active != fire {
            cam.is_active = fire;
        }
    }
    pending.0.clear();
}

pub fn cameras_window(
    mut ctx: EguiContexts,
    frames: Res<CameraFrames>,
    sim: Res<SimWorld>,
) -> Result {
    if frames.0.is_empty() {
        return Ok(());
    }
    let mut cams: Vec<_> = frames
        .0
        .iter()
        // Weak: `CameraFrames` holds the strong handle; bevy_egui frees the id with the image.
        .map(|(&key, image)| (key, ctx.add_image(EguiTextureHandle::Weak(image.id()))))
        .collect();
    cams.sort_by_key(|&(key, _)| key);
    egui::Window::new("Cameras")
        .default_pos([10.0, 400.0])
        .show(ctx.ctx_mut()?, |ui| {
            for ((id, i), tex) in cams {
                let Some(sensor) = sim.robot(id).map(|r| &r.config.sensors[i]) else {
                    continue;
                };
                let SensorConfig::Camera(spec) = sensor.kind else {
                    continue;
                };
                ui.label(format!("{id} {}", sensor.common.frame_id));
                let size = egui::vec2(spec.width as f32, spec.height as f32);
                let size = size * (MAX_DISPLAY_WIDTH / size.x).min(1.0);
                ui.image(egui::load::SizedTexture::new(tex, size));
            }
        });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn square_image_keeps_fov() {
        let spec = CameraSpec {
            width: 100,
            height: 100,
            fov: 1.2,
            ..default()
        };
        assert!((vertical_fov(&spec) - 1.2).abs() < 1e-6);
    }

    #[test]
    fn wide_image_narrows_vertical_fov() {
        let spec = CameraSpec {
            width: 200,
            height: 100,
            fov: FRAC_PI_2,
            ..default()
        };
        // tan(v/2) = tan(45°) · 1/2.
        assert!((vertical_fov(&spec) - 2.0 * 0.5f32.atan()).abs() < 1e-6);
    }

    #[test]
    fn camera_looks_along_sensor_heading_with_z_up() {
        let t = camera_transform(Pose2D {
            x: 0.1,
            y: 0.2,
            theta: FRAC_PI_2,
        });
        assert!((t.forward().as_vec3() - Vec3::Y).length() < 1e-6);
        assert!((t.up().as_vec3() - Vec3::Z).length() < 1e-6);
        assert_eq!(t.translation, Vec3::new(0.1, 0.2, CAMERA_Z));
    }
}
