//! Keyboard teleop of the selected robot: W/Up S/Down drive, A/Left D/Right strafe (omni only),
//! Q/E turn.

use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use stdr_core::{KinematicKind, RobotId, Twist2D};

use crate::plot::PlotterSample;
use crate::sim::{Selection, SimCommand, SimWorld};

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct TeleopKeys {
    pub forward: bool,
    pub backward: bool,
    pub left: bool,
    pub right: bool,
    pub turn_left: bool,
    pub turn_right: bool,
}

impl TeleopKeys {
    pub fn from_input(k: &ButtonInput<KeyCode>) -> Self {
        Self {
            forward: k.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]),
            backward: k.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]),
            left: k.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]),
            right: k.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]),
            turn_left: k.pressed(KeyCode::KeyQ),
            turn_right: k.pressed(KeyCode::KeyE),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct TeleopSpeeds {
    /// m/s
    pub linear: f64,
    /// rad/s
    pub angular: f64,
}

impl Default for TeleopSpeeds {
    fn default() -> Self {
        Self {
            linear: 0.5,
            angular: 1.0,
        }
    }
}

/// Opposing keys cancel; strafe only for omni robots (REP-103: +y and +angular are left).
pub fn teleop_twist(keys: TeleopKeys, kind: KinematicKind, speeds: TeleopSpeeds) -> Twist2D {
    let axis = |pos: bool, neg: bool, speed: f64| match (pos, neg) {
        (true, false) => speed,
        (false, true) => -speed,
        _ => 0.0,
    };
    Twist2D {
        linear_x: axis(keys.forward, keys.backward, speeds.linear),
        linear_y: if kind == KinematicKind::Omni {
            axis(keys.left, keys.right, speeds.linear)
        } else {
            0.0
        },
        angular_z: axis(keys.turn_left, keys.turn_right, speeds.angular),
    }
}

#[derive(Resource, Default)]
pub struct TeleopState {
    /// The robot teleop last sent a non-zero command to; it gets the stop.
    pub driving: Option<RobotId>,
    pub speeds: TeleopSpeeds,
}

pub struct TeleopPlugin;

impl Plugin for TeleopPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TeleopState>()
            .init_resource::<EguiWantsInput>()
            .add_systems(Update, teleop_input.after(PlotterSample))
            .add_systems(EguiPrimaryContextPass, teleop_window);
    }
}

/// Drives the selected robot while keys are held. The stop (exactly one zero command) goes to
/// the robot that was driving, whether keys were released, the selection moved, the robot was
/// deleted, or egui took the keyboard.
pub fn teleop_input(
    keys: Res<ButtonInput<KeyCode>>,
    egui_input: Res<EguiWantsInput>,
    sel: Res<Selection>,
    sim: Res<SimWorld>,
    mut st: ResMut<TeleopState>,
    mut cmd: MessageWriter<SimCommand>,
) {
    let drive = sel
        .robot
        .filter(|_| !egui_input.wants_keyboard_input())
        .and_then(|id| Some((id, sim.robot(id)?.config.kinematic.kind)))
        .map(|(id, kind)| {
            (
                id,
                teleop_twist(TeleopKeys::from_input(&keys), kind, st.speeds),
            )
        })
        .filter(|(_, twist)| *twist != Twist2D::default());
    if let Some(prev) = st.driving
        && drive.is_none_or(|(id, _)| id != prev)
    {
        cmd.write(SimCommand::CmdVel {
            id: prev,
            twist: Twist2D::default(),
        });
        st.driving = None;
    }
    if let Some((id, twist)) = drive {
        cmd.write(SimCommand::CmdVel { id, twist });
        st.driving = Some(id);
    }
}

fn teleop_window(
    mut ctx: EguiContexts,
    sel: Res<Selection>,
    mut st: ResMut<TeleopState>,
) -> Result {
    let Some(id) = sel.robot else { return Ok(()) };
    egui::Window::new("Teleop")
        .default_pos([10.0, 500.0])
        .show(ctx.ctx_mut()?, |ui| {
            ui.label(format!("Driving: {id}"));
            ui.separator();
            ui.add(egui::Slider::new(&mut st.speeds.linear, 0.05..=3.0).text("Linear (m/s)"));
            ui.add(egui::Slider::new(&mut st.speeds.angular, 0.1..=4.0).text("Angular (rad/s)"));
            ui.separator();
            ui.weak("W/S forward/back  A/D strafe (omni)  Q/E turn; arrows also work");
        });
    Ok(())
}

// C++ `TeleopControllerTest.*` (minus `KinematicTypeFromString*`: `KinematicKind` is parsed at load).
#[cfg(test)]
#[allow(non_snake_case)]
mod TeleopController {
    use super::*;

    const IDEAL: KinematicKind = KinematicKind::Ideal;
    const OMNI: KinematicKind = KinematicKind::Omni;

    fn twist(keys: TeleopKeys, kind: KinematicKind) -> Twist2D {
        teleop_twist(keys, kind, TeleopSpeeds::default())
    }

    #[test]
    fn NoKeysHeldReturnsZero() {
        assert_eq!(twist(TeleopKeys::default(), IDEAL), Twist2D::default());
    }

    #[test]
    fn ForwardProducesPositiveLinearX() {
        let keys = TeleopKeys {
            forward: true,
            ..Default::default()
        };
        assert_eq!(
            twist(keys, IDEAL),
            Twist2D {
                linear_x: 0.5,
                ..Default::default()
            }
        );
    }

    #[test]
    fn BackwardProducesNegativeLinearX() {
        let keys = TeleopKeys {
            backward: true,
            ..Default::default()
        };
        assert_eq!(
            twist(keys, IDEAL),
            Twist2D {
                linear_x: -0.5,
                ..Default::default()
            }
        );
    }

    #[test]
    fn OpposingForwardBackwardCancel() {
        let keys = TeleopKeys {
            forward: true,
            backward: true,
            ..Default::default()
        };
        assert_eq!(twist(keys, IDEAL).linear_x, 0.0);
    }

    #[test]
    fn TurnLeftRightMapToAngularZ() {
        let left = TeleopKeys {
            turn_left: true,
            ..Default::default()
        };
        let right = TeleopKeys {
            turn_right: true,
            ..Default::default()
        };
        assert_eq!(twist(left, IDEAL).angular_z, 1.0);
        assert_eq!(twist(right, IDEAL).angular_z, -1.0);
    }

    #[test]
    fn OpposingTurnsCancel() {
        let keys = TeleopKeys {
            turn_left: true,
            turn_right: true,
            ..Default::default()
        };
        assert_eq!(twist(keys, IDEAL).angular_z, 0.0);
    }

    #[test]
    fn DifferentialIgnoresStrafe() {
        for keys in [
            TeleopKeys {
                left: true,
                ..Default::default()
            },
            TeleopKeys {
                right: true,
                ..Default::default()
            },
        ] {
            assert_eq!(twist(keys, IDEAL).linear_y, 0.0);
        }
    }

    #[test]
    fn OmniStrafeLeftProducesPositiveLinearY() {
        let left = TeleopKeys {
            left: true,
            ..Default::default()
        };
        let right = TeleopKeys {
            right: true,
            ..Default::default()
        };
        assert_eq!(twist(left, OMNI).linear_y, 0.5);
        assert_eq!(twist(right, OMNI).linear_y, -0.5);
    }

    #[test]
    fn OmniOpposingStrafesCancel() {
        let keys = TeleopKeys {
            left: true,
            right: true,
            ..Default::default()
        };
        assert_eq!(twist(keys, OMNI).linear_y, 0.0);
    }

    #[test]
    fn CombinedForwardAndTurnProducesBoth() {
        let keys = TeleopKeys {
            forward: true,
            turn_left: true,
            ..Default::default()
        };
        let t = twist(keys, IDEAL);
        assert_eq!((t.linear_x, t.angular_z), (0.5, 1.0));
    }

    #[test]
    fn SpeedsScaleOutput() {
        let keys = TeleopKeys {
            forward: true,
            turn_left: true,
            ..Default::default()
        };
        let speeds = TeleopSpeeds {
            linear: 2.5,
            angular: 3.0,
        };
        let t = teleop_twist(keys, IDEAL, speeds);
        assert_eq!((t.linear_x, t.angular_z), (2.5, 3.0));
    }
}
