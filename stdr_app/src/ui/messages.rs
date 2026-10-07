use std::collections::VecDeque;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use crate::sim::SimEvent;

/// Kept messages; older ones are dropped.
const MAX_MESSAGES: usize = 200;

/// Human-readable sim history, newest last.
#[derive(Resource, Default)]
pub struct MessageLog(pub VecDeque<String>);

pub fn message_text(e: &SimEvent) -> String {
    match e {
        SimEvent::MapLoaded(path) => format!("Loaded map: {}", path.display()),
        SimEvent::RobotSpawned(id) => format!("Spawned robot: {id}"),
        SimEvent::RobotDeleted(id) => format!("Deleted robot: {id}"),
        SimEvent::Reset => "Simulation reset".into(),
        SimEvent::Paused => "Simulation paused".into(),
        SimEvent::Resumed => "Simulation started".into(),
        SimEvent::FellBehind(cap) => format!(
            "Simulation fell behind real-time; discarded backlog beyond {cap:.2} s to catch up"
        ),
        SimEvent::Log(s) => s.clone(),
    }
}

pub fn log_sim_events(mut events: MessageReader<SimEvent>, mut log: ResMut<MessageLog>) {
    for e in events.read() {
        let text = message_text(e);
        info!("{text}");
        log.0.push_back(text);
        if log.0.len() > MAX_MESSAGES {
            log.0.pop_front();
        }
    }
}

pub fn messages_window(mut ctx: EguiContexts, log: Res<MessageLog>) -> Result {
    egui::Window::new("Messages")
        .default_open(false)
        .anchor(egui::Align2::RIGHT_BOTTOM, [-10.0, -40.0])
        .show(ctx.ctx_mut()?, |ui| {
            egui::ScrollArea::vertical()
                .stick_to_bottom(true)
                .max_height(200.0)
                .show(ui, |ui| {
                    for m in &log.0 {
                        ui.label(m);
                    }
                });
        });
    Ok(())
}
