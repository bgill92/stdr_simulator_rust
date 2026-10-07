use std::collections::VecDeque;

use bevy::prelude::*;

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
