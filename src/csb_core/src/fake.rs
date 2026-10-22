//! An in-memory, deterministic [`SimBackend`] for unit tests of what does not need a
//! simulator: the coordinator's decoding and replies.
//!
//! It records every call (shared between clones, so a test keeps one to inspect while the
//! coordinator owns another), keeps the spawned names, advances its clock by the step on
//! each frame, echoes every known entity's status and fails a batch on an unknown one, as a
//! real backend does. [`FakeBackend::failing`] makes every request fail with one error.

use std::cell::RefCell;
use std::collections::BTreeSet;
use std::rc::Rc;
use std::time::Duration;

use crate::proto::simulation_api_schema::TrafficSignal;
use crate::sim::{
    BackendError, EntityStatus, Episode, Goal, Sensor, SimBackend, SpawnRequest, StatusBatch,
    StatusOutcome,
};
use crate::weather::WeatherReply;
use crate::weather::WeatherRequest;

/// The fake's refusal of every sensor.
pub const NO_SENSORS: &str = "the fake backend has no sensors";

/// One call the fake received.
#[derive(Debug, Clone, PartialEq)]
pub enum Call {
    Initialize(Episode),
    UpdateFrame(f64),
    UpdateStepTime(f64),
    Spawn(SpawnRequest),
    Despawn(String),
    UpdateEntityStatus(StatusBatch),
    UpdateEntityGoal(String, Goal),
    UpdateTrafficLights(usize),
    AttachSensor(Sensor),
}

#[derive(Debug, Default)]
struct State {
    calls: Vec<Call>,
    entities: BTreeSet<String>,
    step_time: f64,
    time_ns: i64,
    fail_with: Option<BackendError>,
}

#[derive(Debug, Clone, Default)]
pub struct FakeBackend {
    state: Rc<RefCell<State>>,
}

impl FakeBackend {
    /// A fake whose every request fails with `error`.
    pub fn failing(error: BackendError) -> Self {
        let fake = Self::default();
        fake.state.borrow_mut().fail_with = Some(error);
        fake
    }

    pub fn calls(&self) -> Vec<Call> {
        self.state.borrow().calls.clone()
    }

    pub fn last_call(&self) -> Call {
        self.calls().last().cloned().expect("no call")
    }

    /// Record `call`, and the error every request fails with, if any.
    fn record(&self, call: Call) -> Result<(), BackendError> {
        let mut state = self.state.borrow_mut();
        state.calls.push(call);
        match &state.fail_with {
            Some(e) => Err(e.clone()),
            None => Ok(()),
        }
    }
}

impl SimBackend for FakeBackend {
    fn before_serving(&mut self) {}

    fn initialize(&mut self, episode: &Episode) -> Result<i64, BackendError> {
        self.record(Call::Initialize(episode.clone()))?;
        let mut state = self.state.borrow_mut();
        state.entities.clear();
        state.step_time = episode.step_time;
        Ok(state.time_ns)
    }

    fn update_frame(&mut self, current_simulation_time: f64) -> Result<i64, BackendError> {
        self.record(Call::UpdateFrame(current_simulation_time))?;
        let mut state = self.state.borrow_mut();
        state.time_ns += (state.step_time * 1e9).round() as i64;
        Ok(state.time_ns)
    }

    fn update_step_time(&mut self, step_time: f64) -> Result<(), BackendError> {
        self.record(Call::UpdateStepTime(step_time))?;
        self.state.borrow_mut().step_time = step_time;
        Ok(())
    }

    fn spawn(&mut self, request: &SpawnRequest) -> Result<(), BackendError> {
        self.record(Call::Spawn(request.clone()))?;
        self.state
            .borrow_mut()
            .entities
            .insert(request.name.clone());
        Ok(())
    }

    fn despawn(&mut self, name: &str) -> Result<(), BackendError> {
        self.record(Call::Despawn(name.to_string()))?;
        match self.state.borrow_mut().entities.remove(name) {
            true => Ok(()),
            false => Err(BackendError::Failed(format!("Entity '{name}' not found"))),
        }
    }

    fn update_entity_status(&mut self, batch: StatusBatch) -> StatusOutcome {
        if let Err(e) = self.record(Call::UpdateEntityStatus(batch.clone())) {
            return StatusOutcome {
                entities: Vec::new(),
                failure: Some(e.to_string()),
            };
        }
        let state = self.state.borrow();
        let (known, unknown): (Vec<EntityStatus>, Vec<EntityStatus>) = batch
            .entities
            .into_iter()
            .partition(|e| state.entities.contains(&e.name));
        StatusOutcome {
            entities: known,
            failure: (!unknown.is_empty()).then(|| {
                let names: Vec<String> = unknown.iter().map(|e| format!("'{}'", e.name)).collect();
                format!("UpdateEntityStatus: unknown {}", names.join(", "))
            }),
        }
    }

    fn update_entity_goal(&mut self, name: &str, goal: &Goal) -> Result<(), BackendError> {
        self.record(Call::UpdateEntityGoal(name.to_string(), goal.clone()))
    }

    fn update_traffic_lights(&mut self, signals: &[TrafficSignal]) -> Result<(), BackendError> {
        self.record(Call::UpdateTrafficLights(signals.len()))
    }

    fn attach_sensor(&mut self, sensor: Sensor) -> Result<(), BackendError> {
        self.record(Call::AttachSensor(sensor))?;
        Err(BackendError::Unsupported(NO_SENSORS.into()))
    }

    fn get_weather(&mut self) -> WeatherReply {
        WeatherReply::failed("the fake backend has no weather", None)
    }

    fn set_weather(&mut self, _request: WeatherRequest) -> WeatherReply {
        WeatherReply::failed("the fake backend has no weather", None)
    }

    fn entity_count(&self) -> usize {
        self.state.borrow().entities.len()
    }

    fn take_tick_time(&mut self) -> Duration {
        Duration::ZERO
    }

    fn probe_connection_idle(&mut self) {}

    fn wedged(&self) -> Option<&str> {
        None
    }

    fn server_lost(&self) -> Option<&str> {
        None
    }

    fn abandon_after_hang(&mut self) {}

    fn shutdown(&mut self) {}
}
