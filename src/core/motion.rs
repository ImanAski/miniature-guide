//! Motion core — a 3-axis stage model that runs either against the real
//! ESP301 link or in simulation, plus the bookkeeping the UI needs.
//!
//! [`MotionHub`] is the single source of truth for "where are the motors
//! right now": widgets read it, drive it through commands, and never talk to
//! the serial layer themselves.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use crate::esp301::{ControllerStatus, LinkConfig, MotorState, SerialDriver};

/// Axis labels, index-aligned with [`Axes`].
pub const AXES: [&str; 3] = ["X", "Y", "Z"];

/// Position tolerance below which a move counts as finished (mm).
const POS_EPS: f64 = 1e-6;
/// Maximum simulation step, guards against huge frame gaps.
const MAX_DT: f64 = 0.1;
/// Minimum travel between recorded trail samples (mm).
const TRAIL_MIN_STEP: f64 = 0.02;
const VEL_EPS: f64 = 1e-9;
/// Maximum age of a trail sample (ms).
const TRAIL_MAX_AGE_MS: u128 = 120;

// ─── Axes ──────────────────────────────────────────────────────────────

/// A position or velocity in machine millimetres.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Axes {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Axes {
    pub const ZERO: Axes = Axes {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Axes { x, y, z }
    }

    pub fn from_array(a: [f64; 3]) -> Self {
        Axes::new(a[0], a[1], a[2])
    }

    pub fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    pub fn get(self, i: usize) -> f64 {
        match i {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }

    pub fn length(self) -> f64 {
        self.x.hypot(self.y).hypot(self.z)
    }

    pub fn distance_to(self, o: Axes) -> f64 {
        (self - o).length()
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    /// Replace NaN/inf components with zero.
    pub fn sanitized(self) -> Axes {
        Axes::new(
            if self.x.is_finite() { self.x } else { 0.0 },
            if self.y.is_finite() { self.y } else { 0.0 },
            if self.z.is_finite() { self.z } else { 0.0 },
        )
    }

    /// Clamp each axis into the travel limits.
    pub fn clamp(self, limits: &TravelLimits) -> Axes {
        Axes::new(
            self.x.clamp(limits.min.x, limits.max.x),
            self.y.clamp(limits.min.y, limits.max.y),
            self.z.clamp(limits.min.z, limits.max.z),
        )
    }
}

impl std::ops::Add for Axes {
    type Output = Axes;
    fn add(self, o: Axes) -> Axes {
        Axes::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl std::ops::Sub for Axes {
    type Output = Axes;
    fn sub(self, o: Axes) -> Axes {
        Axes::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl std::ops::Mul<f64> for Axes {
    type Output = Axes;
    fn mul(self, k: f64) -> Axes {
        Axes::new(self.x * k, self.y * k, self.z * k)
    }
}

impl std::fmt::Display for Axes {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "({}, {}, {})",
            crate::core::geo::fmt_f(self.x),
            crate::core::geo::fmt_f(self.y),
            crate::core::geo::fmt_f(self.z)
        )
    }
}

// ─── Travel limits ─────────────────────────────────────────────────────

/// Soft travel limits enforced by the simulator (mm).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TravelLimits {
    pub min: Axes,
    pub max: Axes,
}

impl Default for TravelLimits {
    fn default() -> Self {
        TravelLimits {
            min: Axes::new(-250.0, -250.0, 0.0),
            max: Axes::new(250.0, 250.0, 120.0),
        }
    }
}

// ─── Simulator ─────────────────────────────────────────────────────────

/// A trapezoidal-velocity 3-axis stage model.
///
/// Velocity ramps toward `min(feed, sqrt(2·accel·remaining))`, so a long move
/// cruises at `feed` and a short move never overshoots.
#[derive(Debug, Clone)]
pub struct SimMachine {
    pub pos: Axes,
    pub vel: Axes,
    pub target: Axes,
    /// Maximum travel speed along the path (mm/s).
    pub feed: f64,
    /// Acceleration / deceleration (mm/s²).
    pub accel: f64,
    pub limits: TravelLimits,
    pub homed: bool,
    pub moving: bool,
    /// Set while decelerating to a ramped stop.
    pub(crate) ramp_stop: bool,
}

impl SimMachine {
    pub fn new(feed: f64, accel: f64) -> Self {
        SimMachine {
            pos: Axes::ZERO,
            vel: Axes::ZERO,
            target: Axes::ZERO,
            feed: feed.max(0.0),
            accel: accel.max(0.0),
            limits: TravelLimits::default(),
            homed: false,
            moving: false,
            ramp_stop: false,
        }
    }

    pub fn remaining(&self) -> f64 {
        self.pos.distance_to(self.target)
    }

    pub fn is_idle(&self) -> bool {
        !self.moving && self.vel.length() <= POS_EPS
    }

    /// Rough time-to-target in seconds; may be `INFINITY` at zero feedrate.
    pub fn eta(&self) -> f64 {
        let d = self.remaining();
        if d <= POS_EPS {
            0.0
        } else if self.feed <= 0.0 {
            f64::INFINITY
        } else {
            d / self.feed
        }
    }

    pub fn move_to(&mut self, a: Axes) {
        self.ramp_stop = false;
        self.target = a.sanitized().clamp(&self.limits);
        self.moving = self.pos.distance_to(self.target) > POS_EPS;
    }

    /// Relative move on one axis.
    pub fn jog(&mut self, axis: usize, direction: f64, step: f64) {
        let delta = Axes::ZERO;
        let mut d = delta;
        match axis {
            0 => d.x = direction * step,
            1 => d.y = direction * step,
            _ => d.z = direction * step,
        }
        self.move_to(self.target + d);
    }

    /// Ramp to a full stop where the machine stands.
    pub fn stop(&mut self) {
        self.ramp_stop = false;
        self.target = self.pos;
        self.moving = false;
    }

    /// Decelerate to a stop; the machine comes to rest wherever it lands.
    pub fn stop_ramped(&mut self) {
        self.ramp_stop = true;
        self.moving = self.vel.length() > VEL_EPS;
    }

    pub fn is_ramp_stopping(&self) -> bool {
        self.ramp_stop
    }

    pub fn home(&mut self) {
        self.move_to(Axes::ZERO);
        self.homed = true;
    }

    pub fn reset(&mut self) {
        self.pos = Axes::ZERO;
        self.vel = Axes::ZERO;
        self.target = Axes::ZERO;
        self.moving = false;
        self.ramp_stop = false;
        self.homed = false;
    }

    pub fn step(&mut self, dt: f64) {
        let dt = dt.clamp(0.0, MAX_DT);
        if self.ramp_stop {
            self.step_ramp_stop(dt);
            return;
        }
        let delta = self.target - self.pos;
        let dist = delta.length();
        if dist <= POS_EPS || dt <= 0.0 {
            if dist <= POS_EPS {
                self.pos = self.target;
                self.vel = Axes::ZERO;
                self.moving = false;
            }
            return;
        }

        let accel = self.accel.max(1e-3);
        let speed = self.vel.length();
        let v_des = self.feed.max(0.0).min((2.0 * accel * dist).sqrt());
        let next_speed = if speed < v_des {
            (speed + accel * dt).min(v_des)
        } else {
            (speed - accel * dt).max(v_des)
        };

        let dir = delta * (1.0 / dist);
        self.vel = dir * next_speed;

        let mut next = self.pos + self.vel * dt;
        if next.distance_to(self.target) > dist {
            next = self.target;
        }
        let clamped = next.clamp(&self.limits);
        if clamped != next {
            self.vel = Axes::ZERO;
        }
        self.pos = clamped;

        if self.pos.distance_to(self.target) <= POS_EPS {
            self.pos = self.target;
            self.vel = Axes::ZERO;
            self.moving = false;
            if self.pos == Axes::ZERO {
                self.homed = true;
            }
        } else {
            self.moving = true;
        }
    }

    fn step_ramp_stop(&mut self, dt: f64) {
        let accel = self.accel.max(1e-3);
        let speed = self.vel.length();
        let next_speed = (speed - accel * dt).max(0.0);
        let dir = if speed > VEL_EPS {
            self.vel * (1.0 / speed)
        } else {
            Axes::ZERO
        };

        let mut decel_speed = next_speed;

        let moved = (dir * decel_speed) * dt;
        let next = (self.pos + moved).clamp(&self.limits);
        if next != self.pos + moved {
            decel_speed = 0.0;
        }
        self.pos = next;

        if decel_speed <= VEL_EPS {
            self.vel = Axes::ZERO;
            self.target = self.pos;
            self.moving = false;
            self.ramp_stop = false;
        } else {
            self.vel = dir * decel_speed;
            self.moving = true;
        }
    }
}

// ─── Hub ───────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveMode {
    Simulated,
    Hardware,
}

impl DriveMode {
    pub fn label(self) -> &'static str {
        match self {
            DriveMode::Simulated => "Simulated",
            DriveMode::Hardware => "Hardware",
        }
    }

    pub const ALL: [DriveMode; 2] = [DriveMode::Simulated, DriveMode::Hardware];
}

/// Owns the drive: either the simulator or the serial link, plus cached status
/// and a position trail for plotting.
pub struct MotionHub {
    pub mode: DriveMode,
    pub sim: SimMachine,
    pub link: Option<SerialDriver>,
    pub status: ControllerStatus,
    pub trail: VecDeque<Axes>,
    pub trail_max: usize,
    pub poll_interval: Duration,
    pub link_error: Option<String>,
    commanded: Axes,
    last_tick: Instant,
    last_poll: Instant,
    last_trail_at: Instant,
    last_trail_pos: Axes,
}

impl MotionHub {
    pub fn new(feed: f64, accel: f64) -> Self {
        let now = Instant::now();
        MotionHub {
            mode: DriveMode::Simulated,
            sim: SimMachine::new(feed, accel),
            link: None,
            status: status_of(Axes::ZERO, false, false),
            trail: VecDeque::new(),
            trail_max: 4000,
            poll_interval: Duration::from_millis(200),
            link_error: None,
            commanded: Axes::ZERO,
            last_tick: now,
            last_poll: now,
            last_trail_at: now,
            last_trail_pos: Axes::ZERO,
        }
    }

    pub fn is_linked(&self) -> bool {
        self.link.is_some()
    }

    /// Advance the drive. Call once per frame before drawing.
    pub fn tick(&mut self) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_tick).as_secs_f64();
        self.last_tick = now;

        match self.mode {
            DriveMode::Simulated => {
                self.sim.step(dt);
                self.status = status_of(self.sim.pos, self.sim.moving, self.sim.homed);
            }
            DriveMode::Hardware => {
                if self.link.is_some() && now.duration_since(self.last_poll) >= self.poll_interval {
                    self.last_poll = now;
                    self.poll();
                }
            }
        }

        self.record_trail(now);
    }

    fn poll(&mut self) {
        let Some(link) = self.link.as_mut() else {
            return;
        };
        match link.status() {
            Ok(s) => {
                self.status = s;
                self.link_error = None;
            }
            Err(e) => self.link_error = Some(e.to_string()),
        }
    }

    fn record_trail(&mut self, now: Instant) {
        let pos = self.pos();
        let moved = pos.distance_to(self.last_trail_pos);
        let aged = now.duration_since(self.last_trail_at).as_millis();
        if moved < TRAIL_MIN_STEP && aged < TRAIL_MAX_AGE_MS {
            return;
        }
        self.last_trail_pos = pos;
        self.last_trail_at = now;
        if self.trail.len() >= self.trail_max {
            self.trail.pop_front();
        }
        self.trail.push_back(pos);
    }

    pub fn pos(&self) -> Axes {
        match self.mode {
            DriveMode::Simulated => self.sim.pos,
            DriveMode::Hardware => {
                Axes::new(self.status.pos_x, self.status.pos_y, self.status.pos_z)
            }
        }
    }

    pub fn vel(&self) -> Axes {
        match self.mode {
            DriveMode::Simulated => self.sim.vel,
            DriveMode::Hardware => Axes::ZERO,
        }
    }

    pub fn target(&self) -> Axes {
        match self.mode {
            DriveMode::Simulated => self.sim.target,
            DriveMode::Hardware => self.commanded,
        }
    }

    pub fn remaining(&self) -> f64 {
        match self.mode {
            DriveMode::Simulated => self.sim.remaining(),
            DriveMode::Hardware => 0.0,
        }
    }

    pub fn eta(&self) -> f64 {
        match self.mode {
            DriveMode::Simulated => self.sim.eta(),
            DriveMode::Hardware => 0.0,
        }
    }

    pub fn is_moving(&self) -> bool {
        match self.mode {
            DriveMode::Simulated => self.sim.moving,
            DriveMode::Hardware => self.status.motor_state == MotorState::Moving,
        }
    }

    pub fn feed(&self) -> f64 {
        self.sim.feed
    }

    pub fn set_feed(&mut self, feed: f64) -> Result<(), String> {
        self.sim.feed = feed.max(0.0);
        match self.link.as_mut() {
            Some(link) => link.set_velocity(self.sim.feed).map_err(|e| e.to_string()),
            None => Ok(()),
        }
    }

    pub fn connect(&mut self, cfg: &LinkConfig) -> Result<(), String> {
        let driver = SerialDriver::open(cfg).map_err(|e| e.to_string())?;
        let name = cfg.port.clone();
        self.link = Some(driver);
        self.link_error = None;
        self.commanded = Axes::new(self.status.pos_x, self.status.pos_y, self.status.pos_z);
        self.last_poll = Instant::now() - self.poll_interval;
        self.mode = DriveMode::Hardware;
        log::info!("motion: connected to {name}");
        Ok(())
    }

    pub fn disconnect(&mut self) {
        if let Some(mut link) = self.link.take() {
            let _ = link.stop();
        }
        self.link_error = None;
        self.mode = DriveMode::Simulated;
        log::info!("motion: disconnected");
    }

    pub fn move_to(&mut self, a: Axes) -> Result<(), String> {
        let a = a.sanitized();
        match self.mode {
            DriveMode::Simulated => {
                self.sim.move_to(a);
                Ok(())
            }
            DriveMode::Hardware => {
                let link = self.link.as_mut().ok_or("controller not connected")?;
                link.move_absolute_xyz(a.x, a.y, a.z)
                    .map_err(|e| e.to_string())?;
                self.commanded = a;
                Ok(())
            }
        }
    }

    /// Relative move on one axis, applying `feed` first.
    pub fn jog(&mut self, axis: usize, direction: f64, step: f64, feed: f64) -> Result<(), String> {
        self.set_feed(feed)?;
        match self.mode {
            DriveMode::Simulated => {
                self.sim.jog(axis, direction, step);
                Ok(())
            }
            DriveMode::Hardware => {
                let link = self.link.as_mut().ok_or("controller not connected")?;
                let (dx, dy, dz) = match axis {
                    0 => (direction * step, 0.0, 0.0),
                    1 => (0.0, direction * step, 0.0),
                    _ => (0.0, 0.0, direction * step),
                };
                link.jog_relative(dx, dy, dz).map_err(|e| e.to_string())?;
                self.commanded = self.commanded + Axes::new(dx, dy, dz).clamp(&self.sim.limits);
                Ok(())
            }
        }
    }

    pub fn stop(&mut self, ramped: bool) -> Result<(), String> {
        match self.mode {
            DriveMode::Simulated => {
                if ramped {
                    self.sim.stop_ramped();
                } else {
                    self.sim.stop();
                }
                Ok(())
            }
            DriveMode::Hardware => {
                let link = self.link.as_mut().ok_or("controller not connected")?;
                if ramped {
                    link.stop_smooth()
                } else {
                    link.stop()
                }
                .map_err(|e| e.to_string())
            }
        }
    }

    pub fn home(&mut self, axes: &[usize]) -> Result<(), String> {
        match self.mode {
            DriveMode::Simulated => {
                self.sim.home();
                Ok(())
            }
            DriveMode::Hardware => {
                let link = self.link.as_mut().ok_or("controller not connected")?;
                let codes: Vec<u8> = axes.iter().map(|&i| b'X' + i as u8).collect();
                link.home(&codes).map_err(|e| e.to_string())
            }
        }
    }

    pub fn clear_trail(&mut self) {
        self.trail.clear();
        self.last_trail_pos = self.pos();
        self.last_trail_at = Instant::now();
    }
}

fn status_of(pos: Axes, moving: bool, homed: bool) -> ControllerStatus {
    ControllerStatus {
        pos_x: pos.x,
        pos_y: pos.y,
        pos_z: pos.z,
        motor_state: if moving {
            MotorState::Moving
        } else {
            MotorState::Stopped
        },
        err_code: None,
        buffer_count: 0,
        is_home: homed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_for(m: &mut SimMachine, seconds: f64, dt: f64) {
        let n = (seconds / dt).round() as usize;
        for _ in 0..n {
            m.step(dt);
        }
    }

    #[test]
    fn converges_to_target() {
        let mut m = SimMachine::new(50.0, 500.0);
        m.move_to(Axes::new(10.0, -5.0, 2.0));
        run_for(&mut m, 5.0, 1.0 / 120.0);
        assert!(m.pos.distance_to(m.target) < 1e-6, "pos={:?}", m.pos);
        assert!(!m.moving);
        assert_eq!(m.vel, Axes::ZERO);
    }

    #[test]
    fn never_exceeds_feedrate() {
        let mut m = SimMachine::new(20.0, 1000.0);
        m.move_to(Axes::new(200.0, 0.0, 0.0));
        let mut peak: f64 = 0.0;
        for _ in 0..2000 {
            m.step(1.0 / 240.0);
            peak = peak.max(m.vel.length());
        }
        assert!(peak <= 20.0 + 1e-6, "peak {peak}");
        assert!(peak > 19.0, "should reach cruise speed, got {peak}");
    }

    #[test]
    fn clamps_to_travel_limits() {
        let mut m = SimMachine::new(50.0, 500.0);
        m.move_to(Axes::new(9999.0, -9999.0, -50.0));
        assert_eq!(m.target, Axes::new(250.0, -250.0, 0.0));
        run_for(&mut m, 60.0, 1.0 / 120.0);
        assert_eq!(m.pos, Axes::new(250.0, -250.0, 0.0));

        m.move_to(Axes::new(9999.0, 9999.0, 9999.0));
        assert_eq!(m.target, m.limits.max);
        run_for(&mut m, 60.0, 1.0 / 120.0);
        assert_eq!(m.pos, m.limits.max);
    }

    #[test]
    fn short_move_does_not_overshoot() {
        let mut m = SimMachine::new(100.0, 5000.0);
        m.move_to(Axes::new(0.05, 0.0, 0.0));
        run_for(&mut m, 1.0, 1.0 / 240.0);
        assert_eq!(m.pos.x, 0.05);
    }

    #[test]
    fn jog_is_relative_to_target() {
        let mut m = SimMachine::new(50.0, 500.0);
        m.jog(1, 1.0, 2.5);
        assert_eq!(m.target.y, 2.5);
        m.jog(1, -1.0, 1.0);
        assert_eq!(m.target.y, 1.5);
    }

    #[test]
    fn stop_freezes_position() {
        let mut m = SimMachine::new(50.0, 500.0);
        m.move_to(Axes::new(100.0, 0.0, 0.0));
        run_for(&mut m, 0.2, 1.0 / 120.0);
        m.stop();
        let frozen = m.pos;
        run_for(&mut m, 2.0, 1.0 / 120.0);
        assert_eq!(m.pos, frozen);
    }

    #[test]
    fn ramp_stop_decelerates_and_stays_put() {
        let mut m = SimMachine::new(50.0, 500.0);
        m.move_to(Axes::new(100.0, 0.0, 0.0));
        run_for(&mut m, 0.2, 1.0 / 120.0);
        assert!(m.moving);

        m.stop_ramped();
        assert!(m.is_ramp_stopping());

        let mut peak: f64 = 0.0;
        for _ in 0..240 {
            m.step(1.0 / 120.0);
            peak = peak.max(m.vel.length());
        }
        assert!(!m.is_ramp_stopping(), "ramp stop must finish");
        assert!(!m.moving);
        assert_eq!(m.vel.length(), 0.0, "velocity must reach zero");

        let parked = m.pos;
        assert!(parked.x > 0.0 && parked.x < 100.0, "stops short of target");
        run_for(&mut m, 2.0, 1.0 / 120.0);
        assert_eq!(m.pos, parked, "must not creep after stopping");
    }

    #[test]
    fn new_command_cancels_ramp_stop() {
        let mut m = SimMachine::new(50.0, 500.0);
        m.move_to(Axes::new(100.0, 0.0, 0.0));
        run_for(&mut m, 0.2, 1.0 / 120.0);
        m.stop_ramped();
        m.step(1.0 / 120.0);
        m.move_to(Axes::new(80.0, 0.0, 0.0));
        assert!(!m.is_ramp_stopping());
        run_for(&mut m, 3.0, 1.0 / 120.0);
        assert!((m.pos.x - 80.0).abs() < 1e-3, "resumed to new target");
    }

    #[test]
    fn eta_is_zero_at_target() {
        let m = SimMachine::new(50.0, 500.0);
        assert_eq!(m.eta(), 0.0);
    }

    #[test]
    fn sanitized_replaces_nan() {
        let a = Axes::new(f64::NAN, 1.0, f64::INFINITY).sanitized();
        assert_eq!(a, Axes::new(0.0, 1.0, 0.0));
    }

    #[test]
    fn hub_defaults_to_simulation() {
        let mut hub = MotionHub::new(30.0, 400.0);
        assert_eq!(hub.mode, DriveMode::Simulated);
        assert!(!hub.is_linked());

        hub.move_to(Axes::new(5.0, 0.0, 0.0)).unwrap();
        assert!(hub.is_moving());

        run_for(&mut hub.sim, 1.0, 1.0 / 120.0);
        hub.record_trail(Instant::now());

        assert_eq!(hub.pos().x, 5.0);
        assert!(!hub.is_moving());
        assert!(!hub.trail.is_empty(), "trail should record motion");

        hub.clear_trail();
        assert!(hub.trail.is_empty());
    }

    #[test]
    fn hardware_commands_fail_without_link() {
        let mut hub = MotionHub::new(30.0, 400.0);
        hub.mode = DriveMode::Hardware;
        assert!(hub.move_to(Axes::new(1.0, 2.0, 3.0)).is_err());
        assert!(hub.jog(0, 1.0, 1.0, 10.0).is_err());
        assert!(hub.stop(false).is_err());
    }
}
