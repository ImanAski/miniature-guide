//! Path follower — walks the stage along loaded geometry, waypoint by waypoint.
//!
//! Geometry is flattened into a list of XY waypoints once, then [`PathRunner`]
//! commands them one at a time through [`MotionHub`]: command the next point,
//! wait until the stage is idle, skip the points it reached, repeat.
//!
//! Beam discipline: when started with `fire`, the laser is switched on with
//! the first commanded waypoint and every way out of a run (finish, failure,
//! [`PathRunner::stop`]) switches it back off.

use std::time::{Duration, Instant};

use crate::core::geo::{Shape, fmt_f};
use crate::core::motion::{Axes, MotionHub, TravelLimits};

/// Consecutive waypoints closer than this (mm) are collapsed into one.
const MIN_WP_GAP: f64 = 1e-4;
/// A waypoint counts as reached when the stage is within this distance (mm).
const REACH_TOL: f64 = 0.005;
/// How long a commanded leg may stay idle before the waypoint is declared
/// unreachable (covers the hardware status poll interval).
const UNREACH_GRACE: Duration = Duration::from_secs(2);

/// Why a run ended.
#[derive(Debug, Clone, PartialEq)]
pub enum PathEvent {
    /// The stage walked every waypoint.
    Finished { steps: usize },
    /// The run aborted before completing.
    Failed(String),
}

/// Sequential waypoint walker over a flattened copy of the loaded shapes.
#[derive(Debug)]
pub struct PathRunner {
    waypoints: Vec<Axes>,
    index: usize,
    running: bool,
    commanded: bool,
    commanded_at: Option<Instant>,
    grace: Duration,
    /// Fire the laser while the run is live.
    fire: bool,
    /// The beam has been armed for this run (set once, so an operator
    /// switching it off mid-run keeps control).
    armed: bool,
}

impl Default for PathRunner {
    fn default() -> Self {
        PathRunner {
            waypoints: Vec::new(),
            index: 0,
            running: false,
            commanded: false,
            commanded_at: None,
            grace: UNREACH_GRACE,
            fire: false,
            armed: false,
        }
    }
}

impl PathRunner {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Waypoints reached so far and total count.
    pub fn progress(&self) -> (usize, usize) {
        (self.index, self.waypoints.len())
    }

    /// Flatten `shapes` into waypoints on the plane at height `z` and begin
    /// following them. Returns the waypoint count. Refuses geometry outside
    /// `limits` so the user gets the full bounds report up front.
    ///
    /// `fire` arms the laser on the first commanded waypoint; every run end
    /// switches the beam off again.
    pub fn start(
        &mut self,
        shapes: &[Shape],
        z: f64,
        tolerance: f64,
        limits: &TravelLimits,
        fire: bool,
    ) -> Result<usize, String> {
        if shapes.is_empty() {
            return Err("no geometry loaded".to_string());
        }
        let waypoints = build_waypoints(shapes, z, tolerance.max(1e-4));
        if waypoints.is_empty() {
            return Err("path has no segments".to_string());
        }
        check_limits(&waypoints, limits)?;
        let steps = waypoints.len();
        self.waypoints = waypoints;
        self.index = 0;
        self.running = true;
        self.commanded = false;
        self.commanded_at = None;
        self.fire = fire;
        self.armed = false;
        Ok(steps)
    }

    /// Halt the run where it stands (the stage keeps its current target) and
    /// switch the beam off. Taking the hub guarantees no run can end with the
    /// laser left firing.
    pub fn stop(&mut self, hub: &mut MotionHub) {
        let was_running = self.running;
        self.running = false;
        self.commanded = false;
        self.commanded_at = None;
        if was_running {
            let _ = hub.set_laser(false);
        }
    }

    /// Advance the run. Call once per frame while the stage ticks; returns an
    /// event when the run finishes or aborts.
    pub fn tick(&mut self, hub: &mut MotionHub) -> Option<PathEvent> {
        if !self.running || hub.is_moving() {
            return None;
        }

        // The stage is idle: skip every waypoint it has already reached.
        let pos = hub.pos();
        let mut advanced = false;
        while let Some(wp) = self.waypoints.get(self.index)
            && pos.distance_to(*wp) <= REACH_TOL
        {
            self.index += 1;
            advanced = true;
        }
        if advanced {
            self.commanded = false;
            self.commanded_at = None;
        }

        if self.index >= self.waypoints.len() {
            let steps = self.waypoints.len();
            self.stop(hub);
            return Some(PathEvent::Finished { steps });
        }

        if self.commanded {
            // Already commanded this waypoint and the stage went idle without
            // reaching it — wait out the grace period, then give up.
            let waited = self.commanded_at.is_some_and(|t| t.elapsed() >= self.grace);
            if !waited {
                return None;
            }
            let wp = self.waypoints[self.index];
            let lim = hub.sim.limits;
            self.stop(hub);
            return Some(PathEvent::Failed(format!(
                "waypoint {} ({}, {}) not reached — stage stopped short (limits x [{}, {}] y [{}, {}] mm)",
                self.index,
                fmt_f(wp.x),
                fmt_f(wp.y),
                fmt_f(lim.min.x),
                fmt_f(lim.max.x),
                fmt_f(lim.min.y),
                fmt_f(lim.max.y)
            )));
        }

        let wp = self.waypoints[self.index];
        // Arm the beam before the stage starts moving (the stage is idle here).
        if self.fire && !self.armed {
            if let Err(e) = hub.set_laser(true) {
                self.stop(hub);
                return Some(PathEvent::Failed(format!("laser on failed: {e}")));
            }
            self.armed = true;
        }
        match hub.move_to(wp) {
            Ok(()) => {
                self.commanded = true;
                self.commanded_at = Some(Instant::now());
                None
            }
            Err(e) => {
                self.stop(hub);
                Some(PathEvent::Failed(e))
            }
        }
    }
}

/// Flatten shapes into a waypoint list at height `z`, dropping segments
/// shorter than [`MIN_WP_GAP`].
fn build_waypoints(shapes: &[Shape], z: f64, tolerance: f64) -> Vec<Axes> {
    let mut out: Vec<Axes> = Vec::new();
    for shape in shapes {
        for elem in &shape.elements {
            for p in elem.approx_segments(tolerance) {
                let wp = Axes::new(p.x, p.y, z);
                if out.last().is_none_or(|l| l.distance_to(wp) > MIN_WP_GAP) {
                    out.push(wp);
                }
            }
        }
    }
    out
}

/// Reject waypoints outside the travel limits, reporting the path bounding
/// box next to the allowed range so the drawing/units problem is obvious.
fn check_limits(waypoints: &[Axes], limits: &TravelLimits) -> Result<(), String> {
    let outside = waypoints
        .iter()
        .filter(|w| {
            w.x < limits.min.x
                || w.x > limits.max.x
                || w.y < limits.min.y
                || w.y > limits.max.y
                || w.z < limits.min.z
                || w.z > limits.max.z
        })
        .count();
    if outside == 0 {
        return Ok(());
    }

    let mut min = waypoints[0];
    let mut max = waypoints[0];
    for w in waypoints {
        min.x = min.x.min(w.x);
        min.y = min.y.min(w.y);
        min.z = min.z.min(w.z);
        max.x = max.x.max(w.x);
        max.y = max.y.max(w.y);
        max.z = max.z.max(w.z);
    }
    Err(format!(
        "geometry exceeds travel limits: path x [{}, {}] y [{}, {}] mm \
         vs stage x [{}, {}] y [{}, {}] mm \
         ({outside} of {} waypoints outside)",
        fmt_f(min.x),
        fmt_f(max.x),
        fmt_f(min.y),
        fmt_f(max.y),
        fmt_f(limits.min.x),
        fmt_f(limits.max.x),
        fmt_f(limits.min.y),
        fmt_f(limits.max.y),
        waypoints.len()
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::geo::{Line, PathElement, Point};

    fn line_shape(x0: f64, x1: f64) -> Shape {
        Shape::new(
            vec![PathElement::Line(Line::new(
                Point::new(x0, 0.0),
                Point::new(x1, 0.0),
            ))],
            false,
        )
    }

    fn run(mut runner: PathRunner, hub: &mut MotionHub, max_steps: usize) -> Option<PathEvent> {
        for _ in 0..max_steps {
            if let Some(ev) = runner.tick(hub) {
                return Some(ev);
            }
            hub.sim.step(1.0 / 240.0);
        }
        None
    }

    #[test]
    fn follows_a_line_to_completion() {
        let mut hub = MotionHub::new(500.0, 5000.0);
        let mut runner = PathRunner::new();
        assert_eq!(
            runner.start(
                &[line_shape(0.0, 10.0)],
                0.0,
                0.01,
                &TravelLimits::default(),
                false,
            ),
            Ok(2)
        );
        assert!(runner.is_running());

        let ev = run(runner, &mut hub, 100_000);
        assert_eq!(ev, Some(PathEvent::Finished { steps: 2 }));
        assert!((hub.pos().x - 10.0).abs() < 1e-3, "pos {}", hub.pos().x);
    }

    #[test]
    fn start_rejects_out_of_limits_geometry() {
        let mut runner = PathRunner::new();
        let err = runner
            .start(
                &[line_shape(0.0, 9999.0)],
                0.0,
                0.01,
                &TravelLimits::default(),
                false,
            )
            .unwrap_err();
        assert!(!runner.is_running(), "must not start the run");
        assert!(err.contains("exceeds travel limits"), "{err}");
        assert!(err.contains("9999"), "path extent reported: {err}");
        assert!(err.contains("250"), "stage range reported: {err}");
    }

    #[test]
    fn flags_a_stalled_stage() {
        let mut hub = MotionHub::new(500.0, 5000.0);
        let mut runner = PathRunner::new();
        runner.grace = Duration::ZERO;
        runner
            .start(
                &[line_shape(0.0, 10.0)],
                0.0,
                0.01,
                &TravelLimits::default(),
                false,
            )
            .unwrap();
        // Pretend the beam was firing, then shrink the limits after starting
        // so the stage stops short of wp 1.
        hub.set_laser(true).unwrap();
        hub.sim.limits.max.x = 5.0;

        match run(runner, &mut hub, 100_000) {
            Some(PathEvent::Failed(msg)) => {
                assert!(msg.contains("stopped short"), "{msg}");
                assert!(msg.contains("limits"), "{msg}");
                assert!(!hub.laser_on(), "a failed run must leave the beam off");
            }
            other => panic!("expected failure, got {other:?}"),
        }
    }

    #[test]
    fn start_rejects_empty_geometry() {
        let mut runner = PathRunner::new();
        assert!(
            runner
                .start(&[], 0.0, 0.01, &TravelLimits::default(), false)
                .is_err()
        );
        assert!(!runner.is_running());
    }

    #[test]
    fn stop_halts_the_run_and_the_beam() {
        let mut hub = MotionHub::new(50.0, 500.0);
        let mut runner = PathRunner::new();
        runner
            .start(
                &[line_shape(0.0, 100.0)],
                0.0,
                0.01,
                &TravelLimits::default(),
                false,
            )
            .unwrap();
        for _ in 0..10 {
            runner.tick(&mut hub);
            hub.sim.step(1.0 / 240.0);
        }
        assert!(runner.is_running());
        hub.set_laser(true).unwrap();

        runner.stop(&mut hub);
        assert!(!runner.is_running());
        assert!(!hub.laser_on(), "stopping the run kills the beam");
        assert_eq!(runner.tick(&mut hub), None);
    }

    #[test]
    fn fire_arms_the_beam_and_the_finish_kills_it() {
        let mut hub = MotionHub::new(500.0, 5000.0);
        let mut runner = PathRunner::new();
        runner
            .start(
                &[line_shape(0.0, 10.0)],
                0.0,
                0.01,
                &TravelLimits::default(),
                true,
            )
            .unwrap();
        assert!(!hub.laser_on(), "no beam before the first command");

        runner.tick(&mut hub);
        assert!(hub.laser_on(), "beam fires while the run is live");
        hub.set_laser(false).unwrap(); // operator override mid-run
        runner.tick(&mut hub);
        hub.sim.step(1.0 / 240.0);
        // Armed once: the runner must not re-fire behind the operator's back.
        assert!(!hub.laser_on(), "operator override sticks");

        let ev = run(runner, &mut hub, 100_000);
        assert_eq!(ev, Some(PathEvent::Finished { steps: 2 }));
        assert!(!hub.laser_on(), "beam dies when the run finishes");
    }

    #[test]
    fn fire_without_a_link_fails_the_run_before_moving() {
        let mut hub = MotionHub::new(500.0, 5000.0);
        hub.mode = crate::core::motion::DriveMode::Hardware;
        let mut runner = PathRunner::new();
        runner
            .start(
                &[line_shape(0.0, 10.0)],
                0.0,
                0.01,
                &TravelLimits::default(),
                true,
            )
            .unwrap();

        match runner.tick(&mut hub) {
            Some(PathEvent::Failed(msg)) => {
                assert!(msg.contains("laser on failed"), "{msg}");
                assert!(!runner.is_running());
            }
            other => panic!("expected a laser failure, got {other:?}"),
        }
    }
}
