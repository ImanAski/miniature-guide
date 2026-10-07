use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{BufRead, BufReader, Write};
use std::time::{Duration, Instant};

use std::fmt;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("Failed to open port '{port}': {source}")]
    OpenPort {
        port: String,
        source: Box<dyn std::error::Error + Send + Sync>,
    },
    #[error("Serial I/O error: {0}")]
    Serial(#[from] std::io::Error),
    #[error("Timeout: {0}")]
    Timeout(String),
    #[error("Unexpected response: expected '{expected}', got '{got}'")]
    UnexpectedResponse { expected: String, got: String },
}

#[derive(Debug, Clone)]
pub struct LinkConfig {
    pub port: String,
    pub baud: u32,
    pub timeout: Duration,
}

#[derive(Debug, Clone)]
pub struct ControllerStatus {
    pub pos_x: f64,
    pub pos_y: f64,
    pub pos_z: f64,
    pub motor_state: MotorState,
    pub err_code: Option<i32>,
    pub buffer_count: u32,
    pub is_home: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum MotorState {
    Stopped,
    Moving,
    Error,
}

impl Default for MotorState {
    fn default() -> Self {
        MotorState::Stopped
    }
}

impl fmt::Display for MotorState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MotorState::Stopped => write!(f, "Stopped"),
            MotorState::Moving => write!(f, "Moving"),
            MotorState::Error => write!(f, "Error"),
        }
    }
}

pub struct SerialDriver {
    reader: BufReader<Box<dyn SerialPort>>,
    writer: Box<dyn SerialPort>,
    name: String,
    timeout: Duration,
}

impl SerialDriver {
    /// Open a connection to the ESP301
    pub fn open(cfg: &LinkConfig) -> Result<Self, Error> {
        let builder = serialport::new(&cfg.port, cfg.baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None) // ESP301 typically doesn't use HW flow
            .timeout(cfg.timeout);

        let port = builder.open().map_err(|e| Error::OpenPort {
            port: cfg.port.clone(),
            source: Box::new(e),
        })?;

        // Give controller time to initialize
        std::thread::sleep(Duration::from_millis(100));

        Ok(SerialDriver {
            reader: BufReader::new(port.try_clone().unwrap()),
            writer: port,
            name: cfg.port.clone(),
            timeout: cfg.timeout,
        })
    }

    /// Port identifier this driver was opened on (for logs/status).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Send a raw command string (with CR terminator)
    pub fn send(&mut self, cmd: &str) -> Result<(), Error> {
        let full = format!("{}\r", cmd);
        self.writer
            .write_all(full.as_bytes())
            .map_err(Error::Serial)?;
        self.writer.flush().map_err(Error::Serial)?;
        Ok(())
    }

    /// Read a single line response (with timeout)
    pub fn read_line(&mut self) -> Result<String, Error> {
        let mut line = String::new();
        let start = Instant::now();

        loop {
            let remaining = self.timeout.saturating_sub(start.elapsed());
            if remaining.is_zero() {
                return Err(Error::Timeout("read_line exceeded timeout".to_string()));
            }

            match self.reader.read_line(&mut line) {
                Ok(0) => {
                    return Err(Error::Timeout("connection closed by remote".to_string()));
                }
                Ok(_) => {
                    // Remove CR/LF
                    let trimmed = line.trim_end_matches(|c| c == '\r' || c == '\n');
                    return Ok(trimmed.to_string());
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    if start.elapsed() >= self.timeout {
                        return Err(Error::Timeout("read_line timed out".to_string()));
                    }
                    continue;
                }
                Err(e) => return Err(Error::Serial(e)),
            }
        }
    }

    /// Send a command and read its response
    pub fn query(&mut self, cmd: &str) -> Result<String, Error> {
        self.send(cmd)?;
        self.read_line()
    }

    /// Send a batch of commands (semicolon-separated), drain responses
    pub fn send_batch(&mut self, commands: &[&str]) -> Result<(), Error> {
        let batch = commands.join(";");
        self.send(&batch)?;
        // ESP301 returns one response per command
        for _ in commands {
            let _ = self.read_line()?;
        }
        Ok(())
    }

    /// Check if controller is alive
    pub fn ping(&mut self) -> Result<String, Error> {
        self.query("I")
    }

    /// Get current position
    pub fn get_position(&mut self) -> Result<(f64, f64, f64), Error> {
        let resp = self.query("QPOS")?;
        // Response format: "QPOS,1,1.23,-4.56,7.89"
        let parts: Vec<&str> = resp.split(',').collect();
        if parts.len() >= 5 {
            let x: f64 = parts[2].parse().unwrap_or(0.0);
            let y: f64 = parts[3].parse().unwrap_or(0.0);
            let z: f64 = parts[4].parse().unwrap_or(0.0);
            Ok((x, y, z))
        } else {
            Err(Error::UnexpectedResponse {
                expected: "QPOS,<slot>,<x>,<y>,<z>".to_string(),
                got: resp,
            })
        }
    }

    /// Get motor state
    pub fn get_motor_state(&mut self) -> Result<MotorState, Error> {
        let resp = self.query("QSTAT")?;
        // Response contains status flags
        if resp.contains("0") && !resp.contains("1") {
            Ok(MotorState::Stopped)
        } else if resp.contains("MOVE") || resp.contains("1") {
            Ok(MotorState::Moving)
        } else {
            Ok(MotorState::Stopped)
        }
    }

    /// Get full controller status
    pub fn status(&mut self) -> Result<ControllerStatus, Error> {
        let (pos_x, pos_y, pos_z) = self.get_position()?;
        let motor = self.get_motor_state()?;

        // Check for errors
        let err_code = if self.query("QERR")?.starts_with("0") {
            None
        } else {
            Some(1) // Simplified — actual error code parsing
        };

        // Check homing status
        let is_home = self.query("QHOME")?.contains("1");

        Ok(super::ControllerStatus {
            pos_x,
            pos_y,
            pos_z,
            motor_state: motor,
            err_code,
            buffer_count: 0, // Would need QBUF command
            is_home,
        })
    }

    /// Move to absolute position
    pub fn move_absolute(&mut self, x: f64, y: f64, z: f64) -> Result<(), Error> {
        // SYNCH command for coordinated move
        let cmd = format!("SYNCH,{},{}", x, y);
        self.send(&cmd)?;
        if z != 0.0 {
            let z_cmd = format!("Z,{}", z);
            self.send(&z_cmd)?;
        }
        Ok(())
    }

    /// Move absolute with Z
    pub fn move_absolute_xyz(&mut self, x: f64, y: f64, z: f64) -> Result<(), Error> {
        let cmd = format!("A,{},{}", x, y);
        self.send(&cmd)?;
        let z_cmd = format!("Z,{}", z);
        self.send(&z_cmd)?;
        Ok(())
    }

    /// Relative jog move (ESP301 `G0` with per-axis deltas, e.g. `G0x1.000y-0.500`)
    pub fn jog_relative(&mut self, x: f64, y: f64, z: f64) -> Result<(), Error> {
        let mut cmd = String::from("G0");
        for (letter, v) in [('x', x), ('y', y), ('z', z)] {
            if v != 0.0 {
                cmd.push(letter);
                cmd.push_str(&format!("{:.3}", v));
            }
        }
        self.send(&cmd)
    }

    /// Send motion buffer (batch of moves)
    pub fn buffer_move(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        _vel: f64,
        _acc: f64,
    ) -> Result<(), Error> {
        // SYM — buffer a move (synch move with velocity/acceleration)
        let cmd = format!("SYNCH,{},{}", x, y);
        self.send(&cmd)?;
        if z != 0.0 {
            self.send(&format!("Z,{}", z))?;
        }
        Ok(())
    }

    /// Execute buffer (start motion)
    pub fn execute_buffer(&mut self) -> Result<(), Error> {
        // ESP301 starts executing buffer entries automatically
        // May need SYNCH command to trigger
        Ok(())
    }

    /// Stop all motion immediately
    pub fn stop(&mut self) -> Result<(), Error> {
        self.send("STOP")?;
        Ok(())
    }

    /// Stop all motion smoothly
    pub fn stop_smooth(&mut self) -> Result<(), Error> {
        self.send("STOPO")?; // Stop with deceleration
        Ok(())
    }

    /// Home all axes
    pub fn home(&mut self, axes: &[u8]) -> Result<(), Error> {
        for &axis in axes {
            self.send(&format!("H{}", axis))?;
            let _ = self.read_line()?;
        }
        Ok(())
    }

    /// Set velocity (units/s)
    pub fn set_velocity(&mut self, vel: f64) -> Result<(), Error> {
        self.send(&format!("V,{}", vel))?;
        Ok(())
    }

    /// Set acceleration (units/s²)
    pub fn set_acceleration(&mut self, acc: f64) -> Result<(), Error> {
        self.send(&format!("A,{}", acc))?;
        Ok(())
    }

    /// Set laser output (digital output control)
    /// pin: 1-8 (ESP301 digital output channels)
    /// state: 1=on, 0=off
    pub fn set_digital_out(&mut self, pin: u8, state: bool) -> Result<(), Error> {
        let state_val = if state { 1 } else { 0 };
        self.send(&format!("DO{},{}", pin, state_val))?;
        Ok(())
    }

    /// Close the connection
    pub fn close(&mut self) -> Result<(), Error> {
        self.stop()?;
        Ok(())
    }
}

impl Drop for SerialDriver {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}
