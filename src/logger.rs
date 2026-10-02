//! Logger module — structured logging with timestamps and git info.
//!
//! Console output comes from `env_logger`; the same lines are appended to a
//! per-session file under the log directory. Installing happens once, on the
//! first [`Logger::init`].

use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};

use chrono::Local;
use log::{Level, LevelFilter};
use uuid::Uuid;

/// Session log file, set by [`Logger::init`].
static SINK: OnceLock<Mutex<File>> = OnceLock::new();

/// A log entry with full context
#[derive(Debug, Clone)]
pub struct LogEntry {
    pub timestamp: String,
    pub level: Level,
    pub module: &'static str,
    pub message: String,
    pub git_commit: String,
    pub session_id: String,
}

impl std::fmt::Display for LogEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "[{}] {} {} [{}/{}] {}",
            self.timestamp, self.level, self.git_commit, self.session_id, self.module, self.message,
        )
    }
}

/// The application logger singleton
#[derive(Debug)]
pub struct Logger {
    log_dir: PathBuf,
    session_id: String,
    git_commit: String,
}

impl Logger {
    /// Initialize the logger with a log directory. File failures degrade to
    /// console-only logging instead of taking the app down.
    pub fn init(log_dir: PathBuf) -> Self {
        let session_id = Uuid::new_v4().to_string();
        let git_commit = Self::get_git_commit();

        if let Err(e) = std::fs::create_dir_all(&log_dir) {
            eprintln!("lithowrite: cannot create {}: {e}", log_dir.display());
        }

        let path = log_dir.join(format!("{}.log", Local::now().format("%Y%m%d_%H%M%S")));
        match OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(&path)
        {
            Ok(file) => {
                let _ = SINK.set(Mutex::new(file));
            }
            Err(e) => eprintln!("lithowrite: cannot open {}: {e}", path.display()),
        }

        let _ = env_logger::Builder::new()
            .filter_level(LevelFilter::Info)
            .format(emit)
            .try_init();

        Logger {
            log_dir,
            session_id,
            git_commit,
        }
    }

    fn get_git_commit() -> String {
        if let Ok(git) = std::process::Command::new("git")
            .args(&["rev-parse", "--short", "HEAD"])
            .output()
        {
            if git.status.success() {
                return String::from_utf8_lossy(&git.stdout).trim().to_string();
            }
        }
        "unknown".to_string()
    }

    pub fn session_id(&self) -> &str {
        &self.session_id
    }

    pub fn git_commit(&self) -> &str {
        &self.git_commit
    }

    pub fn log_dir(&self) -> &PathBuf {
        &self.log_dir
    }
}

/// Console + file sink installed as the `log` crate backend.
fn emit(buf: &mut env_logger::fmt::Formatter, record: &log::Record<'_>) -> std::io::Result<()> {
    let ts = Local::now().format("%Y-%m-%d %H:%M:%S%.3f");
    writeln!(
        buf,
        "{} [{}] {} {}",
        ts,
        record.level(),
        record.module_path().unwrap_or("-"),
        record.args()
    )?;

    if let Some(file) = SINK.get()
        && let Ok(mut f) = file.lock()
    {
        let _ = writeln!(
            f,
            "[{ts}] {} {} {}",
            record.level(),
            record.module_path().unwrap_or("-"),
            record.args()
        );
        let _ = f.flush();
    }
    Ok(())
}
