//! Logger module — structured logging with timestamps and git info.

use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::PathBuf;
use std::sync::Mutex;

use chrono::Local;
use log::Level;
use uuid::Uuid;

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
pub struct Logger {
    log_dir: PathBuf,
    session_id: String,
    git_commit: String,
    file: Mutex<File>,
}

impl Logger {
    /// Initialize the logger with a log directory
    pub fn init(log_dir: PathBuf) -> Result<Self, std::io::Error> {
        let session_id = Uuid::new_v4().to_string();
        let git_commit = Self::get_git_commit();

        // Create log directory
        std::fs::create_dir_all(&log_dir)?;

        // Create log file with timestamp
        let now = Local::now();
        let log_file_name = format!("{}.log", now.format("%Y%m%d_%H%M%S"));
        let log_path = log_dir.join(&log_file_name);

        let file = OpenOptions::new()
            .create(true)
            .write(true)
            .append(false)
            .open(&log_path)?;

        // Init env_logger for console/stderr
        env_logger::Builder::new()
            .filter_level(log::LevelFilter::Info)
            .format(|buf, record| {
                let ts = Local::now().format("%H:%M:%S%.3f");
                writeln!(
                    buf,
                    "{} [{}] {} {}",
                    ts,
                    record.level(),
                    record.module_path().unwrap_or("unknown"),
                    record.args()
                )
            })
            .init();

        Ok(Logger {
            log_dir,
            session_id,
            git_commit,
            file: Mutex::new(file),
        })
    }

    fn get_git_commit() -> String {
        // Fallback: git describe from env
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

    fn log_to_file(&self, entry: &LogEntry) {
        if let Ok(mut file) = self.file.lock() {
            let _ = writeln!(file, "{}", entry);
            let _ = file.flush();
        }
    }
}
