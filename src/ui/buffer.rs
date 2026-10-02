//! Bounded in-memory log used by the UI log panel.

use std::collections::VecDeque;

use chrono::Local;
use log::Level;

#[derive(Debug, Clone)]
pub struct LogEntry {
    pub time: String,
    pub level: Level,
    pub module: &'static str,
    pub message: String,
}

#[derive(Debug)]
pub struct LogBuffer {
    entries: VecDeque<LogEntry>,
    capacity: usize,
    pub min_level: Level,
}

impl LogBuffer {
    pub fn new(capacity: usize) -> Self {
        LogBuffer {
            entries: VecDeque::with_capacity(capacity.min(1024)),
            capacity: capacity.max(1),
            min_level: Level::Info,
        }
    }

    /// Record an entry and mirror it to the process logger (console + file).
    pub fn push(&mut self, level: Level, module: &'static str, message: impl Into<String>) {
        let message = message.into();
        match level {
            Level::Error => log::error!("{module}: {message}"),
            Level::Warn => log::warn!("{module}: {message}"),
            Level::Info => log::info!("{module}: {message}"),
            Level::Debug => log::debug!("{module}: {message}"),
            Level::Trace => log::trace!("{module}: {message}"),
        }
        if self.entries.len() >= self.capacity {
            self.entries.pop_front();
        }
        self.entries.push_back(LogEntry {
            time: Local::now().format("%H:%M:%S%.3f").to_string(),
            level,
            module,
            message,
        });
    }

    pub fn error(&mut self, module: &'static str, message: impl Into<String>) {
        self.push(Level::Error, module, message);
    }

    pub fn warn(&mut self, module: &'static str, message: impl Into<String>) {
        self.push(Level::Warn, module, message);
    }

    pub fn info(&mut self, module: &'static str, message: impl Into<String>) {
        self.push(Level::Info, module, message);
    }

    pub fn visible(&self) -> impl Iterator<Item = &LogEntry> {
        self.entries
            .iter()
            .filter(move |e| e.level <= self.min_level)
    }

    pub fn last(&self) -> Option<&LogEntry> {
        self.entries.back()
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

impl Default for LogBuffer {
    fn default() -> Self {
        LogBuffer::new(2000)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drops_oldest_when_full() {
        let mut buf = LogBuffer::new(2);
        buf.info("t", "one");
        buf.info("t", "two");
        buf.info("t", "three");
        assert_eq!(buf.len(), 2);
        assert_eq!(buf.visible().next().unwrap().message, "two");
    }

    #[test]
    fn filters_by_level() {
        let mut buf = LogBuffer::new(8);
        buf.error("t", "bad");
        buf.info("t", "ok");
        buf.min_level = Level::Error;
        assert_eq!(buf.visible().count(), 1);
    }
}
