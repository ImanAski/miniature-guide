pub const CR: &str = "\r";

#[derive(Debug, Clone, PartialEq)]
pub struct Command {
    pub prefix: Option<u8>,
    pub mnemonic: String,
    pub params: Vec<Param>,
    pub query: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Param {
    Int(i64),
    Float(f64),
    Raw(String),
}

impl From<i32> for Param {
    fn from(v: i32) -> Self {
        Param::Int(v as i64)
    }
}

impl From<i64> for Param {
    fn from(v: i64) -> Self {
        Param::Int(v)
    }
}

impl From<u8> for Param {
    fn from(v: u8) -> Self {
        Param::Int(v as i64)
    }
}

impl From<u16> for Param {
    fn from(v: u16) -> Self {
        Param::Int(v as i64)
    }
}

impl From<usize> for Param {
    fn from(v: usize) -> Self {
        Param::Int(v as i64)
    }
}

impl From<f64> for Param {
    fn from(v: f64) -> Self {
        Param::Float(v)
    }
}

impl From<&str> for Param {
    fn from(v: &str) -> Self {
        Param::Raw(v.to_string())
    }
}

fn num(v: f64) -> String {
    format!("{:.3}", v)
}

impl Param {
    pub fn render(&self) -> String {
        match self {
            Param::Int(i) => i.to_string(),
            Param::Float(f) => num(*f),
            Param::Raw(s) => s.clone(),
        }
    }
}

impl Command {
    pub fn new(mnemonic: &str) -> Self {
        Command {
            prefix: None,
            mnemonic: mnemonic.to_ascii_uppercase(),
            params: Vec::new(),
            query: false,
        }
    }

    pub fn p(mut self, prefix: u8) -> Self {
        self.prefix = Some(prefix);
        self
    }

    pub fn arg<T: Into<Param>>(mut self, v: T) -> Self {
        self.params.push(v.into());
        self
    }

    pub fn arg_opt<T: Into<Param>>(mut self, v: Option<T>) -> Self {
        if let Some(v) = v {
            self.params.push(v.into());
        }
        self
    }

    /// Render to the exact bytes the controller expects (without terminator).
    pub fn encode(&self) -> String {
        use std::fmt::Write;
        let mut s = String::with_capacity(16);
        if let Some(p) = self.prefix {
            let _ = write!(s, "{p}");
        }
        s.push_str(&self.mnemonic);
        if self.query {
            s.push('?');
        } else if !self.params.is_empty() {
            s.push_str(
                &self
                    .params
                    .iter()
                    .map(|p| p.render())
                    .collect::<Vec<_>>()
                    .join(","),
            );
        }
        s
    }
}

impl std::fmt::Display for Command {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.encode())
    }
}

pub fn join(cmds: &[Command]) -> String {
    let mut s = String::new();
    for (i, c) in cmds.iter().enumerate() {
        if i > 0 {
            s.push(';');
        }
        s.push_str(&c.encode());
    }
    s
}
