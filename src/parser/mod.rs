use crate::core::geo::Shape;
use std::path::Path;

pub enum FileType {
    DXF,
    GDS,
    GCODE,
}

impl FileType {
    pub fn extension(&self) -> Vec<&'static str> {
        match self {
            FileType::DXF => vec!["dxf"],
            FileType::GDS => vec!["gds"],
            FileType::GCODE => vec!["gcode"],
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            FileType::DXF => "dxf drawing",
            FileType::GDS => "gds drawing",
            FileType::GCODE => "gcode drawing",
        }
    }
}
pub fn parse_file(path: &Path) -> Result<Vec<Shape>, ParseError> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.to_lowercase())
        .unwrap_or_default();

    match ext.as_str() {
        _ => Err(ParseError::UnsupportedFormat(ext)),
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ParseError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("UnsupportedFormat: {0}")]
    UnsupportedFormat(String),
}
