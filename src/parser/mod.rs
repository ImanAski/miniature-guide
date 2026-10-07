mod affine;
pub mod dxf;
pub mod gds;

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
        "dxf" => Ok(dxf::parse(path)?),
        "gds" => Ok(gds::parse(path)?),
        _ => Err(ParseError::UnsupportedFormat(ext)),
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ParseError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("DXF error: {0}")]
    Dxf(#[from] acadrust::DxfError),

    #[error("GDS error: {0}")]
    Gds(#[from] gds21::GdsError),

    #[error("UnsupportedFormat: {0}")]
    UnsupportedFormat(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `tests/` is not tracked in git, so a fresh checkout (CI) may lack the
    /// fixtures. `None` means "skip"; local runs still execute the test.
    fn fixture(path: &'static str) -> Option<&'static Path> {
        let p = Path::new(path);
        if p.exists() {
            Some(p)
        } else {
            eprintln!("skipping {path}: fixture not present in this checkout");
            None
        }
    }

    #[test]
    fn dispatches_dxf_by_extension() {
        let Some(path) = fixture("tests/data/example.dxf") else {
            return;
        };

        let shapes = parse_file(path).expect("example.dxf should parse");

        assert!(!shapes.is_empty());
    }

    #[test]
    fn dispatches_gds_by_extension() {
        let Some(path) = fixture("tests/data/gds_example.gds") else {
            return;
        };

        let shapes = parse_file(path).expect("gds_example.gds should parse");

        assert!(!shapes.is_empty());
    }

    #[test]
    fn rejects_unknown_extensions() {
        let err = parse_file(Path::new("drawing.gcode")).expect_err("gcode is not supported yet");

        assert!(matches!(err, ParseError::UnsupportedFormat(ext) if ext == "gcode"));
    }

    #[test]
    fn reports_missing_extension() {
        let err = parse_file(Path::new("drawing")).expect_err("no extension");

        assert!(matches!(err, ParseError::UnsupportedFormat(ext) if ext.is_empty()));
    }
}
