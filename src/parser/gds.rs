use std::path::Path;

use gds21::{GdsError, GdsLibrary};

use crate::core::Shape;

pub struct GdsEntry {}

pub fn parse(path: &Path) -> Result<Vec<Shape>, GdsError> {
    let lib = GdsLibrary::load(path);
    match lib {
        Ok(v) => collect(v),
        Err(e) => Err(e),
    }
    collect(lib);
    Ok(vec![])
}

fn collect(lib: GdsLibrary) -> Result<Vec<Shape>, std::io::Error> {}

#[cfg(test)]
mod tests {
    use super::*;

    const EXAMPLE: &str = "tests/data/gds_example.gds";

    #[test]
    fn parser_test() {
        let _ = parse(Path::new(EXAMPLE));
    }
}
