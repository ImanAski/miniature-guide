use acadrust::{DxfError, DxfReader, DxfVersion, EntityType};
use std::{collections::BTreeMap, path::Path};

#[derive(Debug, Clone)]
pub struct DxfEntity {
    version: String,
}

fn entity_variant_name(entity: &EntityType) -> String {
    let dbg = format!("{:?}", entity);
    if let Some(paren_pos) = dbg.find('(') {
        dbg[..paren_pos].to_string()
    } else {
        dbg
    }
}

pub fn parse(path: &Path) -> Result<(), DxfError> {
    let doc = DxfReader::from_file(path)?.read()?;
    // let entity = DxfEntity {
    //     version: doc.version.to_string(),
    // };
    let mut orig_by_type: BTreeMap<String, Vec<&EntityType>> = BTreeMap::new();

    for e in doc.entities() {
        orig_by_type
            .entry(entity_variant_name(e))
            .or_default()
            .push(e);
    }

    for (type_name, orig_entities) in &orig_by_type {
        let count = orig_entities.len();
        for i in 0..count {
            let mut o = orig_entities[i].clone();
            let o_common = o.common();
            normalize_entity(&mut o);
        }
    }
    Ok(())
}

fn normalize_entity(entity: &mut EntityType) {
    match entity {
        EntityType::Arc(a) => println!("it's a arc with len: {}", a.arc_length()),
        EntityType::Circle(a) => println!("circle"),
        EntityType::Line(a) => println!("Line"),
        EntityType::Polyline2D(a) => println!("Polyline 2D"),
        EntityType::Insert(a) => println!("Insert"),
        _ => println!("others"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_dxf_file() {
        let path = Path::new("tests/data/example.dxf");

        let result = parse(path);

        assert!(result.is_ok());
    }
}
