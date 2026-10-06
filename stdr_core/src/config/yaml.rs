//! Include expansion for robot yaml: a node `{filename: F, <kind>_specifications: {...}}` means
//! "F's `<kind>.<kind>_specifications`, overridden field-wise (recursively) by the inline mapping".

use std::path::Path;

use serde_yaml_ng::{Mapping, Value};

use crate::error::CoreError;

/// Bounds include chains so a file that includes itself errors instead of overflowing the stack.
const MAX_INCLUDE_DEPTH: usize = 16;

/// Returns the fully expanded `<kind>_specifications` mapping of `node` (empty when absent).
/// Nested includes such as a sonar file's `noise: {filename: ...}` are expanded too. Every
/// `filename` is relative to `base_dir`.
pub(crate) fn specifications(node: Value, kind: &str, base_dir: &Path) -> Result<Value, CoreError> {
    expand(node, kind, base_dir, 0)
}

fn expand(node: Value, kind: &str, base_dir: &Path, depth: usize) -> Result<Value, CoreError> {
    let key = format!("{kind}_specifications");
    let Value::Mapping(mut node) = node else {
        return Ok(Value::Mapping(Mapping::new()));
    };
    let inline = node.remove(&key).unwrap_or(Value::Null);
    let mut spec = match node.get("filename") {
        Some(file) => {
            if depth >= MAX_INCLUDE_DEPTH {
                return Err(CoreError::Invalid(format!(
                    "include chain deeper than {MAX_INCLUDE_DEPTH} at {file:?}"
                )));
            }
            let rel = file.as_str().ok_or_else(|| {
                CoreError::Invalid(format!("filename must be a string, got {file:?}"))
            })?;
            let path = base_dir.join(rel);
            let text = std::fs::read_to_string(&path).map_err(|source| CoreError::Io {
                path: path.clone(),
                source,
            })?;
            let root: Value = serde_yaml_ng::from_str(&text)
                .map_err(|source| CoreError::Yaml { path, source })?;
            root.get(kind)
                .and_then(|k| k.get(&key))
                .cloned()
                .unwrap_or(Value::Null)
        }
        None => Value::Null,
    };
    deep_merge(&mut spec, inline);
    let Value::Mapping(spec) = spec else {
        return Ok(Value::Mapping(Mapping::new()));
    };

    // A child mapping carrying `filename` is itself an include, keyed by its own name (e.g. `noise`).
    spec.into_iter()
        .map(|(k, v)| {
            let v = match (k.as_str(), &v) {
                (Some(child), Value::Mapping(m)) if m.contains_key("filename") => {
                    let child_key = format!("{child}_specifications");
                    let expanded = expand(v, child, base_dir, depth + 1)?;
                    Value::Mapping(Mapping::from_iter([(Value::from(child_key), expanded)]))
                }
                _ => v,
            };
            Ok((k, v))
        })
        .collect::<Result<Mapping, CoreError>>()
        .map(Value::Mapping)
}

/// Mapping onto mapping merges per key, recursively; a null overlay keeps the base (an empty yaml
/// key overrides nothing); any other overlay replaces the base.
fn deep_merge(base: &mut Value, overlay: Value) {
    match (base, overlay) {
        (_, Value::Null) => {}
        (Value::Mapping(base), Value::Mapping(overlay)) => {
            for (k, v) in overlay {
                match base.get_mut(&k) {
                    Some(b) => deep_merge(b, v),
                    None => {
                        base.insert(k, v);
                    }
                }
            }
        }
        (base, overlay) => *base = overlay,
    }
}
