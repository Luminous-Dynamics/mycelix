// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Build-validated mapping from curriculum node IDs to lesson assets.

use serde::Deserialize;
use std::collections::HashMap;
use std::sync::OnceLock;

const CONTENT_MANIFEST_JSON: &str = include_str!("../static/content-manifest.json");

static CONTENT_MANIFEST: OnceLock<Result<ContentManifest, String>> = OnceLock::new();

#[derive(Debug, Deserialize)]
struct ContentManifest {
    schema_version: u32,
    lessons: HashMap<String, Vec<LessonAsset>>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct LessonAsset {
    pub path: String,
    pub title: String,
}

fn content_manifest() -> Result<&'static ContentManifest, &'static str> {
    CONTENT_MANIFEST
        .get_or_init(|| {
            let manifest: ContentManifest = serde_json::from_str(CONTENT_MANIFEST_JSON)
                .map_err(|error| format!("invalid content manifest: {error}"))?;
            if manifest.schema_version != 1 {
                return Err(format!(
                    "unsupported content manifest schema {}",
                    manifest.schema_version
                ));
            }
            Ok(manifest)
        })
        .as_ref()
        .map_err(String::as_str)
}

/// All generated lesson assets registered for a curriculum node.
///
/// Multiple lessons may intentionally target one broad standard. The first
/// path is stable and acts as the primary lesson until the UI exposes a lesson
/// chooser.
pub fn lesson_assets(node_id: &str) -> Result<&'static [LessonAsset], &'static str> {
    Ok(match content_manifest()?.lessons.get(node_id) {
        Some(assets) => assets.as_slice(),
        None => &[],
    })
}

pub fn primary_lesson_url(node_id: &str) -> Result<Option<&'static str>, &'static str> {
    Ok(lesson_assets(node_id)?
        .first()
        .map(|asset| asset.path.as_str()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_in_manifest_is_valid_and_nonempty() {
        let manifest = content_manifest().expect("manifest must parse");
        assert_eq!(manifest.schema_version, 1);
        assert!(!manifest.lessons.is_empty());
        assert!(manifest.lessons.values().flatten().all(|asset| {
            asset.path.starts_with("/curriculum/generated/") && !asset.title.is_empty()
        }));
    }
}
