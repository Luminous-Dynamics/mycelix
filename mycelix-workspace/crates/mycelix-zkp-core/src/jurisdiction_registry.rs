// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
//! Published jurisdiction bounding-box sets.
//!
//! The registry is ordinary geographic metadata. It carries no proof,
//! attestation, governance, residency, tax, or legal authority. Proof systems may
//! reference a registry/box identity only through separately qualified theorems.

use crate::jurisdiction_types::JurisdictionBox;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::HashMap;

/// A named collection of jurisdiction bounding boxes.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JurisdictionRegistry {
    /// Human-readable registry name.
    pub name: String,
    /// Semver-style version string.
    pub version: String,
    /// ISO 8601 publication timestamp.
    pub published_at: String,
    /// Optional publisher DID or web URL.
    pub publisher: Option<String>,
    /// All boxes in this registry, indexed by `JurisdictionBox::id`.
    pub boxes: Vec<JurisdictionBox>,
}

impl JurisdictionRegistry {
    /// Content-addressed digest over the canonical box-list representation.
    ///
    /// This digest identifies registry content; it does not endorse the registry
    /// or establish legal/geographic authority.
    pub fn digest(&self) -> [u8; 32] {
        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-JURISDICTION-REGISTRY:v1:");
        hasher.update(self.name.as_bytes());
        hasher.update(b":");
        hasher.update(self.version.as_bytes());
        hasher.update(b":");

        let mut ids: Vec<&str> = self.boxes.iter().map(|b| b.id.as_str()).collect();
        ids.sort_unstable();
        for id in ids {
            let b = self.boxes.iter().find(|b| b.id == id).expect("id present");
            hasher.update(b.id.as_bytes());
            hasher.update(b":");
            hasher.update(b.lat_min_biased.to_le_bytes());
            hasher.update(b.lat_max_biased.to_le_bytes());
            hasher.update(b.lng_min_biased.to_le_bytes());
            hasher.update(b.lng_max_biased.to_le_bytes());
            hasher.update(b";");
        }

        let digest = hasher.finalize();
        let mut out = [0u8; 32];
        out.copy_from_slice(&digest);
        out
    }

    pub fn box_by_id(&self, id: &str) -> Option<&JurisdictionBox> {
        self.boxes.iter().find(|b| b.id == id)
    }

    pub fn boxes_with_prefix(&self, prefix: &str) -> Vec<&JurisdictionBox> {
        self.boxes
            .iter()
            .filter(|b| b.id.starts_with(prefix))
            .collect()
    }

    pub fn boxes_containing(&self, lat_degrees: f64, lng_degrees: f64) -> Vec<&JurisdictionBox> {
        self.boxes
            .iter()
            .filter(|b| b.contains_degrees(lat_degrees, lng_degrees))
            .collect()
    }

    pub fn len(&self) -> usize {
        self.boxes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.boxes.is_empty()
    }

    pub fn index(&self) -> HashMap<&str, &JurisdictionBox> {
        self.boxes.iter().map(|b| (b.id.as_str(), b)).collect()
    }
}

/// Coarse starter registry for US tax-residency demonstrations.
///
/// This seed is metadata only and must not be interpreted as an authoritative
/// legal residency determination.
pub fn seed_registry_us_tax_residency() -> JurisdictionRegistry {
    JurisdictionRegistry {
        name: "US-tax-residency".to_string(),
        version: "v1-seed".to_string(),
        published_at: "2026-04-18T00:00:00Z".to_string(),
        publisher: Some("did:mycelix:luminous-dynamics".to_string()),
        boxes: vec![
            JurisdictionBox::from_degrees(
                "US-tax-residency-v1-conus",
                24.396308,
                49.384358,
                -125.0,
                -66.93457,
            )
            .expect("valid CONUS box"),
            JurisdictionBox::from_degrees(
                "US-tax-residency-v1-alaska",
                51.214183,
                71.538800,
                -179.148909,
                -129.979510,
            )
            .expect("valid Alaska box"),
            JurisdictionBox::from_degrees(
                "US-tax-residency-v1-hawaii",
                18.776344,
                22.236,
                -160.247,
                -154.806,
            )
            .expect("valid Hawaii box"),
        ],
    }
}

/// Coarse starter registry for South African SARS demonstrations.
///
/// This seed is metadata only and must not be interpreted as an authoritative
/// tax-residency or legal-location determination.
pub fn seed_registry_sa_sars() -> JurisdictionRegistry {
    JurisdictionRegistry {
        name: "ZA-SARS".to_string(),
        version: "v1-seed".to_string(),
        published_at: "2026-04-18T00:00:00Z".to_string(),
        publisher: Some("did:mycelix:luminous-dynamics".to_string()),
        boxes: vec![
            JurisdictionBox::from_degrees(
                "ZA-SARS-v1-mainland",
                -34.833333,
                -22.125,
                16.448056,
                32.891667,
            )
            .expect("valid South Africa mainland box"),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn us_seed_has_three_boxes() {
        let r = seed_registry_us_tax_residency();
        assert_eq!(r.len(), 3);
        assert!(r.box_by_id("US-tax-residency-v1-conus").is_some());
        assert!(r.box_by_id("US-tax-residency-v1-alaska").is_some());
        assert!(r.box_by_id("US-tax-residency-v1-hawaii").is_some());
    }

    #[test]
    fn sa_seed_has_one_box() {
        let r = seed_registry_sa_sars();
        assert_eq!(r.len(), 1);
        assert!(r.box_by_id("ZA-SARS-v1-mainland").is_some());
    }

    #[test]
    fn prefix_query_works() {
        let r = seed_registry_us_tax_residency();
        assert_eq!(r.boxes_with_prefix("US-tax-residency-v1-").len(), 3);
        assert!(r.boxes_with_prefix("ZA-").is_empty());
    }

    #[test]
    fn containment_query_uses_safe_geometry() {
        let r = seed_registry_sa_sars();
        let matches = r.boxes_containing(-26.1625, 27.8725);
        assert_eq!(matches.len(), 1);
        assert_eq!(matches[0].id, "ZA-SARS-v1-mainland");
        assert!(r.boxes_containing(f64::NAN, 27.8725).is_empty());
        assert!(r.boxes_containing(-26.1625, f64::INFINITY).is_empty());
    }

    #[test]
    fn digest_deterministic_across_reorderings() {
        let mut a = seed_registry_us_tax_residency();
        let d_a = a.digest();
        a.boxes.reverse();
        assert_eq!(d_a, a.digest());
    }

    #[test]
    fn digest_distinguishes_registries() {
        assert_ne!(
            seed_registry_us_tax_residency().digest(),
            seed_registry_sa_sars().digest()
        );
    }

    #[test]
    fn empty_registry_has_len_zero() {
        let r = JurisdictionRegistry {
            name: "empty".to_string(),
            version: "0".to_string(),
            published_at: "2026-04-18T00:00:00Z".to_string(),
            publisher: None,
            boxes: vec![],
        };
        assert!(r.is_empty());
        assert_eq!(r.len(), 0);
    }

    #[test]
    fn index_lookup_round_trips() {
        let r = seed_registry_us_tax_residency();
        let idx = r.index();
        assert_eq!(idx.len(), 3);
        assert_eq!(
            idx["US-tax-residency-v1-conus"].id,
            "US-tax-residency-v1-conus"
        );
    }
}
