// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provenance contract for observations captured by forestry robots.
//!
//! This module records what a robot observed and the provenance needed to
//! revisit it. A review state is not a timber-grade, species, legal-harvest,
//! or structural-use certification. Physical accuracy still needs suitable
//! ground truth, calibration evidence, sampling design, and independent review.

use super::wood_products::WoodCalibrationStatus;
use hdi::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// What kind of forest/worksite item was observed.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ForestObservationKind {
    TreeCandidate,
    TerrainPatch,
    LogDeck,
    TrailSegment,
    Obstacle,
    Other,
}

/// Primary sensor or measurement channel responsible for the observation.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ForestSensorModality {
    Rgb,
    StereoDepth,
    Lidar,
    Gnss,
    Imu,
    WheelOdometry,
    MoistureProbe,
    Other,
}

/// Distinguish simulated artifacts from observations captured by physical sensors.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ForestObservationEvidenceClass {
    SyntheticSimulation,
    HardwareInTheLoop,
    FieldCaptured,
}

/// Review lifecycle for inventory use, deliberately separate from wood grading.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ForestObservationReview {
    Captured,
    PendingReview,
    ReviewedForInventory,
    Rejected,
}

/// A provenance envelope around a robot-captured forestry observation.
///
/// Coordinates are integer millimetres in the explicitly named local/reference
/// frame; they are not assumed to be latitude/longitude. Digest fields identify
/// the mission receipt, map snapshot, and retained raw sensor artifact. Related
/// measurement IDs resolve separately to WoodMeasurementEvidence records.
/// This envelope does not, by itself, establish that a detector output is true.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ForestRobotObservation {
    pub observation_id: String,
    pub mission_id: String,
    pub robot_id: String,
    pub platform_class: String,
    pub kind: ForestObservationKind,
    pub evidence_class: ForestObservationEvidenceClass,
    pub sensor_modality: ForestSensorModality,
    pub sensor_id: Option<String>,
    pub calibration_status: WoodCalibrationStatus,
    pub calibration_record_ref: Option<String>,
    pub captured_at: Timestamp,
    pub coordinate_frame_ref: String,
    pub x_mm: i64,
    pub y_mm: i64,
    pub z_mm: i64,
    /// Must be explicit and non-zero; zero would imply perfect localization.
    pub position_uncertainty_mm: u32,
    pub mission_receipt_sha256: String,
    pub map_sha256: String,
    /// Sorted, unique opaque IDs of Sol Atlas RoboticsTerrainEvidenceEnvelope records.
    /// The consumer must resolve each reference and compare its map snapshot and
    /// coordinate frame; this string is not an authenticity or hazard-clearance proof.
    pub spatial_terrain_evidence_refs: Vec<String>,
    pub raw_artifact_sha256: String,
    /// References to separately validated WoodMeasurementEvidence records.
    pub related_measurement_ids: Vec<String>,
    pub wood_batch_id: Option<String>,
    pub site_permission_ref: String,
    pub source_refs: Vec<String>,
    pub uncertainty_note: Option<String>,
    pub review_status: ForestObservationReview,
    pub reviewer_ref: Option<String>,
    pub rejection_reason: Option<String>,
}

fn nonblank(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

impl ForestRobotObservation {
    /// Validate envelope structure, provenance references and review transitions.
    ///
    /// This does not verify signatures, resolve referenced objects, validate map
    /// geometry, independently confirm detections, or grant machine authority.
    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();

        for (field, value) in [
            ("observation_id", self.observation_id.as_str()),
            ("mission_id", self.mission_id.as_str()),
            ("robot_id", self.robot_id.as_str()),
            ("platform_class", self.platform_class.as_str()),
            ("coordinate_frame_ref", self.coordinate_frame_ref.as_str()),
            ("site_permission_ref", self.site_permission_ref.as_str()),
        ] {
            if !nonblank(value) {
                errors.push(format!("{field} is required"));
            }
            if value.len() > 200 {
                errors.push(format!("{field} must be <= 200 characters"));
            }
        }

        for (field, digest) in [
            ("mission_receipt_sha256", self.mission_receipt_sha256.as_str()),
            ("map_sha256", self.map_sha256.as_str()),
            ("raw_artifact_sha256", self.raw_artifact_sha256.as_str()),
        ] {
            if !valid_sha256(digest) {
                errors.push(format!("{field} must be 64 lowercase hexadecimal characters"));
            }
        }

        if self.position_uncertainty_mm == 0 {
            errors.push("position_uncertainty_mm must be > 0".to_owned());
        }

        if self.spatial_terrain_evidence_refs.len() > 128 {
            errors.push("spatial_terrain_evidence_refs must contain <= 128 items".to_owned());
        }
        let mut terrain_refs = BTreeSet::new();
        let mut previous_terrain_ref: Option<&str> = None;
        for terrain_ref in &self.spatial_terrain_evidence_refs {
            if !nonblank(terrain_ref) {
                errors.push("spatial terrain evidence refs must not be blank".to_owned());
            }
            if terrain_ref.len() > 500 {
                errors.push("spatial terrain evidence refs must be <= 500 characters".to_owned());
            }
            if !terrain_refs.insert(terrain_ref) {
                errors.push(format!("duplicate spatial terrain evidence ref: {terrain_ref}"));
            }
            if previous_terrain_ref.is_some_and(|previous| previous >= terrain_ref.as_str()) {
                errors.push("spatial_terrain_evidence_refs must be sorted and unique".to_owned());
            }
            previous_terrain_ref = Some(terrain_ref);
        }
        if matches!(
            self.kind,
            ForestObservationKind::TerrainPatch
                | ForestObservationKind::TrailSegment
                | ForestObservationKind::Obstacle
        ) && self.spatial_terrain_evidence_refs.is_empty()
        {
            errors.push(
                "terrain, trail, and obstacle observations require a Sol Atlas terrain evidence reference".to_owned(),
            );
        }
        if self.sensor_id.as_ref().is_some_and(|value| !nonblank(value)) {
            errors.push("sensor_id must be omitted rather than blank".to_owned());
        }
        if self
            .calibration_record_ref
            .as_ref()
            .is_some_and(|value| !nonblank(value))
        {
            errors.push("calibration_record_ref must be omitted rather than blank".to_owned());
        }
        if self.calibration_status == WoodCalibrationStatus::Current
            && (self.sensor_id.is_none() || self.calibration_record_ref.is_none())
        {
            errors.push(
                "current calibration requires sensor_id and calibration_record_ref".to_owned(),
            );
        }
        if matches!(
            self.calibration_status,
            WoodCalibrationStatus::Expired | WoodCalibrationStatus::Unknown
        ) && self
            .uncertainty_note
            .as_deref()
            .is_none_or(|note| !nonblank(note))
        {
            errors.push(
                "expired or unknown calibration requires an uncertainty_note".to_owned(),
            );
        }

        if self.related_measurement_ids.len() > 128 {
            errors.push("related_measurement_ids must contain <= 128 items".to_owned());
        }
        let mut measurement_ids = BTreeSet::new();
        for measurement_id in &self.related_measurement_ids {
            if !nonblank(measurement_id) {
                errors.push("related measurement IDs must not be blank".to_owned());
            }
            if measurement_id.len() > 200 {
                errors.push("related measurement IDs must be <= 200 characters".to_owned());
            }
            if !measurement_ids.insert(measurement_id) {
                errors.push(format!("duplicate related measurement ID: {measurement_id}"));
            }
        }

        if self.source_refs.is_empty() {
            errors.push("at least one source reference is required".to_owned());
        }
        if self.source_refs.len() > 128 {
            errors.push("source_refs must contain <= 128 items".to_owned());
        }
        let mut sources = BTreeSet::new();
        for source in &self.source_refs {
            if !nonblank(source) {
                errors.push("source references must not be blank".to_owned());
            }
            if source.len() > 500 {
                errors.push("source references must be <= 500 characters".to_owned());
            }
            if !sources.insert(source) {
                errors.push(format!("duplicate source reference: {source}"));
            }
        }

        if self
            .wood_batch_id
            .as_ref()
            .is_some_and(|value| !nonblank(value))
        {
            errors.push("wood_batch_id must be omitted rather than blank".to_owned());
        }
        if self
            .uncertainty_note
            .as_deref()
            .is_some_and(|note| !nonblank(note))
        {
            errors.push("uncertainty_note must be omitted rather than blank".to_owned());
        }

        if self.evidence_class != ForestObservationEvidenceClass::FieldCaptured
            && self.review_status == ForestObservationReview::ReviewedForInventory
        {
            errors.push(
                "only field-captured observations may be ReviewedForInventory".to_owned(),
            );
        }

        match self.review_status {
            ForestObservationReview::ReviewedForInventory => {
                if self.reviewer_ref.as_deref().is_none_or(|value| !nonblank(value)) {
                    errors.push(
                        "ReviewedForInventory requires a reviewer_ref".to_owned(),
                    );
                }
                if self.rejection_reason.is_some() {
                    errors.push("reviewed observations must not have a rejection_reason".to_owned());
                }
            }
            ForestObservationReview::Rejected => {
                if self.reviewer_ref.as_deref().is_none_or(|value| !nonblank(value)) {
                    errors.push("Rejected requires a reviewer_ref".to_owned());
                }
                if self
                    .rejection_reason
                    .as_deref()
                    .is_none_or(|value| !nonblank(value))
                {
                    errors.push("Rejected requires a rejection_reason".to_owned());
                }
            }
            ForestObservationReview::Captured | ForestObservationReview::PendingReview => {
                if self.reviewer_ref.is_some() {
                    errors.push(
                        "unreviewed observations must not carry a reviewer_ref".to_owned(),
                    );
                }
                if self.rejection_reason.is_some() {
                    errors.push(
                        "unreviewed observations must not carry a rejection_reason".to_owned(),
                    );
                }
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation() -> ForestRobotObservation {
        ForestRobotObservation {
            observation_id: "forest-observation-001".to_owned(),
            mission_id: "forest-mission-001".to_owned(),
            robot_id: "scout-001".to_owned(),
            platform_class: "forest_scout_v0".to_owned(),
            kind: ForestObservationKind::TreeCandidate,
            evidence_class: ForestObservationEvidenceClass::FieldCaptured,
            sensor_modality: ForestSensorModality::Lidar,
            sensor_id: Some("lidar-001".to_owned()),
            calibration_status: WoodCalibrationStatus::Current,
            calibration_record_ref: Some("calibration-record-001".to_owned()),
            captured_at: Timestamp::from_micros(1_800_000_000_000_000),
            coordinate_frame_ref: "site-local-frame-v1".to_owned(),
            x_mm: 125_000,
            y_mm: -18_500,
            z_mm: 4_200,
            position_uncertainty_mm: 35,
            mission_receipt_sha256: "a".repeat(64),
            map_sha256: "b".repeat(64),
            spatial_terrain_evidence_refs: vec![],
            raw_artifact_sha256: "c".repeat(64),
            related_measurement_ids: vec![
                "measurement-tree-001-dbh".to_owned(),
                "measurement-tree-001-height".to_owned(),
            ],
            wood_batch_id: None,
            site_permission_ref: "site-permission-001".to_owned(),
            source_refs: vec![
                "raw-artifact-store://artifact-001".to_owned(),
                "mission-receipt://receipt-001".to_owned(),
            ],
            uncertainty_note: None,
            review_status: ForestObservationReview::Captured,
            reviewer_ref: None,
            rejection_reason: None,
        }
    }

    #[test]
    fn valid_observation_round_trips_with_traceable_digests() {
        let record = observation();
        assert!(record.validate().is_ok());
        let encoded = serde_json::to_string(&record).expect("observation serializes");
        let decoded: ForestRobotObservation =
            serde_json::from_str(&encoded).expect("observation deserializes");
        assert_eq!(decoded, record);
    }

    #[test]
    fn terrain_and_obstacle_observations_require_sol_atlas_evidence() {
        let mut record = observation();
        record.kind = ForestObservationKind::TerrainPatch;
        let errors = record.validate().expect_err("terrain observation needs spatial evidence");
        assert!(errors.iter().any(|error| error.contains("Sol Atlas terrain evidence reference")));
        record.spatial_terrain_evidence_refs = vec!["sol-atlas-terrain-evidence-001".to_owned()];
        assert!(record.validate().is_ok());
    }

    #[test]
    fn spatial_terrain_evidence_refs_must_be_canonical_and_unique() {
        let mut record = observation();
        record.spatial_terrain_evidence_refs = vec![
            "terrain-ref-z".to_owned(),
            "terrain-ref-a".to_owned(),
        ];
        assert!(record
            .validate()
            .expect_err("unsorted references must fail")
            .iter()
            .any(|error| error.contains("must be sorted and unique")));
        record.spatial_terrain_evidence_refs = vec![
            "terrain-ref-a".to_owned(),
            "terrain-ref-a".to_owned(),
        ];
        assert!(record
            .validate()
            .expect_err("duplicate references must fail")
            .iter()
            .any(|error| error.contains("duplicate spatial terrain evidence ref")));
    }

    #[test]
    fn tree_candidate_may_omit_terrain_handoff() {
        let record = observation();
        assert_eq!(record.kind, ForestObservationKind::TreeCandidate);
        assert!(record.spatial_terrain_evidence_refs.is_empty());
        assert!(record.validate().is_ok());
    }

    #[test]
    fn malformed_artifact_or_map_digest_is_rejected() {
        let mut record = observation();
        record.raw_artifact_sha256 = "not-a-sha".to_owned();
        record.map_sha256 = "G".repeat(64);
        let errors = record.validate().expect_err("malformed digest should fail");
        assert!(errors.iter().any(|error| error.contains("raw_artifact_sha256")));
        assert!(errors.iter().any(|error| error.contains("map_sha256")));
    }

    #[test]
    fn current_calibration_requires_sensor_identity_and_record() {
        let mut record = observation();
        record.sensor_id = None;
        record.calibration_record_ref = None;
        assert!(
            record
                .validate()
                .expect_err("current calibration needs provenance")
                .iter()
                .any(|error| error.contains("current calibration"))
        );
    }

    #[test]
    fn expired_or_unknown_calibration_requires_uncertainty_note() {
        let mut record = observation();
        record.calibration_status = WoodCalibrationStatus::Unknown;
        record.uncertainty_note = None;
        assert!(
            record
                .validate()
                .expect_err("unknown calibration needs disclosure")
                .iter()
                .any(|error| error.contains("uncertainty_note"))
        );
    }

    #[test]
    fn zero_pose_uncertainty_is_rejected() {
        let mut record = observation();
        record.position_uncertainty_mm = 0;
        assert!(
            record
                .validate()
                .expect_err("zero implies unjustified perfect localization")
                .iter()
                .any(|error| error.contains("position_uncertainty_mm"))
        );
    }

    #[test]
    fn duplicate_related_measurements_are_rejected() {
        let mut record = observation();
        let existing = record.related_measurement_ids[0].clone();
        record.related_measurement_ids.push(existing);
        assert!(
            record
                .validate()
                .expect_err("duplicates must remain visible")
                .iter()
                .any(|error| error.contains("duplicate related measurement ID"))
        );
    }

    #[test]
    fn reviewed_inventory_observation_requires_reviewer_reference() {
        let mut record = observation();
        record.review_status = ForestObservationReview::ReviewedForInventory;
        assert!(
            record
                .validate()
                .expect_err("reviewed state needs actor reference")
                .iter()
                .any(|error| error.contains("reviewer_ref"))
        );
        record.reviewer_ref = Some("reviewer-001".to_owned());
        assert!(record.validate().is_ok());
    }

    #[test]
    fn synthetic_simulation_cannot_be_marked_reviewed_for_inventory() {
        let mut record = observation();
        record.evidence_class = ForestObservationEvidenceClass::SyntheticSimulation;
        record.review_status = ForestObservationReview::ReviewedForInventory;
        record.reviewer_ref = Some("reviewer-001".to_owned());
        assert!(
            record
                .validate()
                .expect_err("synthetic evidence cannot stand in for field capture")
                .iter()
                .any(|error| error.contains("only field-captured"))
        );
    }

    #[test]
    fn rejected_observation_requires_review_actor_and_reason() {
        let mut record = observation();
        record.review_status = ForestObservationReview::Rejected;
        assert!(
            record
                .validate()
                .expect_err("rejection needs review provenance")
                .iter()
                .any(|error| error.contains("rejection_reason"))
        );
        record.reviewer_ref = Some("reviewer-001".to_owned());
        record.rejection_reason = Some("tree candidate was a duplicate".to_owned());
        assert!(record.validate().is_ok());
    }

    #[test]
    fn unreviewed_observations_cannot_smuggle_review_fields() {
        let mut record = observation();
        record.reviewer_ref = Some("reviewer-001".to_owned());
        assert!(
            record
                .validate()
                .expect_err("unreviewed record cannot claim review provenance")
                .iter()
                .any(|error| error.contains("unreviewed observations"))
        );
    }

    #[test]
    fn unknown_fields_are_rejected_during_deserialization() {
        let mut value = serde_json::to_value(observation()).expect("record serializes");
        value
            .as_object_mut()
            .expect("object")
            .insert("timber_grade_certified".to_owned(), serde_json::json!(true));
        assert!(
            serde_json::from_value::<ForestRobotObservation>(value).is_err(),
            "observation envelope must not silently accept invented certification fields"
        );
    }
}
