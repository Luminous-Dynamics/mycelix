//! Deterministic, prototype-only compiler for a bounded modular desk family.
//!
//! This crate turns explicitly bounded dimensions/material/finish parameters into a
//! stable BOM-like cut-part list and candidate process steps. It is NOT CAD, a
//! nesting optimizer, structural analysis, product qualification, or a production
//! release authority. Every emitted packet is marked release-ineligible.

use serde::{Deserialize, Serialize};
use std::fmt;
use std::io::Read;

pub const INPUT_SCHEMA_VERSION: &str = "furniture-desk-input-v1";
pub const PACKET_SCHEMA_VERSION: &str = "furniture-compiler-packet-v2";
pub const SERIALIZATION_PROFILE: &str = "canonical-binary-v2";
pub const MAX_INPUT_BYTES: usize = 64 * 1024;

const FAMILY_ID: &str = "modular_desk_prototype";
const FAMILY_REVISION: u32 = 1;
const FAMILY_RULESET_VERSION: &str = "modular-desk-rules-v1";
const GEOMETRY_RULESET_ID: &str = "rectangular-panel-envelopes-v1";
const PANEL_THICKNESS_MM: u32 = 18;
const REAR_APRON_HEIGHT_MM: u32 = 100;
const MIN_WIDTH_MM: u32 = 600;
const MAX_WIDTH_MM: u32 = 1600;
const MIN_DEPTH_MM: u32 = 400;
const MAX_DEPTH_MM: u32 = 900;
const MIN_HEIGHT_MM: u32 = 650;
const MAX_HEIGHT_MM: u32 = 850;

/// Restrict input to known fields and enums. Dimensions use integer millimetres.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeskConfiguration {
    pub schema_version: String,
    pub width_mm: u32,
    pub depth_mm: u32,
    pub height_mm: u32,
    pub panel_thickness_mm: u32,
    pub material_id: MaterialId,
    pub finish_id: FinishId,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MaterialId {
    BirchPlywood,
    MelamineMdf,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FinishId {
    ClearCoat,
    WhiteMelamine,
    BlackMelamine,
}

/// Single source of truth for validation and the design-digest rules encoding.
const ALLOWED_MATERIAL_FINISHES: [(MaterialId, FinishId); 3] = [
    (MaterialId::BirchPlywood, FinishId::ClearCoat),
    (MaterialId::MelamineMdf, FinishId::WhiteMelamine),
    (MaterialId::MelamineMdf, FinishId::BlackMelamine),
];

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CutPart {
    /// Stable semantic identifier, not a supplier SKU.
    pub part_id: String,
    pub quantity: u32,
    /// Local part axes in millimetres; length is the intended grain direction.
    pub length_mm: u32,
    pub width_mm: u32,
    pub thickness_mm: u32,
    pub material_id: MaterialId,
    pub finish_id: FinishId,
    pub grain_axis: GrainAxis,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GrainAxis {
    Length,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CandidateProcessStep {
    pub sequence: u32,
    pub operation_code: String,
    /// This is a proposed process step, not a partner-approved routing.
    pub requires_partner_mapping: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReleaseGate {
    pub configuration_bounds_passed: bool,
    pub release_eligible: bool,
    pub blockers: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PacketCore {
    pub packet_schema_version: String,
    pub serialization_profile: String,
    pub family_id: String,
    pub family_revision: u32,
    pub family_ruleset_version: String,
    pub geometry_ruleset_id: String,
    pub configuration: DeskConfiguration,
    pub cut_parts: Vec<CutPart>,
    pub candidate_process_steps: Vec<CandidateProcessStep>,
    pub release_gate: ReleaseGate,
    pub design_digest: String,
    pub cut_list_digest: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CompiledPacket {
    pub core: PacketCore,
    pub packet_digest: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IssueCode {
    WrongSchemaVersion,
    WidthOutOfRange,
    DepthOutOfRange,
    HeightOutOfRange,
    UnsupportedThickness,
    IncompatibleFinish,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidationIssue {
    pub code: IssueCode,
    pub message: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CompileError {
    InputTooLarge,
    InvalidJson(String),
    InvalidConfiguration(Vec<ValidationIssue>),
}

impl fmt::Display for CompileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InputTooLarge => write!(f, "input exceeds {MAX_INPUT_BYTES} bytes"),
            Self::InvalidJson(message) => write!(f, "invalid input JSON: {message}"),
            Self::InvalidConfiguration(issues) => {
                write!(f, "configuration rejected ({} issue(s))", issues.len())?;
                for issue in issues {
                    write!(f, "; {:?}: {}", issue.code, issue.message)?;
                }
                Ok(())
            }
        }
    }
}

impl std::error::Error for CompileError {}

pub fn compile_json(input: &str) -> Result<CompiledPacket, CompileError> {
    if input.len() > MAX_INPUT_BYTES {
        return Err(CompileError::InputTooLarge);
    }
    let config: DeskConfiguration = serde_json::from_str(input)
        .map_err(|error| CompileError::InvalidJson(error.to_string()))?;
    compile(config)
}

pub fn compile(config: DeskConfiguration) -> Result<CompiledPacket, CompileError> {
    let issues = validate(&config);
    if !issues.is_empty() {
        return Err(CompileError::InvalidConfiguration(issues));
    }

    // This output describes panel envelopes only. It intentionally does not invent
    // joints, drilling coordinates, tolerances, a hardware bill, or load ratings.
    let side_height_mm = config.height_mm - config.panel_thickness_mm;
    let apron_length_mm = config.width_mm - (2 * config.panel_thickness_mm);
    let mut cut_parts = vec![
        CutPart {
            part_id: "desk_top".to_owned(),
            quantity: 1,
            length_mm: config.width_mm,
            width_mm: config.depth_mm,
            thickness_mm: config.panel_thickness_mm,
            material_id: config.material_id,
            finish_id: config.finish_id,
            grain_axis: GrainAxis::Length,
        },
        CutPart {
            part_id: "side_panel".to_owned(),
            quantity: 2,
            length_mm: side_height_mm,
            width_mm: config.depth_mm,
            thickness_mm: config.panel_thickness_mm,
            material_id: config.material_id,
            finish_id: config.finish_id,
            grain_axis: GrainAxis::Length,
        },
        CutPart {
            part_id: "rear_apron".to_owned(),
            quantity: 1,
            length_mm: apron_length_mm,
            width_mm: REAR_APRON_HEIGHT_MM,
            thickness_mm: config.panel_thickness_mm,
            material_id: config.material_id,
            finish_id: config.finish_id,
            grain_axis: GrainAxis::Length,
        },
    ];
    cut_parts.sort_by(|left, right| left.part_id.cmp(&right.part_id));

    let candidate_process_steps = vec![
        CandidateProcessStep {
            sequence: 1,
            operation_code: "panel_cutting".to_owned(),
            requires_partner_mapping: true,
        },
        CandidateProcessStep {
            sequence: 2,
            operation_code: "edge_finishing".to_owned(),
            requires_partner_mapping: true,
        },
        CandidateProcessStep {
            sequence: 3,
            operation_code: "joinery_and_hardware_definition".to_owned(),
            requires_partner_mapping: true,
        },
        CandidateProcessStep {
            sequence: 4,
            operation_code: "inspection_and_packaging".to_owned(),
            requires_partner_mapping: true,
        },
    ];

    let design_digest = digest_design_identity(
        FAMILY_ID,
        FAMILY_REVISION,
        FAMILY_RULESET_VERSION,
        GEOMETRY_RULESET_ID,
        &config,
    );
    let cut_list_digest = digest_cut_list(&cut_parts);

    let core = PacketCore {
        packet_schema_version: PACKET_SCHEMA_VERSION.to_owned(),
        serialization_profile: SERIALIZATION_PROFILE.to_owned(),
        family_id: FAMILY_ID.to_owned(),
        family_revision: FAMILY_REVISION,
        family_ruleset_version: FAMILY_RULESET_VERSION.to_owned(),
        geometry_ruleset_id: GEOMETRY_RULESET_ID.to_owned(),
        configuration: config,
        cut_parts,
        candidate_process_steps,
        release_gate: ReleaseGate {
            configuration_bounds_passed: true,
            release_eligible: false,
            blockers: vec![
                "joinery_and_fastener_schedule_not_defined".to_owned(),
                "structural_stability_and_load_qualification_not_provided".to_owned(),
                "material_finish_and_edge_process_not_partner_qualified".to_owned(),
                "supplier_and_machine_profile_not_bound".to_owned(),
                "qualified_nesting_output_not_generated".to_owned(),
                "inspection_and_acceptance_criteria_not_defined".to_owned(),
                "packaging_and_delivery_plan_not_bound".to_owned(),
            ],
        },
        design_digest,
        cut_list_digest,
    };
    let packet_digest = digest_packet_core(&core);

    Ok(CompiledPacket { core, packet_digest })
}

/// Verify all packet digest bindings. This verifies byte-level integrity only;
/// it does not validate physical safety, geometry semantics, or manufactureability.
pub fn verify_integrity(packet: &CompiledPacket) -> Result<bool, CompileError> {
    // Recompile from the bound configuration and compare every generated field.
    // Recomputing only the digests would let a caller change the release gate and
    // recompute the outer digest. This check therefore also enforces the prototype's
    // permanently blocked gate and canonical candidate parts/process list.
    let expected = match compile(packet.core.configuration.clone()) {
        Ok(expected) => expected,
        Err(CompileError::InvalidConfiguration(_)) => return Ok(false),
        Err(error) => return Err(error),
    };
    Ok(packet == &expected)
}

/// Canonical binary encoding v1:
/// - strings/byte sequences: unsigned 64-bit big-endian byte length, then UTF-8 bytes;
/// - list lengths: unsigned 64-bit big-endian element count;
/// - integers: unsigned 32-bit big-endian;
/// - booleans: one byte (0 or 1);
/// - enums: explicit one-byte discriminants in the match functions below;
/// - aggregate fields: the exact order written by each encode_* function.
/// This is a protocol of its own, not RFC 8785 JSON. Changing order, units,
/// discriminants, or numeric representation requires a new profile version.
fn put_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn put_len(out: &mut Vec<u8>, value: usize) {
    out.extend_from_slice(&(value as u64).to_be_bytes());
}

fn put_str(out: &mut Vec<u8>, value: &str) {
    put_len(out, value.len());
    out.extend_from_slice(value.as_bytes());
}

fn encode_material(out: &mut Vec<u8>, value: MaterialId) {
    out.push(match value {
        MaterialId::BirchPlywood => 1,
        MaterialId::MelamineMdf => 2,
    });
}

fn encode_finish(out: &mut Vec<u8>, value: FinishId) {
    out.push(match value {
        FinishId::ClearCoat => 1,
        FinishId::WhiteMelamine => 2,
        FinishId::BlackMelamine => 3,
    });
}

fn encode_configuration(out: &mut Vec<u8>, value: &DeskConfiguration) {
    put_str(out, &value.schema_version);
    put_u32(out, value.width_mm);
    put_u32(out, value.depth_mm);
    put_u32(out, value.height_mm);
    put_u32(out, value.panel_thickness_mm);
    encode_material(out, value.material_id);
    encode_finish(out, value.finish_id);
}

/// Bind the fixed constraints and allowed finish matrix into the design identity.
/// If generation semantics change beyond these explicit fields, increment
/// FAMILY_RULESET_VERSION and/or GEOMETRY_RULESET_ID before releasing the change.
fn encode_family_rules(out: &mut Vec<u8>) {
    put_u32(out, MIN_WIDTH_MM);
    put_u32(out, MAX_WIDTH_MM);
    put_u32(out, MIN_DEPTH_MM);
    put_u32(out, MAX_DEPTH_MM);
    put_u32(out, MIN_HEIGHT_MM);
    put_u32(out, MAX_HEIGHT_MM);
    put_u32(out, PANEL_THICKNESS_MM);
    put_u32(out, REAR_APRON_HEIGHT_MM);

    // Serialize the exact allowlist used by validation, in its declared order.
    put_len(out, ALLOWED_MATERIAL_FINISHES.len());
    for (material, finish) in ALLOWED_MATERIAL_FINISHES {
        encode_material(out, material);
        encode_finish(out, finish);
    }
}

fn encode_cut_parts(out: &mut Vec<u8>, parts: &[CutPart]) {
    put_len(out, parts.len());
    for part in parts {
        put_str(out, &part.part_id);
        put_u32(out, part.quantity);
        put_u32(out, part.length_mm);
        put_u32(out, part.width_mm);
        put_u32(out, part.thickness_mm);
        encode_material(out, part.material_id);
        encode_finish(out, part.finish_id);
        out.push(match part.grain_axis {
            GrainAxis::Length => 1,
        });
    }
}

fn encode_process_steps(out: &mut Vec<u8>, steps: &[CandidateProcessStep]) {
    put_len(out, steps.len());
    for step in steps {
        put_u32(out, step.sequence);
        put_str(out, &step.operation_code);
        out.push(u8::from(step.requires_partner_mapping));
    }
}

fn encode_release_gate(out: &mut Vec<u8>, gate: &ReleaseGate) {
    out.push(u8::from(gate.configuration_bounds_passed));
    out.push(u8::from(gate.release_eligible));
    put_len(out, gate.blockers.len());
    for blocker in &gate.blockers {
        put_str(out, blocker);
    }
}

fn hash_canonical(domain: &str, bytes: &[u8]) -> String {
    let mut hasher = blake3::Hasher::new_derive_key("Luminous Dynamics Furniture Compiler v1");
    hasher.update(domain.as_bytes());
    hasher.update(&[0]);
    hasher.update(&(bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
    hasher.finalize().to_hex().to_string()
}

fn digest_design_identity(
    family_id: &str,
    family_revision: u32,
    ruleset_version: &str,
    geometry_ruleset_id: &str,
    config: &DeskConfiguration,
) -> String {
    let mut bytes = Vec::new();
    put_str(&mut bytes, family_id);
    put_u32(&mut bytes, family_revision);
    put_str(&mut bytes, ruleset_version);
    put_str(&mut bytes, geometry_ruleset_id);
    encode_family_rules(&mut bytes);
    encode_configuration(&mut bytes, config);
    hash_canonical("design-spec-v2", &bytes)
}

fn digest_cut_list(parts: &[CutPart]) -> String {
    let mut bytes = Vec::new();
    encode_cut_parts(&mut bytes, parts);
    hash_canonical("cut-list-v1", &bytes)
}

fn digest_packet_core(core: &PacketCore) -> String {
    let mut bytes = Vec::new();
    put_str(&mut bytes, &core.packet_schema_version);
    put_str(&mut bytes, &core.serialization_profile);
    put_str(&mut bytes, &core.family_id);
    put_u32(&mut bytes, core.family_revision);
    put_str(&mut bytes, &core.family_ruleset_version);
    put_str(&mut bytes, &core.geometry_ruleset_id);
    encode_configuration(&mut bytes, &core.configuration);
    encode_cut_parts(&mut bytes, &core.cut_parts);
    encode_process_steps(&mut bytes, &core.candidate_process_steps);
    encode_release_gate(&mut bytes, &core.release_gate);
    put_str(&mut bytes, &core.design_digest);
    put_str(&mut bytes, &core.cut_list_digest);
    hash_canonical("manufacturing-packet-v2", &bytes)
}

pub fn validate(config: &DeskConfiguration) -> Vec<ValidationIssue> {
    let mut issues = Vec::new();

    if config.schema_version != INPUT_SCHEMA_VERSION {
        issues.push(ValidationIssue {
            code: IssueCode::WrongSchemaVersion,
            message: format!("schema_version must be {INPUT_SCHEMA_VERSION}"),
        });
    }
    if !(MIN_WIDTH_MM..=MAX_WIDTH_MM).contains(&config.width_mm) {
        issues.push(ValidationIssue {
            code: IssueCode::WidthOutOfRange,
            message: format!("width_mm must be between {MIN_WIDTH_MM} and {MAX_WIDTH_MM} inclusive"),
        });
    }
    if !(MIN_DEPTH_MM..=MAX_DEPTH_MM).contains(&config.depth_mm) {
        issues.push(ValidationIssue {
            code: IssueCode::DepthOutOfRange,
            message: format!("depth_mm must be between {MIN_DEPTH_MM} and {MAX_DEPTH_MM} inclusive"),
        });
    }
    if !(MIN_HEIGHT_MM..=MAX_HEIGHT_MM).contains(&config.height_mm) {
        issues.push(ValidationIssue {
            code: IssueCode::HeightOutOfRange,
            message: format!("height_mm must be between {MIN_HEIGHT_MM} and {MAX_HEIGHT_MM} inclusive"),
        });
    }
    if config.panel_thickness_mm != PANEL_THICKNESS_MM {
        issues.push(ValidationIssue {
            code: IssueCode::UnsupportedThickness,
            message: format!(
                "panel_thickness_mm must be exactly {PANEL_THICKNESS_MM} for prototype family revision {FAMILY_REVISION}"
            ),
        });
    }
    let finish_valid =
        ALLOWED_MATERIAL_FINISHES.contains(&(config.material_id, config.finish_id));
    if !finish_valid {
        issues.push(ValidationIssue {
            code: IssueCode::IncompatibleFinish,
            message: "finish_id is not in the approved prototype material/finish matrix".to_owned(),
        });
    }

    issues
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_config() -> DeskConfiguration {
        DeskConfiguration {
            schema_version: INPUT_SCHEMA_VERSION.to_owned(),
            width_mm: 1200,
            depth_mm: 600,
            height_mm: 750,
            panel_thickness_mm: 18,
            material_id: MaterialId::BirchPlywood,
            finish_id: FinishId::ClearCoat,
        }
    }

    #[test]
    fn canonical_encoding_uses_explicit_big_endian_integer_bytes() {
        let mut bytes = Vec::new();
        put_u32(&mut bytes, 0x0102_0304);
        assert_eq!(bytes, vec![1, 2, 3, 4]);
    }

    #[test]
    fn canonical_configuration_encoding_has_a_fixed_golden_vector() {
        let config = valid_config();
        let mut actual = Vec::new();
        encode_configuration(&mut actual, &config);

        let mut expected = vec![0, 0, 0, 0, 0, 0, 0, 23];
        expected.extend_from_slice(b"furniture-desk-input-v1");
        expected.extend_from_slice(&1200_u32.to_be_bytes());
        expected.extend_from_slice(&600_u32.to_be_bytes());
        expected.extend_from_slice(&750_u32.to_be_bytes());
        expected.extend_from_slice(&18_u32.to_be_bytes());
        expected.extend_from_slice(&[1, 1]); // BirchPlywood, ClearCoat
        assert_eq!(actual, expected);
    }

    #[test]
    fn design_identity_changes_when_ruleset_identity_changes() {
        let config = valid_config();
        let baseline = digest_design_identity(
            FAMILY_ID,
            FAMILY_REVISION,
            FAMILY_RULESET_VERSION,
            GEOMETRY_RULESET_ID,
            &config,
        );
        let changed_ruleset = digest_design_identity(
            FAMILY_ID,
            FAMILY_REVISION,
            "modular-desk-rules-next",
            GEOMETRY_RULESET_ID,
            &config,
        );
        let changed_geometry = digest_design_identity(
            FAMILY_ID,
            FAMILY_REVISION,
            FAMILY_RULESET_VERSION,
            "jointed-panel-geometry-next",
            &config,
        );
        assert_ne!(baseline, changed_ruleset);
        assert_ne!(baseline, changed_geometry);
    }

    #[test]
    fn same_frozen_input_produces_same_packet_and_digests() {
        let first = compile(valid_config()).expect("valid bounded configuration");
        let second = compile(valid_config()).expect("same valid configuration");
        assert_eq!(first, second);
        assert!(verify_integrity(&first).expect("verify packet"));
    }

    #[test]
    fn dimensions_change_design_cut_list_and_packet_digests() {
        let first = compile(valid_config()).expect("valid bounded configuration");
        let mut altered = valid_config();
        altered.width_mm += 1;
        let second = compile(altered).expect("still in bounds");
        assert_ne!(first.core.design_digest, second.core.design_digest);
        assert_ne!(first.core.cut_list_digest, second.core.cut_list_digest);
        assert_ne!(first.packet_digest, second.packet_digest);
    }

    #[test]
    fn compiler_emits_expected_panel_envelopes_and_stable_order() {
        let packet = compile(valid_config()).expect("valid bounded configuration");
        let parts = &packet.core.cut_parts;
        assert_eq!(
            parts.iter().map(|part| part.part_id.as_str()).collect::<Vec<_>>(),
            vec!["desk_top", "rear_apron", "side_panel"]
        );
        let side = parts.iter().find(|part| part.part_id == "side_panel").unwrap();
        assert_eq!(side.quantity, 2);
        assert_eq!(side.length_mm, 732);
        assert_eq!(side.width_mm, 600);
        let apron = parts.iter().find(|part| part.part_id == "rear_apron").unwrap();
        assert_eq!(apron.length_mm, 1164);
        assert_eq!(apron.width_mm, REAR_APRON_HEIGHT_MM);
    }

    #[test]
    fn all_packets_are_explicitly_not_release_eligible() {
        let packet = compile(valid_config()).expect("valid bounded configuration");
        assert!(packet.core.release_gate.configuration_bounds_passed);
        assert!(!packet.core.release_gate.release_eligible);
        assert!(!packet.core.release_gate.blockers.is_empty());
        assert!(
            packet
                .core
                .candidate_process_steps
                .iter()
                .all(|step| step.requires_partner_mapping)
        );
    }

    #[test]
    fn invalid_dimensions_and_material_finish_mismatches_fail_closed() {
        let mut bad_dimensions = valid_config();
        bad_dimensions.width_mm = 599;
        let errors = compile(bad_dimensions).expect_err("too narrow");
        assert!(matches!(errors, CompileError::InvalidConfiguration(ref issues)
            if issues.iter().any(|issue| issue.code == IssueCode::WidthOutOfRange)));

        let mut bad_finish = valid_config();
        bad_finish.finish_id = FinishId::WhiteMelamine;
        let errors = compile(bad_finish).expect_err("incompatible material/finish");
        assert!(matches!(errors, CompileError::InvalidConfiguration(ref issues)
            if issues.iter().any(|issue| issue.code == IssueCode::IncompatibleFinish)));

        let mut bad_thickness = valid_config();
        bad_thickness.panel_thickness_mm = 24;
        assert!(matches!(
            compile(bad_thickness),
            Err(CompileError::InvalidConfiguration(_))
        ));
    }

    #[test]
    fn every_dimension_range_rejects_both_sides_of_its_bounds() {
        let cases = [
            (599, 600, 750, IssueCode::WidthOutOfRange),
            (1601, 600, 750, IssueCode::WidthOutOfRange),
            (1200, 399, 750, IssueCode::DepthOutOfRange),
            (1200, 901, 750, IssueCode::DepthOutOfRange),
            (1200, 600, 649, IssueCode::HeightOutOfRange),
            (1200, 600, 851, IssueCode::HeightOutOfRange),
        ];
        for (width_mm, depth_mm, height_mm, expected_code) in cases {
            let mut config = valid_config();
            config.width_mm = width_mm;
            config.depth_mm = depth_mm;
            config.height_mm = height_mm;
            let error = compile(config).expect_err("outside bound must fail");
            assert!(matches!(
                error,
                CompileError::InvalidConfiguration(ref issues)
                    if issues.iter().any(|issue| issue.code == expected_code)
            ));
        }
    }

    #[test]
    fn unsupported_input_schema_version_is_rejected() {
        let mut config = valid_config();
        config.schema_version = "furniture-desk-input-next".to_owned();
        assert!(matches!(
            compile(config),
            Err(CompileError::InvalidConfiguration(ref issues))
                if issues.iter().any(|issue| issue.code == IssueCode::WrongSchemaVersion)
        ));
    }

    #[test]
    fn inclusive_dimension_boundaries_are_accepted() {
        for (width_mm, depth_mm, height_mm) in [
            (600, 400, 650),
            (1600, 900, 850),
        ] {
            let mut config = valid_config();
            config.width_mm = width_mm;
            config.depth_mm = depth_mm;
            config.height_mm = height_mm;
            assert!(compile(config).is_ok(), "bounds should be inclusive");
        }
    }

    #[test]
    fn melamine_material_accepts_only_melamine_finishes() {
        let mut white = valid_config();
        white.material_id = MaterialId::MelamineMdf;
        white.finish_id = FinishId::WhiteMelamine;
        assert!(compile(white).is_ok());

        let mut black = valid_config();
        black.material_id = MaterialId::MelamineMdf;
        black.finish_id = FinishId::BlackMelamine;
        assert!(compile(black).is_ok());
    }

    #[test]
    fn tampered_packet_is_detected() {
        let mut packet = compile(valid_config()).expect("valid bounded configuration");
        packet.core.cut_parts[0].length_mm += 1;
        assert!(!verify_integrity(&packet).expect("recompute digests"));
    }

    #[test]
    fn recomputed_outer_digest_cannot_remove_release_block() {
        let mut packet = compile(valid_config()).expect("valid bounded configuration");
        packet.core.release_gate.release_eligible = true;
        packet.packet_digest =
            digest("manufacturing-packet-v1", &packet.core).expect("digest modified packet");
        assert!(!verify_integrity(&packet).expect("verify altered packet"));
    }

    #[test]
    fn unknown_json_fields_are_rejected() {
        let input = r#"{
            "schema_version":"furniture-desk-input-v1",
            "width_mm":1200,"depth_mm":600,"height_mm":750,
            "panel_thickness_mm":18,
            "material_id":"birch_plywood",
            "finish_id":"clear_coat",
            "approve_for_production":true
        }"#;
        assert!(matches!(compile_json(input), Err(CompileError::InvalidJson(_))));
    }

    #[test]
    fn oversized_input_is_rejected_before_json_parsing() {
        let input = " ".repeat(MAX_INPUT_BYTES + 1);
        assert_eq!(compile_json(&input).expect_err("must be bounded"), CompileError::InputTooLarge);
    }
}
