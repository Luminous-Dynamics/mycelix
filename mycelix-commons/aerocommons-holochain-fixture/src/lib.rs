use hdi::prelude::*;
use serde::{Deserialize, Serialize};

/// Test-only payload used to drive the machine-checkable authority-ceiling corpus.
///
/// This crate is intentionally isolated from Mycelix production zomes. It exercises
/// the real Holochain 0.7 / HDI 0.8 validation surface without introducing an
/// engineering authority or production evidence model.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[hdk_entry_helper]
pub struct FixtureEntry {
    pub case_id: String,
    pub dependency: Option<ActionHash>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Fixture(FixtureEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    EvidenceIndex,
}

fn invalid(message: &str) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(message.to_string()))
}

fn validate_fixture_entry(entry: FixtureEntry) -> ExternResult<ValidateCallbackResult> {
    match entry.case_id.as_str() {
        "AC-AUTH-001" => invalid("EntryHash must not be treated as an engineering Artifact or Configuration identity"),
        "AC-AUTH-002" => invalid("ActionHash must not be treated as an Execution or Observation identity"),
        "AC-AUTH-003" => {
            if let Some(dependency) = entry.dependency {
                let _ = must_get_valid_record(dependency)?;
            }
            invalid("Attestation must not satisfy a Measurement evidence role")
        }
        "AC-AUTH-004" => Ok(ValidateCallbackResult::Valid),
        "AC-AUTH-005" => {
            if let Some(dependency) = entry.dependency {
                let _ = must_get_valid_record(dependency)?;
            }
            invalid("Simulation must not become Observation because numeric values match")
        }
        "AC-AUTH-006" => {
            let dependency = entry
                .dependency
                .ok_or_else(|| wasm_error!("AC-AUTH-006 requires a dependency ActionHash"))?;
            let _ = must_get_valid_record(dependency)?;
            Ok(ValidateCallbackResult::Valid)
        }
        "AC-AUTH-008" => Ok(ValidateCallbackResult::Valid),
        "AC-AUTH-009" => {
            if let Some(dependency) = entry.dependency {
                let _ = must_get_valid_record(dependency)?;
            }
            invalid("A certification projection must not establish certification authority")
        }
        "AC-AUTH-010" => invalid("Foreign STEP/AP242/QIF identifiers must not be silently promoted to native identity"),
        "AC-AUTH-011" => {
            let dependency = entry
                .dependency
                .ok_or_else(|| wasm_error!("AC-AUTH-011 requires a missing dependency ActionHash"))?;
            let _ = must_get_valid_record(dependency)?;
            Ok(ValidateCallbackResult::Valid)
        }
        "AC-AUTH-012" => {
            if let Some(dependency) = entry.dependency {
                let _ = must_get_valid_record(dependency)?;
            }
            invalid("A superseded epistemic relation must not become current without lifecycle evidence")
        }
        "AC-AUTH-013" => Ok(ValidateCallbackResult::Valid),
        "AC-AUTH-014" => {
            if let Some(dependency) = entry.dependency {
                let _ = must_get_valid_record(dependency)?;
            }
            invalid("Prediction and Observation roles must not collapse through numeric equality")
        }
        other => invalid(&format!("unknown authority-ceiling fixture case: {other}")),
    }
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::CreateRecord(OpRecord::CreateEntry {
            app_entry: EntryTypes::Fixture(entry),
            ..
        }) => validate_fixture_entry(entry),

        FlatOp::Link(OpLink::CreateLink {
            link_type: LinkTypes::EvidenceIndex,
            ..
        }) => invalid(
            "current mutable link state must not be used as deterministic validation evidence",
        ),

        FlatOp::Link(OpLink::DeleteLink { .. }) => invalid(
            "link deletion metadata must not be interpreted as current mutable state evidence",
        ),

        _ => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entry_cases_execute_the_declared_authority_boundary() {
        let cases = [
            ("AC-AUTH-001", false),
            ("AC-AUTH-002", false),
            ("AC-AUTH-003", false),
            ("AC-AUTH-004", true),
            ("AC-AUTH-005", false),
            ("AC-AUTH-008", true),
            ("AC-AUTH-009", false),
            ("AC-AUTH-010", false),
            ("AC-AUTH-012", false),
            ("AC-AUTH-013", true),
            ("AC-AUTH-014", false),
        ];

        for (case_id, expected_valid) in cases {
            let result = validate_fixture_entry(FixtureEntry {
                case_id: case_id.to_string(),
                dependency: None,
            })
            .expect("non-dependency fixture cases should return a validation result");

            assert_eq!(
                matches!(result, ValidateCallbackResult::Valid),
                expected_valid,
                "unexpected result for {case_id}: {result:?}"
            );
        }
    }

    #[test]
    fn unknown_case_fails_closed() {
        let result = validate_fixture_entry(FixtureEntry {
            case_id: "UNKNOWN".to_string(),
            dependency: None,
        })
        .expect("unknown fixture IDs should produce Invalid, not a host error");

        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn dependency_cases_require_explicit_addresses() {
        for case_id in ["AC-AUTH-006", "AC-AUTH-011"] {
            let result = validate_fixture_entry(FixtureEntry {
                case_id: case_id.to_string(),
                dependency: None,
            });

            assert!(
                result.is_err(),
                "{case_id} must not silently omit its addressable dependency"
            );
        }
    }

    // AC-AUTH-011 intentionally delegates unresolved-dependency behavior to must_get_valid_record.
    // A real conductor test is required to exercise the UnresolvedDependencies result; a host
    // unit test must not pretend to establish it.
}
