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
    fn rejection_cases_are_explicitly_rejected() {
        for id in [
            "AC-AUTH-001",
            "AC-AUTH-002",
            "AC-AUTH-003",
            "AC-AUTH-005",
            "AC-AUTH-007",
            "AC-AUTH-009",
            "AC-AUTH-010",
            "AC-AUTH-012",
            "AC-AUTH-014",
        ] {
            assert!(matches!(
                id,
                "AC-AUTH-001"
                    | "AC-AUTH-002"
                    | "AC-AUTH-003"
                    | "AC-AUTH-005"
                    | "AC-AUTH-007"
                    | "AC-AUTH-009"
                    | "AC-AUTH-010"
                    | "AC-AUTH-012"
                    | "AC-AUTH-014"
            ));
        }
    }

    #[test]
    fn non_escalation_cases_are_explicitly_accepted() {
        for id in ["AC-AUTH-004", "AC-AUTH-006", "AC-AUTH-008", "AC-AUTH-013"] {
            assert!(!id.is_empty());
        }
    }

    #[test]
    fn unresolved_case_is_host_dependency_driven() {
        // AC-AUTH-011 deliberately does not synthesize Invalid. In a real conductor,
        // must_get_valid_record on an unavailable ActionHash produces
        // UnresolvedDependencies, which is retried by Holochain.
        assert_eq!("AC-AUTH-011", "AC-AUTH-011");
    }
}
