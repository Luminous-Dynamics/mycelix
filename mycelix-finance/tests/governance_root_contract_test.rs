use std::fs;
use std::path::Path;

const INTEGRITY_ZOMES: &[&str] = &[
    "recognition",
    "staking",
    "tend",
    "treasury",
];

#[test]
fn governance_root_contract_is_symmetric_across_all_finance_integrity_zomes() {
    for zome in INTEGRITY_ZOMES {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../zomes")
            .join(zome)
            .join("integrity/src/lib.rs");
        let source = fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));

        for required in [
            "GovernanceBootstrapRoot(GovernanceAgentRegistration),",
            "fn validate_create_governance_bootstrap_root(",
            "must_get_agent_activity(",
            "ChainFilter::new(action.prev_action.clone())",
            "EntryTypes::GovernanceBootstrapRoot(registration)",
            "EntryTypes::GovernanceBootstrapRoot(_)",
            "GovernanceAgentRegistration(registration)",
        ] {
            assert!(
                source.contains(required),
                "{} missing required governance-root contract fragment: {required}",
                path.display()
            );
        }
    }
}

#[test]
fn governance_root_coordinators_select_root_only_without_predecessor() {
    for zome in INTEGRITY_ZOMES {
        let package = match *zome {
            "staking" => "staking",
            "treasury" => "treasury",
            "tend" => "tend",
            "recognition" => "recognition",
            _ => unreachable!(),
        };
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../zomes")
            .join(zome)
            .join("coordinator/src/lib.rs");
        let source = fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));

        assert!(
            source.contains("let witness = GovernanceAgentRegistration {"),
            "{} must construct the shared governance witness payload",
            path.display()
        );
        assert!(
            source.contains("let witness_hash = match predecessor {"),
            "{} must choose the entry type from predecessor presence",
            path.display()
        );
        assert!(
            source.contains("None => create_entry(&EntryTypes::GovernanceBootstrapRoot(witness))?"),
            "{} must create the singleton root only for the root case",
            path.display()
        );
        assert!(
            source.contains("Some(_) => create_entry(&EntryTypes::GovernanceAgentRegistration(witness))?"),
            "{} must create a successor witness when a predecessor exists",
            path.display()
        );

        // Keep the variable intentionally used so the package mapping remains
        // explicit and the test is easy to extend per-zome.
        assert!(!package.is_empty());
    }
}
