use std::collections::HashSet;

use mycelix_integral_shell::{
    ContractKind, ExternalStatus, IntegralSystem, MaturityState, ADAPTER_BOUNDARIES,
    CURRENT_EXTERNAL_CONTRACTS, NAVIGATION_SURFACES,
};

#[test]
fn current_external_contract_ids_are_unique() {
    let mut ids = HashSet::new();
    for contract in CURRENT_EXTERNAL_CONTRACTS {
        assert!(ids.insert(contract.id), "duplicate external contract id: {}", contract.id);
    }
}

#[test]
fn current_phase0_statuses_are_preserved() {
    let mut data_structures = 0;
    let mut interfaces = 0;

    for contract in CURRENT_EXTERNAL_CONTRACTS {
        match contract.kind {
            ContractKind::DataStructure => {
                data_structures += 1;
                assert_eq!(contract.status, ExternalStatus::Draft);
            }
            ContractKind::Interface => {
                interfaces += 1;
                assert_eq!(contract.status, ExternalStatus::Pending);
            }
        }
    }

    assert_eq!(data_structures, 6);
    assert_eq!(interfaces, 3);
}

#[test]
fn all_five_integral_systems_have_distinct_adapter_namespaces() {
    let systems: HashSet<_> = ADAPTER_BOUNDARIES.iter().map(|b| b.system).collect();
    let namespaces: HashSet<_> = ADAPTER_BOUNDARIES.iter().map(|b| b.namespace).collect();

    assert_eq!(systems.len(), 5);
    assert_eq!(namespaces.len(), 5);
    assert!(systems.contains(&IntegralSystem::Cds));
    assert!(systems.contains(&IntegralSystem::Oad));
    assert!(systems.contains(&IntegralSystem::Itc));
    assert!(systems.contains(&IntegralSystem::Cos));
    assert!(systems.contains(&IntegralSystem::Frs));

    assert!(ADAPTER_BOUNDARIES
        .iter()
        .all(|b| b.composes_existing_mycelix_owners && b.owns_integral_normative_semantics));
}

#[test]
fn navigation_contains_all_five_systems() {
    let systems: HashSet<_> = NAVIGATION_SURFACES
        .iter()
        .filter_map(|entry| match entry.surface {
            mycelix_integral_shell::NavigationSurface::System(system) => Some(system),
            _ => None,
        })
        .collect();

    assert_eq!(systems.len(), 5);
}

#[test]
fn prequalification_maturity_never_renders_operational() {
    assert!(!MaturityState::Designed.is_operational());
    assert!(!MaturityState::SourceImplemented.is_operational());
    assert!(!MaturityState::LocallyTested.is_operational());
    assert!(MaturityState::RepositoryQualified.is_operational());
    assert!(MaturityState::PilotObserved.is_operational());
}
