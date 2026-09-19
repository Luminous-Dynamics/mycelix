// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use constitutional_authority::{
    branch_can_exercise, constituent_can_exercise, guardian_can_exercise, principal_can_exercise,
    AuthorityPrincipal, Branch, ConstitutionalPower, Guardian,
};

fn branch_owners(power: ConstitutionalPower) -> Vec<Branch> {
    Branch::ALL
        .into_iter()
        .filter(|branch| branch_can_exercise(*branch, power))
        .collect()
}

fn guardian_owners(power: ConstitutionalPower) -> Vec<Guardian> {
    Guardian::ALL
        .into_iter()
        .filter(|guardian| guardian_can_exercise(*guardian, power))
        .collect()
}

#[test]
fn every_enumerated_power_has_exactly_one_constitutional_owner_class() {
    for power in ConstitutionalPower::ALL {
        let branches = branch_owners(power);
        let guardians = guardian_owners(power);
        let constituent = if constituent_can_exercise(power) { 1 } else { 0 };
        let owner_count = branches.len() + guardians.len() + constituent;

        assert_eq!(
            owner_count, 1,
            "power {power:?} must have exactly one owner class; branches={branches:?}, guardians={guardians:?}, constituent={constituent}"
        );
    }
}

#[test]
fn no_power_is_shared_between_two_constituted_branches() {
    for power in ConstitutionalPower::ALL {
        let owners = branch_owners(power);
        assert!(
            owners.len() <= 1,
            "power {power:?} is shared by multiple branches: {owners:?}"
        );
    }
}

#[test]
fn constituent_powers_are_not_branch_or_guardian_powers() {
    for power in ConstitutionalPower::ALL {
        if constituent_can_exercise(power) {
            assert!(branch_owners(power).is_empty(), "{power:?} leaked to a branch");
            assert!(guardian_owners(power).is_empty(), "{power:?} leaked to a guardian");
        }
    }
}

#[test]
fn guardian_powers_are_not_branch_powers() {
    for power in ConstitutionalPower::ALL {
        if !guardian_owners(power).is_empty() {
            assert!(branch_owners(power).is_empty(), "{power:?} leaked to a branch");
            assert!(
                !constituent_can_exercise(power),
                "{power:?} leaked to constituent sovereignty"
            );
        }
    }
}

#[test]
fn every_branch_has_at_least_one_exclusive_power() {
    for branch in Branch::ALL {
        let count = ConstitutionalPower::ALL
            .into_iter()
            .filter(|power| branch_can_exercise(branch, *power))
            .count();
        assert!(count > 0, "branch {branch:?} has no constitutional power");
    }
}

#[test]
fn every_guardian_has_at_least_one_narrow_power() {
    for guardian in Guardian::ALL {
        let count = ConstitutionalPower::ALL
            .into_iter()
            .filter(|power| guardian_can_exercise(guardian, *power))
            .count();
        assert!(count > 0, "guardian {guardian:?} has no constitutional power");
    }
}

#[test]
fn automated_agents_remain_outside_all_constitutional_owner_classes() {
    for power in ConstitutionalPower::ALL {
        assert!(!principal_can_exercise(
            AuthorityPrincipal::AutomatedAgent,
            power
        ));
    }
}
