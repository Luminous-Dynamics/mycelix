use std::collections::{BTreeMap, BTreeSet};

use crate::canonical::{
    canonical_dependency_bytes, derive_graph_commitment, derive_group_commitment,
    derive_leg_commitment,
};
use crate::{
    BoundedText, Commitment32, CoordinationGroupClassV1, CoordinationGroupV1,
    DependencySpecV1, GraphError, SettlementDependencyV1, SettlementGraphInputV1,
    SettlementGraphV1, SettlementLegRoleV1, SettlementLegV1, MAX_DEPENDENCIES,
    MAX_GROUPS, MAX_GROUP_MEMBERS, MAX_LEGS, MIN_LEGS,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum DependencyNode {
    Leg(Commitment32),
    StrongGroup(Commitment32),
}

pub fn build_settlement_graph_v1(
    input: SettlementGraphInputV1,
) -> Result<SettlementGraphV1, GraphError> {
    validate_counts(&input)?;

    let SettlementGraphInputV1 {
        economic_effect_commitment,
        graph_profile,
        temporal_profile,
        legs: leg_specs,
        dependencies: dependency_specs,
        coordination_groups: group_specs,
    } = input;

    let mut aliases = BTreeMap::<BoundedText, Commitment32>::new();
    let mut semantic_leg_ids = BTreeSet::<Commitment32>::new();
    let mut semantic_idempotency_refs = BTreeSet::<BoundedText>::new();
    let mut roles = BTreeMap::<Commitment32, SettlementLegRoleV1>::new();
    let mut legs = Vec::with_capacity(leg_specs.len());

    for spec in leg_specs {
        validate_leg_spec(&spec)?;
        if aliases.contains_key(&spec.alias) {
            return Err(GraphError::DuplicateLegAlias);
        }

        let leg_id = derive_leg_commitment(economic_effect_commitment, &graph_profile, &spec)?;
        if !semantic_leg_ids.insert(leg_id) {
            return Err(GraphError::DuplicateSemanticLeg);
        }
        if !semantic_idempotency_refs.insert(spec.semantic_idempotency_ref.clone()) {
            return Err(GraphError::DuplicateSemanticIdempotencyRef);
        }
        aliases.insert(spec.alias.clone(), leg_id);
        roles.insert(leg_id, spec.role);

        legs.push(SettlementLegV1 {
            leg_id,
            adapter_profile: spec.adapter_profile,
            rail: spec.rail,
            network: spec.network,
            role: spec.role,
            source_subject: spec.source_subject,
            destination_subject: spec.destination_subject,
            amount: spec.amount,
            asset_unit_profile: spec.asset_unit_profile,
            required_finality_profile: spec.required_finality_profile,
            semantic_idempotency_ref: spec.semantic_idempotency_ref,
            purpose_profile: spec.purpose_profile,
            delivery_asset_subject: spec.delivery_asset_subject,
        });
    }
    legs.sort_by_key(SettlementLegV1::leg_id);

    let dependencies = resolve_dependencies(dependency_specs, &aliases)?;
    let coordination_groups = resolve_groups(group_specs, &aliases, &roles, &graph_profile)?;
    validate_connected_graph(&legs, &dependencies, &coordination_groups)?;
    validate_dependency_dag(&legs, &dependencies, &coordination_groups)?;

    let mut graph = SettlementGraphV1 {
        economic_effect_commitment,
        graph_profile,
        temporal_profile,
        legs,
        dependencies,
        coordination_groups,
        graph_commitment: Commitment32::from_bytes([0_u8; 32]),
    };
    graph.graph_commitment = derive_graph_commitment(&graph)?;
    Ok(graph)
}

fn validate_counts(input: &SettlementGraphInputV1) -> Result<(), GraphError> {
    if input.legs.len() < MIN_LEGS {
        return Err(GraphError::TooFewLegs);
    }
    if input.legs.len() > MAX_LEGS {
        return Err(GraphError::TooManyLegs);
    }
    if input.dependencies.len() > MAX_DEPENDENCIES {
        return Err(GraphError::TooManyDependencies);
    }
    if input.coordination_groups.len() > MAX_GROUPS {
        return Err(GraphError::TooManyGroups);
    }
    Ok(())
}

fn validate_leg_spec(spec: &crate::SettlementLegSpecV1) -> Result<(), GraphError> {
    if spec.amount.atomic_units() == 0 {
        return Err(GraphError::ZeroAmount);
    }

    let valid = match spec.role {
        SettlementLegRoleV1::Payment => spec.delivery_asset_subject.is_none(),
        SettlementLegRoleV1::Delivery => spec.delivery_asset_subject.is_some(),
        SettlementLegRoleV1::Auxiliary => spec.purpose_profile.is_some(),
    };

    if valid {
        Ok(())
    } else {
        Err(GraphError::InvalidLegRoleSemantics)
    }
}

fn resolve_dependencies(
    specs: Vec<DependencySpecV1>,
    aliases: &BTreeMap<BoundedText, Commitment32>,
) -> Result<Vec<SettlementDependencyV1>, GraphError> {
    let mut dependencies = BTreeMap::<Vec<u8>, SettlementDependencyV1>::new();

    for spec in specs {
        let dependency = match spec {
            DependencySpecV1::Requires {
                leg_alias,
                prerequisite_alias,
            } => {
                let leg_id = resolve_alias(aliases, &leg_alias)?;
                let prerequisite_id = resolve_alias(aliases, &prerequisite_alias)?;
                if leg_id == prerequisite_id {
                    return Err(GraphError::SelfDependency);
                }
                SettlementDependencyV1::Requires {
                    leg_id,
                    prerequisite_id,
                }
            }
            DependencySpecV1::Before {
                before_alias,
                after_alias,
            } => {
                let before_id = resolve_alias(aliases, &before_alias)?;
                let after_id = resolve_alias(aliases, &after_alias)?;
                if before_id == after_id {
                    return Err(GraphError::SelfDependency);
                }
                SettlementDependencyV1::Before {
                    before_id,
                    after_id,
                }
            }
            DependencySpecV1::ConditionalOnEvidence {
                leg_alias,
                predicate_profile,
            } => SettlementDependencyV1::ConditionalOnEvidence {
                leg_id: resolve_alias(aliases, &leg_alias)?,
                predicate_profile,
            },
        };

        let key = canonical_dependency_bytes(&dependency)?;
        if dependencies.insert(key, dependency).is_some() {
            return Err(GraphError::DuplicateDependency);
        }
    }

    Ok(dependencies.into_values().collect())
}

fn resolve_groups(
    specs: Vec<crate::CoordinationGroupSpecV1>,
    aliases: &BTreeMap<BoundedText, Commitment32>,
    roles: &BTreeMap<Commitment32, SettlementLegRoleV1>,
    graph_profile: &crate::SemanticProfileRefV1,
) -> Result<Vec<CoordinationGroupV1>, GraphError> {
    let mut group_aliases = BTreeSet::new();
    let mut seen_members = BTreeSet::new();
    let mut semantic_group_ids = BTreeSet::new();
    let mut groups = Vec::with_capacity(specs.len());

    for spec in specs {
        if !group_aliases.insert(spec.alias) {
            return Err(GraphError::DuplicateGroupAlias);
        }
        validate_group_size(spec.class, spec.member_aliases.len())?;

        let mut member_ids = BTreeSet::new();
        for member_alias in spec.member_aliases {
            let member_id = resolve_alias(aliases, &member_alias)?;
            if !member_ids.insert(member_id) {
                return Err(GraphError::DuplicateGroupMember);
            }
        }
        if member_ids.len() > MAX_GROUP_MEMBERS {
            return Err(GraphError::InvalidGroupSize);
        }
        for member_id in &member_ids {
            if !seen_members.insert(*member_id) {
                return Err(GraphError::LegInMultipleGroups);
            }
        }

        let member_leg_ids: Vec<_> = member_ids.into_iter().collect();
        validate_group_roles(spec.class, &member_leg_ids, roles)?;
        let group_id = derive_group_commitment(
            graph_profile,
            spec.class,
            &spec.coordination_profile,
            &member_leg_ids,
        )?;
        if !semantic_group_ids.insert(group_id) {
            return Err(GraphError::DuplicateSemanticGroup);
        }

        groups.push(CoordinationGroupV1 {
            group_id,
            class: spec.class,
            coordination_profile: spec.coordination_profile,
            member_leg_ids,
        });
    }

    groups.sort_by_key(CoordinationGroupV1::group_id);
    Ok(groups)
}

fn validate_group_size(
    class: CoordinationGroupClassV1,
    size: usize,
) -> Result<(), GraphError> {
    let valid = match class {
        CoordinationGroupClassV1::Pvp | CoordinationGroupClassV1::Dvp => size == 2,
        CoordinationGroupClassV1::AllOrNone | CoordinationGroupClassV1::Saga => {
            (2..=MAX_GROUP_MEMBERS).contains(&size)
        }
    };
    if valid {
        Ok(())
    } else {
        Err(GraphError::InvalidGroupSize)
    }
}

fn validate_group_roles(
    class: CoordinationGroupClassV1,
    member_leg_ids: &[Commitment32],
    roles: &BTreeMap<Commitment32, SettlementLegRoleV1>,
) -> Result<(), GraphError> {
    let mut payment_count = 0_usize;
    let mut delivery_count = 0_usize;

    for member_id in member_leg_ids {
        match roles
            .get(member_id)
            .copied()
            .ok_or(GraphError::InternalInvariant)?
        {
            SettlementLegRoleV1::Payment => payment_count += 1,
            SettlementLegRoleV1::Delivery => delivery_count += 1,
            SettlementLegRoleV1::Auxiliary => {}
        }
    }

    let valid = match class {
        CoordinationGroupClassV1::Pvp => payment_count == 2 && delivery_count == 0,
        CoordinationGroupClassV1::Dvp => payment_count == 1 && delivery_count == 1,
        CoordinationGroupClassV1::AllOrNone | CoordinationGroupClassV1::Saga => true,
    };

    if valid {
        Ok(())
    } else {
        Err(GraphError::InvalidCoordinationRoleComposition)
    }
}

fn resolve_alias(
    aliases: &BTreeMap<BoundedText, Commitment32>,
    alias: &BoundedText,
) -> Result<Commitment32, GraphError> {
    aliases
        .get(alias)
        .copied()
        .ok_or(GraphError::UnknownLegAlias)
}

fn validate_connected_graph(
    legs: &[SettlementLegV1],
    dependencies: &[SettlementDependencyV1],
    groups: &[CoordinationGroupV1],
) -> Result<(), GraphError> {
    let mut adjacency = BTreeMap::<Commitment32, BTreeSet<Commitment32>>::new();
    for leg in legs {
        adjacency.insert(leg.leg_id, BTreeSet::new());
    }

    let mut connect = |left: Commitment32, right: Commitment32| -> Result<(), GraphError> {
        adjacency
            .get_mut(&left)
            .ok_or(GraphError::InternalInvariant)?
            .insert(right);
        adjacency
            .get_mut(&right)
            .ok_or(GraphError::InternalInvariant)?
            .insert(left);
        Ok(())
    };

    for dependency in dependencies {
        match dependency {
            SettlementDependencyV1::Requires {
                leg_id,
                prerequisite_id,
            } => connect(*leg_id, *prerequisite_id)?,
            SettlementDependencyV1::Before {
                before_id,
                after_id,
            } => connect(*before_id, *after_id)?,
            SettlementDependencyV1::ConditionalOnEvidence { .. } => {}
        }
    }

    for group in groups {
        let Some(first) = group.member_leg_ids.first().copied() else {
            return Err(GraphError::InternalInvariant);
        };
        for member in group.member_leg_ids.iter().copied().skip(1) {
            connect(first, member)?;
        }
    }

    let Some(start) = legs.first().map(SettlementLegV1::leg_id) else {
        return Err(GraphError::InternalInvariant);
    };
    let mut visited = BTreeSet::new();
    let mut frontier = vec![start];

    while let Some(current) = frontier.pop() {
        if !visited.insert(current) {
            continue;
        }
        let neighbors = adjacency
            .get(&current)
            .ok_or(GraphError::InternalInvariant)?;
        frontier.extend(neighbors.iter().copied());
    }

    if visited.len() == legs.len() {
        Ok(())
    } else {
        Err(GraphError::DisconnectedGraph)
    }
}

fn validate_dependency_dag(
    legs: &[SettlementLegV1],
    dependencies: &[SettlementDependencyV1],
    groups: &[CoordinationGroupV1],
) -> Result<(), GraphError> {
    let mut strong_group_by_leg = BTreeMap::<Commitment32, Commitment32>::new();
    for group in groups {
        if group.class.is_strongly_coupled() {
            for member in &group.member_leg_ids {
                strong_group_by_leg.insert(*member, group.group_id);
            }
        }
    }

    let node_for = |leg_id: Commitment32| {
        strong_group_by_leg
            .get(&leg_id)
            .copied()
            .map(DependencyNode::StrongGroup)
            .unwrap_or(DependencyNode::Leg(leg_id))
    };

    let mut nodes = BTreeSet::new();
    for leg in legs {
        nodes.insert(node_for(leg.leg_id));
    }

    let mut adjacency = BTreeMap::<DependencyNode, BTreeSet<DependencyNode>>::new();
    let mut indegree = BTreeMap::<DependencyNode, usize>::new();
    for node in &nodes {
        adjacency.insert(*node, BTreeSet::new());
        indegree.insert(*node, 0);
    }

    for dependency in dependencies {
        let edge = match dependency {
            SettlementDependencyV1::Requires {
                leg_id,
                prerequisite_id,
            } => Some((node_for(*prerequisite_id), node_for(*leg_id))),
            SettlementDependencyV1::Before {
                before_id,
                after_id,
            } => Some((node_for(*before_id), node_for(*after_id))),
            SettlementDependencyV1::ConditionalOnEvidence { .. } => None,
        };

        if let Some((from, to)) = edge {
            if from == to {
                return Err(GraphError::DependencyInsideStrongGroup);
            }
            let targets = adjacency
                .get_mut(&from)
                .ok_or(GraphError::InternalInvariant)?;
            if targets.insert(to) {
                let target_indegree = indegree
                    .get_mut(&to)
                    .ok_or(GraphError::InternalInvariant)?;
                *target_indegree += 1;
            }
        }
    }

    let mut ready: BTreeSet<_> = indegree
        .iter()
        .filter_map(|(node, degree)| (*degree == 0).then_some(*node))
        .collect();
    let mut visited = 0_usize;

    while let Some(node) = ready.pop_first() {
        visited += 1;
        if let Some(targets) = adjacency.get(&node) {
            for target in targets {
                let degree = indegree
                    .get_mut(target)
                    .ok_or(GraphError::InternalInvariant)?;
                *degree = degree.checked_sub(1).ok_or(GraphError::InternalInvariant)?;
                if *degree == 0 {
                    ready.insert(*target);
                }
            }
        }
    }

    if visited == nodes.len() {
        Ok(())
    } else {
        Err(GraphError::DependencyCycle)
    }
}
