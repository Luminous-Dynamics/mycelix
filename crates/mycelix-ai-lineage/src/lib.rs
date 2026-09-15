// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![no_std]

extern crate alloc;

use alloc::collections::BTreeSet;
use alloc::vec::Vec;

pub const PROTOCOL_VERSION: &str = "mycelix-amsap-ai-lineage-v0.1";
pub const MAX_BRANCHES: usize = 1024;
pub const MAX_MERGE_PARENTS: usize = 1024;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IdentifierKind {
    Model,
    Lineage,
    Instance,
    Civic,
    Checkpoint,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineageError {
    ZeroIdentifier(IdentifierKind),
    TooFewForkChildren,
    TooManyForkChildren,
    DuplicateChildLineage,
    DuplicateChildInstance,
    ForkChildReusesParentLineage,
    ForkChildReusesParentInstance,
    ForkModelMismatch,
    TooFewMergeParents,
    TooManyMergeParents,
    DuplicateMergeParentLineage,
    DuplicateMergeParentInstance,
    MergeResultReusesParentLineage,
    MergeResultReusesParentInstance,
    RestoreResultReusesHistoricalLineage,
    RestoreResultReusesCurrentLineage,
    RestoreResultReusesCurrentInstance,
    RestoreModelMismatch,
    ModelMutationKeepsSameModel,
    ModelMutationKeepsSameInstance,
}

macro_rules! identifier_type {
    ($name:ident, $kind:expr) => {
        #[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
        pub struct $name([u8; 32]);

        impl $name {
            pub fn new(bytes: [u8; 32]) -> Result<Self, LineageError> {
                if bytes == [0; 32] {
                    return Err(LineageError::ZeroIdentifier($kind));
                }
                Ok(Self(bytes))
            }

            pub const fn as_bytes(&self) -> &[u8; 32] {
                &self.0
            }
        }
    };
}

identifier_type!(ModelId, IdentifierKind::Model);
identifier_type!(LineageId, IdentifierKind::Lineage);
identifier_type!(InstanceId, IdentifierKind::Instance);
identifier_type!(CivicId, IdentifierKind::Civic);
identifier_type!(CheckpointId, IdentifierKind::Checkpoint);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OperationalSubjectRef {
    pub model_id: ModelId,
    pub lineage_id: LineageId,
    pub instance_id: InstanceId,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SubjectRef {
    operational: OperationalSubjectRef,
    existing_civic_id: Option<CivicId>,
}

impl SubjectRef {
    pub const fn operational_only(operational: OperationalSubjectRef) -> Self {
        Self {
            operational,
            existing_civic_id: None,
        }
    }

    /// Attaches a reference to a CivicId granted elsewhere.
    /// This crate does not create civic standing or entitlement from lineage.
    pub const fn with_existing_civic_id(
        operational: OperationalSubjectRef,
        civic_id: CivicId,
    ) -> Self {
        Self {
            operational,
            existing_civic_id: Some(civic_id),
        }
    }

    pub const fn operational(&self) -> &OperationalSubjectRef {
        &self.operational
    }

    pub const fn existing_civic_id(&self) -> Option<CivicId> {
        self.existing_civic_id
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ForkChild {
    pub model_id: ModelId,
    pub lineage_id: LineageId,
    pub instance_id: InstanceId,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ForkEvent {
    pub parent: OperationalSubjectRef,
    pub checkpoint_id: CheckpointId,
    pub children: Vec<ForkChild>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MergeEvent {
    pub parents: Vec<OperationalSubjectRef>,
    pub result: OperationalSubjectRef,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CheckpointRestoreEvent {
    pub checkpoint_model_id: ModelId,
    pub historical_lineage_id: LineageId,
    pub checkpoint_id: CheckpointId,
    pub current_lineage_id: LineageId,
    pub current_instance_id: InstanceId,
    pub restored_model_id: ModelId,
    pub restored_lineage_id: LineageId,
    pub restored_instance_id: InstanceId,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ModelMutationEvent {
    pub lineage_id: LineageId,
    pub prior_model_id: ModelId,
    pub successor_model_id: ModelId,
    pub prior_instance_id: InstanceId,
    pub successor_instance_id: InstanceId,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LineageEvent {
    Fork(ForkEvent),
    Merge(MergeEvent),
    CheckpointRestore(CheckpointRestoreEvent),
    ModelMutation(ModelMutationEvent),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineageEventType {
    Fork,
    Merge,
    CheckpointRestore,
    ModelMutation,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ValidatedLineageEvent {
    event: LineageEvent,
}

impl ValidatedLineageEvent {
    pub const fn event(&self) -> &LineageEvent {
        &self.event
    }

    pub const fn event_type(&self) -> LineageEventType {
        match &self.event {
            LineageEvent::Fork(_) => LineageEventType::Fork,
            LineageEvent::Merge(_) => LineageEventType::Merge,
            LineageEvent::CheckpointRestore(_) => LineageEventType::CheckpointRestore,
            LineageEvent::ModelMutation(_) => LineageEventType::ModelMutation,
        }
    }

    pub const fn creates_civic_identity(&self) -> bool {
        false
    }

    pub const fn grants_legal_standing(&self) -> bool {
        false
    }

    pub const fn grants_governance_authority(&self) -> bool {
        false
    }

    pub const fn grants_currentness(&self) -> bool {
        false
    }

    pub const fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn validate_lineage_event(event: LineageEvent) -> Result<ValidatedLineageEvent, LineageError> {
    match &event {
        LineageEvent::Fork(fork) => validate_fork(fork)?,
        LineageEvent::Merge(merge) => validate_merge(merge)?,
        LineageEvent::CheckpointRestore(restore) => validate_restore(restore)?,
        LineageEvent::ModelMutation(mutation) => validate_model_mutation(mutation)?,
    }
    Ok(ValidatedLineageEvent { event })
}

fn validate_fork(event: &ForkEvent) -> Result<(), LineageError> {
    if event.children.len() < 2 {
        return Err(LineageError::TooFewForkChildren);
    }
    if event.children.len() > MAX_BRANCHES {
        return Err(LineageError::TooManyForkChildren);
    }

    let mut lineages = BTreeSet::new();
    let mut instances = BTreeSet::new();
    for child in &event.children {
        if child.model_id != event.parent.model_id {
            return Err(LineageError::ForkModelMismatch);
        }
        if child.lineage_id == event.parent.lineage_id {
            return Err(LineageError::ForkChildReusesParentLineage);
        }
        if child.instance_id == event.parent.instance_id {
            return Err(LineageError::ForkChildReusesParentInstance);
        }
        if !lineages.insert(child.lineage_id) {
            return Err(LineageError::DuplicateChildLineage);
        }
        if !instances.insert(child.instance_id) {
            return Err(LineageError::DuplicateChildInstance);
        }
    }
    Ok(())
}

fn validate_merge(event: &MergeEvent) -> Result<(), LineageError> {
    if event.parents.len() < 2 {
        return Err(LineageError::TooFewMergeParents);
    }
    if event.parents.len() > MAX_MERGE_PARENTS {
        return Err(LineageError::TooManyMergeParents);
    }

    let mut lineages = BTreeSet::new();
    let mut instances = BTreeSet::new();
    for parent in &event.parents {
        if parent.lineage_id == event.result.lineage_id {
            return Err(LineageError::MergeResultReusesParentLineage);
        }
        if parent.instance_id == event.result.instance_id {
            return Err(LineageError::MergeResultReusesParentInstance);
        }
        if !lineages.insert(parent.lineage_id) {
            return Err(LineageError::DuplicateMergeParentLineage);
        }
        if !instances.insert(parent.instance_id) {
            return Err(LineageError::DuplicateMergeParentInstance);
        }
    }
    Ok(())
}

fn validate_restore(event: &CheckpointRestoreEvent) -> Result<(), LineageError> {
    if event.restored_model_id != event.checkpoint_model_id {
        return Err(LineageError::RestoreModelMismatch);
    }
    if event.restored_lineage_id == event.historical_lineage_id {
        return Err(LineageError::RestoreResultReusesHistoricalLineage);
    }
    if event.restored_lineage_id == event.current_lineage_id {
        return Err(LineageError::RestoreResultReusesCurrentLineage);
    }
    if event.restored_instance_id == event.current_instance_id {
        return Err(LineageError::RestoreResultReusesCurrentInstance);
    }
    Ok(())
}

fn validate_model_mutation(event: &ModelMutationEvent) -> Result<(), LineageError> {
    if event.prior_model_id == event.successor_model_id {
        return Err(LineageError::ModelMutationKeepsSameModel);
    }
    if event.prior_instance_id == event.successor_instance_id {
        return Err(LineageError::ModelMutationKeepsSameInstance);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    fn bytes(value: u8) -> [u8; 32] {
        [value; 32]
    }

    fn model(value: u8) -> ModelId {
        ModelId::new(bytes(value)).expect("nonzero model")
    }

    fn lineage(value: u8) -> LineageId {
        LineageId::new(bytes(value)).expect("nonzero lineage")
    }

    fn instance(value: u8) -> InstanceId {
        InstanceId::new(bytes(value)).expect("nonzero instance")
    }

    fn checkpoint(value: u8) -> CheckpointId {
        CheckpointId::new(bytes(value)).expect("nonzero checkpoint")
    }

    fn operational(m: u8, l: u8, i: u8) -> OperationalSubjectRef {
        OperationalSubjectRef {
            model_id: model(m),
            lineage_id: lineage(l),
            instance_id: instance(i),
        }
    }

    #[test]
    fn identifiers_reject_zero() {
        assert_eq!(
            ModelId::new([0; 32]),
            Err(LineageError::ZeroIdentifier(IdentifierKind::Model))
        );
        assert_eq!(
            CivicId::new([0; 32]),
            Err(LineageError::ZeroIdentifier(IdentifierKind::Civic))
        );
    }

    #[test]
    fn civic_identity_is_explicitly_external_to_operational_identity() {
        let op = operational(1, 2, 3);
        let no_civic = SubjectRef::operational_only(op);
        assert_eq!(no_civic.existing_civic_id(), None);

        let civic = CivicId::new(bytes(4)).expect("nonzero civic id");
        let referenced = SubjectRef::with_existing_civic_id(op, civic);
        assert_eq!(referenced.existing_civic_id(), Some(civic));
        assert_eq!(*referenced.operational(), op);
    }

    #[test]
    fn fork_requires_distinct_new_lineages_and_instances() {
        let event = LineageEvent::Fork(ForkEvent {
            parent: operational(1, 2, 3),
            checkpoint_id: checkpoint(4),
            children: vec![
                ForkChild {
                    model_id: model(1),
                    lineage_id: lineage(5),
                    instance_id: instance(6),
                },
                ForkChild {
                    model_id: model(1),
                    lineage_id: lineage(7),
                    instance_id: instance(8),
                },
            ],
        });
        let qualified = validate_lineage_event(event).expect("valid fork");
        assert_eq!(qualified.event_type(), LineageEventType::Fork);
        assert!(!qualified.creates_civic_identity());
    }

    #[test]
    fn fork_rejects_duplicate_child_lineage() {
        let event = LineageEvent::Fork(ForkEvent {
            parent: operational(1, 2, 3),
            checkpoint_id: checkpoint(4),
            children: vec![
                ForkChild {
                    model_id: model(1),
                    lineage_id: lineage(5),
                    instance_id: instance(6),
                },
                ForkChild {
                    model_id: model(1),
                    lineage_id: lineage(5),
                    instance_id: instance(7),
                },
            ],
        });
        assert_eq!(
            validate_lineage_event(event),
            Err(LineageError::DuplicateChildLineage)
        );
    }

    #[test]
    fn fork_rejects_implicit_model_mutation() {
        let event = LineageEvent::Fork(ForkEvent {
            parent: operational(1, 2, 3),
            checkpoint_id: checkpoint(4),
            children: vec![
                ForkChild {
                    model_id: model(1),
                    lineage_id: lineage(5),
                    instance_id: instance(6),
                },
                ForkChild {
                    model_id: model(9),
                    lineage_id: lineage(7),
                    instance_id: instance(8),
                },
            ],
        });
        assert_eq!(
            validate_lineage_event(event),
            Err(LineageError::ForkModelMismatch)
        );
    }

    #[test]
    fn merge_preserves_distinct_parent_history() {
        let event = LineageEvent::Merge(MergeEvent {
            parents: vec![operational(1, 2, 3), operational(1, 4, 5)],
            result: operational(6, 7, 8),
        });
        assert!(validate_lineage_event(event).is_ok());
    }

    #[test]
    fn merge_rejects_duplicate_parent_lineage() {
        let event = LineageEvent::Merge(MergeEvent {
            parents: vec![operational(1, 2, 3), operational(4, 2, 5)],
            result: operational(6, 7, 8),
        });
        assert_eq!(
            validate_lineage_event(event),
            Err(LineageError::DuplicateMergeParentLineage)
        );
    }

    #[test]
    fn checkpoint_restore_creates_a_new_branch() {
        let event = LineageEvent::CheckpointRestore(CheckpointRestoreEvent {
            checkpoint_model_id: model(1),
            historical_lineage_id: lineage(2),
            checkpoint_id: checkpoint(3),
            current_lineage_id: lineage(4),
            current_instance_id: instance(5),
            restored_model_id: model(1),
            restored_lineage_id: lineage(6),
            restored_instance_id: instance(7),
        });
        assert!(validate_lineage_event(event).is_ok());
    }

    #[test]
    fn checkpoint_restore_cannot_rewrite_current_lineage() {
        let event = LineageEvent::CheckpointRestore(CheckpointRestoreEvent {
            checkpoint_model_id: model(1),
            historical_lineage_id: lineage(2),
            checkpoint_id: checkpoint(3),
            current_lineage_id: lineage(4),
            current_instance_id: instance(5),
            restored_model_id: model(1),
            restored_lineage_id: lineage(4),
            restored_instance_id: instance(7),
        });
        assert_eq!(
            validate_lineage_event(event),
            Err(LineageError::RestoreResultReusesCurrentLineage)
        );
    }

    #[test]
    fn model_mutation_is_explicit_and_preserves_lineage_id() {
        let event = LineageEvent::ModelMutation(ModelMutationEvent {
            lineage_id: lineage(1),
            prior_model_id: model(2),
            successor_model_id: model(3),
            prior_instance_id: instance(4),
            successor_instance_id: instance(5),
        });
        let qualified = validate_lineage_event(event).expect("valid model mutation");
        assert_eq!(qualified.event_type(), LineageEventType::ModelMutation);
    }

    #[test]
    fn validated_lineage_event_grants_no_authority() {
        let event = LineageEvent::ModelMutation(ModelMutationEvent {
            lineage_id: lineage(1),
            prior_model_id: model(2),
            successor_model_id: model(3),
            prior_instance_id: instance(4),
            successor_instance_id: instance(5),
        });
        let qualified = validate_lineage_event(event).expect("valid event");
        assert!(!qualified.creates_civic_identity());
        assert!(!qualified.grants_legal_standing());
        assert!(!qualified.grants_governance_authority());
        assert!(!qualified.grants_currentness());
        assert!(!qualified.grants_external_effect_authority());
    }
}
