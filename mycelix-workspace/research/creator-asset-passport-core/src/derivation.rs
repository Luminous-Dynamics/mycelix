//! Rights-aware derivation lineage for creative and cultural assets.
//!
//! This module extends the existing CSCP usage-grant core. It is deliberately
//! conservative: a derivation operation needs an explicit grant for every
//! source and every ancestor; later use of the derived asset needs an exact
//! grant for the output and every ancestor. No rights are inferred from asset
//! relationships, signatures, embeddings, or model output.
//!
//! The graph is currently in-memory and is not yet embedded in the portable
//! passport snapshot. It is a reference implementation for conformance work,
//! not a durable registry or legal authority verifier.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fmt;

use crate::{
    AssetId, AssetRegistry, DerivationId, GrantId, PartyId, PermissionDecision, PermissionDenial,
    UsageGrant, UseAction, UseRequest, evaluate_usage,
};

const MAX_FIELD_BYTES: usize = 256;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DerivationError {
    EmptyField,
    FieldTooLong,
    InvalidFieldCharacter,
    NoSourceAssets,
    DuplicateSourceAsset,
    OutputIsSourceAsset,
    InvalidToolDigest,
    NonDerivationOperation,
    AssetNotRegistered(AssetId),
    OutputAlreadyDerived,
    CycleDetected,
    InvalidUseRequest,
    SourceNotAuthorized {
        asset_id: AssetId,
        reason: PermissionDenial,
    },
}

impl fmt::Display for DerivationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for DerivationError {}

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct AuthorizationGrant {
    pub resource_asset_id: AssetId,
    pub grant_id: GrantId,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DerivationRecord {
    id: DerivationId,
    output_asset_id: AssetId,
    source_asset_ids: Vec<AssetId>,
    actor: PartyId,
    operation: UseAction,
    purpose: String,
    occurred_at: u64,
    tool_digest_hex: String,
    evidence_ref: String,
    authorization_grants: Vec<AuthorizationGrant>,
}

impl DerivationRecord {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: DerivationId,
        output_asset_id: AssetId,
        mut source_asset_ids: Vec<AssetId>,
        actor: PartyId,
        operation: UseAction,
        purpose: impl AsRef<str>,
        occurred_at: u64,
        tool_digest_hex: impl AsRef<str>,
        evidence_ref: impl AsRef<str>,
    ) -> Result<Self, DerivationError> {
        if source_asset_ids.is_empty() {
            return Err(DerivationError::NoSourceAssets);
        }
        if source_asset_ids.iter().any(|source| source == &output_asset_id) {
            return Err(DerivationError::OutputIsSourceAsset);
        }
        source_asset_ids.sort();
        if source_asset_ids.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(DerivationError::DuplicateSourceAsset);
        }
        if !operation.is_derivation_operation() {
            return Err(DerivationError::NonDerivationOperation);
        }

        let purpose = validate_text(purpose.as_ref())?;
        let evidence_ref = validate_text(evidence_ref.as_ref())?;
        let digest = tool_digest_hex.as_ref().trim();
        if digest.len() != 64 || !digest.bytes().all(|byte| byte.is_ascii_hexdigit()) {
            return Err(DerivationError::InvalidToolDigest);
        }

        Ok(Self {
            id,
            output_asset_id,
            source_asset_ids,
            actor,
            operation,
            purpose,
            occurred_at,
            tool_digest_hex: digest.to_ascii_lowercase(),
            evidence_ref,
            authorization_grants: Vec::new(),
        })
    }

    pub fn id(&self) -> &DerivationId {
        &self.id
    }

    pub fn output_asset_id(&self) -> &AssetId {
        &self.output_asset_id
    }

    pub fn source_asset_ids(&self) -> &[AssetId] {
        &self.source_asset_ids
    }

    pub fn actor(&self) -> &PartyId {
        &self.actor
    }

    pub fn operation(&self) -> UseAction {
        self.operation
    }

    pub fn purpose(&self) -> &str {
        &self.purpose
    }

    pub fn occurred_at(&self) -> u64 {
        self.occurred_at
    }

    pub fn tool_digest_hex(&self) -> &str {
        &self.tool_digest_hex
    }

    pub fn evidence_ref(&self) -> &str {
        &self.evidence_ref
    }

    /// Exact source/ancestor grants used when this derivation was authorized.
    /// These IDs are audit references, not cryptographic proof of issuer authority.
    pub fn authorization_grants(&self) -> &[AuthorizationGrant] {
        &self.authorization_grants
    }
}

fn validate_text(value: &str) -> Result<String, DerivationError> {
    let value = value.trim();
    if value.is_empty() {
        return Err(DerivationError::EmptyField);
    }
    if value.len() > MAX_FIELD_BYTES {
        return Err(DerivationError::FieldTooLong);
    }
    if value.chars().any(char::is_control) {
        return Err(DerivationError::InvalidFieldCharacter);
    }
    Ok(value.to_owned())
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct DerivationGraph {
    by_output: BTreeMap<AssetId, DerivationRecord>,
}

impl DerivationGraph {
    pub fn new() -> Self {
        Self::default()
    }

    /// Authorize a transformation against every source and each source's
    /// complete existing ancestry before recording the output lineage.
    ///
    /// Any failure leaves the graph unchanged. This uses CSCP's existing
    /// evaluate_usage semantics; it does not create a second grant store.
    pub fn record_authorized(
        &mut self,
        registry: &AssetRegistry,
        grants: &[UsageGrant],
        mut record: DerivationRecord,
    ) -> Result<(), DerivationError> {
        self.validate_record_assets(registry, &record)?;

        let mut authorization_grants = BTreeSet::new();
        for source_asset_id in &record.source_asset_ids {
            let request = UseRequest::new(
                source_asset_id.clone(),
                record.actor.clone(),
                record.operation,
                &record.purpose,
                record.occurred_at,
            )
            .map_err(|_| DerivationError::InvalidUseRequest)?;

            match evaluate_derived_usage(registry, self, grants, &request)? {
                DerivedPermissionDecision::Allowed { authorization_grants: refs } => {
                    authorization_grants.extend(refs);
                }
                DerivedPermissionDecision::Denied { asset_id, reason } => {
                    return Err(DerivationError::SourceNotAuthorized { asset_id, reason });
                }
            }
        }

        record.authorization_grants = authorization_grants.into_iter().collect();
        self.by_output
            .insert(record.output_asset_id.clone(), record);
        Ok(())
    }

    fn validate_record_assets(
        &self,
        registry: &AssetRegistry,
        record: &DerivationRecord,
    ) -> Result<(), DerivationError> {
        if self.by_output.contains_key(&record.output_asset_id) {
            return Err(DerivationError::OutputAlreadyDerived);
        }
        if registry.get(&record.output_asset_id).is_none() {
            return Err(DerivationError::AssetNotRegistered(
                record.output_asset_id.clone(),
            ));
        }
        for source in &record.source_asset_ids {
            if registry.get(source).is_none() {
                return Err(DerivationError::AssetNotRegistered(source.clone()));
            }
            // A source that already descends from the proposed output would
            // make the new edge cyclic.
            if self.ancestors_of(source)?.contains(&record.output_asset_id) {
                return Err(DerivationError::CycleDetected);
            }
        }
        Ok(())
    }

    pub fn get(&self, output_asset_id: &AssetId) -> Option<&DerivationRecord> {
        self.by_output.get(output_asset_id)
    }

    /// Deterministic, sorted list of all known source ancestors, nearest and
    /// distant alike. A repeated ancestor in a multi-parent graph is returned once.
    pub fn ancestors_of(&self, asset_id: &AssetId) -> Result<Vec<AssetId>, DerivationError> {
        let mut ancestors = BTreeSet::new();
        let mut expanded = BTreeSet::new();
        let mut active = BTreeSet::new();
        self.collect_ancestors(asset_id, &mut ancestors, &mut expanded, &mut active)?;
        Ok(ancestors.into_iter().collect())
    }

    fn collect_ancestors(
        &self,
        asset_id: &AssetId,
        ancestors: &mut BTreeSet<AssetId>,
        expanded: &mut BTreeSet<AssetId>,
        active: &mut BTreeSet<AssetId>,
    ) -> Result<(), DerivationError> {
        // Check the active recursion stack before the expanded set: a node can
        // already have been discovered on this same path and still form a cycle.
        if active.contains(asset_id) {
            return Err(DerivationError::CycleDetected);
        }
        if !expanded.insert(asset_id.clone()) {
            return Ok(());
        }
        active.insert(asset_id.clone());
        if let Some(record) = self.by_output.get(asset_id) {
            for source in &record.source_asset_ids {
                ancestors.insert(source.clone());
                self.collect_ancestors(source, ancestors, expanded, active)?;
            }
        }
        active.remove(asset_id);
        Ok(())
    }

    pub fn records(&self) -> impl Iterator<Item = &DerivationRecord> {
        self.by_output.values()
    }

    pub fn len(&self) -> usize {
        self.by_output.len()
    }

    pub fn is_empty(&self) -> bool {
        self.by_output.is_empty()
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DerivedPermissionDecision {
    Allowed {
        authorization_grants: Vec<AuthorizationGrant>,
    },
    Denied {
        asset_id: AssetId,
        reason: PermissionDenial,
    },
}

/// Evaluate one operation on a derived resource and every source ancestor.
///
/// A derived asset is permitted only if:
/// - the output asset has an exact active grant; and
/// - every source ancestor has an exact active grant for the same grantee,
///   operation, purpose and time.
///
/// This is intentionally conservative. No permission is inherited or widened.
/// Explicit authority-based declassification is not implemented in this crate.
pub fn evaluate_derived_usage(
    registry: &AssetRegistry,
    graph: &DerivationGraph,
    grants: &[UsageGrant],
    request: &UseRequest,
) -> Result<DerivedPermissionDecision, DerivationError> {
    let mut resources = vec![request.asset_id.clone()];
    resources.extend(graph.ancestors_of(&request.asset_id)?);

    let mut authorization_grants = Vec::new();
    for asset_id in resources {
        let scoped_request = UseRequest::new(
            asset_id.clone(),
            request.grantee_id.clone(),
            request.action,
            &request.purpose,
            request.at,
        )
        .map_err(|_| DerivationError::InvalidUseRequest)?;

        match evaluate_usage(registry, grants, &scoped_request) {
            PermissionDecision::Allowed { grant_id } => {
                authorization_grants.push(AuthorizationGrant {
                    resource_asset_id: asset_id,
                    grant_id,
                });
            }
            PermissionDecision::Denied(reason) => {
                return Ok(DerivedPermissionDecision::Denied { asset_id, reason });
            }
        }
    }

    authorization_grants.sort();
    Ok(DerivedPermissionDecision::Allowed {
        authorization_grants,
    })
}
