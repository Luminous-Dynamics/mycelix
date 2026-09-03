use std::collections::BTreeMap;

use mycelix_infrastructure_types::{InfrastructureErrorV1, PartyIdV1, StableIdV1};
use mycelix_nix_cache::{NixCacheCatalogV1, NixCacheEntryV1, NixStoreHashV1};
use thiserror::Error;

use crate::{
    project_remote_exposure_v1, ExposureAssuranceV1, ExposureEndpointIdV1, ExposureErrorV1,
    ExposureEvidenceCoverageV1, RemoteExposureGrantEvidenceV1, RemoteExposurePolicyV1,
    RemoteExposureRevocationEvidenceV1, RemoteExposureSnapshotV1, RemoteReaderV1,
};

const SERVING_SNAPSHOT_DOMAIN_V1: &str = "content-fabric/nix-remote-serving-snapshot";
const SERVING_SNAPSHOT_SCHEMA_V1: u16 = 1;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum RemoteServingSnapshotErrorV1 {
    #[error("projection was created under a different remote exposure policy")]
    PolicyMismatch,
    #[error("serving-safe remote exposure requires complete authority coverage")]
    IncompleteCoverage,
    #[error("serving-safe remote exposure requires a policy that itself requires complete coverage")]
    PolicyAllowsIncompleteCoverage,
    #[error("serving-safe remote exposure requires cryptographically verified grant assurance")]
    AssuranceTooLow,
    #[error("serving snapshot refresh interval must be non-zero")]
    ZeroRefreshInterval,
    #[error("serving snapshot is missing grant evidence {0:?}")]
    MissingGrantEvidence(StableIdV1),
    #[error("serving snapshot has no positive serving lifetime")]
    NoServingLifetime,
    #[error(
        "serving snapshot is not valid at {now_unix_ms}; valid interval is [{evaluation_time_unix_ms}, {serve_until_unix_ms})"
    )]
    SnapshotNotValidAt {
        now_unix_ms: u64,
        evaluation_time_unix_ms: u64,
        serve_until_unix_ms: u64,
    },
    #[error(transparent)]
    Infrastructure(#[from] InfrastructureErrorV1),
}

#[derive(Debug, Error)]
pub enum RemoteServingProjectionErrorV1 {
    #[error(transparent)]
    Projection(#[from] ExposureErrorV1),
    #[error(transparent)]
    Promotion(#[from] RemoteServingSnapshotErrorV1),
}

/// Build a serving-safe remote exposure artifact from one exact evidence set.
///
/// Projection and serving-horizon derivation intentionally happen inside this
/// single API call so callers cannot project with one revocation set and then
/// omit known authority transitions while deriving the serving deadline.
#[allow(clippy::too_many_arguments)]
pub fn project_remote_serving_snapshot_v1(
    catalog: &NixCacheCatalogV1,
    grants: &[RemoteExposureGrantEvidenceV1],
    revocations: &[RemoteExposureRevocationEvidenceV1],
    evaluation_time_unix_ms: u64,
    coverage: ExposureEvidenceCoverageV1,
    policy: &RemoteExposurePolicyV1,
    refresh_interval_ms: u64,
) -> Result<RemoteServingSnapshotV1, RemoteServingProjectionErrorV1> {
    let projection = project_remote_exposure_v1(
        catalog,
        grants,
        revocations,
        evaluation_time_unix_ms,
        coverage,
        policy,
    )?;
    Ok(RemoteServingSnapshotV1::from_projection(
        projection,
        policy,
        grants,
        revocations,
        refresh_interval_ms,
    )?)
}

/// Snapshot that is safe to hand to a future non-loopback serving adapter.
///
/// Unlike `RemoteExposureSnapshotV1`, this type can only be constructed from a
/// projection tied to the exact supplied strict policy, with complete authority
/// coverage and cryptographically verified grant assurance. It also carries an
/// exclusive serving deadline derived from policy refresh, exposed grant expiry,
/// grant-evidence freshness, and already-visible scheduled revocations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoteServingSnapshotV1 {
    id: StableIdV1,
    snapshot: RemoteExposureSnapshotV1,
    refresh_interval_ms: u64,
    serve_until_unix_ms: u64,
}

impl RemoteServingSnapshotV1 {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn from_projection(
        snapshot: RemoteExposureSnapshotV1,
        policy: &RemoteExposurePolicyV1,
        grants: &[RemoteExposureGrantEvidenceV1],
        revocations: &[RemoteExposureRevocationEvidenceV1],
        refresh_interval_ms: u64,
    ) -> Result<Self, RemoteServingSnapshotErrorV1> {
        if snapshot.policy_id() != policy.id() || snapshot.endpoint() != policy.endpoint() {
            return Err(RemoteServingSnapshotErrorV1::PolicyMismatch);
        }
        if snapshot.coverage() != ExposureEvidenceCoverageV1::CompleteForAuthority {
            return Err(RemoteServingSnapshotErrorV1::IncompleteCoverage);
        }
        if !policy.require_complete_coverage() {
            return Err(RemoteServingSnapshotErrorV1::PolicyAllowsIncompleteCoverage);
        }
        if policy.required_assurance() != ExposureAssuranceV1::CryptographicallyVerified {
            return Err(RemoteServingSnapshotErrorV1::AssuranceTooLow);
        }
        if refresh_interval_ms == 0 {
            return Err(RemoteServingSnapshotErrorV1::ZeroRefreshInterval);
        }

        let evaluation = snapshot.evaluation_time_unix_ms();
        let mut serve_until = evaluation.saturating_add(refresh_interval_ms);
        let visible_grants = grants
            .iter()
            .filter(|grant| grant.authored_at_unix_ms() <= evaluation)
            .map(|grant| (grant.id(), grant))
            .collect::<BTreeMap<_, _>>();

        for exposed in snapshot.entries() {
            for grant_id in exposed.grant_ids() {
                let grant = visible_grants
                    .get(grant_id)
                    .copied()
                    .ok_or(RemoteServingSnapshotErrorV1::MissingGrantEvidence(*grant_id))?;

                // The projected state already validated these facts. Rechecking
                // the authority/endpoint relation here keeps the serving wrapper
                // fail-closed if projection internals are refactored later.
                if grant.endpoint() != snapshot.endpoint() || grant.issuer() != snapshot.authority() {
                    return Err(RemoteServingSnapshotErrorV1::PolicyMismatch);
                }

                serve_until = serve_until.min(grant.valid_until_unix_ms());
                serve_until = serve_until.min(
                    grant
                        .authored_at_unix_ms()
                        .saturating_add(policy.max_evidence_age_ms()),
                );

                // A revocation that was already visible when the projection was
                // made can change serving authority at its future effective time.
                // Below-policy or pre-grant evidence is intentionally outside
                // this grant's trusted revocation universe and cannot shorten
                // the serving horizon.
                for revocation in revocations.iter().filter(|revocation| {
                    revocation.grant_id() == *grant_id
                        && revocation.issuer() == grant.issuer()
                        && revocation.authored_at_unix_ms() >= grant.authored_at_unix_ms()
                        && revocation.authored_at_unix_ms() <= evaluation
                        && revocation.effective_at_unix_ms() > evaluation
                        && revocation.assurance() >= policy.required_assurance()
                }) {
                    serve_until = serve_until.min(revocation.effective_at_unix_ms());
                }
            }
        }

        if serve_until <= evaluation {
            return Err(RemoteServingSnapshotErrorV1::NoServingLifetime);
        }

        let refresh = refresh_interval_ms.to_be_bytes();
        let deadline = serve_until.to_be_bytes();
        let projection_id = snapshot.id();
        let policy_id = policy.id();
        let id = StableIdV1::derive(
            SERVING_SNAPSHOT_DOMAIN_V1,
            SERVING_SNAPSHOT_SCHEMA_V1,
            &[&projection_id.0, &policy_id.0, &refresh, &deadline],
        )?;

        Ok(Self {
            id,
            snapshot,
            refresh_interval_ms,
            serve_until_unix_ms: serve_until,
        })
    }

    pub fn id(&self) -> StableIdV1 {
        self.id
    }

    pub fn projection_id(&self) -> StableIdV1 {
        self.snapshot.id()
    }

    pub fn policy_id(&self) -> StableIdV1 {
        self.snapshot.policy_id()
    }

    pub fn endpoint(&self) -> ExposureEndpointIdV1 {
        self.snapshot.endpoint()
    }

    pub fn authority(&self) -> PartyIdV1 {
        self.snapshot.authority()
    }

    pub fn evaluation_time_unix_ms(&self) -> u64 {
        self.snapshot.evaluation_time_unix_ms()
    }

    pub fn refresh_interval_ms(&self) -> u64 {
        self.refresh_interval_ms
    }

    /// Exclusive deadline. A future serving adapter must stop using this
    /// snapshot when `now_unix_ms >= serve_until_unix_ms()`.
    pub fn serve_until_unix_ms(&self) -> u64 {
        self.serve_until_unix_ms
    }

    pub fn is_valid_at(&self, now_unix_ms: u64) -> bool {
        self.evaluation_time_unix_ms() <= now_unix_ms && now_unix_ms < self.serve_until_unix_ms
    }

    pub fn len(&self) -> usize {
        self.snapshot.len()
    }

    pub fn is_empty(&self) -> bool {
        self.snapshot.is_empty()
    }

    /// Serving-safe lookup. The caller must supply the current authenticated
    /// request time; stale or not-yet-valid snapshots fail before any entry is
    /// returned. Unauthorized and absent entries both remain `Ok(None)`.
    pub fn entry_for_reader_at(
        &self,
        store_hash: &NixStoreHashV1,
        reader: &RemoteReaderV1,
        now_unix_ms: u64,
    ) -> Result<Option<&NixCacheEntryV1>, RemoteServingSnapshotErrorV1> {
        if !self.is_valid_at(now_unix_ms) {
            return Err(RemoteServingSnapshotErrorV1::SnapshotNotValidAt {
                now_unix_ms,
                evaluation_time_unix_ms: self.evaluation_time_unix_ms(),
                serve_until_unix_ms: self.serve_until_unix_ms,
            });
        }
        Ok(self.snapshot.entry_for_reader(store_hash, reader))
    }

    /// Read-only audit view. Serving code should use `entry_for_reader_at`;
    /// this accessor exists for diagnostics/evidence export, not serving.
    pub fn audit_projection(&self) -> &RemoteExposureSnapshotV1 {
        &self.snapshot
    }
}
