use std::{
    cmp::max,
    collections::{BTreeMap, BTreeSet},
};

use mycelix_infrastructure_types::StableIdV1;
use mycelix_nix_cache::{NixCacheCatalogV1, NixCacheEntryV1, NixStoreHashV1};

use crate::{
    make_exposed_entry, ExposureAudienceV1, ExposureErrorV1, ExposureEvidenceCoverageV1,
    GrantExclusionReasonV1, NixExposureObjectIdV1, RemoteExposureDiagnosticV1,
    RemoteExposureGrantEvidenceV1, RemoteExposurePolicyV1,
    RemoteExposureRevocationEvidenceV1, RemoteExposureSnapshotV1, RevocationIgnoreReasonV1,
};

#[derive(Debug)]
struct ExposureBuilderV1 {
    entry: NixCacheEntryV1,
    object_id: NixExposureObjectIdV1,
    audiences: BTreeSet<ExposureAudienceV1>,
    grant_ids: BTreeSet<StableIdV1>,
}

pub fn project_remote_exposure_v1(
    catalog: &NixCacheCatalogV1,
    grants: &[RemoteExposureGrantEvidenceV1],
    revocations: &[RemoteExposureRevocationEvidenceV1],
    evaluation_time_unix_ms: u64,
    coverage: ExposureEvidenceCoverageV1,
    policy: &RemoteExposurePolicyV1,
) -> Result<RemoteExposureSnapshotV1, ExposureErrorV1> {
    if policy.endpoint().stable_id() == StableIdV1::ZERO {
        return Err(ExposureErrorV1::ZeroEndpoint);
    }
    if policy.require_complete_coverage()
        && coverage != ExposureEvidenceCoverageV1::CompleteForAuthority
    {
        return Err(ExposureErrorV1::IncompleteCoverage);
    }

    // Historical replay is causally sliced before deduplication or diagnostics.
    // Future evidence therefore cannot poison an earlier exposure snapshot.
    let mut visible_grants = BTreeMap::<StableIdV1, RemoteExposureGrantEvidenceV1>::new();
    for grant in grants
        .iter()
        .filter(|grant| grant.authored_at_unix_ms() <= evaluation_time_unix_ms)
    {
        match visible_grants.get(&grant.id()) {
            Some(existing) if existing != grant => {
                return Err(ExposureErrorV1::ConflictingGrantId(grant.id()));
            }
            Some(_) => {}
            None => {
                visible_grants.insert(grant.id(), grant.clone());
            }
        }
    }

    let mut visible_revocations =
        BTreeMap::<StableIdV1, RemoteExposureRevocationEvidenceV1>::new();
    for revocation in revocations
        .iter()
        .filter(|revocation| revocation.authored_at_unix_ms() <= evaluation_time_unix_ms)
    {
        match visible_revocations.get(&revocation.id()) {
            Some(existing) if existing != revocation => {
                return Err(ExposureErrorV1::ConflictingRevocationId(revocation.id()));
            }
            Some(_) => {}
            None => {
                visible_revocations.insert(revocation.id(), revocation.clone());
            }
        }
    }

    let mut diagnostics = Vec::new();
    for revocation in visible_revocations.values() {
        if !visible_grants.contains_key(&revocation.grant_id()) {
            diagnostics.push(RemoteExposureDiagnosticV1::RevocationIgnored {
                revocation_id: revocation.id(),
                reason: RevocationIgnoreReasonV1::UnknownGrant,
            });
        }
    }

    let mut exposed = BTreeMap::<NixStoreHashV1, ExposureBuilderV1>::new();
    for grant in visible_grants.values() {
        if grant.issuer() != policy.authority() {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::AuthorityMismatch,
            );
            continue;
        }
        if grant.endpoint() != policy.endpoint() {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::EndpointMismatch,
            );
            continue;
        }
        if grant.assurance() < policy.required_assurance() {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::AssuranceTooLow,
            );
            continue;
        }
        if grant.valid_until_unix_ms() - grant.valid_from_unix_ms()
            > policy.max_grant_lifetime_ms()
        {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::GrantLifetimeTooLong,
            );
            continue;
        }
        if evaluation_time_unix_ms - grant.authored_at_unix_ms() > policy.max_evidence_age_ms() {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::EvidenceTooOld,
            );
            continue;
        }
        if evaluation_time_unix_ms < grant.valid_from_unix_ms() {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::NotYetValid,
            );
            continue;
        }
        if evaluation_time_unix_ms >= grant.valid_until_unix_ms() {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::Expired,
            );
            continue;
        }
        if !policy.allows_audience(grant.audience()) {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::AudienceDisallowed,
            );
            continue;
        }

        let Some(entry) = catalog.entry(grant.store_hash()) else {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::CatalogEntryMissing,
            );
            continue;
        };
        let object_id = NixExposureObjectIdV1::from_entry(entry)?;
        if object_id != grant.object_id() {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::CatalogObjectChanged,
            );
            continue;
        }

        let required_revocation_assurance = max(policy.required_assurance(), grant.assurance());
        let mut revoked = false;
        let mut unresolved_revocation = false;
        for revocation in visible_revocations
            .values()
            .filter(|revocation| revocation.grant_id() == grant.id())
        {
            if revocation.issuer() != grant.issuer() {
                diagnostics.push(RemoteExposureDiagnosticV1::RevocationIgnored {
                    revocation_id: revocation.id(),
                    reason: RevocationIgnoreReasonV1::IssuerMismatch,
                });
                continue;
            }
            if revocation.authored_at_unix_ms() < grant.authored_at_unix_ms() {
                diagnostics.push(RemoteExposureDiagnosticV1::RevocationIgnored {
                    revocation_id: revocation.id(),
                    reason: RevocationIgnoreReasonV1::PredatesGrant,
                });
                continue;
            }
            // A scheduled revocation does not shorten the grant before its
            // declared effective boundary, even though the evidence itself is
            // already visible in the historical snapshot.
            if revocation.effective_at_unix_ms() > evaluation_time_unix_ms {
                continue;
            }
            if revocation.assurance() < policy.required_assurance() {
                // Evidence below the policy's authority threshold cannot prove
                // that it came from the grant authority and must not become a
                // downgrade/availability DoS primitive.
                continue;
            }
            if revocation.assurance() < required_revocation_assurance {
                // Evidence that is trusted strongly enough to enter the
                // authority universe but is weaker than the original grant is
                // genuinely ambiguous. Remote disclosure fails closed.
                unresolved_revocation = true;
                continue;
            }
            revoked = true;
        }
        if unresolved_revocation {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::RevocationUnresolved,
            );
            continue;
        }
        if revoked {
            exclude(
                &mut diagnostics,
                grant.id(),
                GrantExclusionReasonV1::Revoked,
            );
            continue;
        }

        let builder = exposed
            .entry(grant.store_hash().clone())
            .or_insert_with(|| ExposureBuilderV1 {
                entry: entry.clone(),
                object_id,
                audiences: BTreeSet::new(),
                grant_ids: BTreeSet::new(),
            });
        builder.audiences.insert(grant.audience().clone());
        builder.grant_ids.insert(grant.id());
    }

    let entries = exposed
        .into_values()
        .map(|builder| {
            make_exposed_entry(
                builder.entry,
                builder.object_id,
                builder.audiences,
                builder.grant_ids,
            )
        })
        .collect();

    RemoteExposureSnapshotV1::new(
        policy,
        evaluation_time_unix_ms,
        coverage,
        entries,
        diagnostics,
    )
}

fn exclude(
    diagnostics: &mut Vec<RemoteExposureDiagnosticV1>,
    grant_id: StableIdV1,
    reason: GrantExclusionReasonV1,
) {
    diagnostics.push(RemoteExposureDiagnosticV1::GrantExcluded { grant_id, reason });
}
