// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Storage backend that persists DeSci claims to mycelix-knowledge's DHT
//! instead of an in-process HashMap, via a native (non-WASM) Holochain
//! client connection -- mirrors the connection pattern already used by
//! `mycelix-pulse/happ/backend-rs`.
//!
//! Only `store`/`retrieve`/`exists` are implemented against the real DHT.
//! `delete` has no equivalent in mycelix-knowledge's `claims` zome (claims
//! are immutable once created, matching that zome's own design) and
//! `get_full_proof`/`update_quantum_anchor_pointer` are DeSci-specific
//! ZKP/Arweave concerns mycelix-knowledge has no model of at all -- both
//! return an explicit "not supported" error rather than fabricating
//! plausible-looking data.

use crate::claims::{ClaimContent, DesciClaim, EpistemicPosition, EpistemicTier};
use crate::{Error, Result};
use async_trait::async_trait;
use holochain_client::{
    AdminWebsocket, AppInfo, AppWebsocket, AuthorizeSigningCredentialsPayload, CellInfo,
    ClientAgentSigner, ExternIO, IssueAppAuthenticationTokenPayload,
};
use holochain_types::prelude::CellId;
use holochain_zome_types::prelude::Record;
use serde::{Deserialize, Serialize};

/// Map a continuous empirical score (0.0-1.0, as stored in
/// mycelix-knowledge's `Claim.classification.empirical`) back to DeSci's
/// discrete `EpistemicTier`, using the same 0.25-wide bucket boundaries as
/// `mycelix-claim-types::EmpiricalLevel::from_f32`.
fn epistemic_tier_from_score(score: f64) -> EpistemicTier {
    if score >= 0.875 {
        EpistemicTier::E4
    } else if score >= 0.625 {
        EpistemicTier::E3
    } else if score >= 0.375 {
        EpistemicTier::E2
    } else if score >= 0.125 {
        EpistemicTier::E1
    } else {
        EpistemicTier::E0
    }
}

/// Strip a `ws://` or `wss://` scheme prefix, if present, so a
/// conventionally-written conductor URL can be passed to
/// `holochain_client`'s `impl ToSocketAddrs`-based `connect()`.
fn strip_ws_scheme(addr: &str) -> &str {
    addr.strip_prefix("wss://")
        .or_else(|| addr.strip_prefix("ws://"))
        .unwrap_or(addr)
}

/// Parse the `desci-tier:E{0..4}` tag written by
/// `knowledge_claim_from_desci_claim` (via `{:?}` on `EpistemicTier`) back
/// into the exact original tier.
fn parse_epistemic_tier(s: &str) -> Option<EpistemicTier> {
    match s {
        "E0" => Some(EpistemicTier::E0),
        "E1" => Some(EpistemicTier::E1),
        "E2" => Some(EpistemicTier::E2),
        "E3" => Some(EpistemicTier::E3),
        "E4" => Some(EpistemicTier::E4),
        _ => None,
    }
}

/// Wire-format claim matching mycelix-knowledge's `claims` zome `Claim`
/// entry shape (`mycelix-workspace/mycelix-knowledge/zomes/claims/integrity/src/lib.rs`).
/// Defined locally rather than depending on that zome's crate directly --
/// the interchange is over the zome-call boundary (serde), the same
/// pattern Prism's `prism-ui/src/holochain.rs` already uses.
#[derive(Serialize, Deserialize, Debug, Clone)]
struct KnowledgeClaimClassification {
    empirical: f64,
    normative: f64,
    mythic: f64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct KnowledgeClaim {
    id: String,
    content: String,
    classification: KnowledgeClaimClassification,
    author: String,
    sources: Vec<String>,
    tags: Vec<String>,
    claim_type: String,
    confidence: f64,
    expires: Option<holochain_zome_types::prelude::Timestamp>,
    created: holochain_zome_types::prelude::Timestamp,
    updated: holochain_zome_types::prelude::Timestamp,
    version: u32,
}

// Generates `TryFrom<SerializedBytes> for KnowledgeClaim` (and the reverse),
// required by `RecordEntry::to_app_option::<KnowledgeClaim>()` below to
// decode the msgpack-encoded `Entry::App` payload a zome call actually
// returns -- treating the response as `serde_json::Value` (the original,
// wrong approach) failed with "invalid type: byte array, expected any
// valid JSON value" because the wire format is msgpack, not JSON.
holochain_serialized_bytes::holochain_serial!(KnowledgeClaim);

/// Storage backend backed by mycelix-knowledge's DHT.
pub struct KnowledgeDhtStorage {
    app_ws: AppWebsocket,
    cell_id: CellId,
}

impl KnowledgeDhtStorage {
    /// Connect to the shared conductor's `mycelix-knowledge` app and return
    /// a ready-to-use storage backend. Mirrors
    /// `mycelix-pulse/happ/backend-rs/src/services/holochain.rs`'s
    /// `establish_connection` (admin connect -> issue auth token -> app
    /// connect -> app_info -> find cell -> authorize signing).
    ///
    /// `admin_addr`/`app_addr` accept either a plain `host:port` (e.g.
    /// `"localhost:33800"`) or a `ws://`/`wss://`-prefixed URL -- this
    /// `holochain_client` version's `connect()` takes `impl ToSocketAddrs`
    /// directly (matching the crate's own doc examples, e.g.
    /// `"localhost:30000"`), not a URL string, so any scheme prefix is
    /// stripped here to accept both forms (the rest of this codebase's
    /// conductor URLs are conventionally written with a `ws://` prefix,
    /// e.g. in `mycelix-workspace/CLAUDE.md`).
    pub async fn connect(admin_addr: &str, app_addr: &str, app_id: &str) -> Result<Self> {
        let admin_addr = strip_ws_scheme(admin_addr);
        let app_addr = strip_ws_scheme(app_addr);

        let admin_ws = AdminWebsocket::connect(admin_addr, None)
            .await
            .map_err(|e| Error::Network(format!("Admin connection failed: {e}")))?;

        let token_response = admin_ws
            .issue_app_auth_token(IssueAppAuthenticationTokenPayload {
                installed_app_id: app_id.to_string().into(),
                expiry_seconds: 3600,
                single_use: false,
            })
            .await
            .map_err(|e| Error::Network(format!("Failed to issue auth token: {e}")))?;

        // `ClientAgentSigner` wraps `Arc<RwLock<HashMap<CellId, SigningCredentials>>>`
        // internally (it's cheaply `Clone`, sharing the same underlying credential
        // store) -- we keep our own handle so we can call `add_credentials` on it
        // *after* the cell_id is known, while `app_ws` uses a clone of the same
        // signer for its zome calls. Credentials are NOT added automatically by
        // `authorize_signing_credentials` (contrary to what
        // mycelix-pulse/happ/backend-rs's comment claims) -- omitting this step
        // fails every zome call with `SignZomeCallError("Provenance not found")`,
        // caught only by actually running this against a live conductor.
        let signer = ClientAgentSigner::default();

        let app_ws =
            AppWebsocket::connect(app_addr, token_response.token, signer.clone().into(), None)
                .await
                .map_err(|e| Error::Network(format!("App connection failed: {e}")))?;

        let app_info: AppInfo = app_ws
            .app_info()
            .await
            .map_err(|e| Error::Network(format!("Failed to get app info: {e}")))?
            .ok_or_else(|| Error::Network("App info not available".to_string()))?;

        let cell_id = Self::find_cell_id(&app_info)?;

        let credentials = admin_ws
            .authorize_signing_credentials(AuthorizeSigningCredentialsPayload {
                cell_id: cell_id.clone(),
                functions: None,
            })
            .await
            .map_err(|e| Error::Network(format!("Failed to authorize signing: {e}")))?;
        signer.add_credentials(cell_id.clone(), credentials);

        Ok(Self { app_ws, cell_id })
    }

    fn find_cell_id(app_info: &AppInfo) -> Result<CellId> {
        for cells in app_info.cell_info.values() {
            for cell_info in cells {
                match cell_info {
                    CellInfo::Provisioned(cell) => return Ok(cell.cell_id.clone()),
                    CellInfo::Cloned(cell) => return Ok(cell.cell_id.clone()),
                    _ => continue,
                }
            }
        }
        Err(Error::Network(
            "No provisioned cell found in mycelix-knowledge app".to_string(),
        ))
    }

    /// The connected agent's own DID, matching the
    /// `did:mycelix:<agent-pubkey>` format mycelix-knowledge's claims
    /// validator requires (see `require_claim_author_matches_committer`
    /// in that zome's integrity crate) -- this backend must always supply
    /// the real connecting agent's key as `author`, never a placeholder.
    fn agent_did(&self) -> String {
        format!("did:mycelix:{}", self.cell_id.agent_pubkey())
    }

    async fn call_zome<I, O>(&self, zome_name: &str, fn_name: &str, payload: I) -> Result<O>
    where
        I: Serialize + Clone + std::fmt::Debug,
        O: serde::de::DeserializeOwned + std::fmt::Debug,
    {
        let encoded = ExternIO::encode(payload)
            .map_err(|e| Error::Network(format!("Failed to encode payload: {e}")))?;
        let response = self
            .app_ws
            .call_zome(
                self.cell_id.clone().into(),
                zome_name.into(),
                fn_name.into(),
                encoded,
            )
            .await
            .map_err(|e| {
                Error::Network(format!("Zome call {zome_name}/{fn_name} failed: {e:?}"))
            })?;
        response
            .decode()
            .map_err(|e| Error::Network(format!("Failed to decode response: {e}")))
    }

    fn to_knowledge_claim(&self, claim: &DesciClaim) -> KnowledgeClaim {
        knowledge_claim_from_desci_claim(claim, self.agent_did())
    }
}

/// Pure mapping, factored out of `KnowledgeDhtStorage::to_knowledge_claim`
/// so it's unit-testable without a live conductor connection.
fn knowledge_claim_from_desci_claim(claim: &DesciClaim, author_did: String) -> KnowledgeClaim {
    let mut sources: Vec<String> = claim
        .provenance
        .iter()
        .map(|p| match &p.url {
            Some(url) => format!("{} ({}): {}", p.source, p.source_type, url),
            None => format!("{} ({})", p.source, p.source_type),
        })
        .collect();
    // Preserve the original creator identifier as provenance context --
    // it is NOT used as the DHT entry's author (see agent_did()).
    sources.push(format!("desci-creator:{}", claim.creator));

    // `epistemic_tier` and `epistemic_position` are independent fields on
    // DesciClaim (verification-based trust level vs. E-N-M type
    // classification) -- only `epistemic_position` maps onto
    // mycelix-knowledge's `classification`. The discrete tier is preserved
    // via a prefixed tag rather than re-derived from the position's score
    // on the way back, which would silently discard the real value
    // whenever it doesn't coincide with the position's bucket (it did for
    // E2 + the default 0.5 position by coincidence, not by correctness).
    let mut tags = claim.content.keywords.clone();
    tags.push(format!("desci-category:{}", claim.content.category));
    tags.push(format!("desci-tier:{:?}", claim.epistemic_tier));

    KnowledgeClaim {
        id: claim.id.to_string(),
        content: claim.content.description.clone(),
        classification: KnowledgeClaimClassification {
            empirical: claim.epistemic_position.empirical,
            normative: claim.epistemic_position.normative,
            mythic: claim.epistemic_position.mythic,
        },
        author: author_did,
        sources,
        tags,
        claim_type: "Fact".to_string(),
        // Reuses the fingerprint's own real weighted-composite computation
        // (see EpistemicFingerprint::confidence_score) -- not a fabricated
        // value.
        confidence: claim.fingerprint.confidence_score(),
        expires: None,
        created: holochain_zome_types::prelude::Timestamp::from_micros(
            claim.created_at.timestamp_micros(),
        ),
        updated: holochain_zome_types::prelude::Timestamp::from_micros(
            claim.updated_at.timestamp_micros(),
        ),
        version: 1,
    }
}

impl KnowledgeDhtStorage {
    fn from_knowledge_claim(kc: KnowledgeClaim) -> Result<DesciClaim> {
        let id = uuid::Uuid::parse_str(&kc.id)
            .map_err(|e| Error::Storage(format!("Invalid claim id from DHT: {e}")))?;

        let epistemic_position = EpistemicPosition::new(
            kc.classification.empirical,
            kc.classification.normative,
            kc.classification.mythic,
        );

        let mut category: Option<String> = None;
        let mut tier_tag: Option<String> = None;
        let mut keywords = Vec::new();
        for tag in &kc.tags {
            if let Some(c) = tag.strip_prefix("desci-category:") {
                category = Some(c.to_string());
            } else if let Some(t) = tag.strip_prefix("desci-tier:") {
                tier_tag = Some(t.to_string());
            } else {
                keywords.push(tag.clone());
            }
        }
        let category = category.unwrap_or_else(|| "unknown".to_string());
        // Prefer the exact preserved tier; only fall back to deriving from
        // the position's empirical score (an approximation, not a faithful
        // round-trip) for claims that predate this tagging scheme.
        let epistemic_tier = tier_tag
            .and_then(|t| parse_epistemic_tier(&t))
            .unwrap_or_else(|| epistemic_tier_from_score(kc.classification.empirical));

        let content = ClaimContent {
            dataset_hash: String::new(),
            description: kc.content,
            category,
            keywords,
            storage_ref: None,
            reproducibility_score: None,
            license: None,
        };

        let creator = kc
            .sources
            .iter()
            .find_map(|s| s.strip_prefix("desci-creator:"))
            .map(str::to_string)
            .unwrap_or(kc.author);

        let mut claim =
            DesciClaim::with_position(epistemic_tier, epistemic_position, content, creator);
        claim.id = id;
        claim.created_at = chrono::DateTime::from_timestamp_micros(kc.created.as_micros())
            .unwrap_or(claim.created_at);
        claim.updated_at = chrono::DateTime::from_timestamp_micros(kc.updated.as_micros())
            .unwrap_or(claim.updated_at);
        // Provenance (source URLs) is reconstructible from `kc.sources` if
        // needed by a caller, but is intentionally not round-tripped into
        // `claim.provenance` here -- the desci-creator marker aside, this
        // backend stores sources as free-text; full Provenance fidelity
        // (source_type, timestamp, structured metadata) isn't recoverable
        // from mycelix-knowledge's Claim shape and is left empty rather than
        // fabricated.
        Ok(claim)
    }
}

#[async_trait]
impl crate::storage::StorageBackend for KnowledgeDhtStorage {
    async fn store(&self, claim: &DesciClaim) -> Result<String> {
        let kc = self.to_knowledge_claim(claim);
        let _record: Record = self.call_zome("claims", "submit_claim", kc).await?;
        Ok(claim.id.to_string())
    }

    async fn retrieve(&self, cid: &str) -> Result<DesciClaim> {
        let record: Option<Record> = self
            .call_zome("claims", "get_claim", cid.to_string())
            .await?;
        let record = record.ok_or_else(|| Error::NotFound(format!("Claim not found: {cid}")))?;
        let kc: KnowledgeClaim = record
            .entry()
            .to_app_option()
            .map_err(|e| Error::Storage(format!("Failed to parse claim from DHT: {e}")))?
            .ok_or_else(|| Error::Storage("Malformed claim record from DHT".to_string()))?;
        Self::from_knowledge_claim(kc)
    }

    async fn exists(&self, cid: &str) -> Result<bool> {
        match self.retrieve(cid).await {
            Ok(_) => Ok(true),
            Err(Error::NotFound(_)) => Ok(false),
            Err(e) => Err(e),
        }
    }

    async fn delete(&self, _cid: &str) -> Result<()> {
        Err(Error::Storage(
            "not supported: claims are immutable in mycelix-knowledge's DHT".to_string(),
        ))
    }

    async fn get_full_proof(
        &self,
        _id: uuid::Uuid,
    ) -> Result<mycelix_zkp_core::AuthenticatedProof> {
        Err(Error::Storage(
            "not supported by KnowledgeDhtStorage: ZKP proof archival is a separate concern \
             from claims storage, not modeled by mycelix-knowledge's DHT"
                .to_string(),
        ))
    }

    async fn update_quantum_anchor_pointer(&self, _id: uuid::Uuid, _cid: &str) -> Result<()> {
        Err(Error::Storage(
            "not supported by KnowledgeDhtStorage: Arweave anchoring is a separate concern \
             from claims storage, not modeled by mycelix-knowledge's DHT"
                .to_string(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::claims::ClaimContent;

    fn sample_content() -> ClaimContent {
        ClaimContent {
            dataset_hash: "sha256:abcdef".to_string(),
            description: "Water boils at 100C at sea level".to_string(),
            category: "physics".to_string(),
            keywords: vec!["thermodynamics".to_string(), "phase-transition".to_string()],
            storage_ref: None,
            reproducibility_score: None,
            license: None,
        }
    }

    #[test]
    fn strip_ws_scheme_handles_both_forms() {
        assert_eq!(strip_ws_scheme("ws://localhost:8888"), "localhost:8888");
        assert_eq!(strip_ws_scheme("wss://localhost:8888"), "localhost:8888");
        assert_eq!(strip_ws_scheme("localhost:8888"), "localhost:8888");
    }

    #[test]
    fn epistemic_tier_from_score_matches_canonical_e0_e4_boundaries() {
        assert_eq!(epistemic_tier_from_score(0.0), EpistemicTier::E0);
        assert_eq!(epistemic_tier_from_score(0.2), EpistemicTier::E1);
        assert_eq!(epistemic_tier_from_score(0.5), EpistemicTier::E2);
        assert_eq!(epistemic_tier_from_score(0.7), EpistemicTier::E3);
        assert_eq!(epistemic_tier_from_score(1.0), EpistemicTier::E4);
    }

    #[test]
    fn to_knowledge_claim_uses_the_connected_agents_did_not_the_desci_creator() {
        let claim = DesciClaim::new(
            EpistemicTier::E3,
            sample_content(),
            "alice-local-id".to_string(),
        );
        let kc = knowledge_claim_from_desci_claim(&claim, "did:mycelix:uhCAkREALAGENT".to_string());

        assert_eq!(kc.author, "did:mycelix:uhCAkREALAGENT");
        assert!(
            kc.sources
                .iter()
                .any(|s| s == "desci-creator:alice-local-id"),
            "original creator should be preserved as provenance context: {:?}",
            kc.sources
        );
        assert_eq!(kc.id, claim.id.to_string());
        assert_eq!(kc.content, claim.content.description);
    }

    #[test]
    fn desci_claim_round_trips_through_knowledge_claim_mapping() {
        // E4 deliberately does NOT match the default EpistemicPosition's
        // empirical score (0.5, which buckets to E2) -- proves the tier is
        // preserved exactly via the desci-tier tag, not re-derived from
        // the (independent) position score.
        let original = DesciClaim::new(EpistemicTier::E4, sample_content(), "bob".to_string());
        let kc = knowledge_claim_from_desci_claim(&original, "did:mycelix:uhCAkBOB".to_string());
        let restored = KnowledgeDhtStorage::from_knowledge_claim(kc).unwrap();

        assert_eq!(restored.id, original.id);
        assert_eq!(restored.content.description, original.content.description);
        assert_eq!(restored.content.category, original.content.category);
        assert_eq!(restored.content.keywords, original.content.keywords);
        assert_eq!(restored.creator, "bob");
        assert_eq!(restored.epistemic_tier, EpistemicTier::E4);
    }

    /// Real end-to-end verification against a live conductor -- not run in
    /// normal CI (no conductor available there), but exercises the actual
    /// holochain_client connection, a real submit_claim/get_claim zome call,
    /// and the full store()/retrieve() StorageBackend path together.
    ///
    /// To run: install mycelix-knowledge onto a conductor (see
    /// mycelix-workspace/scripts/start-conductor.sh for a disposable one,
    /// NOT the shared 33800/8888 conductor -- see
    /// .claude/rules/CONCURRENT_SESSIONS.md and the root CLAUDE.md
    /// "Conductor hygiene" section for why), then:
    ///   cargo test -p mycelix-desci-core --features holochain \
    ///     --target x86_64-unknown-linux-gnu \
    ///     live_conductor_round_trip -- --ignored --nocapture
    #[tokio::test]
    #[ignore = "requires a live Holochain conductor with mycelix-knowledge installed"]
    async fn live_conductor_round_trip() {
        use crate::storage::StorageBackend;

        let admin_addr =
            std::env::var("DESCI_TEST_ADMIN_ADDR").unwrap_or_else(|_| "localhost:4444".to_string());
        let app_addr =
            std::env::var("DESCI_TEST_APP_ADDR").unwrap_or_else(|_| "localhost:4445".to_string());

        let storage = KnowledgeDhtStorage::connect(&admin_addr, &app_addr, "mycelix-knowledge")
            .await
            .expect("connect to live conductor");

        let claim = DesciClaim::new(
            EpistemicTier::E3,
            sample_content(),
            "live-conductor-test".to_string(),
        );
        let claim_id = claim.id.to_string();

        let stored_cid = storage.store(&claim).await.expect("store claim");
        assert_eq!(stored_cid, claim_id);

        let retrieved = storage.retrieve(&claim_id).await.expect("retrieve claim");
        assert_eq!(retrieved.id, claim.id);
        assert_eq!(retrieved.content.description, claim.content.description);
        assert_eq!(retrieved.epistemic_tier, EpistemicTier::E3);

        assert!(storage.exists(&claim_id).await.expect("exists check"));
        assert!(
            !storage
                .exists("nonexistent-claim-id")
                .await
                .expect("exists check for missing claim")
        );

        println!("Live conductor round-trip succeeded for claim {claim_id}");
    }
}
