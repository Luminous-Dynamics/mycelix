use std::collections::BTreeSet;

use crate::partition::{
    canonical_reconciliation, reconcile_offline_reservations, verify_reconciliation,
    OfflineReservation, ReconciliationDisposition, ReconciliationDecision,
};

pub const ORGANIZATION_COUNT: usize = 24;
pub const HUB_COUNT: usize = 8;
pub const SKU_CATALOG_SIZE: usize = 2_048;
pub const ORDER_COUNT: usize = 12_000;
pub const HOTSPOT_ORDER_COUNT: usize = 64;
pub const HOTSPOT_HUB_INDEX: usize = 0;
pub const HOTSPOT_SKU_INDEX: usize = 0;
pub const GENERATOR_VERSION: &str = "xorshift64star-v1";
pub const SKU_MAPPING_VERSION: &str = "dedicated-zero-plus-stride-997-mod-2047-v1";
pub const HOTSPOT_MIN_QUANTITY: u64 = 1;
pub const HOTSPOT_MAX_QUANTITY: u64 = 3;
pub const ORDINARY_MIN_QUANTITY: u64 = 1;
pub const ORDINARY_MAX_QUANTITY: u64 = 4;
pub const SCENARIO_SEED: u64 = 1_511_506_142;
pub const SNAPSHOT_OBSERVED_AT_MS: u64 = 10_000;
pub const SNAPSHOT_VALID_UNTIL_MS: u64 = 11_000;
pub const RECONCILIATION_NOW_MS: u64 = 10_100;
pub const REQUEST_OBSERVED_AT_MS: u64 = 10_020;
pub const RESERVATION_LEASE_UNTIL_MS: u64 = 10_800;

const COMMITTED_MANIFEST: &str = include_str!("../scenario-manifest.json");

fn expected_manifest_json() -> String {
    format!(
        "{{\"schema\":\"logistics-commons-s0/frozen-scenario-manifest-v1\",\"generator\":\"{}\",\"sku_mapping\":\"{}\",\"seed\":{},\"organization_count\":{},\"hub_count\":{},\"sku_catalog_size\":{},\"order_count\":{},\"hotspot_order_count\":{},\"hotspot_hub_index\":{},\"hotspot_sku_index\":{},\"hotspot_min_quantity\":{},\"hotspot_max_quantity\":{},\"ordinary_min_quantity\":{},\"ordinary_max_quantity\":{},\"snapshot_observed_at_ms\":{},\"snapshot_valid_until_ms\":{},\"reconciliation_now_ms\":{},\"request_observed_at_ms\":{},\"reservation_lease_until_ms\":{}}}",
        GENERATOR_VERSION,
        SKU_MAPPING_VERSION,
        SCENARIO_SEED,
        ORGANIZATION_COUNT,
        HUB_COUNT,
        SKU_CATALOG_SIZE,
        ORDER_COUNT,
        HOTSPOT_ORDER_COUNT,
        HOTSPOT_HUB_INDEX,
        HOTSPOT_SKU_INDEX,
        HOTSPOT_MIN_QUANTITY,
        HOTSPOT_MAX_QUANTITY,
        ORDINARY_MIN_QUANTITY,
        ORDINARY_MAX_QUANTITY,
        SNAPSHOT_OBSERVED_AT_MS,
        SNAPSHOT_VALID_UNTIL_MS,
        RECONCILIATION_NOW_MS,
        REQUEST_OBSERVED_AT_MS,
        RESERVATION_LEASE_UNTIL_MS,
    )
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FrozenScenarioRun {
    pub requests: Vec<OfflineReservation>,
    pub decisions: Vec<ReconciliationDecision>,
    pub independent_violations: Vec<String>,
    pub manifest_matches_generator: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct DeterministicRng {
    state: u64,
}

impl DeterministicRng {
    fn new(seed: u64) -> Self {
        // Xorshift64* is used only to produce reproducible synthetic inputs, never for security.
        Self { state: seed.max(1) }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn index(&mut self, upper: usize) -> usize {
        (self.next_u64() % upper as u64) as usize
    }
}

fn snapshot_available(hub_index: usize, sku_index: usize) -> u64 {
    if hub_index == HOTSPOT_HUB_INDEX && sku_index == HOTSPOT_SKU_INDEX {
        // The dedicated hot spot forces the split-brain last-unit condition.
        1
    } else {
        1 + ((hub_index * 1_009 + sku_index * 313 + (SCENARIO_SEED as usize % 997)) % 16) as u64
    }
}

fn snapshot_revision(hub_index: usize, sku_index: usize) -> u64 {
    100 + ((hub_index * 977 + sku_index * 719 + (SCENARIO_SEED as usize % 7_919)) % 1_000_000) as u64
}

fn organization(index: usize) -> String {
    format!("coop-{index:02}")
}

fn hub(index: usize) -> String {
    format!("hub-{index:02}")
}

fn sku(index: usize) -> String {
    format!("sku-{index:04}")
}

impl FrozenScenarioRun {
    pub fn build() -> Self {
        let mut rng = DeterministicRng::new(SCENARIO_SEED);
        let mut requests = Vec::with_capacity(ORDER_COUNT);

        for order_index in 0..ORDER_COUNT {
            let hotspot = order_index < HOTSPOT_ORDER_COUNT;
            let org_index = if order_index < ORGANIZATION_COUNT {
                order_index
            } else {
                rng.index(ORGANIZATION_COUNT)
            };
            let hub_index = if hotspot {
                HOTSPOT_HUB_INDEX
            } else if order_index < HOTSPOT_ORDER_COUNT + HUB_COUNT {
                order_index - HOTSPOT_ORDER_COUNT
            } else {
                rng.index(HUB_COUNT)
            };
            // Reserve the hot-spot SKU exclusively for the 64 targeted contenders.
            // 997 is coprime to 2,047, so repeated values cover every other SKU.
            let sku_index = if hotspot {
                HOTSPOT_SKU_INDEX
            } else {
                1 + (((order_index - HOTSPOT_ORDER_COUNT) * 997
                    + (SCENARIO_SEED as usize % (SKU_CATALOG_SIZE - 1)))
                    % (SKU_CATALOG_SIZE - 1))
            };
            let quantity = if hotspot {
                HOTSPOT_MIN_QUANTITY + rng.index((HOTSPOT_MAX_QUANTITY - HOTSPOT_MIN_QUANTITY + 1) as usize) as u64
            } else {
                ORDINARY_MIN_QUANTITY + rng.index((ORDINARY_MAX_QUANTITY - ORDINARY_MIN_QUANTITY + 1) as usize) as u64
            };
            let participant_id = organization(org_index);
            let hub_id = hub(hub_index);
            let sku_id = sku(sku_index);

            requests.push(OfflineReservation {
                participant_id: participant_id.clone(),
                node_id: format!("edge-{org_index:02}-{}", order_index % 3),
                reservation_id: format!("order-{order_index:06}"),
                idempotency_key: format!("operation-{order_index:06}"),
                hub_id,
                sku_id,
                quantity,
                snapshot_revision: snapshot_revision(hub_index, sku_index),
                snapshot_available_units: snapshot_available(hub_index, sku_index),
                snapshot_observed_at_ms: SNAPSHOT_OBSERVED_AT_MS,
                snapshot_valid_until_ms: SNAPSHOT_VALID_UNTIL_MS,
                observed_at_ms: REQUEST_OBSERVED_AT_MS,
                lease_until_ms: RESERVATION_LEASE_UNTIL_MS,
            });
        }

        let decisions = reconcile_offline_reservations(&requests, RECONCILIATION_NOW_MS);
        let mut independent_violations =
            verify_reconciliation(&requests, RECONCILIATION_NOW_MS, &decisions);
        let expected_manifest = expected_manifest_json();
        let manifest_matches_generator = COMMITTED_MANIFEST.trim() == expected_manifest.as_str();
        if !manifest_matches_generator {
            independent_violations.push("frozen-manifest-does-not-match-generator".to_owned());
            independent_violations.sort();
        }

        Self {
            requests,
            decisions,
            independent_violations,
            manifest_matches_generator,
        }
    }

    pub fn organization_count(&self) -> usize {
        self.requests.iter().map(|r| r.participant_id.as_str()).collect::<BTreeSet<_>>().len()
    }

    pub fn hub_count(&self) -> usize {
        self.requests.iter().map(|r| r.hub_id.as_str()).collect::<BTreeSet<_>>().len()
    }

    pub fn observed_sku_count(&self) -> usize {
        self.requests.iter().map(|r| r.sku_id.as_str()).collect::<BTreeSet<_>>().len()
    }

    pub fn awaiting_recheck_count(&self) -> usize {
        self.decisions.iter().filter(|d|
            matches!(&d.disposition, ReconciliationDisposition::AwaitingAuthoritativeRecheck)
        ).count()
    }

    pub fn unresolved_conflict_count(&self) -> usize {
        self.decisions.iter().filter(|d|
            matches!(&d.disposition, ReconciliationDisposition::UnresolvedConflict { .. })
        ).count()
    }

    pub fn rejected_count(&self) -> usize {
        self.decisions.iter().filter(|d|
            matches!(&d.disposition, ReconciliationDisposition::Rejected { .. })
        ).count()
    }

    pub fn duplicate_alias_count(&self) -> usize {
        self.decisions.iter().filter(|d|
            matches!(&d.disposition, ReconciliationDisposition::DuplicateOf { .. })
        ).count()
    }

    pub fn hotspot_conflict_count(&self) -> usize {
        self.decisions.iter().filter(|d|
            d.reservation_id.strip_prefix("order-")
                .and_then(|suffix| suffix.parse::<usize>().ok())
                .is_some_and(|order| order < HOTSPOT_ORDER_COUNT)
                && matches!(
                    &d.disposition,
                    ReconciliationDisposition::UnresolvedConflict {
                        reason: "oversubscribed-offline-snapshot",
                        ..
                    }
                )
        ).count()
    }

    pub fn model_invariants_hold(&self) -> bool {
        self.manifest_matches_generator
            && self.requests.len() == ORDER_COUNT
            && self.decisions.len() == ORDER_COUNT
            && self.organization_count() == ORGANIZATION_COUNT
            && self.hub_count() == HUB_COUNT
            && self.observed_sku_count() == SKU_CATALOG_SIZE
            && self.independent_violations.is_empty()
            && self.hotspot_conflict_count() == HOTSPOT_ORDER_COUNT
            && self.awaiting_recheck_count() + self.unresolved_conflict_count()
                + self.rejected_count() + self.duplicate_alias_count() == ORDER_COUNT
    }

    /// Machine-readable summary is evidence about this synthetic model, not production logistics.
    pub fn summary_json(&self) -> String {
        format!(
            "{{\"record\":\"summary\",\"schema\":\"logistics-commons-s0/scenario-summary-v1\",\"seed\":{},\"organizations\":{},\"hubs\":{},\"sku_catalog_size\":{},\"observed_skus\":{},\"orders\":{},\"hotspot_orders\":{},\"hotspot_conflicts\":{},\"awaiting_authoritative_recheck\":{},\"unresolved_conflicts\":{},\"rejected\":{},\"duplicate_aliases\":{},\"independent_verifier_violations\":{},\"manifest_matches_generator\":{},\"model_invariants_hold\":{}}}",
            SCENARIO_SEED,
            self.organization_count(),
            self.hub_count(),
            SKU_CATALOG_SIZE,
            self.observed_sku_count(),
            self.requests.len(),
            HOTSPOT_ORDER_COUNT,
            self.hotspot_conflict_count(),
            self.awaiting_recheck_count(),
            self.unresolved_conflict_count(),
            self.rejected_count(),
            self.duplicate_alias_count(),
            self.independent_violations.len(),
            self.manifest_matches_generator,
            self.model_invariants_hold()
        )
    }

    /// The artifact includes the frozen manifest, every synthetic input, canonical decisions,
    /// and verifier summary. It is byte-comparable and suitable for an external SHA-256 receipt.
    pub fn canonical_artifact(&self) -> String {
        let mut output = String::new();
        output.push_str("logistics-commons-s0/frozen-artifact-v1\n");
        output.push_str(&COMMITTED_MANIFEST.trim().replace('\n', ""));
        output.push('\n');
        output.push_str("inputs-v1\tparticipant\tnode\treservation\tidempotency\thub\tsku\tquantity\trevision\tavailable\tobserved_at\tvalid_until\trequest_observed_at\tlease_until\n");

        let mut inputs = self.requests.iter().collect::<Vec<_>>();
        inputs.sort_by(|a, b| a.reservation_id.cmp(&b.reservation_id));
        for request in inputs {
            output.push_str(&crate::escape_field(&request.participant_id));
            output.push('\t');
            output.push_str(&crate::escape_field(&request.node_id));
            output.push('\t');
            output.push_str(&crate::escape_field(&request.reservation_id));
            output.push('\t');
            output.push_str(&crate::escape_field(&request.idempotency_key));
            output.push('\t');
            output.push_str(&crate::escape_field(&request.hub_id));
            output.push('\t');
            output.push_str(&crate::escape_field(&request.sku_id));
            output.push('\t');
            output.push_str(&request.quantity.to_string());
            output.push('\t');
            output.push_str(&request.snapshot_revision.to_string());
            output.push('\t');
            output.push_str(&request.snapshot_available_units.to_string());
            output.push('\t');
            output.push_str(&request.snapshot_observed_at_ms.to_string());
            output.push('\t');
            output.push_str(&request.snapshot_valid_until_ms.to_string());
            output.push('\t');
            output.push_str(&request.observed_at_ms.to_string());
            output.push('\t');
            output.push_str(&request.lease_until_ms.to_string());
            output.push('\n');
        }

        output.push_str(&canonical_reconciliation(&self.decisions));
        output.push_str(&self.summary_json());
        output.push('\n');
        if !self.independent_violations.is_empty() {
            output.push_str("{\"record\":\"violations\",\"items\":[");
            for (index, violation) in self.independent_violations.iter().enumerate() {
                if index > 0 {
                    output.push(',');
                }
                output.push('"');
                output.push_str(&crate::escape_field(violation));
                output.push('"');
            }
            output.push_str("]}\n");
        }
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn committed_manifest_matches_generator_constants() {
        let expected_manifest = expected_manifest_json();
        assert_eq!(COMMITTED_MANIFEST.trim(), expected_manifest.as_str());
    }

    #[test]
    fn frozen_corpus_covers_all_declared_dimensions_and_hotspot_conflicts() {
        let run = FrozenScenarioRun::build();
        assert_eq!(run.requests.len(), 12_000);
        assert_eq!(run.decisions.len(), 12_000);
        assert_eq!(run.organization_count(), 24);
        assert_eq!(run.hub_count(), 8);
        assert_eq!(run.observed_sku_count(), 2_048);
        assert_eq!(run.hotspot_conflict_count(), 64);
        assert!(run.independent_violations.is_empty(), "{:?}", run.independent_violations);
        assert!(run.model_invariants_hold(), "{}", run.summary_json());
    }

    #[test]
    fn frozen_hotspot_has_exactly_the_declared_contender_set() {
        let run = FrozenScenarioRun::build();
        let hotspot_hub = hub(HOTSPOT_HUB_INDEX);
        let hotspot_sku = sku(HOTSPOT_SKU_INDEX);
        let hotspot_rows = run.requests.iter()
            .filter(|request| request.hub_id == hotspot_hub && request.sku_id == hotspot_sku)
            .collect::<Vec<_>>();
        assert_eq!(hotspot_rows.len(), HOTSPOT_ORDER_COUNT);

        let expected_ids = (0..HOTSPOT_ORDER_COUNT)
            .map(|index| format!("order-{index:06}"))
            .collect::<Vec<_>>();
        let mut actual_ids = hotspot_rows.iter()
            .map(|request| request.reservation_id.clone())
            .collect::<Vec<_>>();
        actual_ids.sort();
        assert_eq!(actual_ids, expected_ids);

        for decision in run.decisions.iter().filter(|decision|
            expected_ids.binary_search(&decision.reservation_id).is_ok())
        {
            assert!(matches!(
                &decision.disposition,
                ReconciliationDisposition::UnresolvedConflict {
                    reason: "oversubscribed-offline-snapshot",
                    contenders
                } if contenders == &expected_ids
            ), "hotspot decision did not carry the exhaustive frozen contender set: {:?}", decision);
        }
    }

    #[test]
    fn large_corpus_reconciliation_is_input_permutation_invariant() {
        let run = FrozenScenarioRun::build();
        let mut reversed = run.requests.clone();
        reversed.reverse();
        let replayed = reconcile_offline_reservations(&reversed, RECONCILIATION_NOW_MS);
        assert_eq!(
            canonical_reconciliation(&run.decisions),
            canonical_reconciliation(&replayed)
        );
        assert!(verify_reconciliation(&reversed, RECONCILIATION_NOW_MS, &replayed).is_empty());
    }

    #[test]
    fn repeated_frozen_corpus_is_byte_identical() {
        let first = FrozenScenarioRun::build().canonical_artifact();
        let second = FrozenScenarioRun::build().canonical_artifact();
        assert_eq!(first.as_bytes(), second.as_bytes());
        assert!(first.contains("\"seed\":1511506142"));
        assert!(first.contains("\"model_invariants_hold\":true"));
        assert!(first.contains("inputs-v1\tparticipant\tnode\treservation"));
    }

    #[test]
    fn violation_record_escapes_json_quotes_and_backslashes() {
        let mut run = FrozenScenarioRun::build();
        run.independent_violations = vec![r#"quote":"\path"#.to_owned()];
        let artifact = run.canonical_artifact();
        assert!(artifact.lines().any(|line| {
            line == r#"{"record":"violations","items":["quote%22:%22%5Cpath"]}"#
        }));
    }

    #[test]
    fn canonical_artifact_contains_every_input_and_decision() {
        let run = FrozenScenarioRun::build();
        let artifact = run.canonical_artifact();
        let input_records = artifact.lines().filter(|line| {
            line.starts_with("coop-")
                && line.split('\t').count() == 13
        }).count();
        let decision_records = artifact.lines().filter(|line| {
            line.starts_with("coop-")
                && (line.contains("AWAITING_AUTHORITATIVE_RECHECK")
                    || line.contains("UNRESOLVED_CONFLICT:")
                    || line.contains("REJECTED:")
                    || line.contains("DUPLICATE_OF:"))
        }).count();
        assert_eq!(input_records, ORDER_COUNT);
        assert_eq!(decision_records, ORDER_COUNT);
    }
}
