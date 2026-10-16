// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Leptos consumer for the Hearth HDK 0.6 authoring-conductor publication API.
//!
//! The mutation's source-chain result is surfaced immediately by the caller.
//! This module performs one non-blocking follow-up receipt query and presents
//! only a validated, action-hash-bound summary. It does not poll or retry.

use leptos::prelude::RwSignal;
use mycelix_leptos_core::holochain_provider::HolochainCtx;
use serde::{Deserialize, Serialize};

use crate::action_feedback::{PublicationEvidence, PublicationEvidenceNotice};
use crate::record_bridge::WireRecord;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
enum WirePublicationEvidenceAvailability {
    NotAvailableToCaller,
    NoOperationData,
    Observed,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
struct WirePublicationOperationEvidence {
    operation_type: String,
    receipt_record_count: u32,
    valid_receipt_record_count: u32,
    rejected_receipt_record_count: u32,
    abandoned_receipt_record_count: u32,
    receipts_complete: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
struct WireActionPublicationEvidence {
    action_hash: Vec<u8>,
    checked_at_micros: i64,
    availability: WirePublicationEvidenceAvailability,
    operation_count: u32,
    receipt_record_count: u32,
    receipt_threshold_complete_operation_count: u32,
    rejected_operation_count: u32,
    all_receipt_thresholds_complete: bool,
    operations: Vec<WirePublicationOperationEvidence>,
}

#[derive(Debug, Serialize)]
struct GetActionPublicationEvidenceInput {
    action_hash: Vec<u8>,
}

fn now_unix_seconds() -> u64 {
    let now_ms = js_sys::Date::now();
    if !now_ms.is_finite() || now_ms < 0.0 {
        0
    } else {
        (now_ms / 1_000.0).floor() as u64
    }
}

fn checked_at_seconds(checked_at_micros: i64) -> Option<u64> {
    if checked_at_micros < 0 {
        None
    } else {
        Some((checked_at_micros as u64) / 1_000_000)
    }
}

fn map_report(
    requested_action_hash: &[u8],
    report: WireActionPublicationEvidence,
) -> PublicationEvidence {
    if report.action_hash.as_slice() != requested_action_hash {
        return PublicationEvidence::Unavailable;
    }

    let Some(checked_at_unix_seconds) = checked_at_seconds(report.checked_at_micros) else {
        return PublicationEvidence::Unavailable;
    };

    let operations_len = u32::try_from(report.operations.len()).unwrap_or(u32::MAX);
    let receipt_record_count = report.operations.iter().fold(0u32, |total, operation| {
        total.saturating_add(operation.receipt_record_count)
    });
    let complete_operation_count = u32::try_from(
        report.operations.iter().filter(|operation| operation.receipts_complete).count()
    )
    .unwrap_or(u32::MAX);
    let rejected_operation_count = u32::try_from(
        report.operations.iter().filter(|operation| operation.rejected_receipt_record_count > 0).count()
    )
    .unwrap_or(u32::MAX);

    // Fail closed on an internally inconsistent response. The UI must not
    // manufacture a success signal from a truncated or malformed DTO.
    let per_operation_counts_valid = report.operations.iter().all(|operation| {
        operation.receipt_record_count
            == operation
                .valid_receipt_record_count
                .saturating_add(operation.rejected_receipt_record_count)
                .saturating_add(operation.abandoned_receipt_record_count)
    });
    let expected_all_complete = !report.operations.is_empty()
        && report.operations.iter().all(|operation| operation.receipts_complete);
    if !per_operation_counts_valid
        || report.operation_count != operations_len
        || report.receipt_record_count != receipt_record_count
        || report.receipt_threshold_complete_operation_count != complete_operation_count
        || report.rejected_operation_count != rejected_operation_count
        || report.all_receipt_thresholds_complete != expected_all_complete
    {
        return PublicationEvidence::Unavailable;
    }

    match report.availability {
        WirePublicationEvidenceAvailability::NotAvailableToCaller => {
            if report.operation_count == 0 && report.receipt_record_count == 0 {
                PublicationEvidence::NotAvailableToCaller
            } else {
                PublicationEvidence::Unavailable
            }
        }
        WirePublicationEvidenceAvailability::NoOperationData => {
            if report.operations.is_empty()
                && report.operation_count == 0
                && report.receipt_record_count == 0
                && !report.all_receipt_thresholds_complete
            {
                PublicationEvidence::NoOperationData { checked_at_unix_seconds }
            } else {
                PublicationEvidence::Unavailable
            }
        }
        WirePublicationEvidenceAvailability::Observed => {
            if report.operations.is_empty() {
                return PublicationEvidence::Unavailable;
            }

            let valid_receipt_count = report.operations.iter().fold(0u32, |total, operation| {
                total.saturating_add(operation.valid_receipt_record_count)
            });
            let rejected_receipt_count = report.operations.iter().fold(0u32, |total, operation| {
                total.saturating_add(operation.rejected_receipt_record_count)
            });
            let abandoned_receipt_count = report.operations.iter().fold(0u32, |total, operation| {
                total.saturating_add(operation.abandoned_receipt_record_count)
            });
            let abandoned_operation_count = u32::try_from(
                report.operations.iter().filter(|operation| operation.abandoned_receipt_record_count > 0).count()
            )
            .unwrap_or(u32::MAX);

            PublicationEvidence::ReceiptsObserved {
                operation_count: report.operation_count,
                receipt_count: report.receipt_record_count,
                valid_receipt_count,
                rejected_receipt_count,
                abandoned_receipt_count,
                complete_operation_count: report.receipt_threshold_complete_operation_count,
                rejected_operation_count: report.rejected_operation_count,
                abandoned_operation_count,
                checked_at_unix_seconds,
            }
        }
    }
}

/// Do one follow-up query for the exact action returned by the user's own write.
/// The caller should emit the source-chain-saved toast before awaiting this
/// function. Receipt failure never rewrites the mutation's earlier result.
pub(crate) async fn check_and_store(
    hc: &HolochainCtx,
    publication_signal: RwSignal<Option<PublicationEvidenceNotice>>,
    record: &WireRecord,
    subject: &'static str,
) {
    let action_hash = record.signed_action.hashed.hash.clone();
    let action_hash_prefix = record.action_hash_b64().chars().take(10).collect::<String>();

    publication_signal.set(Some(PublicationEvidenceNotice {
        subject: subject.to_string(),
        action_hash_prefix: action_hash_prefix.clone(),
        evidence: PublicationEvidence::Checking,
    }));

    if action_hash.len() != 39 {
        publication_signal.set(Some(PublicationEvidenceNotice {
            subject: subject.to_string(),
            action_hash_prefix,
            evidence: PublicationEvidence::Unavailable,
        }));
        return;
    }

    let result = hc
        .call_zome_default::<GetActionPublicationEvidenceInput, WireActionPublicationEvidence>(
            "hearth_bridge",
            "get_action_publication_evidence",
            &GetActionPublicationEvidenceInput {
                action_hash: action_hash.clone(),
            },
        )
        .await;

    let evidence = match result {
        Ok(report) => map_report(&action_hash, report),
        Err(_error) => PublicationEvidence::Unavailable,
    };

    publication_signal.set(Some(PublicationEvidenceNotice {
        subject: subject.to_string(),
        action_hash_prefix,
        evidence,
    }));
}

#[cfg(test)]
mod tests {
    use super::{
        checked_at_seconds, map_report, WireActionPublicationEvidence,
        WirePublicationEvidenceAvailability as Availability, WirePublicationOperationEvidence as Op,
    };
    use crate::action_feedback::PublicationEvidence;

    fn action_hash() -> Vec<u8> {
        vec![7; 39]
    }

    fn operation(
        receipt_count: u32,
        valid: u32,
        rejected: u32,
        abandoned: u32,
        complete: bool,
    ) -> Op {
        Op {
            operation_type: "StoreRecord".into(),
            receipt_record_count: receipt_count,
            valid_receipt_record_count: valid,
            rejected_receipt_record_count: rejected,
            abandoned_receipt_record_count: abandoned,
            receipts_complete: complete,
        }
    }

    fn report(
        availability: Availability,
        operations: Vec<Op>,
    ) -> WireActionPublicationEvidence {
        let operation_count = operations.len() as u32;
        let receipt_record_count = operations.iter().map(|op| op.receipt_record_count).sum();
        let complete_count = operations.iter().filter(|op| op.receipts_complete).count() as u32;
        let rejected_operation_count = operations
            .iter()
            .filter(|op| op.rejected_receipt_record_count > 0)
            .count() as u32;
        let all_complete = !operations.is_empty() && operations.iter().all(|op| op.receipts_complete);
        WireActionPublicationEvidence {
            action_hash: action_hash(),
            checked_at_micros: 1_700_000_000_000_000,
            availability,
            operation_count,
            receipt_record_count,
            receipt_threshold_complete_operation_count: complete_count,
            rejected_operation_count,
            all_receipt_thresholds_complete: all_complete,
            operations,
        }
    }

    #[test]
    fn empty_operation_data_is_not_vacuously_complete() {
        let mapped = map_report(&action_hash(), report(Availability::NoOperationData, vec![]));
        assert!(matches!(mapped, PublicationEvidence::NoOperationData { .. }));
    }

    #[test]
    fn observed_partial_receipts_preserve_each_validation_status() {
        let mapped = map_report(
            &action_hash(),
            report(
                Availability::Observed,
                vec![
                    operation(3, 1, 1, 1, false),
                    operation(1, 1, 0, 0, true),
                ],
            ),
        );
        assert_eq!(
            mapped,
            PublicationEvidence::ReceiptsObserved {
                operation_count: 2,
                receipt_count: 4,
                valid_receipt_count: 2,
                rejected_receipt_count: 1,
                abandoned_receipt_count: 1,
                complete_operation_count: 1,
                rejected_operation_count: 1,
                abandoned_operation_count: 1,
                checked_at_unix_seconds: 1_700_000_000,
            }
        );
    }

    #[test]
    fn report_for_a_different_action_hash_is_unavailable() {
        let mut bad = report(Availability::Observed, vec![operation(1, 1, 0, 0, true)]);
        bad.action_hash = vec![8; 39];
        assert_eq!(map_report(&action_hash(), bad), PublicationEvidence::Unavailable);
    }

    #[test]
    fn inconsistent_totals_fail_closed() {
        let mut bad = report(Availability::Observed, vec![operation(1, 1, 0, 0, true)]);
        bad.receipt_record_count = 0;
        assert_eq!(map_report(&action_hash(), bad), PublicationEvidence::Unavailable);
    }

    #[test]
    fn observed_cannot_mean_zero_operations() {
        let mapped = map_report(&action_hash(), report(Availability::Observed, vec![]));
        assert_eq!(mapped, PublicationEvidence::Unavailable);
    }

    #[test]
    fn not_available_to_caller_does_not_expose_counts() {
        let mapped = map_report(
            &action_hash(),
            report(Availability::NotAvailableToCaller, vec![]),
        );
        assert_eq!(mapped, PublicationEvidence::NotAvailableToCaller);
    }

    #[test]
    fn negative_timestamp_fails_closed() {
        assert_eq!(checked_at_seconds(-1), None);
    }

    #[test]
    fn dto_round_trips_through_msgpack() {
        let original = report(Availability::Observed, vec![operation(2, 2, 0, 0, true)]);
        let encoded = rmp_serde::to_vec_named(&original).unwrap();
        let decoded: WireActionPublicationEvidence = rmp_serde::from_slice(&encoded).unwrap();
        assert_eq!(decoded, original);
        assert!(matches!(
            map_report(&action_hash(), decoded),
            PublicationEvidence::ReceiptsObserved { complete_operation_count: 1, .. }
        ));
    }
}
