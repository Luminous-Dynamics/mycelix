// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Non-blocking publication-evidence status for the latest resident action.
//!
//! This is separate from the source-chain-saved toast. It displays only evidence
//! returned by the authoring conductor and always includes observation freshness
//! and a short action reference; it does not imply global synchronization.

use leptos::prelude::*;

use crate::action_feedback::{publication_evidence_copy, PublicationEvidence};
use crate::hearth_context::use_hearth;

fn evidence_class(evidence: PublicationEvidence) -> &'static str {
    match evidence {
        PublicationEvidence::Checking => "publication-evidence--checking",
        PublicationEvidence::Unavailable | PublicationEvidence::NotAvailableToCaller => {
            "publication-evidence--unknown"
        }
        PublicationEvidence::NoOperationData { .. } => "publication-evidence--unknown",
        PublicationEvidence::ReceiptsObserved {
            rejected_operation_count,
            abandoned_operation_count,
            ..
        } if rejected_operation_count > 0 || abandoned_operation_count > 0 => {
            "publication-evidence--attention"
        }
        PublicationEvidence::ReceiptsObserved {
            operation_count,
            complete_operation_count,
            ..
        } if operation_count > 0 && operation_count == complete_operation_count => {
            "publication-evidence--thresholds"
        }
        PublicationEvidence::ReceiptsObserved { .. } => "publication-evidence--progress",
    }
}

fn now_unix_seconds() -> u64 {
    let now_ms = js_sys::Date::now();
    if !now_ms.is_finite() || now_ms < 0.0 {
        0
    } else {
        (now_ms / 1_000.0).floor() as u64
    }
}

fn evidence_checked_at(evidence: PublicationEvidence) -> Option<u64> {
    match evidence {
        PublicationEvidence::NoOperationData { checked_at_unix_seconds }
        | PublicationEvidence::ReceiptsObserved { checked_at_unix_seconds, .. } => {
            Some(checked_at_unix_seconds)
        }
        _ => None,
    }
}

fn format_timestamp(unix_seconds: u64) -> String {
    let date = js_sys::Date::new(&wasm_bindgen::JsValue::from_f64(
        unix_seconds as f64 * 1_000.0,
    ));
    date.to_iso_string()
        .as_string()
        .unwrap_or_else(|| "timestamp unavailable".into())
}

#[component]
pub fn PublicationEvidenceStatus() -> impl IntoView {
    let publication = use_hearth().publication_evidence;

    view! {
        <Show
            when=move || publication.get().is_some()
            fallback=|| ()
        >
            {move || {
                publication.get().map(|notice| {
                    let class = evidence_class(notice.evidence);
                    let copy = publication_evidence_copy(notice.evidence, now_unix_seconds());
                    let checked_at = evidence_checked_at(notice.evidence)
                        .map(format_timestamp)
                        .unwrap_or_else(|| "observation time unavailable".into());
                    let subject = notice.subject;
                    let action_ref = notice.action_hash_prefix;

                    view! {
                        <aside
                            class=format!("publication-evidence {class}")
                            role="status"
                            aria-live="polite"
                            aria-atomic="true"
                            aria-label="publication evidence for the latest action"
                        >
                            <div class="publication-evidence__copy">
                                <strong>{format!("Publication evidence · {subject}")}</strong>
                                <p>{copy}</p>
                                <small>{format!("Action reference: {action_ref}… · Observed at {checked_at}")}</small>
                            </div>
                        </aside>
                    }
                })
            }}
        </Show>
    }
}

#[cfg(test)]
mod tests {
    use super::evidence_class;
    use crate::action_feedback::PublicationEvidence;

    #[test]
    fn publication_state_classes_do_not_conflate_unknown_with_complete() {
        assert_eq!(
            evidence_class(PublicationEvidence::Unavailable),
            "publication-evidence--unknown"
        );
        assert_eq!(
            evidence_class(PublicationEvidence::NoOperationData {
                checked_at_unix_seconds: 10
            }),
            "publication-evidence--unknown"
        );
        assert_eq!(
            evidence_class(PublicationEvidence::ReceiptsObserved {
                operation_count: 2,
                receipt_count: 1,
                valid_receipt_count: 1,
                rejected_receipt_count: 0,
                abandoned_receipt_count: 0,
                complete_operation_count: 0,
                rejected_operation_count: 0,
                abandoned_operation_count: 0,
                checked_at_unix_seconds: 10,
            }),
            "publication-evidence--progress"
        );
        assert_eq!(
            evidence_class(PublicationEvidence::ReceiptsObserved {
                operation_count: 2,
                receipt_count: 10,
                valid_receipt_count: 10,
                rejected_receipt_count: 0,
                abandoned_receipt_count: 0,
                complete_operation_count: 2,
                rejected_operation_count: 0,
                abandoned_operation_count: 0,
                checked_at_unix_seconds: 10,
            }),
            "publication-evidence--thresholds"
        );
    }

    #[test]
    fn rejection_or_abandonment_has_attention_class_even_if_thresholds_complete() {
        for (rejected, abandoned) in [(1, 0), (0, 1)] {
            assert_eq!(
                evidence_class(PublicationEvidence::ReceiptsObserved {
                    operation_count: 1,
                    receipt_count: 5,
                    valid_receipt_count: 3,
                    rejected_receipt_count: rejected,
                    abandoned_receipt_count: abandoned,
                    complete_operation_count: 1,
                    rejected_operation_count: rejected,
                    abandoned_operation_count: abandoned,
                    checked_at_unix_seconds: 10,
                }),
                "publication-evidence--attention"
            );
        }
    }
}
