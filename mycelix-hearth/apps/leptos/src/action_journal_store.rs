// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Versioned, privacy-minimized browser journal for Hearth action attempts.
//!
//! This module owns local evidence only. A committed Submitted row is still
//! ambiguous after restart and must be recovered as OutcomeUnknown. Browser
//! storage may be evicted or cleared by the user; IndexedDB strict durability is
//! a transaction hint, not a guarantee against every storage/device failure.
//!
//! The adapter deliberately uses native web-sys bindings rather than adding a
//! third-party IndexedDB dependency. Mutations are written only after an
//! IndexedDB readwrite transaction reports completion.

use crate::action_lifecycle::{transition_allowed, ActionKind, AttemptStatus};
use js_sys::{Array, Function, Object, Promise, Reflect};
use serde::{Deserialize, Serialize};
use std::cell::{Cell, RefCell};
use std::rc::Rc;
use wasm_bindgen::{closure::Closure, JsCast, JsValue};
use wasm_bindgen_futures::JsFuture;
use web_sys::{Event, IdbDatabase, IdbObjectStore, IdbOpenDbRequest, IdbRequest, IdbTransaction};

const DATABASE_NAME: &str = "mycelix-hearth-action-journal";
const DATABASE_VERSION: u32 = 1;
const STORE_NAME: &str = "attempts";
const SCHEMA_VERSION: u16 = 1;

/// Safe, explicit failures. Raw browser/record content is never included.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JournalError {
    IndexedDbUnavailable,
    OpenFailed,
    MissingObjectStore,
    StrictDurabilityUnavailable,
    RequestFailed,
    CommitFailed,
    SerializationFailed,
    CorruptRecord,
    UnsupportedSchemaVersion(u16),
    InvalidRecord,
    DuplicateOperation,
    MissingOperation,
    InvalidTransition {
        from: AttemptStatus,
        to: AttemptStatus,
    },
    TimestampRegressed,
}

/// The actual origin-storage posture observed when the adapter opens.
///
/// `persistent_storage_granted` is None when the browser does not expose a
/// usable StorageManager query. False means browser-managed storage can still
/// be evicted under the browser's policy. True is not a promise against user
/// deletion, device loss, or every class of storage failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StoragePosture {
    pub strict_transaction_option_observed: bool,
    pub persistent_storage_granted: Option<bool>,
}

/// Only minimum metadata needed to track/recover one action. Do not add names,
/// message bodies, raw member keys, or arbitrary action payloads here.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JournalEntryV1 {
    schema_version: u16,
    operation_id: String,
    hearth_scope_fingerprint: String,
    target_fingerprint: String,
    action: ActionKind,
    status: AttemptStatus,
    created_at_unix_ms: u64,
    updated_at_unix_ms: u64,
}

impl JournalEntryV1 {
    pub fn new(
        operation_id: impl Into<String>,
        action: ActionKind,
        hearth_scope_fingerprint: impl Into<String>,
        target_fingerprint: impl Into<String>,
        created_at_unix_ms: u64,
    ) -> Result<Self, JournalError> {
        let entry = Self {
            schema_version: SCHEMA_VERSION,
            operation_id: operation_id.into(),
            hearth_scope_fingerprint: hearth_scope_fingerprint.into(),
            target_fingerprint: target_fingerprint.into(),
            action,
            status: AttemptStatus::Prepared,
            created_at_unix_ms,
            updated_at_unix_ms: created_at_unix_ms,
        };
        entry.validate()?;
        Ok(entry)
    }

    pub fn operation_id(&self) -> &str {
        &self.operation_id
    }

    pub const fn action(&self) -> ActionKind {
        self.action
    }

    pub const fn status(&self) -> AttemptStatus {
        self.status
    }

    pub fn target_fingerprint(&self) -> &str {
        &self.target_fingerprint
    }

    pub fn hearth_scope_fingerprint(&self) -> &str {
        &self.hearth_scope_fingerprint
    }

    pub const fn created_at_unix_ms(&self) -> u64 {
        self.created_at_unix_ms
    }

    pub const fn updated_at_unix_ms(&self) -> u64 {
        self.updated_at_unix_ms
    }

    fn validate(&self) -> Result<(), JournalError> {
        if self.schema_version != SCHEMA_VERSION {
            return Err(JournalError::UnsupportedSchemaVersion(self.schema_version));
        }
        if self.operation_id.is_empty()
            || self.operation_id.len() > 80
            || !self.operation_id.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-')
            || !valid_sha256_fingerprint(&self.hearth_scope_fingerprint)
            || !valid_sha256_fingerprint(&self.target_fingerprint)
            || self.updated_at_unix_ms < self.created_at_unix_ms
        {
            return Err(JournalError::InvalidRecord);
        }
        Ok(())
    }

    fn transition(
        &mut self,
        next: AttemptStatus,
        now_unix_ms: u64,
    ) -> Result<(), JournalError> {
        if !transition_allowed(self.status, next) {
            return Err(JournalError::InvalidTransition { from: self.status, to: next });
        }
        if now_unix_ms < self.updated_at_unix_ms {
            return Err(JournalError::TimestampRegressed);
        }
        if self.status != next {
            self.status = next;
            self.updated_at_unix_ms = now_unix_ms;
        }
        self.validate()
    }
}

fn valid_sha256_fingerprint(value: &str) -> bool {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return false;
    };
    hex.len() == 64
        && hex
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

fn decode_record(json: &str) -> Result<JournalEntryV1, JournalError> {
    let entry: JournalEntryV1 =
        serde_json::from_str(json).map_err(|_| JournalError::CorruptRecord)?;
    entry.validate()?;
    Ok(entry)
}

/// Persistence contract used by the live dispatch gate and deterministic tests.
/// A successful write means the adapter observed the transaction's completion
/// event. It does not prove the browser can never clear the origin's storage.
pub trait ActionJournalStore {
    async fn insert_prepared(&self, entry: JournalEntryV1) -> Result<(), JournalError>;

    async fn transition(
        &self,
        operation_id: &str,
        next: AttemptStatus,
        now_unix_ms: u64,
    ) -> Result<JournalEntryV1, JournalError>;

    async fn list(&self) -> Result<Vec<JournalEntryV1>, JournalError>;

    /// Convert each recovered Submitted attempt to OutcomeUnknown and persist
    /// every conversion before returning any recovered history to the caller.
    async fn recover_after_restart(
        &self,
        now_unix_ms: u64,
    ) -> Result<Vec<JournalEntryV1>, JournalError> {
        let entries = self.list().await?;
        for entry in &entries {
            if entry.status() == AttemptStatus::Submitted {
                self.transition(
                    entry.operation_id(),
                    AttemptStatus::OutcomeUnknown,
                    now_unix_ms.max(entry.updated_at_unix_ms()),
                )
                .await?;
            }
        }
        self.list().await
    }
}

/// Browser-backed journal. Available only when IndexedDB and strict transaction
/// durability options are present; unsupported capability fails closed.
#[derive(Clone)]
pub struct IndexedDbJournal {
    database: IdbDatabase,
    posture: StoragePosture,
}

impl IndexedDbJournal {
    pub async fn open() -> Result<Self, JournalError> {
        let window = web_sys::window().ok_or(JournalError::IndexedDbUnavailable)?;
        let factory = window
            .indexed_db()
            .map_err(|_| JournalError::IndexedDbUnavailable)?
            .ok_or(JournalError::IndexedDbUnavailable)?;
        let request = factory
            .open_with_u32(DATABASE_NAME, DATABASE_VERSION)
            .map_err(|_| JournalError::OpenFailed)?;

        // Schema changes must happen synchronously within the version-change event.
        let upgrade_request = request.clone();
        let upgrade = Closure::once_into_js(move |_event: Event| {
            if let Ok(value) = upgrade_request.result() {
                if let Ok(database) = value.dyn_into::<IdbDatabase>() {
                    if !database.object_store_names().contains(STORE_NAME) {
                        let _ = database.create_object_store(STORE_NAME);
                    }
                }
            }
        });
        request.set_onupgradeneeded(Some(upgrade.unchecked_ref()));

        let opened = JsFuture::from(open_database_request(request))
            .await
            .map_err(|_| JournalError::OpenFailed)?;
        let database = opened
            .dyn_into::<IdbDatabase>()
            .map_err(|_| JournalError::OpenFailed)?;

        if !database.object_store_names().contains(STORE_NAME) {
            database.close();
            return Err(JournalError::MissingObjectStore);
        }

        // Test the requested strict transaction option before this adapter is
        // admitted to a dispatch gate. Aborting this empty probe writes nothing.
        let probe = strict_readwrite_transaction(&database)?;
        let _ = probe.abort();

        let posture = StoragePosture {
            strict_transaction_option_observed: true,
            persistent_storage_granted: query_storage_persisted().await,
        };
        Ok(Self { database, posture })
    }

    pub const fn posture(&self) -> StoragePosture {
        self.posture
    }

    async fn write_entry(
        &self,
        entry: &JournalEntryV1,
        insert_only: bool,
    ) -> Result<(), JournalError> {
        entry.validate()?;
        let json =
            serde_json::to_string(entry).map_err(|_| JournalError::SerializationFailed)?;
        let tx = strict_readwrite_transaction(&self.database)?;
        let done = transaction_finished(&tx);
        let store = tx
            .object_store(STORE_NAME)
            .map_err(|_| JournalError::RequestFailed)?;
        let key = JsValue::from_str(entry.operation_id());
        let value = JsValue::from_str(&json);
        let request = if insert_only {
            store.add_with_key(&value, &key)
        } else {
            store.put_with_key(&value, &key)
        }
        .map_err(|_| {
            let _ = tx.abort();
            JournalError::RequestFailed
        })?;

        let request_result = JsFuture::from(request_finished(request)).await;
        if request_result.is_err() {
            // Drain the completion promise so failures are observed from the
            // transaction too; no successful return occurs on request success alone.
            let _ = JsFuture::from(done).await;
            return Err(if insert_only {
                JournalError::DuplicateOperation
            } else {
                JournalError::RequestFailed
            });
        }
        JsFuture::from(done)
            .await
            .map_err(|_| JournalError::CommitFailed)?;
        Ok(())
    }
}

impl ActionJournalStore for IndexedDbJournal {
    async fn insert_prepared(&self, entry: JournalEntryV1) -> Result<(), JournalError> {
        if entry.status() != AttemptStatus::Prepared {
            return Err(JournalError::InvalidRecord);
        }
        self.write_entry(&entry, true).await
    }

    async fn transition(
        &self,
        operation_id: &str,
        next: AttemptStatus,
        now_unix_ms: u64,
    ) -> Result<JournalEntryV1, JournalError> {
        let existing = self
            .list()
            .await?
            .into_iter()
            .find(|entry| entry.operation_id() == operation_id)
            .ok_or(JournalError::MissingOperation)?;
        let mut changed = existing;
        changed.transition(next, now_unix_ms)?;
        self.write_entry(&changed, false).await?;
        Ok(changed)
    }

    async fn list(&self) -> Result<Vec<JournalEntryV1>, JournalError> {
        let tx = self
            .database
            .transaction_with_str(STORE_NAME)
            .map_err(|_| JournalError::RequestFailed)?;
        let done = transaction_finished(&tx);
        let store = tx
            .object_store(STORE_NAME)
            .map_err(|_| JournalError::RequestFailed)?;
        let request = store.get_all().map_err(|_| JournalError::RequestFailed)?;
        let values = JsFuture::from(request_finished(request))
            .await
            .map_err(|_| JournalError::RequestFailed)?;
        JsFuture::from(done)
            .await
            .map_err(|_| JournalError::CommitFailed)?;

        let values = Array::from(&values);
        let mut entries = Vec::with_capacity(values.length() as usize);
        for value in values.iter() {
            let json = value.as_string().ok_or(JournalError::CorruptRecord)?;
            entries.push(decode_record(&json)?);
        }
        entries.sort_by(|a, b| {
            a.created_at_unix_ms()
                .cmp(&b.created_at_unix_ms())
                .then_with(|| a.operation_id().cmp(b.operation_id()))
        });
        Ok(entries)
    }
}

fn strict_readwrite_transaction(
    database: &IdbDatabase,
) -> Result<IdbTransaction, JournalError> {
    let receiver: &JsValue = database.as_ref();
    let transaction = Reflect::get(receiver, &JsValue::from_str("transaction"))
        .map_err(|_| JournalError::StrictDurabilityUnavailable)?
        .dyn_into::<Function>()
        .map_err(|_| JournalError::StrictDurabilityUnavailable)?;

    let stores = Array::new();
    stores.push(&JsValue::from_str(STORE_NAME));
    let options = Object::new();
    Reflect::set(
        &options,
        &JsValue::from_str("durability"),
        &JsValue::from_str("strict"),
    )
    .map_err(|_| JournalError::StrictDurabilityUnavailable)?;
    let arguments = Array::new();
    arguments.push(&stores);
    arguments.push(&JsValue::from_str("readwrite"));
    arguments.push(&options);

    let tx_value = transaction
        .apply(receiver, &arguments)
        .map_err(|_| JournalError::StrictDurabilityUnavailable)?;
    let tx = tx_value
        .dyn_into::<IdbTransaction>()
        .map_err(|_| JournalError::StrictDurabilityUnavailable)?;
    let durability = Reflect::get(tx.as_ref(), &JsValue::from_str("durability"))
        .ok()
        .and_then(|value| value.as_string());
    if durability.as_deref() != Some("strict") {
        let _ = tx.abort();
        return Err(JournalError::StrictDurabilityUnavailable);
    }
    Ok(tx)
}

fn open_database_request(request: IdbOpenDbRequest) -> Promise {
    Promise::new(&mut |resolve, reject| {
        let success_request = request.clone();
        let success_resolve = resolve.clone();
        let success = Closure::once_into_js(move |_event: Event| {
            match success_request.result() {
                Ok(value) => {
                    let _ = success_resolve.call1(&JsValue::UNDEFINED, &value);
                }
                Err(_) => {
                    let _ = success_resolve.call1(
                        &JsValue::UNDEFINED,
                        &JsValue::UNDEFINED,
                    );
                }
            }
        });
        request.set_onsuccess(Some(success.unchecked_ref()));

        let error_reject = reject.clone();
        let error = Closure::once_into_js(move |_event: Event| {
            let _ = error_reject.call1(
                &JsValue::UNDEFINED,
                &JsValue::from_str("IndexedDB open failed"),
            );
        });
        request.set_onerror(Some(error.unchecked_ref()));
    })
}

fn request_finished(request: IdbRequest) -> Promise {
    Promise::new(&mut |resolve, reject| {
        let success_request = request.clone();
        let success_resolve = resolve.clone();
        let success = Closure::once_into_js(move |_event: Event| {
            match success_request.result() {
                Ok(value) => {
                    let _ = success_resolve.call1(&JsValue::UNDEFINED, &value);
                }
                Err(_) => {
                    let _ = success_resolve.call1(
                        &JsValue::UNDEFINED,
                        &JsValue::UNDEFINED,
                    );
                }
            }
        });
        request.set_onsuccess(Some(success.unchecked_ref()));

        let error_reject = reject.clone();
        let error = Closure::once_into_js(move |_event: Event| {
            let _ = error_reject.call1(
                &JsValue::UNDEFINED,
                &JsValue::from_str("IndexedDB request failed"),
            );
        });
        request.set_onerror(Some(error.unchecked_ref()));
    })
}

fn transaction_finished(tx: &IdbTransaction) -> Promise {
    Promise::new(&mut |resolve, reject| {
        let complete_resolve = resolve.clone();
        let complete = Closure::once_into_js(move |_event: Event| {
            let _ = complete_resolve.call0(&JsValue::UNDEFINED);
        });
        tx.set_oncomplete(Some(complete.unchecked_ref()));

        let abort_reject = reject.clone();
        let abort = Closure::once_into_js(move |_event: Event| {
            let _ = abort_reject.call1(
                &JsValue::UNDEFINED,
                &JsValue::from_str("IndexedDB transaction aborted"),
            );
        });
        tx.set_onabort(Some(abort.unchecked_ref()));

        let error_reject = reject.clone();
        let error = Closure::once_into_js(move |_event: Event| {
            let _ = error_reject.call1(
                &JsValue::UNDEFINED,
                &JsValue::from_str("IndexedDB transaction failed"),
            );
        });
        tx.set_onerror(Some(error.unchecked_ref()));
    })
}

async fn query_storage_persisted() -> Option<bool> {
    let window = web_sys::window()?;
    let navigator = window.navigator();
    let storage = Reflect::get(navigator.as_ref(), &JsValue::from_str("storage")).ok()?;
    if storage.is_null() || storage.is_undefined() {
        return None;
    }
    let persisted_fn = Reflect::get(&storage, &JsValue::from_str("persisted"))
        .ok()?
        .dyn_into::<Function>()
        .ok()?;
    let result = persisted_fn.call0(&storage).ok()?.dyn_into::<Promise>().ok()?;
    JsFuture::from(result).await.ok()?.as_bool()
}

#[cfg(test)]
mod tests {
    use super::{decode_record, valid_sha256_fingerprint, JournalEntryV1, JournalError};
    use crate::action_lifecycle::{ActionKind, AttemptStatus};

    const FINGERPRINT: &str =
        "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    fn prepared() -> JournalEntryV1 {
        JournalEntryV1::new(
            "c0ffee00-0000-4000-8000-000000000001",
            ActionKind::CareCompletion,
            FINGERPRINT,
            FINGERPRINT,
            100,
        )
        .unwrap()
    }

    #[test]
    fn versioned_record_round_trip_contains_only_allowlisted_metadata() {
        let entry = prepared();
        let encoded = serde_json::to_string(&entry).unwrap();
        let decoded = decode_record(&encoded).unwrap();
        assert_eq!(decoded, entry);
        assert!(encoded.contains("care_completion"));
        assert!(!encoded.contains("member name"));
        assert!(!encoded.contains("gratitude message"));
        assert!(!encoded.contains("uhCAk"));
    }

    #[test]
    fn raw_or_noncanonical_identity_is_not_accepted_as_a_fingerprint() {
        assert!(valid_sha256_fingerprint(FINGERPRINT));
        assert!(!valid_sha256_fingerprint("A resident name"));
        assert!(!valid_sha256_fingerprint("sha256:ABCDEF"));
        assert!(!valid_sha256_fingerprint("sha256:gggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg"));
        let result = JournalEntryV1::new(
            "op-2",
            ActionKind::Gratitude,
            FINGERPRINT,
            "raw-agent-key",
            1,
        );
        assert_eq!(result, Err(JournalError::InvalidRecord));
    }

    #[test]
    fn unknown_schema_versions_fail_closed() {
        let entry = prepared();
        let encoded = serde_json::to_string(&entry).unwrap().replace(
            "\"schema_version\":1",
            "\"schema_version\":99",
        );
        assert_eq!(
            decode_record(&encoded),
            Err(JournalError::UnsupportedSchemaVersion(99))
        );
    }

    #[test]
    fn unknown_fields_fail_closed() {
        let encoded = serde_json::to_string(&prepared())
            .unwrap()
            .replace("}", ",\"gratitude_message\":\"private text\"}");
        assert_eq!(decode_record(&encoded), Err(JournalError::CorruptRecord));
    }

    #[test]
    fn submitted_attempt_can_only_be_recovered_as_unknown() {
        let mut entry = prepared();
        entry.transition(AttemptStatus::Submitted, 110).unwrap();
        entry.transition(AttemptStatus::OutcomeUnknown, 120).unwrap();
        assert_eq!(entry.status(), AttemptStatus::OutcomeUnknown);
        assert!(!entry.status().dispatch_eligible());
        assert_eq!(
            entry.transition(AttemptStatus::Submitted, 130),
            Err(JournalError::InvalidTransition {
                from: AttemptStatus::OutcomeUnknown,
                to: AttemptStatus::Submitted,
            })
        );
    }

    #[test]
    fn timestamps_cannot_move_backwards_and_terminal_result_is_preserved() {
        let mut entry = prepared();
        entry.transition(AttemptStatus::Submitted, 110).unwrap();
        assert_eq!(
            entry.transition(AttemptStatus::OutcomeUnknown, 109),
            Err(JournalError::TimestampRegressed)
        );
        entry.transition(AttemptStatus::SourceChainCommitted, 120).unwrap();
        assert_eq!(
            entry.transition(AttemptStatus::OutcomeUnknown, 130),
            Err(JournalError::InvalidTransition {
                from: AttemptStatus::SourceChainCommitted,
                to: AttemptStatus::OutcomeUnknown,
            })
        );
    }
}
