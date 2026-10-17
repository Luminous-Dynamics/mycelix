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
use leptos::prelude::*;
use serde::{Deserialize, Serialize};
use std::cell::{Cell, RefCell};
use std::rc::Rc;
use wasm_bindgen::{closure::Closure, JsCast, JsValue};
use wasm_bindgen_futures::JsFuture;
use web_sys::{Event, IdbDatabase, IdbOpenDbRequest, IdbRequest, IdbTransaction};

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
        if !valid_operation_id(&self.operation_id)
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

/// Generate a random UUID using the browser's cryptographic RNG. If the
/// required secure-context API is unavailable, fail closed instead of falling
/// back to timestamps, Math.random, or counters.
pub fn new_operation_id() -> Result<String, JournalError> {
    let window = web_sys::window().ok_or(JournalError::IndexedDbUnavailable)?;
    let crypto = window
        .crypto()
        .map_err(|_| JournalError::IndexedDbUnavailable)?;
    let operation_id = crypto.random_uuid();
    if !valid_operation_id(&operation_id) {
        return Err(JournalError::InvalidRecord);
    }
    Ok(operation_id)
}

/// Domain-separated SHA-256 fingerprint for a canonical, non-secret identity
/// byte representation. This is a pseudonymous locator, not encryption or
/// authorization; public/low-entropy inputs can still be linkable or guessed.
pub async fn sha256_fingerprint(
    domain: &str,
    canonical_bytes: &[u8],
) -> Result<String, JournalError> {
    if domain.is_empty() || domain.len() > 80 || !domain.is_ascii() {
        return Err(JournalError::InvalidRecord);
    }
    let domain_len =
        u32::try_from(domain.len()).map_err(|_| JournalError::InvalidRecord)?;
    let material_len =
        u64::try_from(canonical_bytes.len()).map_err(|_| JournalError::InvalidRecord)?;

    let mut material = Vec::with_capacity(
        b"mycelix-hearth-journal-fingerprint-v1\0".len()
            + 4
            + domain.len()
            + 8
            + canonical_bytes.len(),
    );
    material.extend_from_slice(b"mycelix-hearth-journal-fingerprint-v1\0");
    material.extend_from_slice(&domain_len.to_be_bytes());
    material.extend_from_slice(domain.as_bytes());
    material.extend_from_slice(&material_len.to_be_bytes());
    material.extend_from_slice(canonical_bytes);

    let window = web_sys::window().ok_or(JournalError::IndexedDbUnavailable)?;
    let crypto = window
        .crypto()
        .map_err(|_| JournalError::IndexedDbUnavailable)?;
    let digest = crypto
        .subtle()
        .digest_with_str_and_u8_array("SHA-256", &material)
        .map_err(|_| JournalError::IndexedDbUnavailable)?;
    let digest = JsFuture::from(digest)
        .await
        .map_err(|_| JournalError::IndexedDbUnavailable)?;
    let bytes = js_sys::Uint8Array::new(&digest).to_vec();
    fingerprint_from_digest(&bytes)
}

fn fingerprint_from_digest(digest: &[u8]) -> Result<String, JournalError> {
    if digest.len() != 32 {
        return Err(JournalError::InvalidRecord);
    }
    let mut fingerprint = String::with_capacity(7 + digest.len() * 2);
    fingerprint.push_str("sha256:");
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut fingerprint, "{byte:02x}")
            .map_err(|_| JournalError::SerializationFailed)?;
    }
    Ok(fingerprint)
}

fn valid_operation_id(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() != 36
        || bytes[8] != b'-'
        || bytes[13] != b'-'
        || bytes[18] != b'-'
        || bytes[23] != b'-'
        || bytes[14] != b'4'
        || !matches!(bytes[19], b'8' | b'9' | b'a' | b'b')
    {
        return false;
    }
    bytes.iter().enumerate().all(|(index, byte)| {
        matches!(index, 8 | 13 | 18 | 23)
            || byte.is_ascii_digit()
            || (b'a'..=b'f').contains(byte)
    })
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
            let recovered_status = match entry.status() {
                // The gate never sends while an attempt is merely Prepared.
                AttemptStatus::Prepared => Some(AttemptStatus::NotSent),
                // Submitted means dispatch may have happened, so never replay it.
                AttemptStatus::Submitted => Some(AttemptStatus::OutcomeUnknown),
                _ => None,
            };
            if let Some(recovered_status) = recovered_status {
                self.transition(
                    entry.operation_id(),
                    recovered_status,
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

    /// Open the local store and persist restart recovery before allowing live
    /// dispatch to obtain a handle to the journal.
    pub async fn open_recovered() -> Result<Self, JournalError> {
        let journal = Self::open().await?;
        let now = current_unix_ms()?;
        journal.recover_after_restart(now).await?;
        Ok(journal)
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
            // A generic request error does not establish a duplicate key;
            // quota, corruption, or an aborted transaction can fail here too.
            return Err(JournalError::RequestFailed);
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
        if !valid_operation_id(operation_id) {
            return Err(JournalError::InvalidRecord);
        }

        // Keep the read and write in one IndexedDB transaction so stale status
        // observations cannot overwrite a newer transition from another tab.
        let tx = strict_readwrite_transaction(&self.database)?;
        let done = transaction_finished(&tx);
        let store = tx.object_store(STORE_NAME).map_err(|_| JournalError::RequestFailed)?;
        let key = JsValue::from_str(operation_id);
        let request = store.get(&key).map_err(|_| JournalError::RequestFailed)?;
        let updated = Rc::new(RefCell::new(None::<JournalEntryV1>));
        let failure = Rc::new(Cell::new(None::<JournalError>));

        let callback_request = request.clone();
        let callback_store = store.clone();
        let callback_key = key.clone();
        let callback_id = operation_id.to_string();
        let callback_updated = Rc::clone(&updated);
        let callback_failure = Rc::clone(&failure);
        let success = Closure::once_into_js(move |_event: Event| {
            let value = match callback_request.result() {
                Ok(value) if !value.is_null() && !value.is_undefined() => value,
                _ => {
                    callback_failure.set(Some(JournalError::MissingOperation));
                    return;
                }
            };
            let Some(json) = value.as_string() else {
                callback_failure.set(Some(JournalError::CorruptRecord));
                return;
            };
            let mut entry = match decode_record(&json) {
                Ok(entry) if entry.operation_id() == callback_id => entry,
                Ok(_) => {
                    callback_failure.set(Some(JournalError::CorruptRecord));
                    return;
                }
                Err(error) => {
                    callback_failure.set(Some(error));
                    return;
                }
            };
            if let Err(error) = entry.transition(next, now_unix_ms) {
                callback_failure.set(Some(error));
                return;
            }
            let encoded = match serde_json::to_string(&entry) {
                Ok(encoded) => encoded,
                Err(_) => {
                    callback_failure.set(Some(JournalError::SerializationFailed));
                    return;
                }
            };
            if callback_store
                .put_with_key(&JsValue::from_str(&encoded), &callback_key)
                .is_err()
            {
                callback_failure.set(Some(JournalError::RequestFailed));
                return;
            }
            callback_updated.replace(Some(entry));
        });
        request.set_onsuccess(Some(success.unchecked_ref()));

        if JsFuture::from(done).await.is_err() {
            return Err(failure.get().unwrap_or(JournalError::CommitFailed));
        }
        if let Some(error) = failure.get() {
            return Err(error);
        }
        updated.borrow_mut().take().ok_or(JournalError::CommitFailed)
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
        let values_request = store.get_all().map_err(|_| JournalError::RequestFailed)?;
        let keys_request = store.get_all_keys().map_err(|_| JournalError::RequestFailed)?;
        // Queue both reads before yielding, while this read-only transaction is
        // active. A record must agree with the IndexedDB key under which it sits.
        let values_future = JsFuture::from(request_finished(values_request));
        let keys_future = JsFuture::from(request_finished(keys_request));
        let values = values_future.await.map_err(|_| JournalError::RequestFailed)?;
        let keys = keys_future.await.map_err(|_| JournalError::RequestFailed)?;
        JsFuture::from(done)
            .await
            .map_err(|_| JournalError::CommitFailed)?;

        let values = Array::from(&values);
        let keys = Array::from(&keys);
        if values.length() != keys.length() {
            return Err(JournalError::CorruptRecord);
        }
        let mut entries = Vec::with_capacity(values.length() as usize);
        for (value, key) in values.iter().zip(keys.iter()) {
            let json = value.as_string().ok_or(JournalError::CorruptRecord)?;
            let entry = decode_record(&json)?;
            if key.as_string().as_deref() != Some(entry.operation_id()) {
                return Err(JournalError::CorruptRecord);
            }
            entries.push(entry);
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

fn current_unix_ms() -> Result<u64, JournalError> {
    let now = js_sys::Date::now();
    if !now.is_finite() || now < 0.0 || now > u64::MAX as f64 {
        return Err(JournalError::InvalidRecord);
    }
    Ok(now.floor() as u64)
}

fn upsert_entry_snapshot(
    entries_signal: RwSignal<Vec<JournalEntryV1>>,
    entry: JournalEntryV1,
) {
    entries_signal.update(|entries| {
        if let Some(existing) = entries
            .iter_mut()
            .find(|existing| existing.operation_id() == entry.operation_id())
        {
            *existing = entry;
        } else {
            entries.push(entry);
        }
        entries.sort_by(|a, b| {
            a.created_at_unix_ms()
                .cmp(&b.created_at_unix_ms())
                .then_with(|| a.operation_id().cmp(b.operation_id()))
        });
    });
}

/// One locally prepared attempt. It is not an idempotency key recognized by a
/// zome; it represents local dispatch/recovery evidence only.
#[derive(Clone)]
pub struct PreparedJournalAttempt {
    journal: IndexedDbJournal,
    entries_signal: RwSignal<Vec<JournalEntryV1>>,
    operation_id: String,
    submitted_at_unix_ms: u64,
}

impl PreparedJournalAttempt {
    pub fn operation_id(&self) -> &str {
        &self.operation_id
    }

    /// Persist an observed successful response. False means the remote call
    /// succeeded but its local terminal state could not be confirmed durable.
    pub async fn mark_source_chain_committed(&self) -> bool {
        let Ok(now) = current_unix_ms() else {
            return false;
        };
        let Ok(updated) = self
            .journal
            .transition(
                &self.operation_id,
                AttemptStatus::SourceChainCommitted,
                now.max(self.submitted_at_unix_ms),
            )
            .await
        else {
            return false;
        };
        upsert_entry_snapshot(self.entries_signal, updated);
        true
    }

    /// Persist ambiguity from timeout/transport/decode/restart-related errors.
    /// False does not make the underlying action safe to retry.
    pub async fn mark_outcome_unknown(&self) -> bool {
        let Ok(now) = current_unix_ms() else {
            return false;
        };
        let Ok(updated) = self
            .journal
            .transition(
                &self.operation_id,
                AttemptStatus::OutcomeUnknown,
                now.max(self.submitted_at_unix_ms),
            )
            .await
        else {
            return false;
        };
        upsert_entry_snapshot(self.entries_signal, updated);
        true
    }
}

/// Journal-gate a live dispatch. Returning Ok proves both Prepared and Submitted
/// writes reported transaction completion; callers must not contact the
/// conductor before this function succeeds. Operation IDs remain local.
pub async fn prepare_live_attempt(
    journal_signal: RwSignal<Option<IndexedDbJournal>, LocalStorage>,
    entries_signal: RwSignal<Vec<JournalEntryV1>>,
    action: ActionKind,
    hearth_scope_bytes: &[u8],
    target_identity_bytes: &[u8],
) -> Result<PreparedJournalAttempt, JournalError> {
    if hearth_scope_bytes.is_empty() || target_identity_bytes.is_empty() {
        return Err(JournalError::InvalidRecord);
    }
    let journal = journal_signal
        .get_untracked()
        .ok_or(JournalError::IndexedDbUnavailable)?;
    let operation_id = new_operation_id()?;
    let scope_fingerprint =
        sha256_fingerprint("hearth-scope-v1", hearth_scope_bytes).await?;
    let target_fingerprint =
        sha256_fingerprint("hearth-target-v1", target_identity_bytes).await?;
    let created_at = current_unix_ms()?;
    let entry = JournalEntryV1::new(
        operation_id.clone(),
        action,
        scope_fingerprint,
        target_fingerprint,
        created_at,
    )?;

    // Any error here returns before the caller is permitted to send. The
    // read-only UI snapshot is updated only after the write transaction commits.
    journal.insert_prepared(entry.clone()).await?;
    upsert_entry_snapshot(entries_signal, entry);

    let submitted_at = current_unix_ms()?.max(created_at);
    let submitted = journal
        .transition(&operation_id, AttemptStatus::Submitted, submitted_at)
        .await?;
    upsert_entry_snapshot(entries_signal, submitted);

    Ok(PreparedJournalAttempt {
        journal,
        entries_signal,
        operation_id,
        submitted_at_unix_ms: submitted_at,
    })
}

#[cfg(test)]
#[derive(Default)]
struct MemoryJournal {
    records: RefCell<std::collections::BTreeMap<String, JournalEntryV1>>,
    fail_next_write: Cell<bool>,
}

#[cfg(test)]
impl MemoryJournal {
    fn fail_next_write(&self) {
        self.fail_next_write.set(true);
    }

    fn consume_failure(&self) -> Result<(), JournalError> {
        if self.fail_next_write.replace(false) {
            Err(JournalError::CommitFailed)
        } else {
            Ok(())
        }
    }

    fn insert_prepared_sync(&self, entry: JournalEntryV1) -> Result<(), JournalError> {
        entry.validate()?;
        if entry.status() != AttemptStatus::Prepared {
            return Err(JournalError::InvalidRecord);
        }
        self.consume_failure()?;
        let mut records = self.records.borrow_mut();
        if records.contains_key(entry.operation_id()) {
            return Err(JournalError::DuplicateOperation);
        }
        records.insert(entry.operation_id().to_string(), entry);
        Ok(())
    }

    fn transition_sync(
        &self,
        operation_id: &str,
        next: AttemptStatus,
        now_unix_ms: u64,
    ) -> Result<JournalEntryV1, JournalError> {
        self.consume_failure()?;
        let mut records = self.records.borrow_mut();
        let mut entry = records
            .get(operation_id)
            .cloned()
            .ok_or(JournalError::MissingOperation)?;
        entry.transition(next, now_unix_ms)?;
        records.insert(operation_id.to_string(), entry.clone());
        Ok(entry)
    }

    fn list_sync(&self) -> Vec<JournalEntryV1> {
        let mut entries = self.records.borrow().values().cloned().collect::<Vec<_>>();
        entries.sort_by(|a, b| a.operation_id().cmp(b.operation_id()));
        entries
    }

    fn recover_sync(&self, now_unix_ms: u64) -> Result<Vec<JournalEntryV1>, JournalError> {
        for entry in self.list_sync() {
            let recovered_status = match entry.status() {
                AttemptStatus::Prepared => Some(AttemptStatus::NotSent),
                AttemptStatus::Submitted => Some(AttemptStatus::OutcomeUnknown),
                _ => None,
            };
            if let Some(recovered_status) = recovered_status {
                self.transition_sync(
                    entry.operation_id(),
                    recovered_status,
                    now_unix_ms.max(entry.updated_at_unix_ms()),
                )?;
            }
        }
        Ok(self.list_sync())
    }
}

#[cfg(test)]
impl ActionJournalStore for MemoryJournal {
    async fn insert_prepared(&self, entry: JournalEntryV1) -> Result<(), JournalError> {
        self.insert_prepared_sync(entry)
    }

    async fn transition(
        &self,
        operation_id: &str,
        next: AttemptStatus,
        now_unix_ms: u64,
    ) -> Result<JournalEntryV1, JournalError> {
        self.transition_sync(operation_id, next, now_unix_ms)
    }

    async fn list(&self) -> Result<Vec<JournalEntryV1>, JournalError> {
        Ok(self.list_sync())
    }

    async fn recover_after_restart(
        &self,
        now_unix_ms: u64,
    ) -> Result<Vec<JournalEntryV1>, JournalError> {
        self.recover_sync(now_unix_ms)
    }
}

#[cfg(test)]
mod tests {
    use super::{
        decode_record, valid_sha256_fingerprint, JournalEntryV1, JournalError, MemoryJournal,
    };
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
    fn injected_prepared_commit_failure_leaves_no_dispatchable_record() {
        let journal = MemoryJournal::default();
        journal.fail_next_write();
        assert_eq!(
            journal.insert_prepared_sync(prepared()),
            Err(JournalError::CommitFailed)
        );
        assert!(journal.list_sync().is_empty());
    }

    #[test]
    fn restart_recovery_marks_prepared_attempt_as_not_sent() {
        let journal = MemoryJournal::default();
        journal.insert_prepared_sync(prepared()).unwrap();

        let recovered = journal.recover_sync(120).unwrap();
        assert_eq!(recovered.len(), 1);
        assert_eq!(recovered[0].status(), AttemptStatus::NotSent);
        assert!(recovered[0].status().is_terminal());
        assert!(!recovered[0].status().dispatch_eligible());
    }

    #[test]
    fn restart_recovery_persists_unknown_before_exposing_rows() {
        let journal = MemoryJournal::default();
        journal.insert_prepared_sync(prepared()).unwrap();
        journal
            .transition_sync(
                "c0ffee00-0000-4000-8000-000000000001",
                AttemptStatus::Submitted,
                110,
            )
            .unwrap();

        let recovered = journal.recover_sync(120).unwrap();
        assert_eq!(recovered.len(), 1);
        assert_eq!(recovered[0].status(), AttemptStatus::OutcomeUnknown);
        assert_eq!(
            journal.list_sync()[0].status(),
            AttemptStatus::OutcomeUnknown
        );
        assert!(!recovered[0].status().dispatch_eligible());
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
    fn fingerprint_encoding_requires_exact_sha256_length_and_is_lowercase() {
        assert_eq!(
            super::fingerprint_from_digest(&[0u8; 32]).unwrap(),
            format!("sha256:{}", "00".repeat(32))
        );
        assert_eq!(
            super::fingerprint_from_digest(&[0xab; 32]).unwrap(),
            format!("sha256:{}", "ab".repeat(32))
        );
        assert_eq!(
            super::fingerprint_from_digest(&[0u8; 31]),
            Err(JournalError::InvalidRecord)
        );
    }

    #[test]
    fn raw_or_noncanonical_identity_is_not_accepted_as_a_fingerprint() {
        assert!(valid_sha256_fingerprint(FINGERPRINT));
        assert!(!valid_sha256_fingerprint("A resident name"));
        assert!(!valid_sha256_fingerprint("sha256:ABCDEF"));
        assert!(!valid_sha256_fingerprint("sha256:gggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg"));
        let result = JournalEntryV1::new(
            "c0ffee00-0000-4000-8000-000000000001",
            ActionKind::Gratitude,
            FINGERPRINT,
            "raw-agent-key",
            1,
        );
        assert_eq!(result, Err(JournalError::InvalidRecord));

        let non_uuid = JournalEntryV1::new(
            "op-2",
            ActionKind::CareCompletion,
            FINGERPRINT,
            FINGERPRINT,
            1,
        );
        assert_eq!(non_uuid, Err(JournalError::InvalidRecord));
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
