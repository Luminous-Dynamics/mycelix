// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Crash-safe journal wrapper for Hearth edge adapters.
//!
//! HTH-AUTO-004A establishes one invariant:
//!
//! > Restart must never transform uncertainty into permission to repeat a side effect.
//!
//! The wrapper durably records `Prepared -> Dispatching` before calling a real adapter.
//! After a crash, `Dispatching`, `Ambiguous`, and `Accepted` states enter reconciliation-only
//! mode: the command is not sent again automatically. The edge runtime may ask for verification,
//! but a new physical side effect requires an explicit fresh execution after uncertainty is
//! resolved.
//!
//! `Rejected`, `AdapterUnavailable`, and `NotAttempted` are treated as definite non-execution.
//! Adapter implementations MUST preserve that semantic contract. `TimedOut` and
//! `TransportError` are always ambiguous.

use hearth_automation_types::{
    ActionSpec, CommandStatus, EvidenceRef, OutcomeExpectation, OutcomePolicy, VerificationStatus,
};
use hearth_edge::{EdgeAdapter, VerificationReport};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    path::{Path, PathBuf},
};

pub const JOURNAL_SCHEMA_VERSION: u16 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum JournalDurability {
    /// Volatile test-only state. Must never guard a real household side effect.
    Volatile,
    /// Best-effort filesystem persistence without a full directory-fsync guarantee.
    BestEffort,
    /// Atomic replace + file fsync + parent-directory fsync.
    Durable,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum JournalPhase {
    /// Intent/action identity is durable; no dispatch marker has been written.
    Prepared,
    /// Durable marker written immediately before invoking the adapter.
    Dispatching { attempt: u16 },
    /// Adapter contract proves no side effect occurred.
    DefinitelyNotExecuted { attempt: u16 },
    /// Transport outcome cannot prove whether the side effect happened.
    Ambiguous { attempt: u16 },
    /// Adapter accepted the command. Physical/digital outcome remains unverified.
    Accepted { attempt: u16 },
    /// Requested outcome was independently verified.
    Verified,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryDisposition {
    SafeToDispatch,
    ReconcileOnly,
    AlreadyVerified,
}

impl JournalPhase {
    pub fn recovery_disposition(&self) -> RecoveryDisposition {
        match self {
            Self::Prepared | Self::DefinitelyNotExecuted { .. } => {
                RecoveryDisposition::SafeToDispatch
            }
            Self::Dispatching { .. } | Self::Ambiguous { .. } | Self::Accepted { .. } => {
                RecoveryDisposition::ReconcileOnly
            }
            Self::Verified => RecoveryDisposition::AlreadyVerified,
        }
    }

    fn next_attempt(&self) -> u16 {
        match self {
            Self::Dispatching { attempt }
            | Self::DefinitelyNotExecuted { attempt }
            | Self::Ambiguous { attempt }
            | Self::Accepted { attempt } => attempt.saturating_add(1),
            Self::Prepared | Self::Verified => 1,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct JournalRecord {
    pub schema_version: u16,
    pub idempotency_key: String,
    pub action: ActionSpec,
    pub phase: JournalPhase,
    pub first_seen_micros: i64,
    pub updated_at_micros: i64,
    pub generation: u64,
    pub last_command: Option<CommandStatus>,
    pub last_verification: Option<VerificationStatus>,
    pub evidence: Vec<EvidenceRef>,
}

impl JournalRecord {
    pub fn prepared(
        idempotency_key: impl Into<String>,
        action: ActionSpec,
        now_micros: i64,
    ) -> Self {
        Self {
            schema_version: JOURNAL_SCHEMA_VERSION,
            idempotency_key: idempotency_key.into(),
            action,
            phase: JournalPhase::Prepared,
            first_seen_micros: now_micros,
            updated_at_micros: now_micros,
            generation: 1,
            last_command: None,
            last_verification: None,
            evidence: Vec::new(),
        }
    }

    pub fn validate(&self) -> Result<(), JournalError> {
        if self.schema_version != JOURNAL_SCHEMA_VERSION {
            return Err(JournalError::InvalidRecord(format!(
                "unsupported journal schema version {}",
                self.schema_version
            )));
        }
        if self.idempotency_key.trim().is_empty() {
            return Err(JournalError::InvalidRecord(
                "idempotency key must not be empty".into(),
            ));
        }
        self.action
            .validate("action")
            .map_err(|error| JournalError::InvalidRecord(error.to_string()))?;
        if self.updated_at_micros < self.first_seen_micros {
            return Err(JournalError::InvalidRecord(
                "updated_at_micros precedes first_seen_micros".into(),
            ));
        }
        if self.generation == 0 {
            return Err(JournalError::InvalidRecord(
                "generation must be at least 1".into(),
            ));
        }
        Ok(())
    }

    fn advance(
        &self,
        phase: JournalPhase,
        now_micros: i64,
        last_command: Option<CommandStatus>,
        last_verification: Option<VerificationStatus>,
        evidence: Vec<EvidenceRef>,
    ) -> Self {
        let mut next = self.clone();
        next.phase = phase;
        next.updated_at_micros = now_micros;
        next.generation = self.generation.saturating_add(1);
        if let Some(command) = last_command {
            next.last_command = Some(command);
        }
        if let Some(verification) = last_verification {
            next.last_verification = Some(verification);
        }
        if !evidence.is_empty() {
            next.evidence = evidence;
        }
        next
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum JournalError {
    Io(String),
    Serialization(String),
    InvalidRecord(String),
    GenerationRegression { current: u64, attempted: u64 },
    IdempotencyCollision,
    DurableJournalRequired,
}

impl std::fmt::Display for JournalError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(message) => write!(f, "journal I/O error: {message}"),
            Self::Serialization(message) => write!(f, "journal serialization error: {message}"),
            Self::InvalidRecord(message) => write!(f, "invalid journal record: {message}"),
            Self::GenerationRegression { current, attempted } => write!(
                f,
                "journal generation regression: current={current}, attempted={attempted}"
            ),
            Self::IdempotencyCollision => {
                write!(f, "idempotency key already belongs to a different action")
            }
            Self::DurableJournalRequired => {
                write!(f, "real household side effects require a durable journal")
            }
        }
    }
}

impl std::error::Error for JournalError {}

impl From<std::io::Error> for JournalError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value.to_string())
    }
}

impl From<serde_json::Error> for JournalError {
    fn from(value: serde_json::Error) -> Self {
        Self::Serialization(value.to_string())
    }
}

pub trait ExecutionJournal {
    fn durability(&self) -> JournalDurability;
    fn load(&self, idempotency_key: &str) -> Result<Option<JournalRecord>, JournalError>;
    fn store(&mut self, record: &JournalRecord) -> Result<(), JournalError>;
}

#[derive(Debug, Default, Clone)]
pub struct MemoryJournal {
    records: BTreeMap<String, JournalRecord>,
}

impl MemoryJournal {
    pub fn records(&self) -> &BTreeMap<String, JournalRecord> {
        &self.records
    }
}

impl ExecutionJournal for MemoryJournal {
    fn durability(&self) -> JournalDurability {
        JournalDurability::Volatile
    }

    fn load(&self, idempotency_key: &str) -> Result<Option<JournalRecord>, JournalError> {
        Ok(self.records.get(idempotency_key).cloned())
    }

    fn store(&mut self, record: &JournalRecord) -> Result<(), JournalError> {
        record.validate()?;
        if let Some(current) = self.records.get(&record.idempotency_key)
            && record.generation <= current.generation
        {
            return Err(JournalError::GenerationRegression {
                current: current.generation,
                attempted: record.generation,
            });
        }
        self.records
            .insert(record.idempotency_key.clone(), record.clone());
        Ok(())
    }
}

/// Simple crash-safe local journal for the NixOS/Linux Hearth edge target.
///
/// On Unix, `store` writes a new file, fsyncs it, atomically renames it over the
/// previous generation, and fsyncs the containing directory. Other platforms are
/// reported as `BestEffort`, so `JournaledAdapter::new_physical` refuses them.
#[derive(Debug, Clone)]
pub struct FileJournal {
    root: PathBuf,
}

impl FileJournal {
    pub fn open(root: impl Into<PathBuf>) -> Result<Self, JournalError> {
        let root = root.into();
        fs::create_dir_all(&root)?;
        Ok(Self { root })
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    fn path_for(&self, key: &str) -> PathBuf {
        self.root.join(format!("{}.json", hex_encode(key.as_bytes())))
    }

    fn temporary_path_for(&self, key: &str, generation: u64) -> PathBuf {
        self.root.join(format!(
            ".{}.{}.{}.tmp",
            hex_encode(key.as_bytes()),
            std::process::id(),
            generation
        ))
    }
}

impl ExecutionJournal for FileJournal {
    fn durability(&self) -> JournalDurability {
        #[cfg(unix)]
        {
            JournalDurability::Durable
        }
        #[cfg(not(unix))]
        {
            JournalDurability::BestEffort
        }
    }

    fn load(&self, idempotency_key: &str) -> Result<Option<JournalRecord>, JournalError> {
        let path = self.path_for(idempotency_key);
        let mut file = match File::open(path) {
            Ok(file) => file,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(error) => return Err(error.into()),
        };
        let mut bytes = Vec::new();
        file.read_to_end(&mut bytes)?;
        let record: JournalRecord = serde_json::from_slice(&bytes)?;
        record.validate()?;
        Ok(Some(record))
    }

    fn store(&mut self, record: &JournalRecord) -> Result<(), JournalError> {
        record.validate()?;
        if let Some(current) = self.load(&record.idempotency_key)?
            && record.generation <= current.generation
        {
            return Err(JournalError::GenerationRegression {
                current: current.generation,
                attempted: record.generation,
            });
        }

        let bytes = serde_json::to_vec(record)?;
        let target = self.path_for(&record.idempotency_key);
        let temporary = self.temporary_path_for(&record.idempotency_key, record.generation);

        let write_result = (|| -> Result<(), JournalError> {
            let mut file = OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&temporary)?;
            file.write_all(&bytes)?;
            file.sync_all()?;
            drop(file);
            fs::rename(&temporary, &target)?;

            #[cfg(unix)]
            {
                File::open(&self.root)?.sync_all()?;
            }
            Ok(())
        })();

        if write_result.is_err() {
            let _ = fs::remove_file(&temporary);
        }
        write_result
    }
}

fn hex_encode(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CommandCertainty {
    DefinitelyNotExecuted,
    Ambiguous,
    Accepted,
}

fn command_certainty(status: &CommandStatus) -> CommandCertainty {
    match status {
        CommandStatus::NotAttempted
        | CommandStatus::Rejected { .. }
        | CommandStatus::AdapterUnavailable => CommandCertainty::DefinitelyNotExecuted,
        CommandStatus::TimedOut | CommandStatus::TransportError { .. } => {
            CommandCertainty::Ambiguous
        }
        CommandStatus::Accepted => CommandCertainty::Accepted,
    }
}

/// Crash-safe adapter decorator.
///
/// For a previously ambiguous/accepted/dispatching key, `execute` returns
/// `Accepted` *without invoking the inner adapter*. This deliberately routes the
/// existing edge runtime into its verification path. If verification cannot
/// establish the requested outcome, the runtime may retry verification using
/// the same key, but the decorator will not redispatch the side effect.
///
/// The `Accepted` returned during recovery means "continue reconciliation", not
/// "a new command was sent". Durable journal state remains the execution truth.
pub struct JournaledAdapter<A, J> {
    inner: A,
    journal: J,
    physical: bool,
}

impl<A, J> JournaledAdapter<A, J>
where
    A: EdgeAdapter,
    J: ExecutionJournal,
{
    pub fn new_physical(inner: A, journal: J) -> Result<Self, JournalError> {
        if journal.durability() != JournalDurability::Durable {
            return Err(JournalError::DurableJournalRequired);
        }
        Ok(Self {
            inner,
            journal,
            physical: true,
        })
    }

    pub fn new_for_test(inner: A, journal: J) -> Self {
        Self {
            inner,
            journal,
            physical: false,
        }
    }

    pub fn journal(&self) -> &J {
        &self.journal
    }

    pub fn inner(&self) -> &A {
        &self.inner
    }

    pub fn into_parts(self) -> (A, J) {
        (self.inner, self.journal)
    }

    fn load_compatible(
        &self,
        key: &str,
        action: &ActionSpec,
    ) -> Result<Option<JournalRecord>, JournalError> {
        let record = self.journal.load(key)?;
        if let Some(record) = &record
            && record.action != *action
        {
            return Err(JournalError::IdempotencyCollision);
        }
        Ok(record)
    }

    fn persist(&mut self, record: &JournalRecord) -> Result<(), CommandStatus> {
        self.journal.store(record).map_err(|error| {
            CommandStatus::TransportError {
                message: format!("durable journal update failed: {error}"),
            }
        })
    }
}

impl<A, J> EdgeAdapter for JournaledAdapter<A, J>
where
    A: EdgeAdapter,
    J: ExecutionJournal,
{
    fn name(&self) -> &str {
        self.inner.name()
    }

    fn supports_capability(&self, capability: &str) -> bool {
        self.inner.supports_capability(capability)
    }

    fn supports(&self, action: &ActionSpec) -> bool {
        self.inner.supports(action)
    }

    fn execute(
        &mut self,
        idempotency_key: &str,
        action: &ActionSpec,
        timeout_ms: u64,
        now_micros: i64,
    ) -> CommandStatus {
        if self.physical && self.journal.durability() != JournalDurability::Durable {
            return CommandStatus::Rejected {
                reason: JournalError::DurableJournalRequired.to_string(),
            };
        }

        let mut record = match self.load_compatible(idempotency_key, action) {
            Ok(Some(record)) => record,
            Ok(None) => {
                let prepared =
                    JournalRecord::prepared(idempotency_key, action.clone(), now_micros);
                if let Err(status) = self.persist(&prepared) {
                    return status;
                }
                prepared
            }
            Err(error) => {
                return CommandStatus::Rejected {
                    reason: error.to_string(),
                };
            }
        };

        match record.phase.recovery_disposition() {
            RecoveryDisposition::AlreadyVerified | RecoveryDisposition::ReconcileOnly => {
                // Never resend an uncertain or already-verified side effect.
                // Returning Accepted enters EdgeRuntime's separate verify path.
                return CommandStatus::Accepted;
            }
            RecoveryDisposition::SafeToDispatch => {}
        }

        let attempt = record.phase.next_attempt();
        record = record.advance(
            JournalPhase::Dispatching { attempt },
            now_micros,
            None,
            None,
            Vec::new(),
        );
        if let Err(status) = self.persist(&record) {
            return status;
        }

        let command = self
            .inner
            .execute(idempotency_key, action, timeout_ms, now_micros);

        let phase = match command_certainty(&command) {
            CommandCertainty::DefinitelyNotExecuted => {
                JournalPhase::DefinitelyNotExecuted { attempt }
            }
            CommandCertainty::Ambiguous => JournalPhase::Ambiguous { attempt },
            CommandCertainty::Accepted => JournalPhase::Accepted { attempt },
        };

        let next = record.advance(
            phase,
            now_micros,
            Some(command.clone()),
            None,
            Vec::new(),
        );
        if let Err(status) = self.persist(&next) {
            // `Dispatching` was already durable before the adapter call, so a
            // failed post-command journal update still recovers fail-closed.
            return status;
        }

        command
    }

    fn verify(
        &mut self,
        idempotency_key: &str,
        expectations: &[OutcomeExpectation],
        policy: &OutcomePolicy,
        now_micros: i64,
    ) -> VerificationReport {
        let Some(mut record) = (match self.journal.load(idempotency_key) {
            Ok(record) => record,
            Err(error) => {
                return VerificationReport::failed(format!(
                    "journal load failed during verification: {error}"
                ));
            }
        }) else {
            return VerificationReport::failed(
                "verification refused because no durable dispatch record exists",
            );
        };

        if record.phase == JournalPhase::Verified
            && record
                .last_verification
                .as_ref()
                .is_some_and(VerificationStatus::is_verified)
        {
            return VerificationReport::verified(record.evidence.clone());
        }

        let report =
            self.inner
                .verify(idempotency_key, expectations, policy, now_micros);

        let phase = if report.status.is_verified() {
            JournalPhase::Verified
        } else {
            record.phase.clone()
        };

        record = record.advance(
            phase,
            now_micros,
            None,
            Some(report.status.clone()),
            report.evidence.clone(),
        );

        if let Err(error) = self.journal.store(&record) {
            return VerificationReport::failed(format!(
                "journal update failed during verification: {error}"
            ));
        }

        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_edge::SyntheticAdapter;

    fn action() -> ActionSpec {
        ActionSpec {
            capability: "home.appliance.control".into(),
            target: hearth_automation_types::EntityRef {
                kind: "appliance".into(),
                id: "washer".into(),
            },
            operation: "stop".into(),
            arguments: BTreeMap::new(),
        }
    }

    fn outcome() -> OutcomePolicy {
        OutcomePolicy {
            expectations: vec![OutcomeExpectation::State(
                hearth_automation_types::StateExpectation {
                    subject: hearth_automation_types::EntityRef {
                        kind: "appliance".into(),
                        id: "washer".into(),
                    },
                    attribute: "running".into(),
                    op: hearth_automation_types::ComparisonOp::Eq,
                    expected: hearth_automation_types::AutomationValue::Bool(false),
                },
            )],
            verify_within_ms: 5_000,
            evidence: hearth_automation_types::EvidenceRequirement::default(),
            on_unverified: hearth_automation_types::UnverifiedDisposition::StopAndNotify,
        }
    }

    #[test]
    fn physical_wrapper_rejects_volatile_journal() {
        let adapter =
            SyntheticAdapter::new("synthetic", ["home.appliance.control".to_string()]);
        let result = JournaledAdapter::new_physical(adapter, MemoryJournal::default());
        assert!(matches!(result, Err(JournalError::DurableJournalRequired)));
    }

    #[test]
    fn crash_after_dispatch_enters_reconciliation_without_redispatch() {
        let inner = SyntheticAdapter::new(
            "synthetic",
            ["home.appliance.control".to_string()],
        )
        .with_commands(vec![CommandStatus::Accepted])
        .with_verifications(vec![VerificationReport::failed("not yet")]);

        let mut journal = MemoryJournal::default();
        let mut record = JournalRecord::prepared("plan:p:step:s", action(), 10);
        record = record.advance(
            JournalPhase::Dispatching { attempt: 1 },
            11,
            None,
            None,
            Vec::new(),
        );
        journal.store(&record).unwrap();

        let mut wrapped = JournaledAdapter::new_for_test(inner, journal);
        let command = wrapped.execute("plan:p:step:s", &action(), 1_000, 20);

        assert_eq!(command, CommandStatus::Accepted);
        assert!(wrapped.inner().command_invocations.is_empty());
    }

    #[test]
    fn timeout_becomes_ambiguous_and_is_not_redispatched() {
        let inner = SyntheticAdapter::new(
            "synthetic",
            ["home.appliance.control".to_string()],
        )
        .with_commands(vec![CommandStatus::TimedOut, CommandStatus::Accepted])
        .with_verifications(vec![VerificationReport::failed("unknown")]);

        let mut wrapped =
            JournaledAdapter::new_for_test(inner, MemoryJournal::default());

        let first = wrapped.execute("plan:p:step:s", &action(), 1_000, 10);
        assert_eq!(first, CommandStatus::TimedOut);

        let second = wrapped.execute("plan:p:step:s", &action(), 1_000, 20);
        assert_eq!(second, CommandStatus::Accepted);

        assert_eq!(wrapped.inner().command_invocations.len(), 1);
    }

    #[test]
    fn definite_non_execution_may_be_dispatched_again() {
        let inner = SyntheticAdapter::new(
            "synthetic",
            ["home.appliance.control".to_string()],
        )
        .with_commands(vec![
            CommandStatus::AdapterUnavailable,
            CommandStatus::Accepted,
        ]);

        let mut wrapped =
            JournaledAdapter::new_for_test(inner, MemoryJournal::default());

        let first = wrapped.execute("plan:p:step:s", &action(), 1_000, 10);
        assert_eq!(first, CommandStatus::AdapterUnavailable);

        let second = wrapped.execute("plan:p:step:s", &action(), 1_000, 20);
        assert_eq!(second, CommandStatus::Accepted);

        assert_eq!(wrapped.inner().command_invocations.len(), 2);
    }

    #[test]
    fn verified_record_survives_replay_without_inner_execute() {
        let inner = SyntheticAdapter::new(
            "synthetic",
            ["home.appliance.control".to_string()],
        )
        .with_commands(vec![CommandStatus::Accepted])
        .with_verifications(vec![VerificationReport::verified(Vec::new())]);

        let mut wrapped =
            JournaledAdapter::new_for_test(inner, MemoryJournal::default());

        assert_eq!(
            wrapped.execute("plan:p:step:s", &action(), 1_000, 10),
            CommandStatus::Accepted
        );
        assert!(
            wrapped
                .verify("plan:p:step:s", &outcome().expectations, &outcome(), 11)
                .status
                .is_verified()
        );

        let before = wrapped.inner().command_invocations.len();
        assert_eq!(
            wrapped.execute("plan:p:step:s", &action(), 1_000, 20),
            CommandStatus::Accepted
        );
        assert_eq!(wrapped.inner().command_invocations.len(), before);
    }

    #[test]
    fn idempotency_key_collision_is_rejected() {
        let inner =
            SyntheticAdapter::new("synthetic", ["home.appliance.control".to_string()]);
        let mut journal = MemoryJournal::default();
        journal
            .store(&JournalRecord::prepared(
                "plan:p:step:s",
                action(),
                10,
            ))
            .unwrap();

        let mut other = action();
        other.operation = "start".into();

        let mut wrapped = JournaledAdapter::new_for_test(inner, journal);
        let result = wrapped.execute("plan:p:step:s", &other, 1_000, 20);

        assert!(matches!(result, CommandStatus::Rejected { .. }));
        assert!(wrapped.inner().command_invocations.is_empty());
    }
}
