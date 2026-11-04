#![forbid(unsafe_code)]
//! FIN-ECO-006 v1: exact evidence-bound accounting journal projection.
//!
//! Authority flows in one direction:
//! qualified economic fact -> versioned accounting policy -> balanced journal.
//! A balanced journal is not proof that a source event was authorized or settled.
//! This crate does not authorize actions, create capacity, value assets, convert
//! currencies, or claim complete books.
//!
//! The first source adapter accepts only constructor-controlled QualifiedSettlement
//! values. Other economic sources require their own qualified adapters.

use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_finance_exact::{AssetAmount, AssetId};
use mycelix_finance_settlement::{QualifiedSettlement, ReversalModel};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const ACCOUNTING_JOURNAL_SCHEMA_V1: u16 = 1;
pub const MAX_ACCOUNTS: usize = 100_000;
pub const MAX_UNIT_PROFILES: usize = 10_000;
pub const MAX_POSTINGS_PER_ENTRY: usize = 256;
pub const MAX_ID_BYTES: usize = 512;
pub const MAX_DECIMAL_SCALE: u8 = 18;

const LEDGER_DOMAIN: &[u8] = b"mycelix:fin-eco-006:ledger-profile:v1\0";
const UNIT_DOMAIN: &[u8] = b"mycelix:fin-eco-006:asset-unit:v1\0";
const POLICY_DOMAIN: &[u8] = b"mycelix:fin-eco-006:projection-policy:v1\0";
const SOURCE_DOMAIN: &[u8] = b"mycelix:fin-eco-006:settlement-source:v1\0";
const ENTRY_DOMAIN: &[u8] = b"mycelix:fin-eco-006:journal-entry:v1\0";
const SOURCE_APPLICATION_DOMAIN: &[u8] = b"mycelix:fin-eco-006:source-application:v1\0";

fn valid_id(value: &ReferenceId) -> bool {
    !value.as_str().trim().is_empty()
        && value.as_str().len() <= MAX_ID_BYTES
        && !value.as_str().chars().any(char::is_control)
}

#[derive(Clone, Debug, Default)]
struct CanonicalWriter {
    bytes: Vec<u8>,
}

impl CanonicalWriter {
    fn new(domain: &[u8]) -> Self {
        Self { bytes: domain.to_vec() }
    }
    fn field(&mut self, value: &[u8]) {
        self.bytes.extend_from_slice(&(value.len() as u64).to_be_bytes());
        self.bytes.extend_from_slice(value);
    }
    fn text(&mut self, value: &str) { self.field(value.as_bytes()); }
    fn u8(&mut self, value: u8) { self.bytes.push(value); }
    fn u16(&mut self, value: u16) { self.bytes.extend_from_slice(&value.to_be_bytes()); }
    fn u64(&mut self, value: u64) { self.bytes.extend_from_slice(&value.to_be_bytes()); }
    fn digest(&mut self, value: Digest32) { self.bytes.extend_from_slice(&value.0); }
    fn finish(self) -> Vec<u8> { self.bytes }
    fn hash(self) -> Digest32 { Digest32(Sha256::digest(self.bytes).into()) }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum AccountingBookDomain {
    Financial,
    Encumbrance,
    Operational,
    Management,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum NormalBalance {
    Debit,
    Credit,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum PostingSide {
    Debit,
    Credit,
}

impl PostingSide {
    pub fn opposite(self) -> Self {
        match self {
            Self::Debit => Self::Credit,
            Self::Credit => Self::Debit,
        }
    }
    fn tag(self) -> u8 {
        match self {
            Self::Debit => 0,
            Self::Credit => 1,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct SettlementProfileBinding {
    pub profile_id: ReferenceId,
    pub revision: u64,
    pub digest: Digest32,
}

impl SettlementProfileBinding {
    pub fn from_qualified_settlement(settlement: &QualifiedSettlement) -> Self {
        Self {
            profile_id: settlement.profile().id.clone(),
            revision: settlement.profile().revision,
            digest: settlement.profile().digest,
        }
    }

    fn validate(&self) -> Result<(), AccountingError> {
        if !valid_id(&self.profile_id) {
            return Err(AccountingError::InvalidIdentifier);
        }
        if self.revision == 0 {
            return Err(AccountingError::ZeroProfileRevision);
        }
        Ok(())
    }

    fn encode(&self, writer: &mut CanonicalWriter) {
        writer.text(self.profile_id.as_str());
        writer.u64(self.revision);
        writer.digest(self.digest);
    }
}

/// Exact denomination profile. AssetId remains opaque; this profile binds
/// decimal scale and issuer/network/fungibility semantics for one ledger.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AssetUnitProfile {
    pub id: ReferenceId,
    pub revision: u64,
    pub asset_id: AssetId,
    pub decimal_scale: u8,
    pub issuer: Option<ReferenceId>,
    pub network: Option<ReferenceId>,
    pub fungibility_domain: ReferenceId,
}

impl AssetUnitProfile {
    pub fn validate(&self) -> Result<(), AccountingError> {
        if !valid_id(&self.id) || !valid_id(&self.fungibility_domain) {
            return Err(AccountingError::InvalidIdentifier);
        }
        if self.revision == 0 {
            return Err(AccountingError::ZeroProfileRevision);
        }
        if self.decimal_scale > MAX_DECIMAL_SCALE {
            return Err(AccountingError::InvalidDecimalScale);
        }
        if self.issuer.as_ref().is_some_and(|v| !valid_id(v))
            || self.network.as_ref().is_some_and(|v| !valid_id(v))
        {
            return Err(AccountingError::InvalidIdentifier);
        }
        Ok(())
    }

    fn canonical_bytes(&self) -> Result<Vec<u8>, AccountingError> {
        self.validate()?;
        let mut writer = CanonicalWriter::new(UNIT_DOMAIN);
        writer.text(self.id.as_str());
        writer.u64(self.revision);
        writer.text(self.asset_id.as_str());
        writer.u8(self.decimal_scale);
        match &self.issuer {
            Some(value) => { writer.u8(1); writer.text(value.as_str()); }
            None => writer.u8(0),
        }
        match &self.network {
            Some(value) => { writer.u8(1); writer.text(value.as_str()); }
            None => writer.u8(0),
        }
        writer.text(self.fungibility_domain.as_str());
        Ok(writer.finish())
    }

    pub fn commitment(&self) -> Result<Digest32, AccountingError> {
        let mut writer = CanonicalWriter::new(b"mycelix:fin-eco-006:asset-unit-commitment:v1\0");
        writer.field(&self.canonical_bytes()?);
        Ok(writer.hash())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LedgerAccountProfile {
    pub account_id: ReferenceId,
    pub schema_revision: u64,
    pub reporting_entity: ReferenceId,
    pub normal_balance: NormalBalance,
    pub allowed_unit_profiles: BTreeSet<ReferenceId>,
}

impl LedgerAccountProfile {
    fn validate(&self) -> Result<(), AccountingError> {
        if !valid_id(&self.account_id) || !valid_id(&self.reporting_entity) {
            return Err(AccountingError::InvalidIdentifier);
        }
        if self.schema_revision == 0 {
            return Err(AccountingError::ZeroProfileRevision);
        }
        if self.allowed_unit_profiles.is_empty()
            || self.allowed_unit_profiles.iter().any(|v| !valid_id(v))
        {
            return Err(AccountingError::InvalidAccountUnitAllowlist);
        }
        Ok(())
    }

    fn encode(&self, writer: &mut CanonicalWriter) {
        writer.text(self.account_id.as_str());
        writer.u64(self.schema_revision);
        writer.text(self.reporting_entity.as_str());
        writer.u8(match self.normal_balance {
            NormalBalance::Debit => 0,
            NormalBalance::Credit => 1,
        });
        writer.u64(self.allowed_unit_profiles.len() as u64);
        for id in &self.allowed_unit_profiles {
            writer.text(id.as_str());
        }
    }
}

/// Immutable semantic/account taxonomy snapshot for one ledger.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AccountingLedgerProfile {
    pub ledger_id: ReferenceId,
    pub revision: u64,
    pub reporting_entity: ReferenceId,
    pub domain: AccountingBookDomain,
    pub units: BTreeMap<ReferenceId, AssetUnitProfile>,
    pub accounts: BTreeMap<ReferenceId, LedgerAccountProfile>,
}

impl AccountingLedgerProfile {
    pub fn validate(&self) -> Result<(), AccountingError> {
        if !valid_id(&self.ledger_id) || !valid_id(&self.reporting_entity) {
            return Err(AccountingError::InvalidIdentifier);
        }
        if self.revision == 0 {
            return Err(AccountingError::ZeroProfileRevision);
        }
        if self.units.is_empty() || self.units.len() > MAX_UNIT_PROFILES {
            return Err(AccountingError::InvalidUnitProfileCount);
        }
        if self.accounts.is_empty() || self.accounts.len() > MAX_ACCOUNTS {
            return Err(AccountingError::InvalidAccountCount);
        }
        for (key, unit) in &self.units {
            if key != &unit.id { return Err(AccountingError::ProfileMapKeyMismatch); }
            unit.validate()?;
        }
        for (key, account) in &self.accounts {
            if key != &account.account_id { return Err(AccountingError::ProfileMapKeyMismatch); }
            account.validate()?;
            if account.reporting_entity != self.reporting_entity {
                return Err(AccountingError::AccountEntityMismatch);
            }
            if account.allowed_unit_profiles.iter().any(|id| !self.units.contains_key(id)) {
                return Err(AccountingError::UnknownUnitProfile);
            }
        }
        Ok(())
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, AccountingError> {
        self.validate()?;
        let mut writer = CanonicalWriter::new(LEDGER_DOMAIN);
        writer.u16(ACCOUNTING_JOURNAL_SCHEMA_V1);
        writer.text(self.ledger_id.as_str());
        writer.u64(self.revision);
        writer.text(self.reporting_entity.as_str());
        writer.u8(match self.domain {
            AccountingBookDomain::Financial => 0,
            AccountingBookDomain::Encumbrance => 1,
            AccountingBookDomain::Operational => 2,
            AccountingBookDomain::Management => 3,
        });
        writer.u64(self.units.len() as u64);
        for unit in self.units.values() {
            writer.field(&unit.canonical_bytes()?);
        }
        writer.u64(self.accounts.len() as u64);
        for account in self.accounts.values() {
            account.encode(&mut writer);
        }
        Ok(writer.finish())
    }

    pub fn commitment(&self) -> Result<Digest32, AccountingError> {
        Ok(Digest32(Sha256::digest(self.canonical_bytes()?).into()))
    }
}

/// The policy admits only exact settlement-profile revisions and a pinned
/// ledger-profile commitment. It is a classification rule, not execution authority.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AccountingProjectionPolicy {
    pub policy_id: ReferenceId,
    pub revision: u64,
    pub ledger_profile_digest: Digest32,
    pub allowed_settlement_profiles: BTreeSet<SettlementProfileBinding>,
    pub minimum_postings: usize,
    pub maximum_postings: usize,
    pub allow_reversals: bool,
}

impl AccountingProjectionPolicy {
    pub fn validate(&self) -> Result<(), AccountingError> {
        if !valid_id(&self.policy_id) {
            return Err(AccountingError::InvalidIdentifier);
        }
        if self.revision == 0 {
            return Err(AccountingError::ZeroProfileRevision);
        }
        if self.allowed_settlement_profiles.is_empty() {
            return Err(AccountingError::EmptySourceProfileAllowlist);
        }
        for profile in &self.allowed_settlement_profiles {
            profile.validate()?;
        }
        if self.minimum_postings < 2
            || self.maximum_postings < self.minimum_postings
            || self.maximum_postings > MAX_POSTINGS_PER_ENTRY
        {
            return Err(AccountingError::InvalidPostingBounds);
        }
        Ok(())
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, AccountingError> {
        self.validate()?;
        let mut writer = CanonicalWriter::new(POLICY_DOMAIN);
        writer.u16(ACCOUNTING_JOURNAL_SCHEMA_V1);
        writer.text(self.policy_id.as_str());
        writer.u64(self.revision);
        writer.digest(self.ledger_profile_digest);
        writer.u64(self.allowed_settlement_profiles.len() as u64);
        for profile in &self.allowed_settlement_profiles {
            profile.encode(&mut writer);
        }
        writer.u64(self.minimum_postings as u64);
        writer.u64(self.maximum_postings as u64);
        writer.u8(u8::from(self.allow_reversals));
        Ok(writer.finish())
    }

    pub fn commitment(&self) -> Result<Digest32, AccountingError> {
        Ok(Digest32(Sha256::digest(self.canonical_bytes()?).into()))
    }
}

/// Constructor-controlled economic source. Its only public constructor binds
/// an exact QualifiedSettlement and its full evidence frontier.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedEconomicSource {
    source_event_id: ReferenceId,
    settlement_subject: ReferenceId,
    attempt_id: ReferenceId,
    rail: ReferenceId,
    network: ReferenceId,
    settlement_profile: SettlementProfileBinding,
    amount: AssetAmount,
    evidence_frontier: Digest32,
    qualified_at_unix_ms: u64,
    source_commitment: Digest32,
}

impl QualifiedEconomicSource {
    pub fn from_qualified_settlement(
        source_event_id: ReferenceId,
        settlement: &QualifiedSettlement,
    ) -> Result<Self, AccountingError> {
        if !valid_id(&source_event_id) {
            return Err(AccountingError::InvalidIdentifier);
        }
        if settlement.amount().atomic_units() == 0 {
            return Err(AccountingError::ZeroSourceAmount);
        }
        let attempt_id = settlement.attempt().as_ref_id().clone();
        let settlement_profile = SettlementProfileBinding::from_qualified_settlement(settlement);
        let mut writer = CanonicalWriter::new(SOURCE_DOMAIN);
        writer.text(source_event_id.as_str());
        writer.text(settlement.subject().as_str());
        writer.text(attempt_id.as_str());
        writer.text(settlement.rail().as_str());
        writer.text(settlement.network().as_str());
        settlement_profile.encode(&mut writer);
        writer.text(settlement.amount().asset().as_str());
        writer.u64(settlement.amount().atomic_units());
        writer.digest(settlement.evidence_frontier());
        writer.u64(settlement.qualified_at_unix_ms());
        writer.u8(match settlement.reversal_model() {
            ReversalModel::MayReverse => 0,
            ReversalModel::DeclaredTerminalByProfile => 1,
        });
        writer.u64(settlement.operations().len() as u64);
        for (operation_id, operation) in settlement.operations() {
            writer.text(operation_id.as_str());
            writer.u64(operation.revision());
            writer.text(operation.amount().asset().as_str());
            writer.u64(operation.amount().atomic_units());
            writer.u64(operation.evidence_ids().len() as u64);
            for evidence_id in operation.evidence_ids() {
                writer.text(evidence_id.as_str());
            }
        }
        let source_commitment = writer.hash();
        Ok(Self {
            source_event_id,
            settlement_subject: settlement.subject().clone(),
            attempt_id,
            rail: settlement.rail().clone(),
            network: settlement.network().clone(),
            settlement_profile,
            amount: settlement.amount().clone(),
            evidence_frontier: settlement.evidence_frontier(),
            qualified_at_unix_ms: settlement.qualified_at_unix_ms(),
            source_commitment,
        })
    }

    pub fn source_event_id(&self) -> &ReferenceId { &self.source_event_id }
    pub fn amount(&self) -> &AssetAmount { &self.amount }
    pub fn source_commitment(&self) -> Digest32 { self.source_commitment }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AccountingAccountRef {
    pub account_id: ReferenceId,
    pub schema_revision: u64,
    pub reporting_entity: ReferenceId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JournalPosting {
    pub account: AccountingAccountRef,
    pub unit_profile_id: ReferenceId,
    pub side: PostingSide,
    pub amount: AssetAmount,
    pub semantic_role: ReferenceId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JournalEntryDraft {
    pub reporting_period: ReferenceId,
    pub effective_at_unix_ms: u64,
    pub source: QualifiedEconomicSource,
    pub postings: Vec<JournalPosting>,
    pub reverses: Option<Digest32>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedJournalEntry {
    journal_id: Digest32,
    ledger_profile_digest: Digest32,
    projection_policy_digest: Digest32,
    source_application_key: Digest32,
    source: QualifiedEconomicSource,
    reporting_entity: ReferenceId,
    reporting_period: ReferenceId,
    effective_at_unix_ms: u64,
    postings: Vec<JournalPosting>,
    reverses: Option<Digest32>,
    canonical_bytes: Vec<u8>,
}

impl QualifiedJournalEntry {
    pub fn journal_id(&self) -> Digest32 { self.journal_id }
    pub fn source_event_id(&self) -> &ReferenceId { self.source.source_event_id() }
    pub fn postings(&self) -> &[JournalPosting] { &self.postings }
    pub fn reverses(&self) -> Option<Digest32> { self.reverses }
    pub fn canonical_bytes(&self) -> &[u8] { &self.canonical_bytes }
    pub fn source_commitment(&self) -> Digest32 { self.source.source_commitment() }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccountingError {
    InvalidIdentifier,
    ZeroProfileRevision,
    InvalidDecimalScale,
    InvalidAccountCount,
    InvalidUnitProfileCount,
    InvalidPostingBounds,
    InvalidAccountUnitAllowlist,
    ProfileMapKeyMismatch,
    AccountEntityMismatch,
    UnknownUnitProfile,
    EmptySourceProfileAllowlist,
    LedgerPolicyMismatch,
    UnsupportedSettlementProfile,
    ZeroSourceAmount,
    InvalidReportingEntity,
    InvalidReportingPeriod,
    InvalidEffectiveTime,
    PostingCountOutOfBounds,
    ZeroPostingAmount,
    PostingAssetMismatch,
    PostingUnitNotAllowed,
    PostingAccountMismatch,
    DuplicatePostingSemantic,
    MultipleUnitProfilesInOneEntry,
    UnbalancedPostingTotals,
    SourceAmountMismatch,
    ReversalNotAllowed,
    LedgerProfileMismatch,
    DuplicateJournalId,
    SourceProjectionConflict,
    ReversalTargetMissing,
    ReversalOfReversal,
    ReversalPostingMismatch,
    ReversalExceedsOriginal,
    CounterOverflow,
}

impl fmt::Display for AccountingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for AccountingError {}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct PostingIdentity {
    account_id: ReferenceId,
    unit_profile_id: ReferenceId,
    side: PostingSide,
    semantic_role: ReferenceId,
}

impl PostingIdentity {
    fn from_posting(posting: &JournalPosting) -> Self {
        Self {
            account_id: posting.account.account_id.clone(),
            unit_profile_id: posting.unit_profile_id.clone(),
            side: posting.side,
            semantic_role: posting.semantic_role.clone(),
        }
    }
}

fn sort_postings(postings: &mut [JournalPosting]) {
    postings.sort_by(|a, b| {
        a.account.account_id.cmp(&b.account.account_id)
            .then_with(|| a.unit_profile_id.cmp(&b.unit_profile_id))
            .then_with(|| a.side.cmp(&b.side))
            .then_with(|| a.semantic_role.cmp(&b.semantic_role))
            .then_with(|| a.amount.atomic_units().cmp(&b.amount.atomic_units()))
    });
}

fn encode_posting(writer: &mut CanonicalWriter, posting: &JournalPosting) {
    writer.text(posting.account.account_id.as_str());
    writer.u64(posting.account.schema_revision);
    writer.text(posting.account.reporting_entity.as_str());
    writer.text(posting.unit_profile_id.as_str());
    writer.u8(posting.side.tag());
    writer.text(posting.amount.asset().as_str());
    writer.u64(posting.amount.atomic_units());
    writer.text(posting.semantic_role.as_str());
}

/// Validate and canonicalize one proposed entry under an exact ledger profile
/// and pinned policy. This v1 profile supports one asset-unit profile per entry.
pub fn qualify_journal_entry(
    ledger: &AccountingLedgerProfile,
    policy: &AccountingProjectionPolicy,
    draft: JournalEntryDraft,
) -> Result<QualifiedJournalEntry, AccountingError> {
    ledger.validate()?;
    policy.validate()?;
    let ledger_digest = ledger.commitment()?;
    let policy_digest = policy.commitment()?;
    if policy.ledger_profile_digest != ledger_digest {
        return Err(AccountingError::LedgerPolicyMismatch);
    }
    if !policy.allowed_settlement_profiles.contains(&draft.source.settlement_profile) {
        return Err(AccountingError::UnsupportedSettlementProfile);
    }
    if draft.source.amount.atomic_units() == 0 {
        return Err(AccountingError::ZeroSourceAmount);
    }
    if draft.reporting_entity != ledger.reporting_entity {
        return Err(AccountingError::InvalidReportingEntity);
    }
    if !valid_id(&draft.reporting_period) {
        return Err(AccountingError::InvalidReportingPeriod);
    }
    if draft.effective_at_unix_ms == 0 {
        return Err(AccountingError::InvalidEffectiveTime);
    }
    if draft.postings.len() < policy.minimum_postings
        || draft.postings.len() > policy.maximum_postings
    {
        return Err(AccountingError::PostingCountOutOfBounds);
    }
    if draft.reverses.is_some() && !policy.allow_reversals {
        return Err(AccountingError::ReversalNotAllowed);
    }

    let source_asset = draft.source.amount.asset();
    let mut selected_unit_profile: Option<ReferenceId> = None;
    let mut identities = BTreeSet::new();
    let mut debits = 0u128;
    let mut credits = 0u128;
    let mut postings = draft.postings;

    for posting in &postings {
        if !valid_id(&posting.semantic_role) {
            return Err(AccountingError::InvalidIdentifier);
        }
        if posting.amount.atomic_units() == 0 {
            return Err(AccountingError::ZeroPostingAmount);
        }
        if posting.amount.asset() != source_asset {
            return Err(AccountingError::PostingAssetMismatch);
        }
        if posting.account.reporting_entity != draft.reporting_entity {
            return Err(AccountingError::InvalidReportingEntity);
        }
        let account = ledger.accounts.get(&posting.account.account_id)
            .ok_or(AccountingError::PostingAccountMismatch)?;
        if account.schema_revision != posting.account.schema_revision
            || account.reporting_entity != posting.account.reporting_entity
        {
            return Err(AccountingError::PostingAccountMismatch);
        }
        if !account.allowed_unit_profiles.contains(&posting.unit_profile_id) {
            return Err(AccountingError::PostingUnitNotAllowed);
        }
        let unit = ledger.units.get(&posting.unit_profile_id)
            .ok_or(AccountingError::UnknownUnitProfile)?;
        if unit.asset_id != *source_asset || unit.asset_id != *posting.amount.asset() {
            return Err(AccountingError::PostingAssetMismatch);
        }
        match &selected_unit_profile {
            Some(selected) if selected != &posting.unit_profile_id => {
                return Err(AccountingError::MultipleUnitProfilesInOneEntry);
            }
            None => selected_unit_profile = Some(posting.unit_profile_id.clone()),
            _ => {}
        }
        if !identities.insert(PostingIdentity::from_posting(posting)) {
            return Err(AccountingError::DuplicatePostingSemantic);
        }
        let total = match posting.side {
            PostingSide::Debit => &mut debits,
            PostingSide::Credit => &mut credits,
        };
        *total = total.checked_add(u128::from(posting.amount.atomic_units()))
            .ok_or(AccountingError::CounterOverflow)?;
    }
    if debits != credits {
        return Err(AccountingError::UnbalancedPostingTotals);
    }
    if debits != u128::from(draft.source.amount.atomic_units()) {
        return Err(AccountingError::SourceAmountMismatch);
    }
    sort_postings(&mut postings);

    let mut source_key_writer = CanonicalWriter::new(SOURCE_APPLICATION_DOMAIN);
    source_key_writer.digest(ledger_digest);
    source_key_writer.text(draft.reporting_entity.as_str());
    source_key_writer.text(draft.source.source_event_id.as_str());
    let source_application_key = source_key_writer.hash();

    let mut writer = CanonicalWriter::new(ENTRY_DOMAIN);
    writer.u16(ACCOUNTING_JOURNAL_SCHEMA_V1);
    writer.digest(ledger_digest);
    writer.digest(policy_digest);
    writer.text(draft.source.source_event_id.as_str());
    writer.digest(draft.source.source_commitment);
    writer.text(draft.reporting_entity.as_str());
    writer.text(draft.reporting_period.as_str());
    writer.u64(draft.effective_at_unix_ms);
    writer.digest(draft.source.evidence_frontier);
    writer.u8(u8::from(draft.reverses.is_some()));
    if let Some(original) = draft.reverses { writer.digest(original); }
    writer.u64(postings.len() as u64);
    for posting in &postings { encode_posting(&mut writer, posting); }
    let canonical_bytes = writer.finish();
    let journal_id = Digest32(Sha256::digest(&canonical_bytes).into());

    Ok(QualifiedJournalEntry {
        journal_id,
        ledger_profile_digest: ledger_digest,
        projection_policy_digest: policy_digest,
        source_application_key,
        source: draft.source,
        reporting_entity: draft.reporting_entity,
        reporting_period: draft.reporting_period,
        effective_at_unix_ms: draft.effective_at_unix_ms,
        postings,
        reverses: draft.reverses,
        canonical_bytes,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AppendDisposition {
    Appended,
    IdempotentReplay,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct ReversalPostingKey {
    account_id: ReferenceId,
    unit_profile_id: ReferenceId,
    original_side: PostingSide,
    semantic_role: ReferenceId,
}

impl ReversalPostingKey {
    fn from_reversal(posting: &JournalPosting) -> Self {
        Self {
            account_id: posting.account.account_id.clone(),
            unit_profile_id: posting.unit_profile_id.clone(),
            original_side: posting.side.opposite(),
            semantic_role: posting.semantic_role.clone(),
        }
    }
}

/// Pure append-only reference book. Durable persistence, distributed
/// idempotency and period-close completeness are separate qualification tracks.
#[derive(Clone, Debug)]
pub struct JournalBook {
    ledger_profile_digest: Digest32,
    entries: BTreeMap<Digest32, QualifiedJournalEntry>,
    source_index: BTreeMap<Digest32, (Digest32, Digest32)>,
    reversed_amounts: BTreeMap<(Digest32, ReversalPostingKey), u64>,
}

impl JournalBook {
    pub fn new(ledger: &AccountingLedgerProfile) -> Result<Self, AccountingError> {
        ledger.validate()?;
        Ok(Self {
            ledger_profile_digest: ledger.commitment()?,
            entries: BTreeMap::new(),
            source_index: BTreeMap::new(),
            reversed_amounts: BTreeMap::new(),
        })
    }

    pub fn entries(&self) -> &BTreeMap<Digest32, QualifiedJournalEntry> {
        &self.entries
    }

    pub fn append(
        &mut self,
        entry: QualifiedJournalEntry,
    ) -> Result<AppendDisposition, AccountingError> {
        if entry.ledger_profile_digest != self.ledger_profile_digest {
            return Err(AccountingError::LedgerProfileMismatch);
        }
        if let Some((source_commitment, journal_id)) =
            self.source_index.get(&entry.source_application_key)
        {
            if source_commitment == &entry.source.source_commitment
                && journal_id == &entry.journal_id
            {
                return Ok(AppendDisposition::IdempotentReplay);
            }
            return Err(AccountingError::SourceProjectionConflict);
        }
        if self.entries.contains_key(&entry.journal_id) {
            return Err(AccountingError::DuplicateJournalId);
        }

        let mut reversal_updates = Vec::new();
        if let Some(original_id) = entry.reverses {
            let original = self.entries.get(&original_id)
                .ok_or(AccountingError::ReversalTargetMissing)?;
            if original.reverses.is_some() {
                return Err(AccountingError::ReversalOfReversal);
            }
            if original.ledger_profile_digest != entry.ledger_profile_digest
                || original.reporting_entity != entry.reporting_entity
            {
                return Err(AccountingError::ReversalPostingMismatch);
            }
            for reversal_posting in &entry.postings {
                let key = ReversalPostingKey::from_reversal(reversal_posting);
                let original_posting = original.postings.iter().find(|posting| {
                    posting.account.account_id == key.account_id
                        && posting.unit_profile_id == key.unit_profile_id
                        && posting.side == key.original_side
                        && posting.semantic_role == key.semantic_role
                }).ok_or(AccountingError::ReversalPostingMismatch)?;
                let previous = self.reversed_amounts
                    .get(&(original_id, key.clone())).copied().unwrap_or(0);
                let updated = previous.checked_add(reversal_posting.amount.atomic_units())
                    .ok_or(AccountingError::CounterOverflow)?;
                if updated > original_posting.amount.atomic_units() {
                    return Err(AccountingError::ReversalExceedsOriginal);
                }
                reversal_updates.push(((original_id, key), updated));
            }
        }

        self.source_index.insert(
            entry.source_application_key,
            (entry.source.source_commitment, entry.journal_id),
        );
        for (key, value) in reversal_updates {
            self.reversed_amounts.insert(key, value);
        }
        self.entries.insert(entry.journal_id, entry);
        Ok(AppendDisposition::Appended)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_business_core::ExecutionAttemptRef;
    use mycelix_finance_settlement::{
        qualify_settlement, FinalityEvidence, FinalityProfile, FinalityProfileRef,
        ObservedSettlementState, SettlementObservation, SettlementSubject,
    };

    fn reference(value: &str) -> ReferenceId {
        ReferenceId::new(value).expect("static reference")
    }

    fn asset() -> AssetId {
        AssetId::new("USD.micro").expect("static asset")
    }

    fn unit_profile() -> AssetUnitProfile {
        AssetUnitProfile {
            id: reference("unit:USD.micro:v1"),
            revision: 1,
            asset_id: asset(),
            decimal_scale: 6,
            issuer: Some(reference("issuer:bank-a")),
            network: Some(reference("network:bank-rail")),
            fungibility_domain: reference("fungibility:bank-a-usd"),
        }
    }

    fn ledger_profile() -> AccountingLedgerProfile {
        let unit = unit_profile();
        let unit_id = unit.id.clone();
        let cash = LedgerAccountProfile {
            account_id: reference("account:cash"),
            schema_revision: 1,
            reporting_entity: reference("entity:treasury"),
            normal_balance: NormalBalance::Debit,
            allowed_unit_profiles: BTreeSet::from([unit_id.clone()]),
        };
        let clearing = LedgerAccountProfile {
            account_id: reference("account:settlement-clearing"),
            schema_revision: 1,
            reporting_entity: reference("entity:treasury"),
            normal_balance: NormalBalance::Credit,
            allowed_unit_profiles: BTreeSet::from([unit_id]),
        };
        AccountingLedgerProfile {
            ledger_id: reference("ledger:treasury"),
            revision: 1,
            reporting_entity: reference("entity:treasury"),
            domain: AccountingBookDomain::Financial,
            units: BTreeMap::from([(unit.id.clone(), unit)]),
            accounts: BTreeMap::from([
                (cash.account_id.clone(), cash),
                (clearing.account_id.clone(), clearing),
            ]),
        }
    }

    fn settlement_profile() -> FinalityProfile {
        FinalityProfile {
            profile_ref: FinalityProfileRef {
                id: reference("finality:bank:v1"),
                revision: 1,
                digest: Digest32::repeat(9),
            },
            rail: reference("rail:bank"),
            network: reference("network:test-bank"),
            required_evidence_kinds: BTreeSet::from([reference("evidence:provider-settlement")]),
            min_distinct_sources: 1,
            max_observation_age_ms: 60_000,
            reversal_model: ReversalModel::MayReverse,
        }
    }

    fn qualified_settlement(source_suffix: &str, amount_units: u64) -> QualifiedSettlement {
        let profile = settlement_profile();
        let subject = SettlementSubject {
            id: reference(&format!("settlement:subject:{source_suffix}")),
            attempt: ExecutionAttemptRef::new(format!("attempt:{source_suffix}"))
                .expect("static attempt"),
            rail: profile.rail.clone(),
            network: profile.network.clone(),
            amount: AssetAmount::new(amount_units, asset()),
            required_profile: profile.profile_ref.clone(),
        };
        let operation_id = reference(&format!("operation:{source_suffix}"));
        let evidence = FinalityEvidence {
            evidence_id: reference(&format!("evidence:{source_suffix}")),
            subject: subject.id.clone(),
            operation_id: operation_id.clone(),
            kind: reference("evidence:provider-settlement"),
            source: reference("source:provider-a"),
            digest: Digest32::repeat(7),
        };
        let observation = SettlementObservation {
            observation_id: reference(&format!("observation:{source_suffix}")),
            subject: subject.id.clone(),
            attempt: subject.attempt.clone(),
            rail: subject.rail.clone(),
            network: subject.network.clone(),
            operation_id,
            revision: 1,
            amount: AssetAmount::new(amount_units, asset()),
            state: ObservedSettlementState::Applied,
            observed_at_unix_ms: 9_000,
            evidence: vec![evidence],
        };
        qualify_settlement(
            &subject, &profile, &[observation], 10_000, Digest32::repeat(8),
        ).expect("valid qualified settlement fixture")
    }

    fn policy(ledger: &AccountingLedgerProfile) -> AccountingProjectionPolicy {
        let profile = settlement_profile();
        AccountingProjectionPolicy {
            policy_id: reference("policy:treasury-settlement-v1"),
            revision: 1,
            ledger_profile_digest: ledger.commitment().expect("ledger profile commitment"),
            allowed_settlement_profiles: BTreeSet::from([SettlementProfileBinding {
                profile_id: profile.profile_ref.id,
                revision: profile.profile_ref.revision,
                digest: profile.profile_ref.digest,
            }]),
            minimum_postings: 2,
            maximum_postings: 16,
            allow_reversals: true,
        }
    }

    fn posting(account: &str, side: PostingSide, units: u64) -> JournalPosting {
        JournalPosting {
            account: AccountingAccountRef {
                account_id: reference(account),
                schema_revision: 1,
                reporting_entity: reference("entity:treasury"),
            },
            unit_profile_id: reference("unit:USD.micro:v1"),
            side,
            amount: AssetAmount::new(units, asset()),
            semantic_role: reference(if account == "account:cash" {
                "role:settlement-cash"
            } else {
                "role:settlement-clearing"
            }),
        }
    }

    fn draft(
        source_suffix: &str,
        amount_units: u64,
        reverses: Option<Digest32>,
    ) -> JournalEntryDraft {
        let source = QualifiedEconomicSource::from_qualified_settlement(
            reference(&format!("economic-event:{source_suffix}")),
            &qualified_settlement(source_suffix, amount_units),
        ).expect("qualified source binding");
        let reversing = reverses.is_some();
        JournalEntryDraft {
            reporting_period: reference("period:2026-11"),
            effective_at_unix_ms: 10_001,
            source,
            postings: vec![
                posting(
                    "account:cash",
                    if reversing { PostingSide::Credit } else { PostingSide::Debit },
                    amount_units,
                ),
                posting(
                    "account:settlement-clearing",
                    if reversing { PostingSide::Debit } else { PostingSide::Credit },
                    amount_units,
                ),
            ],
            reverses,
        }
    }

    #[test]
    fn qualified_settlement_projects_to_exact_balanced_journal() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let entry = qualify_journal_entry(&ledger, &policy, draft("one", 100, None))
            .expect("balanced settlement journal");
        assert_eq!(entry.postings.len(), 2);
        assert_eq!(entry.source_event_id().as_str(), "economic-event:one");
        assert_eq!(entry.journal_id(), Digest32(Sha256::digest(entry.canonical_bytes()).into()));
    }

    #[test]
    fn unbalanced_and_wrong_source_totals_fail_closed() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let mut unbalanced = draft("unbalanced", 100, None);
        unbalanced.postings[1].amount = AssetAmount::new(99, asset());
        assert_eq!(
            qualify_journal_entry(&ledger, &policy, unbalanced),
            Err(AccountingError::UnbalancedPostingTotals)
        );
        let mut wrong_total = draft("wrong-total", 100, None);
        wrong_total.postings[0].amount = AssetAmount::new(101, asset());
        wrong_total.postings[1].amount = AssetAmount::new(101, asset());
        assert_eq!(
            qualify_journal_entry(&ledger, &policy, wrong_total),
            Err(AccountingError::SourceAmountMismatch)
        );
    }

    #[test]
    fn posting_permutation_preserves_identity_but_semantic_change_does_not() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let first = qualify_journal_entry(&ledger, &policy, draft("permutation", 100, None))
            .expect("first journal");
        let mut permuted_draft = draft("permutation", 100, None);
        permuted_draft.postings.reverse();
        let permuted = qualify_journal_entry(&ledger, &policy, permuted_draft)
            .expect("permuted journal");
        assert_eq!(first.journal_id(), permuted.journal_id());
        let mut changed = draft("permutation", 100, None);
        changed.postings[0].semantic_role = reference("role:different-economic-meaning");
        assert_ne!(
            first.journal_id(),
            qualify_journal_entry(&ledger, &policy, changed)
                .expect("balanced but semantically different journal")
                .journal_id()
        );
    }

    #[test]
    fn wrong_entity_account_revision_and_unit_profile_reject() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let mut wrong_entity = draft("entity", 100, None);
        wrong_entity.postings[0].account.reporting_entity = reference("entity:other");
        assert_eq!(
            qualify_journal_entry(&ledger, &policy, wrong_entity),
            Err(AccountingError::InvalidReportingEntity)
        );
        let mut wrong_revision = draft("account", 100, None);
        wrong_revision.postings[0].account.schema_revision = 2;
        assert_eq!(
            qualify_journal_entry(&ledger, &policy, wrong_revision),
            Err(AccountingError::PostingAccountMismatch)
        );
        let mut wrong_unit = draft("unit", 100, None);
        wrong_unit.postings[0].unit_profile_id = reference("unit:USD.nano:v9");
        assert_eq!(
            qualify_journal_entry(&ledger, &policy, wrong_unit),
            Err(AccountingError::PostingUnitNotAllowed)
        );
    }

    #[test]
    fn same_visible_asset_with_different_unit_profile_cannot_be_mixed() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let mut mixed = draft("unit-mix", 100, None);
        mixed.postings[1].unit_profile_id = reference("unit:USD.cents:v1");
        assert_eq!(
            qualify_journal_entry(&ledger, &policy, mixed),
            Err(AccountingError::PostingUnitNotAllowed)
        );
    }

    #[test]
    fn unknown_settlement_profile_is_not_interpreted_heuristically() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let mut source = QualifiedEconomicSource::from_qualified_settlement(
            reference("economic-event:unsupported"),
            &qualified_settlement("unsupported", 100),
        ).expect("source");
        source.settlement_profile.profile_id = reference("finality:unapproved");
        let draft = JournalEntryDraft {
            reporting_period: reference("period:2026-11"),
            effective_at_unix_ms: 10_001,
            source,
            postings: vec![
                posting("account:cash", PostingSide::Debit, 100),
                posting("account:settlement-clearing", PostingSide::Credit, 100),
            ],
            reverses: None,
        };
        assert_eq!(
            qualify_journal_entry(&ledger, &policy, draft),
            Err(AccountingError::UnsupportedSettlementProfile)
        );
    }

    #[test]
    fn journal_book_is_idempotent_and_conflicting_reprojection_rejects() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let entry = qualify_journal_entry(&ledger, &policy, draft("idempotent", 100, None))
            .expect("entry");
        let mut book = JournalBook::new(&ledger).expect("book");
        assert_eq!(book.append(entry.clone()), Ok(AppendDisposition::Appended));
        assert_eq!(book.append(entry), Ok(AppendDisposition::IdempotentReplay));
        assert_eq!(book.entries().len(), 1);

        let mut conflicting = draft("idempotent", 100, None);
        conflicting.postings[0].semantic_role = reference("role:altered");
        let conflicting = qualify_journal_entry(&ledger, &policy, conflicting)
            .expect("balanced but conflicting projection");
        assert_eq!(
            book.append(conflicting),
            Err(AccountingError::SourceProjectionConflict)
        );
        assert_eq!(book.entries().len(), 1);
    }

    #[test]
    fn append_only_partial_reversals_cannot_exceed_original_postings() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let original = qualify_journal_entry(&ledger, &policy, draft("original", 100, None))
            .expect("original journal");
        let original_id = original.journal_id();
        let mut book = JournalBook::new(&ledger).expect("book");
        book.append(original).expect("append original");

        let sixty = qualify_journal_entry(&ledger, &policy, draft("reversal-60", 60, Some(original_id)))
            .expect("qualified partial reversal");
        book.append(sixty).expect("partial reversal");
        let forty = qualify_journal_entry(&ledger, &policy, draft("reversal-40", 40, Some(original_id)))
            .expect("qualified final reversal");
        book.append(forty).expect("remaining original amount");

        let over = qualify_journal_entry(&ledger, &policy, draft("reversal-over", 1, Some(original_id)))
            .expect("balanced but over-reversing source");
        assert_eq!(book.append(over), Err(AccountingError::ReversalExceedsOriginal));
        assert_eq!(book.entries().len(), 3);
    }

    #[test]
    fn reversal_must_target_an_existing_non_reversal_entry() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let unknown = qualify_journal_entry(
            &ledger,
            &policy,
            draft("missing-original", 50, Some(Digest32::repeat(33))),
        ).expect("balanced draft");
        let mut book = JournalBook::new(&ledger).expect("book");
        assert_eq!(book.append(unknown), Err(AccountingError::ReversalTargetMissing));
    }

    #[test]
    fn cross_asset_and_zero_postings_reject() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let mut cross_asset = draft("cross-asset", 100, None);
        cross_asset.postings[1].amount = AssetAmount::new(
            100,
            AssetId::new("EUR.micro").expect("static asset"),
        );
        assert_eq!(
            qualify_journal_entry(&ledger, &policy, cross_asset),
            Err(AccountingError::PostingAssetMismatch)
        );

        let mut zero = draft("zero", 100, None);
        zero.postings[0].amount = AssetAmount::new(0, asset());
        assert_eq!(
            qualify_journal_entry(&ledger, &policy, zero),
            Err(AccountingError::ZeroPostingAmount)
        );
    }

    #[test]
    fn duplicate_semantic_lines_reject() {
        let ledger = ledger_profile();
        let policy = policy(&ledger);
        let mut duplicate = draft("duplicate", 100, None);
        duplicate.postings[1].account.account_id = reference("account:cash");
        duplicate.postings[1].semantic_role = reference("role:settlement-cash");
        assert_eq!(
            qualify_journal_entry(&ledger, &policy, duplicate),
            Err(AccountingError::DuplicatePostingSemantic)
        );
    }
}
