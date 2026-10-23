#![forbid(unsafe_code)]

//! Dependency-free safety reference model for nested community currencies.
//! This is not a Holochain zome, production ledger, or proof of real-world backing.
//! It models identifier non-aliasing, narrow delegated authority, mutual-credit
//! conservation/replay behavior, and bounded per-agreement federation exposure.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Family { MutualCredit, TimeCredit, ReserveBacked, ResourceCredit, Reputation }

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Profile {
    pub id: String,
    pub issuer: String,
    pub symbol: String,
    pub family: Family,
    pub revision: u64,
    pub active: bool,
    pub max_abs_balance: i64,
}

impl Profile {
    pub fn validate(&self) -> Result<(), Error> {
        if self.id.trim().is_empty() || self.issuer.trim().is_empty() {
            return Err(Error::InvalidIdentity);
        }
        if self.symbol.trim().is_empty() { return Err(Error::InvalidSymbol); }
        if self.revision == 0 || self.max_abs_balance <= 0 {
            return Err(Error::InvalidProfile);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Action { Create, Activate, Amend, Suspend, Retire, Issue, Clear }

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Capability {
    pub issuer: String,
    pub delegate: String,
    pub instrument: String,
    pub actions: BTreeSet<Action>,
    pub revision: u64,
    pub expires_at: u64,
    pub max_amount: Option<i64>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Error {
    InvalidIdentity, InvalidSymbol, InvalidProfile, DuplicateId, UnknownInstrument,
    WrongFamily, Inactive, SameParticipant, InvalidAmount, Overflow, BalanceLimit,
    InvalidEventId, ReplayMismatch, InvalidCapability, CapabilityExpired,
    StaleRevision, ActionNotGranted, InvalidAgreement, AgreementPaused,
    ExposureLimit, NothingOutstanding, SettlementTooLarge,
}

pub fn authorize(
    actor: &str, profile: &Profile, action: Action, amount: Option<i64>,
    now: u64, cap: &Capability,
) -> Result<(), Error> {
    if cap.issuer != profile.issuer || cap.delegate != actor || cap.instrument != profile.id {
        return Err(Error::InvalidCapability);
    }
    if !cap.actions.contains(&action) { return Err(Error::ActionNotGranted); }
    if cap.revision != profile.revision { return Err(Error::StaleRevision); }
    if now >= cap.expires_at { return Err(Error::CapabilityExpired); }
    if let (Some(limit), Some(value)) = (cap.max_amount, amount) {
        if value < 0 || value > limit { return Err(Error::ExposureLimit); }
    }
    Ok(())
}

#[derive(Default)]
pub struct Registry(BTreeMap<String, Profile>);

impl Registry {
    pub fn register(&mut self, profile: Profile) -> Result<(), Error> {
        profile.validate()?;
        if self.0.contains_key(&profile.id) { return Err(Error::DuplicateId); }
        self.0.insert(profile.id.clone(), profile);
        Ok(())
    }
    pub fn get(&self, id: &str) -> Result<&Profile, Error> {
        self.0.get(id).ok_or(Error::UnknownInstrument)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Exchange {
    pub id: String,
    pub instrument: String,
    pub profile_revision: u64,
    pub provider: String,
    pub receiver: String,
    pub amount: i64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Apply { Applied, Duplicate }

#[derive(Default)]
pub struct MutualCredit {
    balances: BTreeMap<(String, String), i64>,
    events: BTreeMap<String, Exchange>,
}

impl MutualCredit {
    pub fn balance(&self, instrument: &str, member: &str) -> i64 {
        *self.balances.get(&(instrument.into(), member.into())).unwrap_or(&0)
    }
    pub fn net(&self, instrument: &str) -> i128 {
        self.balances.iter()
            .filter(|((id, _), _)| id == instrument)
            .map(|(_, value)| i128::from(*value)).sum()
    }
    pub fn event_count(&self) -> usize { self.events.len() }

    pub fn apply(&mut self, profile: &Profile, event: Exchange) -> Result<Apply, Error> {
        if event.id.trim().is_empty() { return Err(Error::InvalidEventId); }
        if let Some(prior) = self.events.get(&event.id) {
            return if prior == &event { Ok(Apply::Duplicate) } else { Err(Error::ReplayMismatch) };
        }
        profile.validate()?;
        if event.instrument != profile.id { return Err(Error::UnknownInstrument); }
        if event.profile_revision != profile.revision { return Err(Error::StaleRevision); }
        if profile.family != Family::MutualCredit { return Err(Error::WrongFamily); }
        if !profile.active { return Err(Error::Inactive); }
        if event.provider == event.receiver { return Err(Error::SameParticipant); }
        if event.amount <= 0 { return Err(Error::InvalidAmount); }

        let pk = (profile.id.clone(), event.provider.clone());
        let rk = (profile.id.clone(), event.receiver.clone());
        let p0 = *self.balances.get(&pk).unwrap_or(&0);
        let r0 = *self.balances.get(&rk).unwrap_or(&0);
        let p1 = p0.checked_add(event.amount).ok_or(Error::Overflow)?;
        let r1 = r0.checked_sub(event.amount).ok_or(Error::Overflow)?;
        if p1.unsigned_abs() > profile.max_abs_balance as u64
            || r1.unsigned_abs() > profile.max_abs_balance as u64 {
            return Err(Error::BalanceLimit);
        }

        // Check all constraints before mutating either account.
        self.balances.insert(pk, p1);
        self.balances.insert(rk, r1);
        self.events.insert(event.id.clone(), event);
        Ok(Apply::Applied)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AgreementStatus { Active, Paused }

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Agreement {
    pub id: String,
    pub community_a: String,
    pub community_b: String,
    pub instrument: String,
    pub revision: u64,
    pub max_abs_outstanding: i64,
    pub status: AgreementStatus,
}

#[derive(Clone, Debug, Eq, PartialEq)]
enum ClearingEvent {
    Obligation { agreement: String, instrument: String, revision: u64, delta: i64 },
    Settlement { agreement: String, instrument: String, revision: u64, amount: u64 },
}

#[derive(Default)]
pub struct Clearing {
    agreements: BTreeMap<String, Agreement>,
    outstanding: BTreeMap<String, i64>,
    events: BTreeMap<String, ClearingEvent>,
}

impl Clearing {
    pub fn add(&mut self, a: Agreement) -> Result<(), Error> {
        if a.id.trim().is_empty() || a.community_a.trim().is_empty()
            || a.community_b.trim().is_empty() || a.community_a == a.community_b
            || a.instrument.trim().is_empty() || a.max_abs_outstanding <= 0
            || self.agreements.contains_key(&a.id) {
            return Err(Error::InvalidAgreement);
        }
        self.outstanding.insert(a.id.clone(), 0);
        self.agreements.insert(a.id.clone(), a);
        Ok(())
    }

    pub fn outstanding(&self, id: &str) -> Result<i64, Error> {
        self.outstanding.get(id).copied().ok_or(Error::InvalidAgreement)
    }

    pub fn pause(&mut self, id: &str) -> Result<(), Error> {
        let a = self.agreements.get_mut(id).ok_or(Error::InvalidAgreement)?;
        a.status = AgreementStatus::Paused;
        Ok(())
    }

    pub fn obligation(&mut self, event_id: &str, agreement_id: &str, instrument: &str,
        revision: u64, delta: i64) -> Result<Apply, Error> {
        if event_id.trim().is_empty() { return Err(Error::InvalidEventId); }
        let ev = ClearingEvent::Obligation {
            agreement: agreement_id.into(), instrument: instrument.into(), revision, delta
        };
        if let Some(old) = self.events.get(event_id) {
            return if old == &ev { Ok(Apply::Duplicate) } else { Err(Error::ReplayMismatch) };
        }
        if delta == 0 { return Err(Error::InvalidAmount); }
        let a = self.agreements.get(agreement_id).ok_or(Error::InvalidAgreement)?;
        if a.instrument != instrument { return Err(Error::InvalidAgreement); }
        if a.revision != revision { return Err(Error::StaleRevision); }
        if a.status != AgreementStatus::Active { return Err(Error::AgreementPaused); }
        let next = self.outstanding(agreement_id)?.checked_add(delta).ok_or(Error::Overflow)?;
        if next.unsigned_abs() > a.max_abs_outstanding as u64 { return Err(Error::ExposureLimit); }
        self.outstanding.insert(agreement_id.into(), next);
        self.events.insert(event_id.into(), ev);
        Ok(Apply::Applied)
    }

    /// Settles existing obligations even when the peering edge is paused.
    pub fn settle(&mut self, event_id: &str, agreement_id: &str, instrument: &str,
        revision: u64, amount: u64) -> Result<Apply, Error> {
        if event_id.trim().is_empty() { return Err(Error::InvalidEventId); }
        let ev = ClearingEvent::Settlement {
            agreement: agreement_id.into(), instrument: instrument.into(), revision, amount
        };
        if let Some(old) = self.events.get(event_id) {
            return if old == &ev { Ok(Apply::Duplicate) } else { Err(Error::ReplayMismatch) };
        }
        if amount == 0 { return Err(Error::InvalidAmount); }
        let a = self.agreements.get(agreement_id).ok_or(Error::InvalidAgreement)?;
        if a.instrument != instrument { return Err(Error::InvalidAgreement); }
        if a.revision != revision { return Err(Error::StaleRevision); }
        let current = self.outstanding(agreement_id)?;
        if current == 0 { return Err(Error::NothingOutstanding); }
        if amount > current.unsigned_abs() { return Err(Error::SettlementTooLarge); }
        let n = i64::try_from(amount).map_err(|_| Error::Overflow)?;
        let next = if current > 0 { current.checked_sub(n) } else { current.checked_add(n) }
            .ok_or(Error::Overflow)?;
        self.outstanding.insert(agreement_id.into(), next);
        self.events.insert(event_id.into(), ev);
        Ok(Apply::Applied)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile(id: &str, issuer: &str, family: Family) -> Profile {
        Profile { id: id.into(), issuer: issuer.into(), symbol: "LOCAL".into(),
            family, revision: 1, active: true, max_abs_balance: 10_000 }
    }
    fn exchange(id: &str, profile: &Profile, amount: i64) -> Exchange {
        Exchange { id: id.into(), instrument: profile.id.clone(), profile_revision: profile.revision,
            provider: "alice".into(), receiver: "bob".into(), amount }
    }
    fn agreement(id: &str, instrument: &str, cap: i64) -> Agreement {
        Agreement { id: id.into(), community_a: "a".into(), community_b: "b".into(),
            instrument: instrument.into(), revision: 1, max_abs_outstanding: cap, status: AgreementStatus::Active }
    }

    #[test]
    fn ticker_is_not_identity_and_duplicate_ids_are_rejected() {
        let mut r = Registry::default();
        r.register(profile("cc:orchard", "did:orchard", Family::MutualCredit)).unwrap();
        r.register(profile("cc:makers", "did:makers", Family::MutualCredit)).unwrap();
        assert_eq!(r.get("cc:orchard").unwrap().symbol, r.get("cc:makers").unwrap().symbol);
        assert_eq!(r.register(profile("cc:orchard", "did:intruder", Family::MutualCredit)),
            Err(Error::DuplicateId));
    }

    #[test]
    fn delegated_authority_is_instrument_action_revision_and_expiry_scoped() {
        let p = profile("cc:child", "did:child", Family::MutualCredit);
        let cap = Capability { issuer: p.issuer.clone(), delegate: "did:federation".into(),
            instrument: p.id.clone(), actions: [Action::Amend].into_iter().collect(),
            revision: 1, expires_at: 100, max_amount: Some(10) };
        assert_eq!(authorize("did:federation", &p, Action::Amend, Some(8), 99, &cap), Ok(()));
        assert_eq!(authorize("did:federation", &p, Action::Issue, Some(8), 99, &cap),
            Err(Error::ActionNotGranted));
        assert_eq!(authorize("did:other", &p, Action::Amend, Some(8), 99, &cap),
            Err(Error::InvalidCapability));
        assert_eq!(authorize("did:federation", &p, Action::Amend, Some(8), 100, &cap),
            Err(Error::CapabilityExpired));
        let mut stale = cap.clone(); stale.revision = 0;
        assert_eq!(authorize("did:federation", &p, Action::Amend, Some(8), 99, &stale),
            Err(Error::StaleRevision));
    }

    #[test]
    fn mutual_credit_is_zero_sum_atomic_and_idempotent() {
        let p = profile("cc:orchard", "did:orchard", Family::MutualCredit);
        let mut l = MutualCredit::default();
        assert_eq!(l.apply(&p, exchange("tx:1", &p, 17)), Ok(Apply::Applied));
        assert_eq!(l.net(&p.id), 0);
        assert_eq!(l.apply(&p, exchange("tx:1", &p, 17)), Ok(Apply::Duplicate));
        assert_eq!(l.apply(&p, exchange("tx:1", &p, 18)), Err(Error::ReplayMismatch));
        assert_eq!(l.balance(&p.id, "alice"), 17);
        assert_eq!(l.balance(&p.id, "bob"), -17);
        assert_eq!(l.event_count(), 1);
    }

    #[test]
    fn family_and_limit_fail_closed_without_partial_mutation() {
        let mut p = profile("cc:small", "did:small", Family::MutualCredit);
        p.max_abs_balance = 5;
        let mut l = MutualCredit::default();
        assert_eq!(l.apply(&p, exchange("tx:1", &p, 6)), Err(Error::BalanceLimit));
        assert_eq!(l.balance(&p.id, "alice"), 0);
        assert_eq!(l.balance(&p.id, "bob"), 0);
        assert_eq!(l.event_count(), 0);
        let r = profile("resource:water", "did:watershed", Family::ResourceCredit);
        assert_eq!(l.apply(&r, exchange("tx:2", &r, 1)), Err(Error::WrongFamily));
    }

    #[test]
    fn federation_exposure_is_per_edge_and_paused_edges_still_settle() {
        let mut c = Clearing::default();
        c.add(agreement("edge:ab", "cc:local", 100)).unwrap();
        c.add(agreement("edge:ac", "cc:local", 100)).unwrap();
        assert_eq!(c.obligation("ob:1", "edge:ab", "cc:local", 1, 70), Ok(Apply::Applied));
        assert_eq!(c.obligation("ob:2", "edge:ab", "cc:local", 1, 40), Err(Error::ExposureLimit));
        assert_eq!(c.obligation("ob:3", "edge:ac", "cc:local", 1, 90), Ok(Apply::Applied));
        c.pause("edge:ab").unwrap();
        assert_eq!(c.obligation("ob:4", "edge:ab", "cc:local", 1, 1), Err(Error::AgreementPaused));
        assert_eq!(c.settle("set:1", "edge:ab", "cc:local", 1, 20), Ok(Apply::Applied));
        assert_eq!(c.outstanding("edge:ab"), Ok(50));
        assert_eq!(c.settle("set:1", "edge:ab", "cc:local", 1, 20), Ok(Apply::Duplicate));
    }

    #[test]
    fn settlement_cannot_overrun_outstanding_and_wrong_instrument_is_rejected() {
        let mut c = Clearing::default();
        c.add(agreement("edge:ab", "cc:local", 100)).unwrap();
        c.obligation("ob:1", "edge:ab", "cc:local", 1, -25).unwrap();
        assert_eq!(c.settle("set:1", "edge:ab", "cc:local", 1, 26), Err(Error::SettlementTooLarge));
        assert_eq!(c.obligation("ob:2", "edge:ab", "cc:other", 1, 1), Err(Error::InvalidAgreement));
        assert_eq!(c.outstanding("edge:ab"), Ok(-25));
    }

    #[test]
    fn one_million_generated_exchanges_preserve_net_position() {
        let p = profile("cc:stress", "did:stress", Family::MutualCredit);
        let mut l = MutualCredit::default();
        let names: Vec<String> = (0..31).map(|i| format!("member:{i}")).collect();
        let mut state: u64 = 0x4d59_4345_4c49_5831;
        for i in 0..1_000_000_u64 {
            state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let provider = state as usize % names.len();
            state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let mut receiver = state as usize % names.len();
            if receiver == provider { receiver = (receiver + 1) % names.len(); }
            let amount = ((state >> 32) % 3 + 1) as i64;
            let event = Exchange { id: format!("stress:{i}"), instrument: p.id.clone(),
                profile_revision: p.revision, provider: names[provider].clone(), receiver: names[receiver].clone(), amount };
            l.apply(&p, event).unwrap();
            if i % 10_000 == 0 { assert_eq!(l.net(&p.id), 0, "drift at op {i}"); }
        }
        assert_eq!(l.net(&p.id), 0);
        assert_eq!(l.event_count(), 1_000_000);
    }

    #[test]
    fn one_hundred_thousand_duplicate_retries_never_reapply_credit() {
        let p = profile("cc:retry", "did:retry", Family::MutualCredit);
        let mut l = MutualCredit::default();
        let event = exchange("retry:1", &p, 9);
        assert_eq!(l.apply(&p, event.clone()), Ok(Apply::Applied));
        for _ in 0..100_000 { assert_eq!(l.apply(&p, event.clone()), Ok(Apply::Duplicate)); }
        assert_eq!(l.balance(&p.id, "alice"), 9);
        assert_eq!(l.balance(&p.id, "bob"), -9);
        assert_eq!(l.event_count(), 1);
    }
}
