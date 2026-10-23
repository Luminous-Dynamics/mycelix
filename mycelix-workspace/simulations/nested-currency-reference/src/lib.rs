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
    /// Immutable denomination identity; unit changes require a new instrument ID or explicit migration.
    pub unit_code: String,
    /// Number of fractional decimal places represented by the unit code.
    pub scale: u8,
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
        if self.unit_code.trim().is_empty() || self.scale > 18 { return Err(Error::InvalidProfile); }
        if self.revision == 0 || self.max_abs_balance <= 0 {
            return Err(Error::InvalidProfile);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Action { Create, Activate, Amend, Suspend, Retire, Issue, Clear, Settle }

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
    WrongFamily, Inactive, SameParticipant, InvalidAmount, Overflow, BalanceLimit, InstrumentSemanticsChanged,
    InvalidEventId, ReplayMismatch, InvalidCapability, CapabilityExpired,
    StaleRevision, ProfileConflict, ActionNotGranted, InvalidAgreement, AgreementPaused,
    ExposureLimit, MissingGrossExposureCap, NothingOutstanding, SettlementTooLarge,
}

pub fn authorize(
    actor: &str, registry: &Registry, instrument_id: &str, action: Action,
    amount: Option<i64>, now: u64, cap: &Capability,
) -> Result<(), Error> {
    let profile = registry.get(instrument_id)?;
    profile.validate()?;
    if cap.issuer != profile.issuer || cap.delegate != actor || cap.instrument != profile.id {
        return Err(Error::InvalidCapability);
    }
    if !cap.actions.contains(&action) { return Err(Error::ActionNotGranted); }
    if cap.revision != profile.revision { return Err(Error::StaleRevision); }
    if now >= cap.expires_at { return Err(Error::CapabilityExpired); }

    if let Some(value) = amount {
        if value < 0 { return Err(Error::InvalidAmount); }
    }
    let amount_bearing = matches!(action, Action::Issue | Action::Clear | Action::Settle);
    if matches!(action, Action::Issue | Action::Clear) && !profile.active {
        return Err(Error::Inactive);
    }
    if amount_bearing && amount.is_none() { return Err(Error::InvalidAmount); }
    if amount_bearing && cap.max_amount.is_none() { return Err(Error::InvalidCapability); }

    match (cap.max_amount, amount) {
        (Some(limit), Some(value)) => {
            if limit < 0 { return Err(Error::InvalidCapability); }
            if value > limit { return Err(Error::ExposureLimit); }
            if amount_bearing && value == 0 { return Err(Error::InvalidAmount); }
        }
        (Some(_), None) => return Err(Error::InvalidAmount),
        _ => {}
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
    instrument_profiles: BTreeMap<String, Profile>,
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

    pub fn apply(&mut self, registry: &Registry, event: Exchange) -> Result<Apply, Error> {
        if event.id.trim().is_empty() { return Err(Error::InvalidEventId); }
        if let Some(prior) = self.events.get(&event.id) {
            return if prior == &event { Ok(Apply::Duplicate) } else { Err(Error::ReplayMismatch) };
        }
        let profile = registry.get(&event.instrument)?;
        profile.validate()?;
        if event.profile_revision != profile.revision { return Err(Error::StaleRevision); }
        if let Some(existing) = self.instrument_profiles.get(&profile.id) {
            if existing.issuer != profile.issuer || existing.family != profile.family
                || existing.unit_code != profile.unit_code || existing.scale != profile.scale {
                return Err(Error::InstrumentSemanticsChanged);
            }
            if profile.revision < existing.revision { return Err(Error::StaleRevision); }
            if profile.revision == existing.revision && existing != profile {
                return Err(Error::ProfileConflict);
            }
        }
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
        self.instrument_profiles.insert(profile.id.clone(), profile.clone());
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
    community_gross_caps: BTreeMap<(String, String), u64>,
}

impl Clearing {
    pub fn add(&mut self, a: Agreement) -> Result<(), Error> {
        if a.id.trim().is_empty() || a.community_a.trim().is_empty()
            || a.community_b.trim().is_empty() || a.community_a == a.community_b
            || a.instrument.trim().is_empty() || a.revision == 0 || a.max_abs_outstanding <= 0
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

    /// Set a conservative gross-exposure cap across every peering edge that touches a community.
    /// Existing commitments are counted before the new cap is accepted.
    /// Set a conservative gross-exposure cap for one community in one exact instrument.
    /// Raw amounts from unlike instrument domains are never summed together.
    pub fn set_community_gross_cap(
        &mut self, community: &str, instrument: &str, cap: u64
    ) -> Result<(), Error> {
        if community.trim().is_empty() || instrument.trim().is_empty() {
            return Err(Error::InvalidAmount);
        }
        if self.gross_exposure(community, instrument)? > cap {
            return Err(Error::ExposureLimit);
        }
        self.community_gross_caps.insert((community.into(), instrument.into()), cap);
        Ok(())
    }

    fn gross_exposure(&self, community: &str, instrument: &str) -> Result<u64, Error> {
        let mut total = 0_u64;
        for agreement in self.agreements.values().filter(|a|
            (a.community_a.as_str() == community || a.community_b.as_str() == community)
                && a.instrument == instrument) {
            total = total.checked_add(self.outstanding(&agreement.id)?.unsigned_abs())
                .ok_or(Error::Overflow)?;
        }
        Ok(total)
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
        // A missing aggregate limit is not permission for unbounded federation exposure.
        // Both communities must explicitly configure the exact instrument's cap before
        // any new obligation can be recorded. A zero cap deliberately disables new risk.
        for community in [&a.community_a, &a.community_b] {
            let key = (community.to_string(), instrument.to_string());
            if !self.community_gross_caps.contains_key(&key) {
                return Err(Error::MissingGrossExposureCap);
            }
        }
        let current = self.outstanding(agreement_id)?;
        let next = current.checked_add(delta).ok_or(Error::Overflow)?;
        if next.unsigned_abs() > a.max_abs_outstanding as u64 { return Err(Error::ExposureLimit); }

        // Enforce gross exposure across all peering edges, not only the edge being mutated.
        // Count the replacement balance once; do not mutate any ledger until all caps pass.
        for community in [&a.community_a, &a.community_b] {
            let cap_key = (community.to_string(), instrument.to_string());
            let cap = self.community_gross_caps.get(&cap_key)
                .ok_or(Error::MissingGrossExposureCap)?;
            let before = self.gross_exposure(community, instrument)?;
            let after = before.checked_sub(current.unsigned_abs())
                .and_then(|n| n.checked_add(next.unsigned_abs()))
                .ok_or(Error::Overflow)?;
            if after > *cap { return Err(Error::ExposureLimit); }
        }
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
            family, unit_code: format!("unit:{id}"), scale: 0,
            revision: 1, active: true, max_abs_balance: 10_000 }
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
        let mut registry = Registry::default();
        registry.register(p.clone()).unwrap();
        let cap = Capability { issuer: p.issuer.clone(), delegate: "did:federation".into(),
            instrument: p.id.clone(), actions: [Action::Amend].into_iter().collect(),
            revision: 1, expires_at: 100, max_amount: Some(10) };
        assert_eq!(authorize("did:federation", &registry, &p.id, Action::Amend, Some(8), 99, &cap), Ok(()));
        assert_eq!(authorize("did:federation", &registry, &p.id, Action::Issue, Some(8), 99, &cap),
            Err(Error::ActionNotGranted));
        assert_eq!(authorize("did:other", &registry, &p.id, Action::Amend, Some(8), 99, &cap),
            Err(Error::InvalidCapability));
        assert_eq!(authorize("did:federation", &registry, &p.id, Action::Amend, Some(8), 100, &cap),
            Err(Error::CapabilityExpired));
        let mut stale = cap.clone(); stale.revision = 0;
        assert_eq!(authorize("did:federation", &registry, &p.id, Action::Amend, Some(8), 99, &stale),
            Err(Error::StaleRevision));

        // Regression: Some(limit) + None(amount) must not bypass the capability ceiling.
        assert_eq!(authorize("did:federation", &registry, &p.id, Action::Amend, None, 99, &cap),
            Err(Error::InvalidAmount));
        assert_eq!(authorize("did:federation", &registry, &p.id, Action::Amend, Some(-1), 99, &cap),
            Err(Error::InvalidAmount));

        let issue_cap = Capability { actions: [Action::Issue].into_iter().collect(),
            max_amount: None, ..cap.clone() };
        assert_eq!(authorize("did:federation", &registry, &p.id, Action::Issue, Some(1), 99, &issue_cap),
            Err(Error::InvalidCapability));

        let bounded_issue_cap = Capability { max_amount: Some(10), ..issue_cap.clone() };
        assert_eq!(authorize("did:federation", &registry, &p.id, Action::Issue, None, 99, &bounded_issue_cap),
            Err(Error::InvalidAmount));
        assert_eq!(authorize("did:federation", &registry, &p.id, Action::Issue, Some(1), 99, &bounded_issue_cap), Ok(()));

        let inactive = Profile { active: false, ..p.clone() };
        let mut inactive_registry = Registry::default();
        inactive_registry.register(inactive.clone()).unwrap();
        let inactive_cap = Capability { revision: inactive.revision, ..bounded_issue_cap };
        assert_eq!(authorize("did:federation", &inactive_registry, &inactive.id, Action::Issue, Some(1), 99, &inactive_cap),
            Err(Error::Inactive));

        // Suspending new issuance must not prevent a separately authorized unwind.
        let settle_cap = Capability { actions: [Action::Settle].into_iter().collect(),
            ..inactive_cap };
        assert_eq!(authorize("did:federation", &inactive_registry, &inactive.id, Action::Settle, Some(1), 99, &settle_cap), Ok(()));
    }

    #[test]
    fn equal_tickers_do_not_share_balances_or_net_positions() {
        let orchard = profile("cc:orchard", "did:orchard", Family::MutualCredit);
        let makers = profile("cc:makers", "did:makers", Family::MutualCredit);
        let mut registry = Registry::default();
        registry.register(orchard.clone()).unwrap();
        registry.register(makers.clone()).unwrap();
        let mut l = MutualCredit::default();
        l.apply(&registry, exchange("orchard:tx:1", &orchard, 50)).unwrap();
        l.apply(&registry, exchange("makers:tx:1", &makers, 20)).unwrap();
        assert_eq!(l.net(&orchard.id), 0);
        assert_eq!(l.net(&makers.id), 0);
        assert_eq!(l.balance(&orchard.id, "alice"), 50);
        assert_eq!(l.balance(&makers.id, "alice"), 20);
        assert_eq!(l.balance(&orchard.id, "bob"), -50);
        assert_eq!(l.balance(&makers.id, "bob"), -20);
    }

    #[test]
    fn mutual_credit_is_zero_sum_atomic_and_idempotent() {
        let p = profile("cc:orchard", "did:orchard", Family::MutualCredit);
        let mut registry = Registry::default();
        registry.register(p.clone()).unwrap();
        let mut l = MutualCredit::default();
        assert_eq!(l.apply(&registry, exchange("tx:1", &p, 17)), Ok(Apply::Applied));
        assert_eq!(l.net(&p.id), 0);
        let changed_revision = Profile { revision: 2, ..p.clone() };
        let mut newer_registry = Registry::default();
        newer_registry.register(changed_revision).unwrap();
        assert_eq!(l.apply(&newer_registry, exchange("tx:stale", &p, 1)),
            Err(Error::StaleRevision));
        // A same-revision registry that silently changes units must also be rejected.
        let changed_units = Profile { unit_code: "unit:changed".into(), ..p.clone() };
        let mut changed_unit_registry = Registry::default();
        changed_unit_registry.register(changed_units).unwrap();
        assert_eq!(l.apply(&changed_unit_registry, exchange("tx:unit-change", &p, 1)),
            Err(Error::InstrumentSemanticsChanged));

        // A same-revision policy mutation cannot silently enlarge the credit limit.
        let changed_limit = Profile { max_abs_balance: i64::MAX, ..p.clone() };
        let mut changed_limit_registry = Registry::default();
        changed_limit_registry.register(changed_limit).unwrap();
        assert_eq!(l.apply(&changed_limit_registry, exchange("tx:limit-bypass", &p, 1)),
            Err(Error::ProfileConflict));

        // A policy revision can change limits explicitly while preserving identity and units.
        let revision_two = Profile { revision: 2, max_abs_balance: 20, ..p.clone() };
        let mut revision_two_registry = Registry::default();
        revision_two_registry.register(revision_two.clone()).unwrap();
        let revision_two_event = Exchange { id: "tx:v2".into(), instrument: revision_two.id.clone(),
            profile_revision: 2, provider: "alice".into(), receiver: "bob".into(), amount: 1 };
        assert_eq!(l.apply(&revision_two_registry, revision_two_event), Ok(Apply::Applied));
        assert_eq!(l.apply(&registry, exchange("tx:1", &p, 17)), Ok(Apply::Duplicate));
        assert_eq!(l.apply(&registry, exchange("tx:1", &p, 18)), Err(Error::ReplayMismatch));
        assert_eq!(l.balance(&p.id, "alice"), 18);
        assert_eq!(l.balance(&p.id, "bob"), -18);
        assert_eq!(l.event_count(), 2);
    }

    #[test]
    fn family_and_limit_fail_closed_without_partial_mutation() {
        let mut p = profile("cc:small", "did:small", Family::MutualCredit);
        p.max_abs_balance = 5;
        let mut registry = Registry::default();
        registry.register(p.clone()).unwrap();
        let mut l = MutualCredit::default();
        assert_eq!(l.apply(&registry, exchange("tx:1", &p, 6)), Err(Error::BalanceLimit));
        assert_eq!(l.balance(&p.id, "alice"), 0);
        assert_eq!(l.balance(&p.id, "bob"), 0);
        assert_eq!(l.event_count(), 0);
        let r = profile("resource:water", "did:watershed", Family::ResourceCredit);
        registry.register(r.clone()).unwrap();
        assert_eq!(l.apply(&registry, exchange("tx:2", &r, 1)), Err(Error::WrongFamily));
    }

    #[test]
    fn federation_exposure_is_per_edge_and_paused_edges_still_settle() {
        let mut c = Clearing::default();
        c.add(agreement("edge:ab", "cc:local", 100)).unwrap();
        c.add(agreement("edge:ac", "cc:local", 100)).unwrap();
        c.set_community_gross_cap("a", "cc:local", 300).unwrap();
        c.set_community_gross_cap("b", "cc:local", 300).unwrap();
        c.set_community_gross_cap("c", "cc:local", 300).unwrap();
        assert_eq!(c.obligation("ob:stale", "edge:ab", "cc:local", 2, 7),
            Err(Error::StaleRevision));
        assert_eq!(c.outstanding("edge:ab"), Ok(0));
        assert_eq!(c.obligation("ob:1", "edge:ab", "cc:local", 1, 70), Ok(Apply::Applied));
        assert_eq!(c.obligation("ob:2", "edge:ab", "cc:local", 1, 40), Err(Error::ExposureLimit));
        assert_eq!(c.obligation("ob:3", "edge:ac", "cc:local", 1, 90), Ok(Apply::Applied));
        c.pause("edge:ab").unwrap();
        assert_eq!(c.obligation("ob:4", "edge:ab", "cc:local", 1, 1), Err(Error::AgreementPaused));
        assert_eq!(c.settle("set:1", "edge:ab", "cc:local", 1, 20), Ok(Apply::Applied));
        assert_eq!(c.outstanding("edge:ab"), Ok(50));
        assert_eq!(c.settle("set:1", "edge:ab", "cc:local", 1, 20), Ok(Apply::Duplicate));
        assert_eq!(c.settle("set:1", "edge:ab", "cc:local", 1, 21), Err(Error::ReplayMismatch));
    }

    #[test]
    fn settlement_cannot_overrun_outstanding_and_wrong_instrument_is_rejected() {
        let mut c = Clearing::default();
        c.add(agreement("edge:ab", "cc:local", 100)).unwrap();
        c.set_community_gross_cap("a", "cc:local", 100).unwrap();
        c.set_community_gross_cap("b", "cc:local", 100).unwrap();
        c.obligation("ob:1", "edge:ab", "cc:local", 1, -25).unwrap();
        assert_eq!(c.settle("set:1", "edge:ab", "cc:local", 1, 26), Err(Error::SettlementTooLarge));
        assert_eq!(c.obligation("ob:2", "edge:ab", "cc:other", 1, 1), Err(Error::InvalidAgreement));
        assert_eq!(c.obligation("ob:3", "edge:ab", "cc:local", 2, 1), Err(Error::StaleRevision));
        assert_eq!(c.outstanding("edge:ab"), Ok(-25));
    }

    #[test]
    fn aggregate_gross_exposure_cannot_be_double_committed_across_federations() {
        let mut c = Clearing::default();
        c.add(agreement("edge:ab", "cc:local", 100)).unwrap();
        c.add(agreement("edge:ac", "cc:local", 100)).unwrap();
        c.add(agreement("edge:ad", "cc:other", 100)).unwrap();
        c.set_community_gross_cap("a", "cc:local", 100).unwrap();
        c.set_community_gross_cap("b", "cc:local", 100).unwrap();
        c.set_community_gross_cap("c", "cc:local", 100).unwrap();
        c.set_community_gross_cap("a", "cc:other", 100).unwrap();
        c.set_community_gross_cap("d", "cc:other", 100).unwrap();
        c.obligation("ob:ab-70", "edge:ab", "cc:local", 1, 70).unwrap();
        // Unlike instruments have distinct amount domains and are not summed by raw magnitude.
        c.obligation("ob:ad-90", "edge:ad", "cc:other", 1, 90).unwrap();
        assert_eq!(c.obligation("ob:ac-pending", "edge:ac", "cc:local", 1, 40),
            Err(Error::ExposureLimit));
        assert_eq!(c.outstanding("edge:ac"), Ok(0));
        // A rejected event ID is not consumed: the same ID can retry with an allowed payload.
        assert_eq!(c.obligation("ob:ac-pending", "edge:ac", "cc:local", 1, 30),
            Ok(Apply::Applied));
        assert_eq!(c.gross_exposure("a", "cc:local"), Ok(100));
        assert_eq!(c.gross_exposure("a", "cc:other"), Ok(90));
        assert_eq!(c.outstanding("edge:ab"), Ok(70));
        assert_eq!(c.outstanding("edge:ac"), Ok(30));
        assert_eq!(c.outstanding("edge:ad"), Ok(90));
    }

    #[test]
    fn clearing_requires_explicit_aggregate_caps_and_zero_cap_disables_new_obligations() {
        let mut c = Clearing::default();
        c.add(agreement("edge:ab", "cc:local", 100)).unwrap();

        // Missing aggregate policy is not treated as unlimited credit.
        assert_eq!(c.obligation("ob:1", "edge:ab", "cc:local", 1, 10),
            Err(Error::MissingGrossExposureCap));
        assert_eq!(c.outstanding("edge:ab"), Ok(0));

        // A zero cap explicitly means no new exposure; the failed event ID is not consumed.
        c.set_community_gross_cap("a", "cc:local", 0).unwrap();
        c.set_community_gross_cap("b", "cc:local", 50).unwrap();
        assert_eq!(c.obligation("ob:1", "edge:ab", "cc:local", 1, 10),
            Err(Error::ExposureLimit));
        assert_eq!(c.outstanding("edge:ab"), Ok(0));

        // Once both communities explicitly permit the exposure, that exact event can retry.
        c.set_community_gross_cap("a", "cc:local", 50).unwrap();
        assert_eq!(c.obligation("ob:1", "edge:ab", "cc:local", 1, 10), Ok(Apply::Applied));
        assert_eq!(c.outstanding("edge:ab"), Ok(10));
    }

    #[test]
    fn one_million_generated_exchanges_preserve_net_position() {
        let p = profile("cc:stress", "did:stress", Family::MutualCredit);
        let mut registry = Registry::default();
        registry.register(p.clone()).unwrap();
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
            l.apply(&registry, event).unwrap();
            if i % 10_000 == 0 { assert_eq!(l.net(&p.id), 0, "drift at op {i}"); }
        }
        assert_eq!(l.net(&p.id), 0);
        assert_eq!(l.event_count(), 1_000_000);
    }

    #[test]
    fn one_hundred_thousand_duplicate_retries_never_reapply_credit() {
        let p = profile("cc:retry", "did:retry", Family::MutualCredit);
        let mut registry = Registry::default();
        registry.register(p.clone()).unwrap();
        let mut l = MutualCredit::default();
        let event = exchange("retry:1", &p, 9);
        assert_eq!(l.apply(&registry, event.clone()), Ok(Apply::Applied));
        for _ in 0..100_000 { assert_eq!(l.apply(&registry, event.clone()), Ok(Apply::Duplicate)); }
        assert_eq!(l.balance(&p.id, "alice"), 9);
        assert_eq!(l.balance(&p.id, "bob"), -9);
        assert_eq!(l.event_count(), 1);
    }
}
