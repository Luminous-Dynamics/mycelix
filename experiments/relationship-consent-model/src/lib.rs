use std::collections::{HashSet, VecDeque};

const BOTH_OPTED_IN: u8 = 0b11;
const BOTH_ACCEPTED: u8 = 0b11;
/// Abstract ticks, not wall-clock hours. A product must define an actual policy.
pub const DECLINE_COOLDOWN_TICKS: u64 = 5;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum Actor {
    A,
    B,
}

impl Actor {
    fn bit(self) -> u8 {
        match self {
            Self::A => 0b01,
            Self::B => 0b10,
        }
    }

    fn other(self) -> Self {
        match self {
            Self::A => Self::B,
            Self::B => Self::A,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum CloseReason {
    Declined,
    Withdrawn,
    Expired,
    Blocked,
}

#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub enum Phase {
    Idle,
    Pending {
        epoch: u64,
        accepted_mask: u8,
        opened_at: u64,
        deadline: u64,
    },
    Connected {
        epoch: u64,
        accepted_mask: u8,
    },
    Closed {
        epoch: u64,
        reason: CloseReason,
    },
}

#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct Machine {
    pub phase: Phase,
    pub now: u64,
    pub next_epoch: u64,
    pub opt_in_mask: u8,
    pub blocked_mask: u8,
    /// Current conversation permission, not proof that previously disclosed
    /// information can be made unknown again.
    pub chat_enabled: bool,
    /// Audit-like model counter: each connection transition increments once.
    pub reveal_count: u64,
    pub last_revealed_epoch: Option<u64>,
    /// Pair-level cooldown after decline; ticks are abstract model units.
    pub retry_after: u64,
}

impl Default for Machine {
    fn default() -> Self {
        Self {
            phase: Phase::Idle,
            now: 0,
            next_epoch: 1,
            opt_in_mask: BOTH_OPTED_IN,
            blocked_mask: 0,
            chat_enabled: false,
            reveal_count: 0,
            last_revealed_epoch: None,
            retry_after: 0,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Event {
    Propose { actor: Actor, ttl: u64 },
    Accept { actor: Actor, epoch: u64 },
    Decline { actor: Actor, epoch: u64 },
    Withdraw { actor: Actor, epoch: u64 },
    Block { actor: Actor },
    Unblock { actor: Actor },
    SetDiscoveryOptIn { actor: Actor, enabled: bool },
    AdvanceTime { to: u64 },
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ExplorationReport {
    pub unique_states: usize,
    pub transitions_checked: usize,
    pub maximum_depth_explored: usize,
    pub state_cap_reached: bool,
    pub invariant_violation: bool,
}

impl Machine {
    /// Apply one event. Invalid, stale, replayed, and out-of-order events fail
    /// closed as no-ops. Expiration is processed before any event at the
    /// current time, so accepting exactly at the deadline is too late.
    pub fn apply(&mut self, event: Event) -> bool {
        let before = self.clone();
        self.expire_if_due();

        match event {
            Event::Propose { actor, ttl } => self.propose(actor, ttl),
            Event::Accept { actor, epoch } => self.accept(actor, epoch),
            Event::Decline { actor, epoch } => self.decline(actor, epoch),
            Event::Withdraw { actor, epoch } => self.withdraw(actor, epoch),
            Event::Block { actor } => self.block(actor),
            Event::Unblock { actor } => {
                self.blocked_mask &= !actor.bit();
            }
            Event::SetDiscoveryOptIn { actor, enabled } => {
                if enabled {
                    self.opt_in_mask |= actor.bit();
                } else {
                    self.opt_in_mask &= !actor.bit();
                    // Pending introductions require current discovery permission.
                    // An already connected conversation has a separate consent
                    // lifecycle and is not silently terminated by discovery pause.
                    if matches!(&self.phase, Phase::Pending { .. }) {
                        self.close_current(CloseReason::Withdrawn);
                    }
                }
            }
            Event::AdvanceTime { to } => {
                if to > self.now {
                    self.now = to;
                    self.expire_if_due();
                }
            }
        }

        self != &before
    }

    pub fn invariants_hold(&self) -> bool {
        if self.chat_enabled != matches!(&self.phase, Phase::Connected { .. }) {
            return false;
        }

        if self.opt_in_mask & !BOTH_OPTED_IN != 0 || self.blocked_mask & !BOTH_OPTED_IN != 0 {
            return false;
        }

        match &self.phase {
            Phase::Idle => !self.chat_enabled,
            Phase::Pending {
                epoch,
                accepted_mask,
                opened_at,
                deadline,
            } => {
                *epoch < self.next_epoch
                    && (*accepted_mask == 0b01 || *accepted_mask == 0b10)
                    && *opened_at <= self.now
                    && self.now < *deadline
                    && self.opt_in_mask == BOTH_OPTED_IN
                    && self.blocked_mask == 0
                    && !self.chat_enabled
                    && self.last_revealed_epoch != Some(*epoch)
            }
            Phase::Connected {
                epoch,
                accepted_mask,
            } => {
                *epoch < self.next_epoch
                    && *accepted_mask == BOTH_ACCEPTED
                    && self.blocked_mask == 0
                    && self.chat_enabled
                    && self.last_revealed_epoch == Some(*epoch)
            }
            Phase::Closed { epoch, .. } => {
                *epoch < self.next_epoch && !self.chat_enabled
            }
        }
    }

    fn propose(&mut self, actor: Actor, ttl: u64) {
        if matches!(&self.phase, Phase::Pending { .. } | Phase::Connected { .. })
            || self.now < self.retry_after
            || ttl == 0
            || self.opt_in_mask != BOTH_OPTED_IN
            || self.blocked_mask != 0
            || self.next_epoch == u64::MAX
        {
            return;
        }

        let Some(deadline) = self.now.checked_add(ttl) else {
            return;
        };

        let epoch = self.next_epoch;
        self.next_epoch += 1;
        self.phase = Phase::Pending {
            epoch,
            accepted_mask: actor.bit(),
            opened_at: self.now,
            deadline,
        };
        self.chat_enabled = false;
    }

    fn accept(&mut self, actor: Actor, epoch: u64) {
        let Phase::Pending {
            epoch: active_epoch,
            accepted_mask,
            opened_at,
            deadline,
        } = self.phase.clone()
        else {
            return;
        };

        if active_epoch != epoch
            || self.now >= deadline
            || self.blocked_mask != 0
            || self.opt_in_mask != BOTH_OPTED_IN
        {
            return;
        }

        let next_mask = accepted_mask | actor.bit();
        if next_mask != BOTH_ACCEPTED {
            self.phase = Phase::Pending {
                epoch: active_epoch,
                accepted_mask: next_mask,
                opened_at,
                deadline,
            };
            return;
        }

        // This is the only transition that permits reciprocal-interest reveal
        // and a shared conversation. The model has no profile or message payload.
        self.phase = Phase::Connected {
            epoch: active_epoch,
            accepted_mask: BOTH_ACCEPTED,
        };
        self.chat_enabled = true;
        self.reveal_count = self.reveal_count.saturating_add(1);
        self.last_revealed_epoch = Some(active_epoch);
    }

    fn decline(&mut self, _actor: Actor, epoch: u64) {
        if matches!(&self.phase, Phase::Pending { epoch: active, .. } if *active == epoch) {
            self.retry_after = self
                .now
                .saturating_add(DECLINE_COOLDOWN_TICKS);
            self.close_current(CloseReason::Declined);
        }
    }

    fn withdraw(&mut self, _actor: Actor, epoch: u64) {
        let same_epoch_is_active = match &self.phase {
            Phase::Pending { epoch: active, .. } | Phase::Connected { epoch: active, .. } => {
                *active == epoch
            }
            _ => false,
        };
        if same_epoch_is_active {
            self.close_current(CloseReason::Withdrawn);
        }
    }

    fn block(&mut self, actor: Actor) {
        self.blocked_mask |= actor.bit();
        if matches!(&self.phase, Phase::Pending { .. } | Phase::Connected { .. }) {
            self.close_current(CloseReason::Blocked);
        }
    }

    fn close_current(&mut self, reason: CloseReason) {
        let epoch = match &self.phase {
            Phase::Pending { epoch, .. }
            | Phase::Connected { epoch, .. }
            | Phase::Closed { epoch, .. } => Some(*epoch),
            Phase::Idle => None,
        };
        if let Some(epoch) = epoch {
            self.phase = Phase::Closed { epoch, reason };
        }
        self.chat_enabled = false;
    }

    fn expire_if_due(&mut self) {
        if matches!(&self.phase, Phase::Pending { deadline, .. } if self.now >= *deadline) {
            self.close_current(CloseReason::Expired);
        }
    }

    fn relevant_epoch(&self) -> u64 {
        match &self.phase {
            Phase::Pending { epoch, .. }
            | Phase::Connected { epoch, .. }
            | Phase::Closed { epoch, .. } => *epoch,
            Phase::Idle => self.next_epoch.saturating_sub(1),
        }
    }

    fn events_for_bounded_exploration(&self) -> Vec<Event> {
        let current = self.relevant_epoch();
        let mut epochs = vec![
            0,
            current,
            current.saturating_sub(1),
            self.next_epoch,
        ];
        epochs.sort_unstable();
        epochs.dedup();

        let mut events = vec![
            Event::Propose { actor: Actor::A, ttl: 2 },
            Event::Propose { actor: Actor::B, ttl: 4 },
            Event::Block { actor: Actor::A },
            Event::Block { actor: Actor::B },
            Event::Unblock { actor: Actor::A },
            Event::Unblock { actor: Actor::B },
            Event::SetDiscoveryOptIn { actor: Actor::A, enabled: false },
            Event::SetDiscoveryOptIn { actor: Actor::A, enabled: true },
            Event::SetDiscoveryOptIn { actor: Actor::B, enabled: false },
            Event::SetDiscoveryOptIn { actor: Actor::B, enabled: true },
            Event::AdvanceTime { to: self.now.saturating_add(1) },
            Event::AdvanceTime { to: self.now.saturating_add(2) },
            Event::AdvanceTime { to: self.now.saturating_add(4) },
            // Actor A's counterpart is included symmetrically; this also
            // generates duplicate acceptance and stale-message negatives.
        ];

        for epoch in epochs {
            for actor in [Actor::A, Actor::B] {
                events.push(Event::Accept { actor, epoch });
                events.push(Event::Decline { actor, epoch });
                events.push(Event::Withdraw { actor, epoch });
            }
        }
        events
    }
}

/// Explore distinct states up to a bounded trace depth. This is a small explicit
/// state explorer, not a proof of an unbounded distributed implementation. A cap
/// is reported explicitly so truncated exploration cannot be mistaken for full.
pub fn bounded_model_check(max_depth: usize, max_states: usize) -> ExplorationReport {
    let initial = Machine::default();
    let mut queue = VecDeque::from([(initial.clone(), 0usize)]);
    let mut visited = HashSet::from([initial]);
    let mut report = ExplorationReport {
        unique_states: 1,
        transitions_checked: 0,
        maximum_depth_explored: 0,
        state_cap_reached: false,
        invariant_violation: false,
    };

    while let Some((state, depth)) = queue.pop_front() {
        report.maximum_depth_explored = report.maximum_depth_explored.max(depth);
        if !state.invariants_hold() {
            // Keep the report fail-closed: state_cap_reached is also used as a
            // non-success sentinel by the CLI, which labels the result invalid.
            report.invariant_violation = true;
            break;
        }
        if depth >= max_depth {
            continue;
        }

        for event in state.events_for_bounded_exploration() {
            report.transitions_checked += 1;
            let mut next = state.clone();
            next.apply(event);
            if !next.invariants_hold() {
                report.invariant_violation = true;
                report.unique_states = visited.len();
                return report;
            }
            if visited.insert(next.clone()) {
                if visited.len() >= max_states.max(1) {
                    report.state_cap_reached = true;
                    report.unique_states = visited.len();
                    return report;
                }
                queue.push_back((next, depth + 1));
            }
        }
    }

    report.unique_states = visited.len();
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    fn propose_a(machine: &mut Machine, ttl: u64) {
        assert!(machine.apply(Event::Propose { actor: Actor::A, ttl }));
        assert!(machine.invariants_hold());
    }

    #[test]
    fn one_sided_proposal_does_not_reveal_or_enable_chat() {
        let mut machine = Machine::default();
        propose_a(&mut machine, 5);

        assert!(matches!(
            &machine.phase,
            Phase::Pending { accepted_mask: 0b01, .. }
        ));
        assert_eq!(machine.reveal_count, 0);
        assert!(!machine.chat_enabled);
        assert!(machine.invariants_hold());
    }

    #[test]
    fn second_distinct_acceptance_is_the_only_connect_transition() {
        let mut machine = Machine::default();
        propose_a(&mut machine, 5);

        machine.apply(Event::Accept { actor: Actor::A, epoch: 1 });
        assert_eq!(machine.reveal_count, 0);
        assert!(!machine.chat_enabled);

        machine.apply(Event::Accept { actor: Actor::B, epoch: 1 });
        assert!(matches!(
            &machine.phase,
            Phase::Connected { accepted_mask: BOTH_ACCEPTED, .. }
        ));
        assert_eq!(machine.reveal_count, 1);
        assert!(machine.chat_enabled);
        assert!(machine.invariants_hold());

        // Replayed acceptance is idempotent and cannot count a second reveal.
        machine.apply(Event::Accept { actor: Actor::B, epoch: 1 });
        assert_eq!(machine.reveal_count, 1);
        assert!(machine.invariants_hold());
    }

    #[test]
    fn acceptance_at_deadline_loses_to_expiration() {
        let mut machine = Machine::default();
        propose_a(&mut machine, 2);
        machine.apply(Event::AdvanceTime { to: 2 });
        machine.apply(Event::Accept { actor: Actor::B, epoch: 1 });

        assert!(matches!(
            &machine.phase,
            Phase::Closed { reason: CloseReason::Expired, .. }
        ));
        assert_eq!(machine.reveal_count, 0);
        assert!(!machine.chat_enabled);
        assert!(machine.invariants_hold());
    }

    #[test]
    fn block_and_withdrawal_revoke_current_conversation_permission() {
        let mut blocked = Machine::default();
        propose_a(&mut blocked, 5);
        blocked.apply(Event::Accept { actor: Actor::B, epoch: 1 });
        assert!(blocked.chat_enabled);
        blocked.apply(Event::Block { actor: Actor::B });
        assert!(matches!(
            &blocked.phase,
            Phase::Closed { reason: CloseReason::Blocked, .. }
        ));
        assert!(!blocked.chat_enabled);
        // Previously disclosed information cannot be made unknown again.
        assert_eq!(blocked.reveal_count, 1);
        assert!(blocked.invariants_hold());

        let mut withdrawn = Machine::default();
        propose_a(&mut withdrawn, 5);
        withdrawn.apply(Event::Accept { actor: Actor::B, epoch: 1 });
        withdrawn.apply(Event::Withdraw { actor: Actor::A, epoch: 1 });
        assert!(matches!(
            &withdrawn.phase,
            Phase::Closed { reason: CloseReason::Withdrawn, .. }
        ));
        assert!(!withdrawn.chat_enabled);
        assert!(withdrawn.invariants_hold());
    }

    #[test]
    fn discovery_opt_out_closes_pending_but_not_connected_conversations() {
        let mut pending = Machine::default();
        propose_a(&mut pending, 5);
        pending.apply(Event::SetDiscoveryOptIn { actor: Actor::B, enabled: false });
        assert!(matches!(
            &pending.phase,
            Phase::Closed { reason: CloseReason::Withdrawn, .. }
        ));
        assert!(pending.invariants_hold());

        let mut connected = Machine::default();
        propose_a(&mut connected, 5);
        connected.apply(Event::Accept { actor: Actor::B, epoch: 1 });
        connected.apply(Event::SetDiscoveryOptIn { actor: Actor::B, enabled: false });
        assert!(matches!(&connected.phase, Phase::Connected { .. }));
        assert!(connected.chat_enabled);
        assert!(connected.invariants_hold());
    }

    #[test]
    fn declined_pair_has_cooldown_and_old_epochs_cannot_connect() {
        let mut machine = Machine::default();
        propose_a(&mut machine, 10);
        machine.apply(Event::Decline { actor: Actor::B, epoch: 1 });
        assert!(matches!(
            &machine.phase,
            Phase::Closed { reason: CloseReason::Declined, .. }
        ));

        machine.apply(Event::Propose { actor: Actor::B, ttl: 10 });
        assert!(matches!(&machine.phase, Phase::Closed { .. }));

        machine.apply(Event::AdvanceTime {
            to: DECLINE_COOLDOWN_TICKS,
        });
        machine.apply(Event::Propose { actor: Actor::B, ttl: 10 });
        assert!(matches!(
            &machine.phase,
            Phase::Pending { epoch: 2, .. }
        ));

        machine.apply(Event::Accept { actor: Actor::A, epoch: 1 });
        assert!(matches!(
            &machine.phase,
            Phase::Pending { epoch: 2, accepted_mask: 0b10, .. }
        ));
        machine.apply(Event::Accept { actor: Actor::A, epoch: 2 });
        assert!(matches!(&machine.phase, Phase::Connected { epoch: 2, .. }));
        assert_eq!(machine.reveal_count, 1);
        assert!(machine.invariants_hold());
    }

    #[test]
    fn invariant_checker_rejects_early_reveal_and_one_sided_connection() {
        let mut early_reveal = Machine::default();
        propose_a(&mut early_reveal, 5);
        early_reveal.chat_enabled = true;
        assert!(!early_reveal.invariants_hold());

        let mut one_sided = Machine::default();
        propose_a(&mut one_sided, 5);
        one_sided.phase = Phase::Connected { epoch: 1, accepted_mask: 0b01 };
        one_sided.chat_enabled = true;
        one_sided.last_revealed_epoch = Some(1);
        assert!(!one_sided.invariants_hold());
    }

    #[test]
    fn bounded_state_exploration_checks_invariants_after_each_transition() {
        let report = bounded_model_check(5, 100_000);
        assert!(!report.state_cap_reached, "exploration hit its state cap");
        assert!(!report.invariant_violation, "an invariant failed during exploration");
        assert!(report.unique_states > 10);
        assert!(report.transitions_checked > report.unique_states);
        assert_eq!(report.maximum_depth_explored, 5);
    }
}
