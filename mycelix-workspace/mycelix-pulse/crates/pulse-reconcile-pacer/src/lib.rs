// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
#![forbid(unsafe_code)]
//! Transport-neutral pacing for non-authoritative Pulse realtime wakes.
//!
//! This state machine owns scheduling metadata only. It never carries a
//! message identifier, sender, thread, receipt, verification result, or other
//! durable authority. The first request starts reconciliation immediately.
//! Every completed pass then enters a cooldown. Arbitrarily many wake requests
//! while a pass is running or while its cooldown is active collapse into one
//! pending bit, so sustained untrusted wakes cannot drive back-to-back durable
//! reloads faster than the caller's enforced cooldown interval.
//!
//! The caller MUST serialize access to one scheduler instance and MUST NOT call
//! [`PacedReconcileScheduler::release_cooldown`] before the minimum delay
//! returned by [`PacedReconcileScheduler::complete_pass`] has elapsed. Cooldown
//! epochs make late/stale timer callbacks harmless.

/// Minimum delay the browser convergence must enforce after every completed
/// authoritative reconciliation pass before another pass may start.
pub const MIN_RECONCILE_COOLDOWN_MS: u32 = 1_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CooldownEpoch(u64);

impl CooldownEpoch {
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Single-owner scheduling state. Deliberately not `Clone` or `Copy`.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct PacedReconcileScheduler {
    state: ReconcileState,
    last_epoch: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ReconcileState {
    #[default]
    Ready,
    Running {
        dirty: bool,
    },
    Cooling {
        pending: bool,
        epoch: CooldownEpoch,
    },
}

#[must_use = "request effects must be acted on or reconciliation may be lost"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReconcileRequestEffect {
    /// The scheduler was ready. Start exactly one authoritative pass now.
    StartPass,
    /// A pass is already active; this request is represented by its dirty bit.
    CoalescedRunning,
    /// The scheduler is cooling; this request is represented by its pending bit.
    CoalescedCooling,
}

#[must_use = "completion must arm the returned cooldown before another pass may start"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CooldownInstruction {
    pub epoch: CooldownEpoch,
    pub minimum_delay_ms: u32,
}

#[must_use = "cooldown release effects must be acted on"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CooldownReleaseEffect {
    /// At least one request arrived during the completed pass or cooldown.
    /// Start exactly one authoritative follow-up pass.
    StartPass,
    /// No request remains pending; the scheduler is ready for a future wake.
    BecameReady,
    /// A late or duplicated timer callback was for an older cooldown epoch and
    /// therefore has no scheduling authority.
    StaleIgnored,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReconcileSchedulerError {
    CompletionWhileNotRunning,
    CooldownEpochExhausted,
}

impl PacedReconcileScheduler {
    pub const fn new() -> Self {
        Self {
            state: ReconcileState::Ready,
            last_epoch: 0,
        }
    }

    /// Request one authoritative reconciliation pass after a validated wake,
    /// startup, or reconnect event.
    pub fn request_reconcile(&mut self) -> ReconcileRequestEffect {
        match self.state {
            ReconcileState::Ready => {
                self.state = ReconcileState::Running { dirty: false };
                ReconcileRequestEffect::StartPass
            }
            ReconcileState::Running { .. } => {
                self.state = ReconcileState::Running { dirty: true };
                ReconcileRequestEffect::CoalescedRunning
            }
            ReconcileState::Cooling { epoch, .. } => {
                self.state = ReconcileState::Cooling {
                    pending: true,
                    epoch,
                };
                ReconcileRequestEffect::CoalescedCooling
            }
        }
    }

    /// Complete exactly one authoritative pass and arm a mandatory cooldown.
    ///
    /// The returned epoch must be supplied to `release_cooldown` only after at
    /// least `minimum_delay_ms` has elapsed. Wakes remain represented by one
    /// pending bit throughout that delay.
    pub fn complete_pass(&mut self) -> Result<CooldownInstruction, ReconcileSchedulerError> {
        let ReconcileState::Running { dirty } = self.state else {
            return Err(ReconcileSchedulerError::CompletionWhileNotRunning);
        };
        let next_epoch = self
            .last_epoch
            .checked_add(1)
            .ok_or(ReconcileSchedulerError::CooldownEpochExhausted)?;
        let epoch = CooldownEpoch(next_epoch);
        self.last_epoch = next_epoch;
        self.state = ReconcileState::Cooling {
            pending: dirty,
            epoch,
        };
        Ok(CooldownInstruction {
            epoch,
            minimum_delay_ms: MIN_RECONCILE_COOLDOWN_MS,
        })
    }

    /// Release one cooldown after its externally enforced minimum delay.
    ///
    /// Stale timer callbacks are ignored rather than acquiring authority over
    /// a newer scheduler epoch.
    pub fn release_cooldown(&mut self, epoch: CooldownEpoch) -> CooldownReleaseEffect {
        let ReconcileState::Cooling {
            pending,
            epoch: current_epoch,
        } = self.state
        else {
            return CooldownReleaseEffect::StaleIgnored;
        };

        if epoch != current_epoch {
            return CooldownReleaseEffect::StaleIgnored;
        }

        if pending {
            self.state = ReconcileState::Running { dirty: false };
            CooldownReleaseEffect::StartPass
        } else {
            self.state = ReconcileState::Ready;
            CooldownReleaseEffect::BecameReady
        }
    }

    pub const fn is_running(&self) -> bool {
        matches!(self.state, ReconcileState::Running { .. })
    }

    pub const fn is_cooling(&self) -> bool {
        matches!(self.state, ReconcileState::Cooling { .. })
    }

    pub const fn has_pending_wake(&self) -> bool {
        matches!(
            self.state,
            ReconcileState::Running { dirty: true } | ReconcileState::Cooling { pending: true, .. }
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_wake_starts_immediately() {
        let mut scheduler = PacedReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        assert!(scheduler.is_running());
        assert!(!scheduler.is_cooling());
    }

    #[test]
    fn running_burst_collapses_to_one_dirty_bit_then_cooldown() {
        let mut scheduler = PacedReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        for _ in 0..10_000 {
            assert_eq!(
                scheduler.request_reconcile(),
                ReconcileRequestEffect::CoalescedRunning
            );
        }
        assert!(scheduler.has_pending_wake());

        let cooldown = scheduler.complete_pass().expect("complete running pass");
        assert_eq!(cooldown.minimum_delay_ms, MIN_RECONCILE_COOLDOWN_MS);
        assert!(scheduler.is_cooling());
        assert!(scheduler.has_pending_wake());
        assert_eq!(
            scheduler.release_cooldown(cooldown.epoch),
            CooldownReleaseEffect::StartPass
        );
        assert!(scheduler.is_running());
        assert!(!scheduler.has_pending_wake());
    }

    #[test]
    fn cooldown_burst_collapses_to_one_pending_bit() {
        let mut scheduler = PacedReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        let cooldown = scheduler.complete_pass().expect("complete running pass");
        assert!(!scheduler.has_pending_wake());

        for _ in 0..10_000 {
            assert_eq!(
                scheduler.request_reconcile(),
                ReconcileRequestEffect::CoalescedCooling
            );
        }
        assert!(scheduler.has_pending_wake());
        assert_eq!(
            scheduler.release_cooldown(cooldown.epoch),
            CooldownReleaseEffect::StartPass
        );
    }

    #[test]
    fn clean_pass_cannot_be_followed_until_cooldown_releases() {
        let mut scheduler = PacedReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        let cooldown = scheduler.complete_pass().expect("complete running pass");
        assert!(scheduler.is_cooling());
        assert_eq!(
            scheduler.release_cooldown(cooldown.epoch),
            CooldownReleaseEffect::BecameReady
        );
        assert!(!scheduler.is_running());
        assert!(!scheduler.is_cooling());
    }

    #[test]
    fn wake_during_cooldown_is_not_lost() {
        let mut scheduler = PacedReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        let cooldown = scheduler.complete_pass().expect("complete running pass");
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::CoalescedCooling
        );
        assert_eq!(
            scheduler.release_cooldown(cooldown.epoch),
            CooldownReleaseEffect::StartPass
        );
    }

    #[test]
    fn stale_cooldown_callbacks_cannot_release_newer_epoch() {
        let mut scheduler = PacedReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        let first = scheduler.complete_pass().expect("first completion");
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::CoalescedCooling
        );
        assert_eq!(
            scheduler.release_cooldown(first.epoch),
            CooldownReleaseEffect::StartPass
        );
        let second = scheduler.complete_pass().expect("second completion");
        assert_ne!(first.epoch, second.epoch);

        assert_eq!(
            scheduler.release_cooldown(first.epoch),
            CooldownReleaseEffect::StaleIgnored
        );
        assert!(scheduler.is_cooling());
        assert_eq!(
            scheduler.release_cooldown(second.epoch),
            CooldownReleaseEffect::BecameReady
        );
    }

    #[test]
    fn duplicate_timer_callback_after_release_is_harmless() {
        let mut scheduler = PacedReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        let cooldown = scheduler.complete_pass().expect("complete running pass");
        assert_eq!(
            scheduler.release_cooldown(cooldown.epoch),
            CooldownReleaseEffect::BecameReady
        );
        assert_eq!(
            scheduler.release_cooldown(cooldown.epoch),
            CooldownReleaseEffect::StaleIgnored
        );
    }

    #[test]
    fn completion_outside_running_fails_closed() {
        let mut scheduler = PacedReconcileScheduler::new();
        assert_eq!(
            scheduler.complete_pass(),
            Err(ReconcileSchedulerError::CompletionWhileNotRunning)
        );

        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );
        let _ = scheduler.complete_pass().expect("complete running pass");
        assert_eq!(
            scheduler.complete_pass(),
            Err(ReconcileSchedulerError::CompletionWhileNotRunning)
        );
    }

    #[test]
    fn sustained_wake_storm_requires_one_cooldown_release_per_follow_up() {
        let mut scheduler = PacedReconcileScheduler::new();
        assert_eq!(
            scheduler.request_reconcile(),
            ReconcileRequestEffect::StartPass
        );

        for expected_epoch in 1..=128_u64 {
            for _ in 0..1_000 {
                assert!(matches!(
                    scheduler.request_reconcile(),
                    ReconcileRequestEffect::CoalescedRunning
                        | ReconcileRequestEffect::CoalescedCooling
                ));
            }
            let cooldown = scheduler.complete_pass().expect("complete storm pass");
            assert_eq!(cooldown.epoch.get(), expected_epoch);
            for _ in 0..1_000 {
                assert_eq!(
                    scheduler.request_reconcile(),
                    ReconcileRequestEffect::CoalescedCooling
                );
            }
            assert_eq!(
                scheduler.release_cooldown(cooldown.epoch),
                CooldownReleaseEffect::StartPass
            );
        }
    }
}
