// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Connection-epoch admission for Personal reconciliation.
//!
//! This module deliberately knows nothing about Leptos, Holochain, or Personal
//! domain records. It answers one narrow question: may an asynchronous result
//! that started in a particular usable-session epoch still publish now?

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReconciliationTransition {
    None,
    Start { epoch: u64 },
    Invalidated { epoch: u64, had_completed: bool },
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReconciliationEpoch {
    epoch: u64,
    usable: bool,
    in_flight: bool,
    completed_once: bool,
}

impl ReconciliationEpoch {
    fn advance_epoch(&mut self) {
        self.epoch = self
            .epoch
            .checked_add(1)
            .expect("Personal reconciliation epoch exhausted; refusing token reuse");
    }

    /// Observe whether the conductor + signer session is currently usable.
    ///
    /// A false -> true transition creates a fresh epoch and admits exactly one
    /// reconciliation. A true -> false transition creates another fresh epoch,
    /// immediately invalidating every result from the previously usable one.
    pub fn observe_usable(&mut self, usable: bool) -> ReconciliationTransition {
        match (self.usable, usable) {
            (false, false) | (true, true) => ReconciliationTransition::None,
            (false, true) => {
                self.advance_epoch();
                self.usable = true;
                self.in_flight = true;
                ReconciliationTransition::Start { epoch: self.epoch }
            }
            (true, false) => {
                self.advance_epoch();
                self.usable = false;
                self.in_flight = false;
                ReconciliationTransition::Invalidated {
                    epoch: self.epoch,
                    had_completed: self.completed_once,
                }
            }
        }
    }

    /// Begin an explicit refresh inside the current usable epoch.
    ///
    /// Refreshes coalesce: while one reconciliation is in flight, another
    /// caller cannot start a competing reconciliation for the same epoch.
    pub fn begin_refresh(&mut self) -> Option<u64> {
        if !self.usable || self.in_flight {
            return None;
        }
        self.in_flight = true;
        Some(self.epoch)
    }

    /// Whether a result produced by `epoch` may still publish into state.
    pub fn accepts(&self, epoch: u64) -> bool {
        self.usable && self.epoch == epoch
    }

    /// Mark a reconciliation complete only if it still belongs to this epoch.
    pub fn finish(&mut self, epoch: u64) -> bool {
        if !self.accepts(epoch) || !self.in_flight {
            return false;
        }
        self.in_flight = false;
        self.completed_once = true;
        true
    }

    pub fn current_epoch(&self) -> u64 {
        self.epoch
    }

    pub fn is_usable(&self) -> bool {
        self.usable
    }

    pub fn is_in_flight(&self) -> bool {
        self.in_flight
    }

    pub fn has_completed(&self) -> bool {
        self.completed_once
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_usable_session_starts_one_epoch() {
        let mut gate = ReconciliationEpoch::default();
        assert_eq!(gate.observe_usable(false), ReconciliationTransition::None);
        assert_eq!(
            gate.observe_usable(true),
            ReconciliationTransition::Start { epoch: 1 }
        );
        assert_eq!(gate.observe_usable(true), ReconciliationTransition::None);
        assert!(gate.accepts(1));
        assert!(gate.is_in_flight());
    }

    #[test]
    fn disconnect_immediately_invalidates_old_results() {
        let mut gate = ReconciliationEpoch::default();
        let ReconciliationTransition::Start { epoch } = gate.observe_usable(true) else {
            panic!("first usable session must start reconciliation");
        };

        assert_eq!(
            gate.observe_usable(false),
            ReconciliationTransition::Invalidated {
                epoch: 2,
                had_completed: false,
            }
        );
        assert!(!gate.accepts(epoch));
        assert!(!gate.is_in_flight());
    }

    #[test]
    fn signer_loss_invalidates_the_same_publication_authority() {
        let mut gate = ReconciliationEpoch::default();
        let ReconciliationTransition::Start { epoch } = gate.observe_usable(true) else {
            panic!("usable conductor plus signer must start reconciliation");
        };
        assert!(gate.finish(epoch));

        // `usable=false` is intentionally agnostic about whether the cause was
        // transport loss or signer loss. Either one removes zome-call authority.
        let transition = gate.observe_usable(false);
        assert_eq!(
            transition,
            ReconciliationTransition::Invalidated {
                epoch: 2,
                had_completed: true,
            }
        );
        assert!(!gate.accepts(epoch));
    }

    #[test]
    fn reconnect_gets_a_distinct_publish_epoch() {
        let mut gate = ReconciliationEpoch::default();
        let ReconciliationTransition::Start { epoch: first } = gate.observe_usable(true) else {
            panic!("first session must start");
        };
        gate.observe_usable(false);
        let ReconciliationTransition::Start { epoch: second } = gate.observe_usable(true) else {
            panic!("reconnect must start");
        };

        assert_ne!(first, second);
        assert!(!gate.accepts(first));
        assert!(gate.accepts(second));
    }

    #[test]
    fn stale_completion_cannot_finish_newer_epoch() {
        let mut gate = ReconciliationEpoch::default();
        let ReconciliationTransition::Start { epoch: first } = gate.observe_usable(true) else {
            panic!("first session must start");
        };
        gate.observe_usable(false);
        let ReconciliationTransition::Start { epoch: second } = gate.observe_usable(true) else {
            panic!("second session must start");
        };

        assert!(!gate.finish(first));
        assert!(gate.is_in_flight());
        assert!(gate.finish(second));
        assert!(gate.has_completed());
    }

    #[test]
    fn completed_session_reports_stale_history_on_invalidation() {
        let mut gate = ReconciliationEpoch::default();
        let ReconciliationTransition::Start { epoch } = gate.observe_usable(true) else {
            panic!("session must start");
        };
        assert!(gate.finish(epoch));

        assert_eq!(
            gate.observe_usable(false),
            ReconciliationTransition::Invalidated {
                epoch: 2,
                had_completed: true,
            }
        );
    }

    #[test]
    fn explicit_refreshes_coalesce_inside_epoch() {
        let mut gate = ReconciliationEpoch::default();
        let ReconciliationTransition::Start { epoch } = gate.observe_usable(true) else {
            panic!("session must start");
        };
        assert!(gate.finish(epoch));

        assert_eq!(gate.begin_refresh(), Some(epoch));
        assert_eq!(gate.begin_refresh(), None);
        assert!(gate.finish(epoch));
        assert_eq!(gate.begin_refresh(), Some(epoch));
    }

    #[test]
    #[should_panic(expected = "refusing token reuse")]
    fn epoch_exhaustion_fails_closed_instead_of_wrapping() {
        let mut gate = ReconciliationEpoch {
            epoch: u64::MAX,
            usable: false,
            in_flight: false,
            completed_once: false,
        };
        let _ = gate.observe_usable(true);
    }
}
