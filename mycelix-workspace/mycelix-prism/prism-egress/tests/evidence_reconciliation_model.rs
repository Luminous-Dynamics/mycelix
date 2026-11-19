//! Dependency-free test model for application/kernel egress evidence reconciliation.
//! This is not a production observer, BPF program, or runtime assurance claim.
use std::collections::HashSet;
use std::net::{IpAddr, Ipv4Addr};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Peer { address: IpAddr, port: u16 }

#[derive(Clone, Debug)]
enum ExpectedAction {
    Connected { reported_peer: Peer, authorized_peers: Vec<Peer> },
    DeniedBeforeConnect,
}

#[derive(Clone, Debug)]
struct ApplicationReceipt {
    action_id: &'static str,
    workload_id: u64,
    started_ns: u64,
    finished_ns: u64,
    expected: ExpectedAction,
}

#[derive(Clone, Copy, Debug)]
struct KernelConnectEvent {
    generation: u64,
    sequence: u64,
    socket_cookie: u64,
    workload_id: u64,
    peer: Peer,
    observed_ns: u64,
    allowed: bool,
}

#[derive(Clone, Copy, Debug)]
struct ObserverWindow {
    generation: u64,
    continuous_coverage: bool,
    covered_start_ns: u64,
    covered_end_ns: u64,
    lost_events: u64,
    first_sequence: u64,
    last_sequence: Option<u64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum IncompleteReason {
    ObserverCoverageGap, EventLoss, EventSequenceGap, DuplicateSocketIdentity,
    ObserverGenerationMismatch, MissingKernelEvent, AmbiguousCorrelation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ContradictionReason {
    UnexpectedSocketActivity, UnauthorizedPeer, KernelDeniedConnect, ReportedPeerNotObserved,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ActionOutcome {
    ReconciledConnected, ReconciledDeniedNoSocket,
    Incomplete(IncompleteReason), Contradictory(ContradictionReason),
}

#[derive(Debug)]
struct ReconciliationReport {
    outcomes: Vec<(&'static str, ActionOutcome)>,
    observed_kernel_sequences: Vec<u64>,
    unmatched_kernel_sequences: Vec<u64>,
    observer_error: Option<IncompleteReason>,
}

impl ReconciliationReport {
    fn independently_qualified(&self) -> bool {
        !self.outcomes.is_empty()
            && self.observer_error.is_none()
            && self.unmatched_kernel_sequences.is_empty()
            && self.outcomes.iter().all(|(_, outcome)| matches!(
                outcome, ActionOutcome::ReconciledConnected | ActionOutcome::ReconciledDeniedNoSocket
            ))
    }
}

fn observer_integrity_error(
    observer: ObserverWindow, receipts: &[ApplicationReceipt], events: &[KernelConnectEvent],
) -> Option<IncompleteReason> {
    if !observer.continuous_coverage || receipts.iter().any(|r|
        r.started_ns < observer.covered_start_ns || r.finished_ns > observer.covered_end_ns
    ) { return Some(IncompleteReason::ObserverCoverageGap); }
    if observer.lost_events != 0 { return Some(IncompleteReason::EventLoss); }
    if events.iter().any(|e| e.generation != observer.generation) {
        return Some(IncompleteReason::ObserverGenerationMismatch);
    }
    let mut cookies = HashSet::new();
    if events.iter().any(|e| !cookies.insert(e.socket_cookie)) {
        return Some(IncompleteReason::DuplicateSocketIdentity);
    }
    match (events.is_empty(), observer.last_sequence) {
        (true, None) => {}
        (false, Some(last)) => {
            let count = last.checked_sub(observer.first_sequence).and_then(|n| n.checked_add(1));
            if count != Some(events.len() as u64) { return Some(IncompleteReason::EventSequenceGap); }
            for (i, event) in events.iter().enumerate() {
                if observer.first_sequence.checked_add(i as u64) != Some(event.sequence) {
                    return Some(IncompleteReason::EventSequenceGap);
                }
            }
        }
        _ => return Some(IncompleteReason::EventSequenceGap),
    }
    if events.iter().any(|e| e.observed_ns < observer.covered_start_ns
        || e.observed_ns > observer.covered_end_ns
    ) { return Some(IncompleteReason::ObserverCoverageGap); }
    None
}

fn reconcile(
    observer: ObserverWindow, receipts: &[ApplicationReceipt], events: &[KernelConnectEvent],
) -> ReconciliationReport {
    let observed: Vec<u64> = events.iter().map(|e| e.sequence).collect();
    if let Some(reason) = observer_integrity_error(observer, receipts, events) {
        return ReconciliationReport {
            outcomes: receipts.iter().map(|r|
                (r.action_id, ActionOutcome::Incomplete(reason))
            ).collect(),
            observed_kernel_sequences: observed.clone(),
            unmatched_kernel_sequences: observed,
            observer_error: Some(reason),
        };
    }

    // No authenticated action ID exists in the kernel event. Join only on
    // workload plus monotonic interval; overlapping candidates remain ambiguous.
    let mut by_receipt = vec![Vec::<usize>::new(); receipts.len()];
    let mut ambiguous = vec![false; receipts.len()];
    let mut unmatched = Vec::new();
    for (event_index, event) in events.iter().enumerate() {
        let candidates: Vec<usize> = receipts.iter().enumerate().filter_map(|(i, r)|
            (r.workload_id == event.workload_id
                && event.observed_ns >= r.started_ns
                && event.observed_ns <= r.finished_ns).then_some(i)
        ).collect();
        match candidates.as_slice() {
            [] => unmatched.push(event.sequence),
            [i] => by_receipt[*i].push(event_index),
            _ => for i in candidates { ambiguous[i] = true; },
        }
    }

    let outcomes = receipts.iter().enumerate().map(|(i, receipt)| {
        let outcome = if ambiguous[i] {
            ActionOutcome::Incomplete(IncompleteReason::AmbiguousCorrelation)
        } else {
            match &receipt.expected {
                ExpectedAction::DeniedBeforeConnect => {
                    if by_receipt[i].is_empty() { ActionOutcome::ReconciledDeniedNoSocket }
                    else { ActionOutcome::Contradictory(ContradictionReason::UnexpectedSocketActivity) }
                }
                ExpectedAction::Connected { reported_peer, authorized_peers } => {
                    if by_receipt[i].is_empty() {
                        ActionOutcome::Incomplete(IncompleteReason::MissingKernelEvent)
                    } else {
                        let mut selected_seen = false;
                        let mut conflict = None;
                        // Multiple raced attempts are allowed only inside the same
                        // address lease; the selected peer must be independently seen.
                        for event_index in &by_receipt[i] {
                            let event = events[*event_index];
                            if !authorized_peers.contains(&event.peer) {
                                conflict = Some(ContradictionReason::UnauthorizedPeer); break;
                            }
                            if !event.allowed {
                                conflict = Some(ContradictionReason::KernelDeniedConnect); break;
                            }
                            if event.peer == *reported_peer { selected_seen = true; }
                        }
                        if let Some(reason) = conflict { ActionOutcome::Contradictory(reason) }
                        else if selected_seen { ActionOutcome::ReconciledConnected }
                        else { ActionOutcome::Contradictory(ContradictionReason::ReportedPeerNotObserved) }
                    }
                }
            }
        };
        (receipt.action_id, outcome)
    }).collect();

    ReconciliationReport {
        outcomes, observed_kernel_sequences: observed,
        unmatched_kernel_sequences: unmatched, observer_error: None,
    }
}

fn peer(last: u8, port: u16) -> Peer {
    Peer { address: IpAddr::V4(Ipv4Addr::new(198, 51, 100, last)), port }
}
fn connected_receipt(id: &'static str, start: u64, end: u64) -> ApplicationReceipt {
    let selected = peer(10, 443);
    ApplicationReceipt {
        action_id: id, workload_id: 7, started_ns: start, finished_ns: end,
        expected: ExpectedAction::Connected {
            reported_peer: selected, authorized_peers: vec![selected, peer(11, 443)],
        },
    }
}
fn observer(first: u64, last: Option<u64>) -> ObserverWindow {
    ObserverWindow {
        generation: 4, continuous_coverage: true, covered_start_ns: 0,
        covered_end_ns: 1_000, lost_events: 0, first_sequence: first, last_sequence: last,
    }
}
fn event(sequence: u64, at: u64, destination: Peer) -> KernelConnectEvent {
    KernelConnectEvent {
        generation: 4, sequence, socket_cookie: 900 + sequence, workload_id: 7,
        peer: destination, observed_ns: at, allowed: true,
    }
}

#[test]
fn exact_single_action_reconciles_with_complete_observer_window() {
    let receipt = connected_receipt("action-a", 100, 200);
    let report = reconcile(observer(8, Some(8)), &[receipt], &[event(8, 150, peer(10, 443))]);
    assert!(report.independently_qualified());
    assert_eq!(report.outcomes[0].1, ActionOutcome::ReconciledConnected);
    assert!(report.unmatched_kernel_sequences.is_empty());
}

#[test]
fn observer_event_loss_never_produces_complete_reconciliation() {
    let receipt = connected_receipt("action-loss", 100, 200);
    let mut damaged = observer(8, Some(8)); damaged.lost_events = 1;
    let report = reconcile(damaged, &[receipt], &[event(8, 150, peer(10, 443))]);
    assert!(!report.independently_qualified());
    assert_eq!(report.observer_error, Some(IncompleteReason::EventLoss));
    assert_eq!(report.outcomes[0].1, ActionOutcome::Incomplete(IncompleteReason::EventLoss));
}

#[test]
fn concurrent_actions_with_overlapping_windows_are_ambiguous() {
    let receipts = [
        connected_receipt("left", 100, 220), connected_receipt("right", 120, 240),
    ];
    let report = reconcile(observer(8, Some(8)), &receipts, &[event(8, 180, peer(10, 443))]);
    assert!(!report.independently_qualified());
    assert!(report.outcomes.iter().all(|(_, o)|
        *o == ActionOutcome::Incomplete(IncompleteReason::AmbiguousCorrelation)
    ));
}

#[test]
fn kernel_event_without_application_receipt_is_retained_as_unmatched() {
    let report = reconcile(observer(8, Some(8)), &[], &[event(8, 150, peer(10, 443))]);
    assert!(!report.independently_qualified());
    assert_eq!(report.observed_kernel_sequences, vec![8]);
    assert_eq!(report.unmatched_kernel_sequences, vec![8]);
}

#[test]
fn application_success_without_kernel_observation_is_not_independently_verified() {
    let receipt = connected_receipt("no-event", 100, 200);
    let report = reconcile(observer(8, None), &[receipt], &[]);
    assert!(!report.independently_qualified());
    assert_eq!(report.outcomes[0].1,
        ActionOutcome::Incomplete(IncompleteReason::MissingKernelEvent));
}

#[test]
fn observer_detach_reattach_cannot_retroactively_cover_gap() {
    let receipt = connected_receipt("gap", 100, 300);
    let mut window = observer(1, Some(1));
    window.generation = 5; window.continuous_coverage = false;
    window.covered_start_ns = 200; window.covered_end_ns = 400;
    let mut observed = event(1, 250, peer(10, 443)); observed.generation = 5;
    let report = reconcile(window, &[receipt], &[observed]);
    assert!(!report.independently_qualified());
    assert_eq!(report.outcomes[0].1,
        ActionOutcome::Incomplete(IncompleteReason::ObserverCoverageGap));
}

#[test]
fn event_sequence_gap_invalidates_observer_completeness() {
    let receipt = connected_receipt("sequence-gap", 100, 200);
    let events = [event(41, 140, peer(10, 443)), event(43, 160, peer(11, 443))];
    let report = reconcile(observer(41, Some(43)), &[receipt], &events);
    assert!(!report.independently_qualified());
    assert_eq!(report.observer_error, Some(IncompleteReason::EventSequenceGap));
}

#[test]
fn observed_peer_outside_authorized_set_is_a_contradiction() {
    let receipt = connected_receipt("wrong-peer", 100, 200);
    let report = reconcile(observer(8, Some(8)), &[receipt], &[event(8, 150, peer(99, 443))]);
    assert!(!report.independently_qualified());
    assert_eq!(report.outcomes[0].1,
        ActionOutcome::Contradictory(ContradictionReason::UnauthorizedPeer));
}

#[test]
fn denied_before_connect_reconciles_only_when_no_kernel_event_exists() {
    let receipt = ApplicationReceipt {
        action_id: "preflight-denied", workload_id: 7, started_ns: 100, finished_ns: 200,
        expected: ExpectedAction::DeniedBeforeConnect,
    };
    let report = reconcile(observer(1, None), &[receipt], &[]);
    assert!(report.independently_qualified());
    assert_eq!(report.outcomes[0].1, ActionOutcome::ReconciledDeniedNoSocket);
}
