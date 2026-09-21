// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Qualification of the Leptos reactive primitive selected by HTH-UI-005B2.
//!
//! This test does not exercise Hearth transport or Holochain. It proves the
//! narrower framework premise required by the snapshot-publication contract:
//! `batch` must defer `ImmediateEffect` observers until all signal writes in the
//! publication closure have completed.

use leptos::prelude::*;
use std::sync::{Arc, Mutex};

type Observation = (u32, &'static str);

fn observed_pair_trace(batched: bool) -> Vec<Observation> {
    let owner = Owner::new();

    owner.with(|| {
        let payload = RwSignal::new(0_u32);
        let provenance = RwSignal::new("old");
        let observations = Arc::new(Mutex::new(Vec::<Observation>::new()));
        let observations_for_effect = Arc::clone(&observations);

        let _effect = ImmediateEffect::new(move || {
            observations_for_effect
                .lock()
                .expect("observation lock should remain available")
                .push((payload.get(), provenance.get()));
        });

        if batched {
            batch(|| {
                payload.set(1);
                provenance.set("new");
            });
        } else {
            payload.set(1);
            provenance.set("new");
        }

        let trace = observations
            .lock()
            .expect("observation lock should remain available")
            .clone();
        trace
    })
}

#[test]
fn unbatched_signal_writes_expose_an_intermediate_payload_provenance_pair() {
    let trace = observed_pair_trace(false);

    assert_eq!(trace.first(), Some(&(0, "old")));
    assert!(
        trace.contains(&(1, "old")),
        "the control must demonstrate the mixed state that B2 is intended to prevent: {trace:?}"
    );
    assert_eq!(trace.last(), Some(&(1, "new")));
}

#[test]
fn batch_defers_immediate_effect_until_payload_and_provenance_agree() {
    let trace = observed_pair_trace(true);

    assert_eq!(
        trace,
        vec![(0, "old"), (1, "new")],
        "a batched publication must not expose an intermediate payload/provenance pair"
    );
}
