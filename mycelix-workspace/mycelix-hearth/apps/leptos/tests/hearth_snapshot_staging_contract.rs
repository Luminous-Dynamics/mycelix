// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Host-free structural regressions for HTH-UI-005B1.
//!
//! These tests deliberately inspect the loader source rather than pretending to
//! be browser/e2e qualification. Their job is narrower: fail fast if a future
//! refactor reintroduces direct published-signal writes inside the async source
//! loader or splits Decisions from their current-vote stage.

const SOURCE: &str = include_str!("../src/hearth_truth.rs");

fn section<'a>(start: &str, end: &str) -> &'a str {
    let start_offset = SOURCE
        .find(start)
        .unwrap_or_else(|| panic!("missing source marker: {start}"));
    let tail = &SOURCE[start_offset..];
    let end_offset = tail
        .find(end)
        .unwrap_or_else(|| panic!("missing source marker after {start}: {end}"));
    &tail[..end_offset]
}

#[test]
fn live_loader_stages_without_direct_published_context_writes() {
    let loader = section("async fn load_live_snapshot(", "async fn load_decisions_and_votes(");

    assert!(loader.contains("StagedHearthSnapshot::pending"));
    assert!(loader.contains("snapshot.members ="));
    assert!(loader.contains("snapshot.care_schedules ="));
    assert!(loader.contains("snapshot.decisions = decisions.decisions"));
    assert!(loader.contains("snapshot.votes = decisions.votes"));

    assert!(
        !loader.contains("hearth."),
        "async snapshot construction must not publish directly to HearthCtx"
    );
    assert!(
        !loader.contains("truth."),
        "async snapshot construction must not publish directly to HearthTruthState"
    );
}

#[test]
fn root_failure_and_empty_paths_stage_complete_scoped_replacement() {
    let loader = section("async fn load_live_snapshot(", "async fn load_decisions_and_votes(");

    assert!(loader.contains("mark_live_readable_unavailable(&mut snapshot.availability)"));
    assert!(loader.contains("mark_live_readable_empty(&mut snapshot.availability)"));

    let pending = section("fn pending(my_agent: String) -> Self", "fn publish(");
    for field in [
        "members: Vec::new()",
        "bonds: Vec::new()",
        "care_schedules: Vec::new()",
        "decisions: Vec::new()",
        "votes: Vec::new()",
        "gratitude: Vec::new()",
        "rhythms: Vec::new()",
        "presence: Vec::new()",
    ] {
        assert!(pending.contains(field), "pending snapshot must clear {field}");
    }
}

#[test]
fn decisions_and_votes_remain_one_private_dependency_stage() {
    let loader = section("async fn load_live_snapshot(", "async fn load_decisions_and_votes(");
    assert!(loader.contains(
        "let decisions = load_decisions_and_votes(hc, &hearth_hash, generation, token).await?"
    ));
    assert!(loader.contains("snapshot.decisions = decisions.decisions"));
    assert!(loader.contains("snapshot.votes = decisions.votes"));

    let decision_loader = section("async fn load_decisions_and_votes(", "fn status_message(");
    assert!(decision_loader.contains("-> Option<DecisionStage>"));
    assert!(decision_loader.contains("DecisionStage {"));
    assert!(
        !decision_loader.contains("hearth.decisions.set"),
        "Decision reads must not publish before vote closure is staged"
    );
    assert!(
        !decision_loader.contains("hearth.votes.set"),
        "vote reads must not publish independently"
    );
}

#[test]
fn publish_replaces_all_supported_payloads_and_provenance() {
    let publish = section("fn publish(self, hearth: &HearthCtx", "struct DecisionStage");

    for write in [
        "hearth.current_hearth.set(self.current_hearth)",
        "hearth.my_role.set(self.my_role)",
        "hearth.members.set(self.members)",
        "hearth.bonds.set(self.bonds)",
        "hearth.care_schedules.set(self.care_schedules)",
        "hearth.decisions.set(self.decisions)",
        "hearth.votes.set(self.votes)",
        "hearth.gratitude.set(self.gratitude)",
        "hearth.rhythms.set(self.rhythms)",
        "hearth.presence.set(self.presence)",
        "hearth.my_agent.set(self.my_agent)",
        "truth.availability.set(self.availability)",
    ] {
        assert!(publish.contains(write), "publication is missing: {write}");
    }

    let availability_pos = publish
        .find("truth.availability.set(self.availability)")
        .expect("availability publication marker");
    let last_payload_pos = publish
        .find("hearth.my_agent.set(self.my_agent)")
        .expect("last payload publication marker");
    assert!(
        availability_pos > last_payload_pos,
        "provenance marker must publish after payload assignment within the boundary"
    );
}
