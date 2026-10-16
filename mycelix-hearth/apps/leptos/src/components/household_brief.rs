// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Intent-first household summary.
//!
//! Counts are deterministic navigation aids. They are not a priority score,
//! individual duty assignment, or proof that source data is complete/current.

use leptos::prelude::*;
use leptos_router::components::A;

use crate::hearth_context::use_hearth;
use hearth_leptos_types::{
    CareScheduleStatus, CareScheduleView, DecisionStatus, DecisionView, EmergencyAlertView,
};

fn active_care_plan_count(plans: &[CareScheduleView], hearth_hash: Option<&str>) -> Option<usize> {
    let hearth_hash = hearth_hash?;
    Some(plans
        .iter()
        .filter(|plan| plan.hearth_hash == hearth_hash && plan.status == CareScheduleStatus::Active)
        .count())
}

fn open_decision_count(decisions: &[DecisionView], hearth_hash: Option<&str>) -> Option<usize> {
    let hearth_hash = hearth_hash?;
    Some(decisions
        .iter()
        .filter(|decision| decision.hearth_hash == hearth_hash && decision.status == DecisionStatus::Open)
        .count())
}

fn unresolved_alert_count(alerts: &[EmergencyAlertView], hearth_hash: Option<&str>) -> Option<usize> {
    let hearth_hash = hearth_hash?;
    Some(alerts
        .iter()
        .filter(|alert| alert.hearth_hash == hearth_hash && alert.resolved_at.is_none())
        .count())
}

#[component]
pub fn HouseholdBrief() -> impl IntoView {
    let hearth = use_hearth();
    let current_hearth = hearth.current_hearth;
    let care_plans = hearth.care_schedules;
    let decisions = hearth.decisions;
    let alerts = hearth.emergency_alerts;

    // Each card is keyed to the currently presented Hearth. No current target
    // means no target-scoped counts, not a silent fallback to another Hearth.
    let care_count = move || {
        let hash = current_hearth.get().map(|item| item.hash);
        active_care_plan_count(&care_plans.get(), hash.as_deref())
    };
    let decision_count = move || {
        let hash = current_hearth.get().map(|item| item.hash);
        open_decision_count(&decisions.get(), hash.as_deref())
    };
    let alert_count = move || {
        let hash = current_hearth.get().map(|item| item.hash);
        unresolved_alert_count(&alerts.get(), hash.as_deref())
    };

    view! {
        <section class="household-brief" aria-labelledby="household-brief-title">
            <header class="brief-heading">
                <div>
                    <p class="brief-eyebrow">"A little context"</p>
                    <h2 id="household-brief-title">"At a glance"</h2>
                </div>
                <p class="brief-subtitle">
                    "Simple paths into your household. Counts are not urgency scores."
                </p>
            </header>

            <div class="brief-grid">
                <article class="brief-card">
                    <p class="brief-category">"Care"</p>
                    <p class="brief-number">{move || care_count().map(|count| count.to_string()).unwrap_or_else(|| "—".into())}</p>
                    <h3>"Active care plans"</h3>
                    <p class="brief-description">
                        {move || if care_count().is_some() {
                            "Plans marked active; due times are not inferred here."
                        } else {
                            "No household selected; this count is unavailable."
                        }}
                    </p>
                    <A href="/care" attr:class="brief-link" attr:aria-label="Review care plans">
                        "Review care plans"
                        <span aria-hidden="true">" →"</span>
                    </A>
                </article>

                <article class="brief-card">
                    <p class="brief-category">"Shared decisions"</p>
                    <p class="brief-number">{move || decision_count().map(|count| count.to_string()).unwrap_or_else(|| "—".into())}</p>
                    <h3>"Open decisions"</h3>
                    <p class="brief-description">
                        {move || if decision_count().is_some() {
                            "Household decisions still marked open; this is not a personal vote list."
                        } else {
                            "No household selected; this count is unavailable."
                        }}
                    </p>
                    <A href="/decisions" attr:class="brief-link" attr:aria-label="Explore open household decisions">
                        "Explore decisions"
                        <span aria-hidden="true">" →"</span>
                    </A>
                </article>

                <article class=move || if alert_count() > 0 {
                    "brief-card brief-card--attention"
                } else {
                    "brief-card"
                }>
                    <p class="brief-category">"Safety"</p>
                    <p class="brief-number">{move || alert_count().map(|count| count.to_string()).unwrap_or_else(|| "—".into())}</p>
                    <h3>"Unresolved alerts"</h3>
                    <p class="brief-description">
                        {move || if alert_count().is_some() {
                            "Unresolved alerts in the currently loaded records; zero is not a safety guarantee."
                        } else {
                            "No household selected; this count is unavailable."
                        }}
                    </p>
                    <A href="/emergency" attr:class="brief-link" attr:aria-label="Review safety alerts">
                        "Review alerts"
                        <span aria-hidden="true">" →"</span>
                    </A>
                </article>
            </div>
        </section>
    }
}

#[cfg(test)]
mod tests {
    use super::{active_care_plan_count, open_decision_count, unresolved_alert_count};
    use hearth_leptos_types::{
        AlertType, CareScheduleStatus, CareScheduleView, CareType, DecisionStatus, DecisionType,
        DecisionView, EmergencyAlertView, Recurrence,
    };

    fn plan(hash: &str, status: CareScheduleStatus) -> CareScheduleView {
        CareScheduleView {
            hash: format!("plan-{hash}"),
            hearth_hash: hash.into(),
            care_type: CareType::Chore,
            title: "Example plan".into(),
            description: String::new(),
            assigned_to: "agent-example".into(),
            recurrence: Recurrence::Weekly,
            status,
            completed_at: None,
        }
    }

    fn decision(hash: &str, status: DecisionStatus) -> DecisionView {
        DecisionView {
            hash: format!("decision-{hash}"),
            hearth_hash: hash.into(),
            title: "Example decision".into(),
            description: String::new(),
            decision_type: DecisionType::Consensus,
            eligible_roles: Vec::new(),
            options: Vec::new(),
            deadline: 0,
            quorum_bp: None,
            status,
            created_by: "agent-example".into(),
            created_at: 0,
        }
    }

    fn alert(hash: &str, resolved_at: Option<i64>) -> EmergencyAlertView {
        EmergencyAlertView {
            hash: format!("alert-{hash}"),
            hearth_hash: hash.into(),
            alert_type: AlertType::Custom("test".into()),
            severity: hearth_leptos_types::AlertSeverity::Low,
            message: "Example alert".into(),
            reporter: "agent-example".into(),
            location_hint: None,
            created_at: 0,
            resolved_at,
        }
    }

    #[test]
    fn no_selected_hearth_is_unavailable_not_a_false_zero() {
        assert_eq!(active_care_plan_count(&[plan("a", CareScheduleStatus::Active)], None), None);
        assert_eq!(open_decision_count(&[decision("a", DecisionStatus::Open)], None), None);
        assert_eq!(unresolved_alert_count(&[alert("a", None)], None), None);
    }

    #[test]
    fn care_count_uses_only_active_plans_from_selected_hearth() {
        let plans = [
            plan("a", CareScheduleStatus::Active),
            plan("a", CareScheduleStatus::Paused),
            plan("a", CareScheduleStatus::Completed),
            plan("b", CareScheduleStatus::Active),
        ];
        assert_eq!(active_care_plan_count(&plans, Some("a")), Some(1));
        assert_eq!(active_care_plan_count(&plans, Some("b")), Some(1));
    }

    #[test]
    fn decisions_count_only_open_items_from_selected_hearth() {
        let decisions = [
            decision("a", DecisionStatus::Open),
            decision("a", DecisionStatus::Finalized),
            decision("b", DecisionStatus::Open),
        ];
        assert_eq!(open_decision_count(&decisions, Some("a")), Some(1));
    }

    #[test]
    fn alert_count_excludes_resolved_and_other_hearth_alerts() {
        let alerts = [
            alert("a", None),
            alert("a", Some(100)),
            alert("b", None),
        ];
        assert_eq!(unresolved_alert_count(&alerts, Some("a")), Some(1));
    }

    #[test]
    fn empty_source_lists_produce_zero_counts() {
        assert_eq!(active_care_plan_count(&[], Some("a")), Some(0));
        assert_eq!(open_decision_count(&[], Some("a")), Some(0));
        assert_eq!(unresolved_alert_count(&[], Some("a")), Some(0));
    }
}
