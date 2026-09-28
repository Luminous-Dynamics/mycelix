use cos_conformance::integral_demo_cockpit::{fields_for, project_cockpit, CockpitField};
use cos_conformance::integral_demo_domain::{ProvenanceClass, SourceKind};
use cos_conformance::integral_demo_trace::{
    validate_trace, ExplanationLevel, TraceActor, TraceEvent, TraceKind,
};
use leptos::prelude::*;

const TRACE: [TraceEvent; 12] = [
    event("proposal-1", 1, TraceKind::Proposal, TraceActor::Human, ProvenanceClass::Proposal, None, true),
    event("design-1", 2, TraceKind::Design, TraceActor::System, ProvenanceClass::Design, None, true),
    event("decision-1", 3, TraceKind::Decision, TraceActor::Human, ProvenanceClass::Decision, Some("cds-auth-1"), true),
    event("authorization-1", 4, TraceKind::Authorization, TraceActor::Human, ProvenanceClass::Authorization, Some("production-auth-1"), true),
    event("execution-1", 5, TraceKind::ExecutionIntent, TraceActor::System, ProvenanceClass::ExecutionIntent, Some("production-auth-1"), true),
    event("observation-1", 6, TraceKind::Observation, TraceActor::System, ProvenanceClass::Observation, None, true),
    event("itc-1", 7, TraceKind::ItcProjection, TraceActor::System, ProvenanceClass::Assessment, None, true),
    event("frs-1", 8, TraceKind::FrsAssessment, TraceActor::System, ProvenanceClass::Assessment, None, true),
    event("recommendation-1", 9, TraceKind::Recommendation, TraceActor::Symthaea, ProvenanceClass::Recommendation, None, true),
    event("human-decision-1", 10, TraceKind::HumanDecision, TraceActor::Human, ProvenanceClass::Decision, Some("review-auth-1"), true),
    event("outcome-1", 11, TraceKind::Outcome, TraceActor::System, ProvenanceClass::Outcome, Some("review-auth-1"), true),
    event("appeal-1", 12, TraceKind::Appeal, TraceActor::Human, ProvenanceClass::Appeal, None, true),
];

// D6C boundary: the scenario selector changes presentation context only. The authoritative
// reference trace remains immutable until a scenario receives its own executable fixture.

const SCENARIOS: [Scenario; 8] = [
    Scenario { name: "Normal flow", summary: "A complete bounded path from design through human decision, outcome, and appeal.", status: "Trace validates", note: "The cockpit shows lineage without turning presentation into authority." },
    Scenario { name: "Recommendation declined", summary: "Symthaea provides a recommendation, but the human decision does not have to follow it.", status: "Human choice preserved", note: "Recommendation and decision remain separate provenance classes." },
    Scenario { name: "Uncertain observation", summary: "An observation carries uncertainty all the way through projection and review.", status: "Uncertainty visible", note: "The UI never replaces uncertainty with a confidence score." },
    Scenario { name: "Conflicting evidence", summary: "Two observations disagree and remain distinguishable rather than being silently merged.", status: "Conflict preserved", note: "Disagreement is something to inspect, not something the cockpit resolves." },
    Scenario { name: "Foreign evidence", summary: "Evidence originating at another node remains foreign after recognition.", status: "Origin preserved", note: "Recognition does not become local authorship or local authority." },
    Scenario { name: "Stale design", summary: "A superseded design generation is rejected before it can become current production intent.", status: "Fails closed", note: "The cockpit can explain the rejection without pretending it is an execution result." },
    Scenario { name: "Appealed outcome", summary: "A consequential outcome exposes a recovery and challenge path.", status: "Appeal available", note: "Disagreement remains possible without depending on Symthaea." },
    Scenario { name: "No Symthaea", summary: "The same trace remains understandable when the assistive cognition layer is unavailable.", status: "Graceful fallback", note: "Authority and provenance live in the trace, not in the model." },
];

#[derive(Clone, Copy)]
struct Scenario {
    name: &'static str,
    summary: &'static str,
    status: &'static str,
    note: &'static str,
}

const fn event(
    event_id: &'static str,
    sequence: u32,
    kind: TraceKind,
    actor: TraceActor,
    provenance: ProvenanceClass,
    authority_ref: Option<&'static str>,
    reversible: bool,
) -> TraceEvent {
    TraceEvent {
        event_id,
        sequence,
        kind,
        provenance,
        actor,
        source: SourceKind::Local,
        source_ref: "evidence://integral-demo",
        generation: 7,
        uncertainty_present: true,
        authority_ref,
        reversible,
        challengeable: true,
        recommendation_only: matches!(kind, TraceKind::Recommendation),
        recovery_ref: if matches!(kind, TraceKind::Outcome) { Some("recovery://outcome-1") } else { None },
        appeal_ref: if matches!(kind, TraceKind::Outcome) { Some("appeal://outcome-1") } else { None },
    }
}

#[component]
fn App() -> impl IntoView {
    let (selected, set_selected) = signal(0usize);
    let (level, set_level) = signal(ExplanationLevel::Summary);

    let trace_valid = validate_trace(&TRACE).is_ok();
    let assurance_view = project_cockpit("trace://integral-demo-001", ExplanationLevel::Assurance, &TRACE)
        .expect("static reference trace produces a cockpit view");
    let technical_view = project_cockpit("trace://integral-demo-001", ExplanationLevel::Technical, &TRACE)
        .expect("static reference trace produces a cockpit view");

    view! {
        <main class="shell">
            <header class="hero">
                <div>
                    <p class="eyebrow">"INTEGRAL · REFERENCE NODE"</p>
                    <h1>"Human-readable coordination, backed by evidence lineage."</h1>
                    <p class="lede">"A Leptos cockpit over the executable Mycelix reference model. It explains what happened without becoming the authority for what should happen."</p>
                </div>
                <div class="status-card">
                    <span class="status-dot"></span>
                    <div>
                        <strong>"Reference trace"</strong>
                        <span>{if trace_valid { "Validated" } else { "Invalid" }}</span>
                    </div>
                </div>
            </header>

            <section class="question-bar">
                <span>"Start here:"</span>
                <strong>{move || SCENARIOS[selected.get()].name}</strong>
                <span>{move || SCENARIOS[selected.get()].summary}</span>
            </section>

            <div class="layout">
                <aside class="scenario-panel">
                    <div class="section-heading">
                        <span class="eyebrow">"SCENARIOS"</span>
                        <span class="count">{SCENARIOS.len()}</span>
                    </div>
                    <div class="scenario-list">
                        {SCENARIOS.iter().enumerate().map(|(index, scenario)| {
                            view! {
                                <button
                                    class=move || if selected.get() == index { "scenario active" } else { "scenario" }
                                    on:click=move |_| set_selected.set(index)
                                >
                                    <span class="scenario-name">{scenario.name}</span>
                                    <span class="scenario-status">{scenario.status}</span>
                                </button>
                            }
                        }).collect_view()}
                    </div>
                </aside>

                <section class="cockpit">
                    <div class="section-heading">
                        <div>
                            <span class="eyebrow">"COCKPIT"</span>
                            <h2>{move || SCENARIOS[selected.get()].name}</h2>
                        </div>
                        <div class="levels">
                            {[
                                (ExplanationLevel::Summary, "Summary"),
                                (ExplanationLevel::Rationale, "Rationale"),
                                (ExplanationLevel::Assurance, "Assurance"),
                                (ExplanationLevel::Technical, "Technical"),
                            ].into_iter().map(|(value, label)| {
                                view! {
                                    <button
                                        class=move || if level.get() == value { "level active" } else { "level" }
                                        on:click=move |_| set_level.set(value)
                                    >{label}</button>
                                }
                            }).collect_view()}
                        </div>
                    </div>

                    <div class="scenario-summary">
                        <strong>{move || SCENARIOS[selected.get()].status}</strong>
                        <span>{move || SCENARIOS[selected.get()].note}</span>
                    </div>

                    <div class="facts">
                        {move || {
                            let fields = fields_for(level.get());
                            fields.iter().map(|field| match field {
                                CockpitField::WhatHappened => view! { <Fact title="What happened" value="The trace records a bounded lifecycle; presentation does not invent events." /> }.into_any(),
                                CockpitField::WhoProducedIt => view! { <Fact title="Who produced it" value="Human, system, and Symthaea actors are disclosed." /> }.into_any(),
                                CockpitField::Evidence => view! { <Fact title="Evidence" value="Each displayed fact points back to trace evidence." /> }.into_any(),
                                CockpitField::Authority => view! { <Fact title="Authority" value="Authorization is explicit; recommendations carry none." /> }.into_any(),
                                CockpitField::Uncertainty => view! { <Fact title="Uncertainty" value="Uncertainty remains visible rather than being collapsed into a score." /> }.into_any(),
                                CockpitField::RecommendationStatus => view! { <Fact title="Recommendation vs decision" value="Symthaea may recommend; a human decision remains distinct." /> }.into_any(),
                                CockpitField::Recovery => view! { <Fact title="Recovery" value="Consequential outcomes expose recovery metadata." /> }.into_any(),
                                CockpitField::Challenge => view! { <Fact title="Challenge" value="A challenge/appeal path remains visible without requiring Symthaea." /> }.into_any(),
                                CockpitField::Generation => view! { <Fact title="Generation" value="Schema generation remains explicit in the lineage." /> }.into_any(),
                                CockpitField::Origin => view! { <Fact title="Origin" value="Local versus foreign origin remains attributable." /> }.into_any(),
                            }).collect_view()
                        }}
                    </div>

                    <div class="trace-card">
                        <div class="trace-header">
                            <div>
                                <span class="eyebrow">"MACHINE-READABLE LINEAGE"</span>
                                <h3>"{move || if level.get() == ExplanationLevel::Technical { technical_view.trace_ref } else { assurance_view.trace_ref }}"</h3>
                            </div>
                            <span class="badge">"ReferenceModelOnly"</span>
                        </div>
                        <div class="trace-list">
                            {TRACE.iter().map(|event| {
                                let kind = format!("{:?}", event.kind);
                                let actor = format!("{:?}", event.actor);
                                view! {
                                    <div class="trace-row">
                                        <span class="sequence">{format!("{:02}", event.sequence)}</span>
                                        <div class="trace-main">
                                            <strong>{kind}</strong>
                                            <span>{event.event_id}</span>
                                        </div>
                                        <span class="trace-meta">{actor}</span>
                                        <span class="trace-meta">{format!("gen {}", event.generation)}</span>
                                        <span class="uncertainty">"uncertain"</span>
                                    </div>
                                }
                            }).collect_view()}
                        </div>
                    </div>

                    <div class="principles">
                        <div><strong>"Human authority"</strong><span>"Consequential action requires explicit authorization."</span></div>
                        <div><strong>"Evidence lineage"</strong><span>"Explanation is a view over evidence, never new evidence."</span></div>
                        <div><strong>"Graceful fallback"</strong><span>"The cockpit remains useful without Symthaea."</span></div>
                    </div>
                </section>
            </div>

            <footer>
                <span>"Integral defines the socio-economic semantics."</span>
                <span>"Mycelix makes the meaning operationally verifiable."</span>
                <span>"Symthaea helps humans understand and navigate it."</span>
            </footer>
        </main>
    }
}

#[component]
fn Fact(title: &'static str, value: &'static str) -> impl IntoView {
    view! {
        <article class="fact">
            <span class="fact-title">{title}</span>
            <p>{value}</p>
        </article>
    }
}

fn main() {
    leptos::mount::mount_to_body(|| view! { <App /> });
}
