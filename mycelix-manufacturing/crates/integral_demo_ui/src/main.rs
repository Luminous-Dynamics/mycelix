use cos_conformance::integral_demo_cockpit::{fields_for, project_cockpit, CockpitField};
use cos_conformance::integral_demo_scenarios::{evaluate, fixture_for, ALL_SCENARIOS};
use cos_conformance::integral_demo_trace::{ExplanationLevel, TraceKind};
use leptos::prelude::*;

#[component]
fn App() -> impl IntoView {
    let (selected, set_selected) = signal(0usize);
    let (level, set_level) = signal(ExplanationLevel::Assurance);

    view! {
        <main class="shell">
            <header class="hero">
                <div>
                    <p class="eyebrow">"INTEGRAL · REFERENCE NODE · D6D"</p>
                    <h1>"See the evidence path. Keep human authority visible."</h1>
                    <p class="lede">
                        "An executable Leptos cockpit over the Mycelix reference model. Each scenario is evaluated before it is presented; branch closure and causal relations are checked independently of the UI."
                    </p>
                </div>
                <div class="status-card">
                    <span class="status-dot"></span>
                    <div>
                        <strong>"Executable scenario corpus"</strong>
                        <span>"ReferenceModelOnly"</span>
                    </div>
                </div>
            </header>

            <div class="layout">
                <aside class="scenario-panel">
                    <div class="section-heading">
                        <span class="eyebrow">"SCENARIOS"</span>
                        <span class="count">{ALL_SCENARIOS.len()}</span>
                    </div>
                    <div class="scenario-list">
                        {ALL_SCENARIOS.iter().enumerate().map(|(index, id)| {
                            let result = evaluate(*id);
                            view! {
                                <button
                                    class=move || if selected.get() == index { "scenario active" } else { "scenario" }
                                    on:click=move |_| set_selected.set(index)
                                    aria-label=result.name
                                    aria-pressed=move || selected.get() == index
                                >
                                    <span class="scenario-name">{result.name}</span>
                                    <span class=if result.actual_valid { "scenario-status valid" } else { "scenario-status invalid" }>
                                        {if result.actual_valid { "valid reference path" } else { "fails closed" }}
                                    </span>
                                </button>
                            }
                        }).collect_view()}
                    </div>
                </aside>

                <section class="cockpit">
                    <div class="section-heading">
                        <div>
                            <span class="eyebrow">"COCKPIT"</span>
                            <h2>{move || evaluate(ALL_SCENARIOS[selected.get()]).name}</h2>
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
                                        aria-pressed=move || level.get() == value
                                    >{label}</button>
                                }
                            }).collect_view()}
                        </div>
                    </div>

                    {move || {
                        let id = ALL_SCENARIOS[selected.get()];
                        let result = evaluate(id);
                        let fixture = fixture_for(id);
                        let trace = &fixture.trace.events;
                        let cockpit = project_cockpit(fixture.trace_ref, level.get(), &fixture.trace);
                        let fields = fields_for(level.get());

                        view! {
                            <div class="scenario-summary">
                                <div class="result-line">
                                    <strong>{
    match result.expected {
        cos_conformance::integral_demo_scenarios::ScenarioExpected::PresentationOnly => "Presentation-only scenario",
        cos_conformance::integral_demo_scenarios::ScenarioExpected::Valid if result.actual_valid => "Reference path validates",
        _ if !result.actual_valid => "Reference path rejected",
        _ => "Reference path differs from expectation",
    }
}</strong>
                                    <span class="badge">{result.claim_ceiling}</span>
                                </div>
                                <span>{result.summary}</span>
                                <small>
                                    "Expected: " {format!("{:?}", result.expected)}
                                    " · Actual: " {if result.actual_valid { "valid" } else { "invalid" }}
                                    " · Events: " {result.trace_len}
                                    " · Relations: " {result.relation_count}
                                    " · Symthaea: " {if result.symthaea_used { "assistive" } else { "not required" }}
                                    {result.validation_error.map(|error| view! { <span>" · Boundary: " {format!("{:?}", error)}</span> })}
                                </small>
                            </div>

                            <div class="facts">
                                {fields.iter().map(|field| {
                                    let (title, value) = match field {
                                        CockpitField::WhatHappened => ("What happened", "The scenario engine supplied this trace; the UI does not invent events."),
                                        CockpitField::WhoProducedIt => ("Who produced it", "Actors remain explicit in the machine-readable lineage."),
                                        CockpitField::Evidence => ("Evidence", "Displayed facts retain their source references."),
                                        CockpitField::Authority => ("Authority", "Authorization is explicit and recommendations carry none."),
                                        CockpitField::Uncertainty => ("Uncertainty", "Uncertainty is preserved or the fixture fails closed."),
                                        CockpitField::RecommendationStatus => ("Recommendation vs decision", "A recommendation never becomes a decision by presentation."),
                                        CockpitField::Recovery => ("Recovery", "Consequential outcomes expose recovery metadata."),
                                        CockpitField::Challenge => ("Challenge", "The appeal path remains inspectable."),
                                        CockpitField::Generation => ("Generation", "Schema generation remains explicit."),
                                        CockpitField::Origin => ("Origin", "Foreign evidence remains attributable."),
                                    };
                                    view! { <Fact title=title value=value /> }
                                }).collect_view()}
                            </div>

                            <div class="trace-card" aria-live="polite">
                                <div class="trace-header">
                                    <div>
                                        <span class="eyebrow">"MACHINE-READABLE LINEAGE"</span>
                                        <h3>"trace://integral-demo-d6c"</h3>
                                    </div>
                                    <span class="badge">{if cockpit.is_some() { "projected" } else { "not projected" }}</span>
                                </div>
                                <div class="trace-list">
                                    {trace.iter().map(|event| {
                                        let kind = format!("{:?}", event.kind);
                                        let actor = format!("{:?}", event.actor);
                                        let status = match event.kind {
                                            TraceKind::Recommendation => "recommendation",
                                            TraceKind::HumanDecision => "human decision",
                                            TraceKind::Appeal => "appeal",
                                            _ => "event",
                                        };
                                        view! {
                                            <div class="trace-row">
                                                <span class="sequence">{format!("{:02}", event.sequence)}</span>
                                                <div class="trace-main">
                                                    <strong>{kind}</strong>
                                                    <span>{event.event_id}</span>
                                                </div>
                                                <span class="trace-meta">{actor}</span>
                                                <span class="trace-meta">{format!("{:?}", event.status)}</span>
                                                <span class="trace-meta">{status}</span>
                                                <span class="uncertainty">{if event.uncertainty_present { "uncertain" } else { "uncertainty lost" }}</span>
                                                {fixture.trace.relations.iter().filter(|relation| relation.from_event == event.event_id).map(|relation| {
                                                    view! {
                                                        <span class="trace-relation">
                                                            {format!("{:?} → {}", relation.relation, relation.to_event)}
                                                        </span>
                                                    }
                                                }).collect_view()}
                                            </div>
                                        }
                                    }).collect_view()}
                                </div>
                            </div>
                        }
                    }}

                    <div class="principles">
                        <div><strong>"Human authority"</strong><span>"Consequential action requires explicit authorization."</span></div>
                        <div><strong>"Evidence lineage"</strong><span>"Explanation is a view over evidence, never new evidence."</span></div>
                        <div><strong>"Graceful fallback"</strong><span>"The trace remains useful without Symthaea."</span></div>
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
