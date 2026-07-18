// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use leptos::prelude::*;
use praxis_core::{CourseId, CourseSummary, LEARNING_COORDINATOR_ZOME, LIST_COURSE_SUMMARIES_FN};

use crate::holochain::{
    DataSource, LiveResourceStatus, ResourceState, tracked_data_source, use_holochain,
};
use crate::mode::{AppMode, use_app_mode};

/// Representative courses shown only in explicit Demo mode.
fn demo_courses() -> Vec<CourseSummary> {
    vec![
        CourseSummary {
            course_id: CourseId("demo-rust-fundamentals".into()),
            title: "Rust Fundamentals".into(),
            description: "Learn systems programming with Rust".into(),
            domain: "Programming".into(),
            updated_at: 0,
        },
        CourseSummary {
            course_id: CourseId("demo-distributed-systems".into()),
            title: "Distributed Systems".into(),
            description: "P2P architecture and consensus".into(),
            domain: "Computer Science".into(),
            updated_at: 0,
        },
        CourseSummary {
            course_id: CourseId("demo-regenerative-agriculture".into()),
            title: "Regenerative Agriculture".into(),
            description: "Soil science and permaculture".into(),
            domain: "Agriculture".into(),
            updated_at: 0,
        },
        CourseSummary {
            course_id: CourseId("demo-cooperative-economics".into()),
            title: "Cooperative Economics".into(),
            description: "Mutual aid, commons governance, and solidarity economy".into(),
            domain: "Economics".into(),
            updated_at: 0,
        },
    ]
}

// ---------------------------------------------------------------------------
// Courses page
// ---------------------------------------------------------------------------

#[component]
pub fn CoursesPage() -> impl IntoView {
    let hc = use_holochain();
    let mode = use_app_mode();

    // Data provenance is selected explicitly; Live failures are recorded by
    // HolochainCtx and surfaced in the global mode banner.
    let courses = LocalResource::new(move || {
        let hc = hc.clone();
        let source = tracked_data_source(mode, &hc);
        async move {
            match source {
                DataSource::Demo => ResourceState::Ready(demo_courses()),
                DataSource::Local => ResourceState::Ready(Vec::new()),
                DataSource::LiveWaiting => ResourceState::WaitingForLive,
                DataSource::LiveReady => match hc
                    .call_zome_default::<(), Vec<CourseSummary>>(
                        LEARNING_COORDINATOR_ZOME,
                        LIST_COURSE_SUMMARIES_FN,
                        &(),
                    )
                    .await
                {
                    Ok(data) => ResourceState::Ready(data),
                    Err(_) => ResourceState::LiveError,
                },
            }
        }
    });

    view! {
        <div class="courses-page">
            <div class="page-header">
                <h2>"Courses"</h2>
                <ConnectionStatusTag />
            </div>
            <Suspense fallback=move || view! { <CoursesLoading /> }>
                {move || {
                    courses.get().map(|state| match state {
                        ResourceState::WaitingForLive => {
                            view! { <LiveResourceStatus failed=false /> }.into_any()
                        }
                        ResourceState::LiveError => {
                            view! { <LiveResourceStatus failed=true /> }.into_any()
                        }
                        ResourceState::Ready(data) if data.is_empty() => {
                            let message = match mode.get() {
                                AppMode::Demo => "No demonstration courses are registered.",
                                AppMode::Local => "Network-published courses are not loaded in Local mode. You can still explore the Knowledge Garden and keep local progress.",
                                AppMode::Live => "The connected network returned no courses.",
                            };
                            view! { <div class="data-empty-state">{message}</div> }.into_any()
                        }
                        ResourceState::Ready(data) => view! {
                            <div class="course-grid">
                                {data.into_iter().map(|course| {
                                    view! {
                                        <div class="course-card">
                                            <h3>{course.title}</h3>
                                            <p>{course.description}</p>
                                            <span class="domain-tag">{course.domain}</span>
                                        </div>
                                    }
                                }).collect_view()}
                            </div>
                        }.into_any(),
                    })
                }}
            </Suspense>
        </div>
    }
}

/// Loading skeleton for the courses grid.
#[component]
fn CoursesLoading() -> impl IntoView {
    view! {
        <div class="course-grid">
            <div class="course-card skeleton"></div>
            <div class="course-card skeleton"></div>
            <div class="course-card skeleton"></div>
        </div>
    }
}

/// Small inline tag showing the actual selected data source.
#[component]
fn ConnectionStatusTag() -> impl IntoView {
    let hc = use_holochain();
    let hc_label = hc.clone();
    view! {
        <span class=move || {
            format!("connection-tag {}", hc.status_css_class())
        }>
            {move || hc_label.status_label()}
        </span>
    }
}
