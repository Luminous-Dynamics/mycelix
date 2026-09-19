// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical task-first primary navigation for Mycelix frontends.
//!
//! This module defines only the stable user-facing shell destinations. It does
//! not own routes, domain state, authorization, or domain-local navigation.

use crate::{NavLink, NavTab};

/// The five primary user tasks in the shared v1 shell.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TaskSurface {
    Home,
    Find,
    Create,
    Inbox,
    Me,
}

impl TaskSurface {
    pub const ALL: [Self; 5] = [
        Self::Home,
        Self::Find,
        Self::Create,
        Self::Inbox,
        Self::Me,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Home => "Home",
            Self::Find => "Find",
            Self::Create => "Create",
            Self::Inbox => "Inbox",
            Self::Me => "Me",
        }
    }
}

/// Explicit route binding for the task-first shell.
///
/// No default route strings are invented here: each application chooses the
/// routes it actually implements and can adopt the task shell incrementally.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TaskShellRoutes {
    pub home: &'static str,
    pub find: &'static str,
    pub create: &'static str,
    pub inbox: &'static str,
    pub me: &'static str,
}

impl TaskShellRoutes {
    pub const fn new(
        home: &'static str,
        find: &'static str,
        create: &'static str,
        inbox: &'static str,
        me: &'static str,
    ) -> Self {
        Self {
            home,
            find,
            create,
            inbox,
            me,
        }
    }

    pub const fn href(self, surface: TaskSurface) -> &'static str {
        match surface {
            TaskSurface::Home => self.home,
            TaskSurface::Find => self.find,
            TaskSurface::Create => self.create,
            TaskSurface::Inbox => self.inbox,
            TaskSurface::Me => self.me,
        }
    }

    /// Desktop primary navigation in canonical task order.
    ///
    /// Icons are deliberately omitted in v1 so this helper adds no decorative
    /// accessibility semantics to the existing shell renderer.
    pub fn desktop_nav(self) -> Vec<NavLink> {
        TaskSurface::ALL
            .into_iter()
            .map(|surface| NavLink {
                href: self.href(surface),
                label: surface.label(),
                icon: None,
            })
            .collect()
    }

    /// Mobile primary navigation with the same conceptual order and routes.
    ///
    /// `NavTab` currently requires an icon string, so v1 supplies the empty
    /// string rather than introducing glyphs that the renderer does not yet
    /// mark decorative. Labels remain the complete accessible/visual names.
    pub fn mobile_tabs(self) -> Vec<NavTab> {
        TaskSurface::ALL
            .into_iter()
            .map(|surface| NavTab {
                href: self.href(surface),
                icon: "",
                label: surface.label(),
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::{TaskShellRoutes, TaskSurface};

    fn routes() -> TaskShellRoutes {
        TaskShellRoutes::new("/", "/find", "/create", "/inbox", "/me")
    }

    #[test]
    fn canonical_surface_order_is_stable() {
        assert_eq!(
            TaskSurface::ALL.map(TaskSurface::label),
            ["Home", "Find", "Create", "Inbox", "Me"]
        );
    }

    #[test]
    fn desktop_and_mobile_share_the_same_routes_and_labels() {
        let desktop = routes().desktop_nav();
        let mobile = routes().mobile_tabs();

        assert_eq!(desktop.len(), 5);
        assert_eq!(mobile.len(), 5);
        for (desktop, mobile) in desktop.iter().zip(mobile.iter()) {
            assert_eq!(desktop.href, mobile.href);
            assert_eq!(desktop.label, mobile.label);
        }
    }

    #[test]
    fn apps_are_not_silently_added_to_primary_navigation() {
        let labels = routes()
            .desktop_nav()
            .into_iter()
            .map(|item| item.label)
            .collect::<Vec<_>>();

        assert!(!labels.contains(&"Apps"));
    }
}
