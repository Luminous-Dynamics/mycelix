// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Deterministic local-only matching for the Mycelix Global Finder.
//!
//! This catalog searches only entries already supplied to the current process.
//! It performs no network requests, telemetry, capability lookup, identity
//! verification, or authority inference. The numeric score is textual
//! relevance only; it is not a trust, quality, reputation, or authority score.

use crate::finder::FinderEntry;

/// One local textual match plus its inspectable relevance score.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FinderMatch {
    pub entry: FinderEntry,
    pub relevance_score: u16,
}

/// In-memory catalog of provider-approved Finder entries.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct LocalFinderCatalog {
    entries: Vec<FinderEntry>,
}

impl LocalFinderCatalog {
    pub fn new(entries: Vec<FinderEntry>) -> Self {
        Self { entries }
    }

    pub fn entries(&self) -> &[FinderEntry] {
        &self.entries
    }

    /// Search the local catalog using transparent textual relevance rules.
    ///
    /// Empty queries intentionally return no results. A future Home/recents
    /// surface should supply recency explicitly rather than smuggling it into
    /// Finder relevance.
    pub fn search(&self, query: &str, limit: usize) -> Vec<FinderMatch> {
        let query = normalize(query);
        if query.is_empty() || limit == 0 {
            return Vec::new();
        }

        let mut matches: Vec<FinderMatch> = self
            .entries
            .iter()
            .filter_map(|entry| {
                relevance(entry, &query).map(|relevance_score| FinderMatch {
                    entry: entry.clone(),
                    relevance_score,
                })
            })
            .collect();

        matches.sort_by(|left, right| {
            right
                .relevance_score
                .cmp(&left.relevance_score)
                .then_with(|| normalize(&left.entry.title).cmp(&normalize(&right.entry.title)))
                .then_with(|| left.entry.provider_id.cmp(&right.entry.provider_id))
                .then_with(|| left.entry.id.cmp(&right.entry.id))
        });
        matches.truncate(limit);
        matches
    }
}

fn normalize(value: &str) -> String {
    value.trim().to_lowercase()
}

fn relevance(entry: &FinderEntry, query: &str) -> Option<u16> {
    let title = normalize(&entry.title);
    let mut best = text_score(&title, query, 1_200, 1_000, 850, 700);

    if let Some(subtitle) = &entry.subtitle {
        best = best.max(text_score(
            &normalize(subtitle),
            query,
            700,
            625,
            550,
            450,
        ));
    }

    for keyword in &entry.keywords {
        best = best.max(text_score(
            &normalize(keyword),
            query,
            800,
            700,
            625,
            500,
        ));
    }

    (best > 0).then_some(best)
}

fn text_score(
    text: &str,
    query: &str,
    exact: u16,
    prefix: u16,
    word_prefix: u16,
    contains: u16,
) -> u16 {
    if text == query {
        exact
    } else if text.starts_with(query) {
        prefix
    } else if text
        .split_whitespace()
        .any(|word| word.starts_with(query))
    {
        word_prefix
    } else if text.contains(query) {
        contains
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::LocalFinderCatalog;
    use crate::finder::{FinderEntry, FinderKind};

    #[test]
    fn empty_query_does_not_turn_finder_into_an_implicit_recents_feed() {
        let catalog = LocalFinderCatalog::new(vec![FinderEntry::navigation(
            "knowledge",
            "doc-1",
            FinderKind::Document,
            "Local-first notes",
            "/documents/doc-1",
        )]);

        assert!(catalog.search("", 10).is_empty());
        assert!(catalog.search("   ", 10).is_empty());
    }

    #[test]
    fn exact_title_match_outranks_keyword_only_match() {
        let exact = FinderEntry::navigation(
            "knowledge",
            "doc-exact",
            FinderKind::Document,
            "Local first",
            "/documents/exact",
        );
        let mut keyword = FinderEntry::navigation(
            "projects",
            "project-keyword",
            FinderKind::Project,
            "Architecture notes",
            "/projects/architecture",
        );
        keyword.keywords.push("Local first".into());

        let results = LocalFinderCatalog::new(vec![keyword, exact]).search("local first", 10);

        assert_eq!(results.len(), 2);
        assert_eq!(results[0].entry.id, "doc-exact");
        assert!(results[0].relevance_score > results[1].relevance_score);
    }

    #[test]
    fn result_kind_does_not_change_textual_relevance() {
        let person = FinderEntry::navigation(
            "people",
            "alex",
            FinderKind::Person,
            "Alex",
            "/people/alex",
        );
        let action = FinderEntry::proposed_action(
            "actions",
            "alex-action",
            "Alex",
            "people.message",
            "alex",
        );

        let results = LocalFinderCatalog::new(vec![person, action]).search("Alex", 10);

        assert_eq!(results.len(), 2);
        assert_eq!(results[0].relevance_score, results[1].relevance_score);
    }

    #[test]
    fn result_limit_is_applied_after_deterministic_ranking() {
        let entries = vec![
            FinderEntry::navigation(
                "knowledge",
                "a",
                FinderKind::Document,
                "Alpha local",
                "/a",
            ),
            FinderEntry::navigation(
                "knowledge",
                "b",
                FinderKind::Document,
                "Local beta",
                "/b",
            ),
            FinderEntry::navigation(
                "knowledge",
                "c",
                FinderKind::Document,
                "Gamma local",
                "/c",
            ),
        ];

        assert_eq!(LocalFinderCatalog::new(entries).search("local", 2).len(), 2);
    }
}
