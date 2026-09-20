// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Support Types — Shared enums and DHT index helpers for the Support domain
//!
//! Provides category, priority, status, and sharding helpers used by all three
//! support zome pairs (knowledge, tickets, diagnostics).

use hdi::prelude::*;

// ============================================================================
// SUPPORT CATEGORY
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum SupportCategory {
    Network,
    Hardware,
    Software,
    Holochain,
    Mycelix,
    Security,
    General,
}

// ============================================================================
// TICKET PRIORITY
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TicketPriority {
    Low,
    Medium,
    High,
    Critical,
}

// ============================================================================
// TICKET STATUS
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TicketStatus {
    Open,
    InProgress,
    AwaitingUser,
    Resolved,
    Closed,
}

// ============================================================================
// AUTONOMY LEVEL
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AutonomyLevel {
    Advisory,
    SemiAutonomous,
    FullAutonomous,
}

// ============================================================================
// ACTION TYPE
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ActionType {
    RestartService,
    ClearCache,
    UpdateConfig,
    RunDiagnostic,
    Custom(String),
}

// ============================================================================
// SHARING TIER
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum SharingTier {
    LocalOnly,
    Anonymized,
    Full,
}

// ============================================================================
// DIAGNOSTIC TYPE
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DiagnosticType {
    NetworkCheck,
    DiskSpace,
    ServiceStatus,
    HolochainHealth,
    MemoryUsage,
    Custom(String),
}

// ============================================================================
// DIAGNOSTIC SEVERITY
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DiagnosticSeverity {
    Healthy,
    Warning,
    Error,
    Critical,
}

// ============================================================================
// DIFFICULTY LEVEL
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DifficultyLevel {
    Beginner,
    Intermediate,
    Advanced,
}

// ============================================================================
// ARTICLE SOURCE
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ArticleSource {
    Community,
    PreSeeded,
    SymthaeaGenerated,
}

// ============================================================================
// FLAG REASON
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum FlagReason {
    Harmful,
    Incorrect,
    Outdated,
    Spam,
}

// ============================================================================
// REPUTATION EVENT
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ReputationEvent {
    ResolutionVerified,
    ArticleUpvoted,
    ArticleFlagged,
    HelpProvided,
}

// ============================================================================
// EPISTEMIC STATUS
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EpistemicStatus {
    Certain,
    Probable,
    Uncertain,
    Unknown,
    OutOfDomain,
}

// ============================================================================
// TIME-SHARDED INDEX HELPERS
// ============================================================================

/// Generate a time-sharded anchor path: "support:tickets:2026:02"
///
/// Prevents DHT hot-link degradation by distributing links across monthly anchors.
pub fn sharded_anchor(domain: &str, entity: &str, timestamp: &Timestamp) -> String {
    let (year, month) = year_month_from_timestamp(timestamp);
    format!("{}:{}:{}:{:02}", domain, entity, year, month)
}

/// Generate a hash-sharded anchor: "support:articles:ab" (first byte hex of entry hash)
///
/// Distributes links across 256 buckets based on the first byte of the entry hash.
pub fn hash_sharded_anchor(domain: &str, entity: &str, entry_hash: &EntryHash) -> String {
    let raw = entry_hash.get_raw_39();
    let hex = format!("{:02x}", raw[0]);
    format!("{}:{}:{}", domain, entity, hex)
}

/// Extract the exact proleptic-Gregorian UTC `(year, month)` for a Holochain
/// `Timestamp`.
///
/// Holochain timestamps are signed microseconds since the Unix epoch.  The
/// conversion intentionally uses integer arithmetic only: no host/local
/// timezone, libc clock, floating point, or average-year/month approximation.
/// `div_euclid` gives correct floor semantics for pre-1970 timestamps too.
fn year_month_from_timestamp(ts: &Timestamp) -> (i32, u32) {
    let seconds = ts.as_micros().div_euclid(1_000_000);
    let days_since_epoch = seconds.div_euclid(86_400);
    civil_year_month_from_days(days_since_epoch)
}

/// Convert whole UTC days relative to 1970-01-01 into a Gregorian year/month.
///
/// This is the integer civil-date decomposition described by Howard Hinnant's
/// `civil_from_days` algorithm, expressed with Euclidean division so the same
/// relation remains valid for negative epoch offsets.  The supported range is
/// far wider than any practical Holochain timestamp while all intermediate
/// values remain inside `i64`.
fn civil_year_month_from_days(days_since_epoch: i64) -> (i32, u32) {
    // Shift the epoch so March is month zero.  That makes leap-day handling a
    // property of complete 400-year Gregorian eras rather than a special case.
    let z = days_since_epoch + 719_468;
    let era = z.div_euclid(146_097);
    let day_of_era = z - era * 146_097; // [0, 146096]
    let year_of_era =
        (day_of_era - day_of_era / 1_460 + day_of_era / 36_524 - day_of_era / 146_096)
            / 365; // [0, 399]
    let year = year_of_era + era * 400;
    let day_of_year =
        day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let march_month = (5 * day_of_year + 2) / 153; // [0, 11]
    let month = march_month + if march_month < 10 { 3 } else { -9 }; // [1, 12]
    let year = year + if month <= 2 { 1 } else { 0 };

    (year as i32, month as u32)
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ── Serde roundtrip: SupportCategory ────────────────────────────────

    #[test]
    fn serde_roundtrip_support_category() {
        let variants = vec![
            SupportCategory::Network,
            SupportCategory::Hardware,
            SupportCategory::Software,
            SupportCategory::Holochain,
            SupportCategory::Mycelix,
            SupportCategory::Security,
            SupportCategory::General,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: SupportCategory = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: TicketPriority ─────────────────────────────────

    #[test]
    fn serde_roundtrip_ticket_priority() {
        let variants = vec![
            TicketPriority::Low,
            TicketPriority::Medium,
            TicketPriority::High,
            TicketPriority::Critical,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: TicketPriority = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: TicketStatus ───────────────────────────────────

    #[test]
    fn serde_roundtrip_ticket_status() {
        let variants = vec![
            TicketStatus::Open,
            TicketStatus::InProgress,
            TicketStatus::AwaitingUser,
            TicketStatus::Resolved,
            TicketStatus::Closed,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: TicketStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: AutonomyLevel ──────────────────────────────────

    #[test]
    fn serde_roundtrip_autonomy_level() {
        let variants = vec![
            AutonomyLevel::Advisory,
            AutonomyLevel::SemiAutonomous,
            AutonomyLevel::FullAutonomous,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: AutonomyLevel = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: ActionType ─────────────────────────────────────

    #[test]
    fn serde_roundtrip_action_type() {
        let variants = vec![
            ActionType::RestartService,
            ActionType::ClearCache,
            ActionType::UpdateConfig,
            ActionType::RunDiagnostic,
            ActionType::Custom("install-patch".to_string()),
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: ActionType = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: SharingTier ────────────────────────────────────

    #[test]
    fn serde_roundtrip_sharing_tier() {
        let variants = vec![
            SharingTier::LocalOnly,
            SharingTier::Anonymized,
            SharingTier::Full,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: SharingTier = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: DiagnosticType ─────────────────────────────────

    #[test]
    fn serde_roundtrip_diagnostic_type() {
        let variants = vec![
            DiagnosticType::NetworkCheck,
            DiagnosticType::DiskSpace,
            DiagnosticType::ServiceStatus,
            DiagnosticType::HolochainHealth,
            DiagnosticType::MemoryUsage,
            DiagnosticType::Custom("custom-check".to_string()),
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: DiagnosticType = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: DiagnosticSeverity ─────────────────────────────

    #[test]
    fn serde_roundtrip_diagnostic_severity() {
        let variants = vec![
            DiagnosticSeverity::Healthy,
            DiagnosticSeverity::Warning,
            DiagnosticSeverity::Error,
            DiagnosticSeverity::Critical,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: DiagnosticSeverity = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: DifficultyLevel ────────────────────────────────

    #[test]
    fn serde_roundtrip_difficulty_level() {
        let variants = vec![
            DifficultyLevel::Beginner,
            DifficultyLevel::Intermediate,
            DifficultyLevel::Advanced,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: DifficultyLevel = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: ArticleSource ──────────────────────────────────

    #[test]
    fn serde_roundtrip_article_source() {
        let variants = vec![
            ArticleSource::Community,
            ArticleSource::PreSeeded,
            ArticleSource::SymthaeaGenerated,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: ArticleSource = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: FlagReason ─────────────────────────────────────

    #[test]
    fn serde_roundtrip_flag_reason() {
        let variants = vec![
            FlagReason::Harmful,
            FlagReason::Incorrect,
            FlagReason::Outdated,
            FlagReason::Spam,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: FlagReason = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: ReputationEvent ────────────────────────────────

    #[test]
    fn serde_roundtrip_reputation_event() {
        let variants = vec![
            ReputationEvent::ResolutionVerified,
            ReputationEvent::ArticleUpvoted,
            ReputationEvent::ArticleFlagged,
            ReputationEvent::HelpProvided,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: ReputationEvent = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Serde roundtrip: EpistemicStatus ────────────────────────────────

    #[test]
    fn serde_roundtrip_epistemic_status() {
        let variants = vec![
            EpistemicStatus::Certain,
            EpistemicStatus::Probable,
            EpistemicStatus::Uncertain,
            EpistemicStatus::Unknown,
            EpistemicStatus::OutOfDomain,
        ];
        for v in &variants {
            let json = serde_json::to_string(v).unwrap();
            let back: EpistemicStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, v);
        }
    }

    // ── Time-sharded anchor tests ───────────────────────────────────────

    #[test]
    fn sharded_anchor_format() {
        // 2026-02-15 00:00:00 UTC → 1_771_113_600 seconds → microseconds
        let ts = Timestamp::from_micros(1_771_113_600_000_000);
        let anchor = sharded_anchor("support", "tickets", &ts);
        assert_eq!(anchor, "support:tickets:2026:02");
    }

    #[test]
    fn sharded_anchor_different_months_differ() {
        let ts_jan = Timestamp::from_micros(1_704_067_200_000_000); // 2024-01-01 UTC
        let ts_jun = Timestamp::from_micros(1_717_200_000_000_000); // 2024-06-01 UTC
        let a1 = sharded_anchor("support", "tickets", &ts_jan);
        let a2 = sharded_anchor("support", "tickets", &ts_jun);
        assert_ne!(a1, a2);
    }

    #[test]
    fn sharded_anchor_same_month_same() {
        let ts1 = Timestamp::from_micros(1_771_113_600_000_000);
        let ts2 = Timestamp::from_micros(1_771_113_600_000_000 + 86_400_000_000); // +1 day
        let a1 = sharded_anchor("support", "tickets", &ts1);
        let a2 = sharded_anchor("support", "tickets", &ts2);
        assert_eq!(a1, a2);
    }

    // ── Hash-sharded anchor tests ───────────────────────────────────────

    /// Build a valid EntryHash with the correct 3-byte prefix [0x84, 0x21, 0x24]
    fn make_entry_hash(fill: u8) -> EntryHash {
        let mut raw = vec![0x84, 0x21, 0x24]; // ENTRY_PREFIX
        raw.extend(vec![fill; 36]);
        EntryHash::from_raw_39(raw)
    }

    #[test]
    fn hash_sharded_anchor_format() {
        let hash = make_entry_hash(0xab);
        let anchor = hash_sharded_anchor("support", "articles", &hash);
        // First byte is the prefix 0x84, so shard = "84"
        assert_eq!(anchor, "support:articles:84");
    }

    #[test]
    fn hash_sharded_anchor_different_data_same_prefix() {
        // Both have the same prefix bytes, so same first byte → same shard
        let h1 = make_entry_hash(0x00);
        let h2 = make_entry_hash(0xff);
        let a1 = hash_sharded_anchor("support", "articles", &h1);
        let a2 = hash_sharded_anchor("support", "articles", &h2);
        // Both start with 0x84 (ENTRY_PREFIX), so same shard
        assert_eq!(a1, a2);
        assert_eq!(a1, "support:articles:84");
    }

    #[test]
    fn hash_sharded_anchor_same_first_byte_same_shard() {
        let h1 = make_entry_hash(0xab);
        let h2 = make_entry_hash(0xcd);
        let a1 = hash_sharded_anchor("support", "articles", &h1);
        let a2 = hash_sharded_anchor("support", "articles", &h2);
        // Same prefix → same first byte → same shard
        assert_eq!(a1, a2);
    }

    // ── year_month_from_timestamp tests ─────────────────────────────────

    fn assert_year_month(micros: i64, expected: (i32, u32)) {
        let ts = Timestamp::from_micros(micros);
        assert_eq!(year_month_from_timestamp(&ts), expected);
    }

    #[test]
    fn year_month_epoch_is_1970_01() {
        assert_year_month(0, (1970, 1));
    }

    #[test]
    fn year_month_pre_epoch_uses_floor_day_semantics() {
        assert_year_month(-1, (1969, 12));
        assert_year_month(-1_000_000, (1969, 12));
    }

    #[test]
    fn year_month_handles_leap_day_boundaries() {
        assert_year_month(1_709_164_799_000_000, (2024, 2)); // 2024-02-28 23:59:59Z
        assert_year_month(1_709_164_800_000_000, (2024, 2)); // 2024-02-29 00:00:00Z
        assert_year_month(1_709_251_199_000_000, (2024, 2)); // 2024-02-29 23:59:59Z
        assert_year_month(1_709_251_200_000_000, (2024, 3)); // 2024-03-01 00:00:00Z
    }

    #[test]
    fn year_month_handles_year_boundary() {
        assert_year_month(1_735_689_599_000_000, (2024, 12)); // 2024-12-31 23:59:59Z
        assert_year_month(1_735_689_600_000_000, (2025, 1)); // 2025-01-01 00:00:00Z
    }

    #[test]
    fn year_month_known_regression_2026_09_19() {
        assert_year_month(1_789_776_000_000_000, (2026, 9));
        assert_eq!(
            sharded_anchor(
                "support",
                "tickets",
                &Timestamp::from_micros(1_789_776_000_000_000),
            ),
            "support:tickets:2026:09"
        );
    }

    #[test]
    fn year_month_gregorian_century_rules() {
        // 1900 is not a Gregorian leap year; 2000 is.
        assert_year_month(-2_203_934_400_000_000, (1900, 2)); // 1900-02-28 12:00:00Z
        assert_year_month(-2_203_848_000_000_000, (1900, 3)); // 1900-03-01 12:00:00Z
        assert_year_month(951_825_600_000_000, (2000, 2)); // 2000-02-29 12:00:00Z
    }

    #[test]
    fn civil_year_month_reference_days() {
        assert_eq!(civil_year_month_from_days(0), (1970, 1));
        assert_eq!(civil_year_month_from_days(-1), (1969, 12));
        assert_eq!(civil_year_month_from_days(19_782), (2024, 2)); // 2024-02-29
        assert_eq!(civil_year_month_from_days(20_350), (2025, 9)); // sanity away from boundaries
    }
}
