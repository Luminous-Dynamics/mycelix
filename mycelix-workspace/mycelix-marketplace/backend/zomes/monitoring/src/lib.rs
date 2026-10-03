// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/// Monitoring and Metrics System for Mycelix-Marketplace
///
/// This module provides real-time monitoring, Byzantine attack detection,
/// and operational visibility for the marketplace.
use hdk::prelude::*;
use std::collections::VecDeque;

/// Metric types
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MetricType {
    /// Transaction created
    TransactionCreated,
    /// Transaction completed
    TransactionCompleted,
    /// Transaction disputed
    TransactionDisputed,
    /// Byzantine attempt detected
    ByzantineAttempt,
    /// High-risk agent flagged
    HighRiskAgent,
    /// MATL score updated
    MatlScoreUpdated,
    /// Listing created
    ListingCreated,
    /// Review submitted
    ReviewSubmitted,
    /// Arbitration initiated
    ArbitrationInitiated,
    /// Cache hit
    CacheHit,
    /// Cache miss
    CacheMiss,
    /// Message sent
    MessageSent,
    /// Conversation started
    ConversationStarted,
    /// Conversation blocked (spam)
    ConversationBlocked,
    /// Message read
    MessageRead,
    /// Bridge zome error (cross-app reputation)
    BridgeError,
    /// Bridge reputation reported successfully
    BridgeReputationReported,
    /// Cross-app reputation retrieved
    CrossAppReputationRetrieved,
}

/// Metric event
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MetricEvent {
    pub metric_type: MetricType,
    pub value: f64,
    pub agent: Option<AgentPubKey>,
    pub metadata: Option<String>,
    pub timestamp: Timestamp,
}

impl MetricEvent {
    pub fn new(
        metric_type: MetricType,
        value: f64,
        agent: Option<AgentPubKey>,
        metadata: Option<String>,
    ) -> ExternResult<Self> {
        Self::new_at(metric_type, value, agent, metadata, sys_time()?)
    }

    pub fn new_at(
        metric_type: MetricType,
        value: f64,
        agent: Option<AgentPubKey>,
        metadata: Option<String>,
        timestamp: Timestamp,
    ) -> ExternResult<Self> {
        Ok(Self {
            metric_type,
            value,
            agent,
            metadata,
            timestamp,
        })
    }
}

/// Alert severity levels
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AlertSeverity {
    Info,
    Warning,
    Critical,
}

/// Alert types
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Alert {
    pub severity: AlertSeverity,
    pub message: String,
    pub metric_type: MetricType,
    pub value: f64,
    pub threshold: f64,
    pub timestamp: Timestamp,
}

impl Alert {
    pub fn new(
        severity: AlertSeverity,
        message: String,
        metric_type: MetricType,
        value: f64,
        threshold: f64,
    ) -> ExternResult<Self> {
        Self::new_at(
            severity,
            message,
            metric_type,
            value,
            threshold,
            sys_time()?,
        )
    }

    pub fn new_at(
        severity: AlertSeverity,
        message: String,
        metric_type: MetricType,
        value: f64,
        threshold: f64,
        timestamp: Timestamp,
    ) -> ExternResult<Self> {
        Ok(Self {
            severity,
            message,
            metric_type,
            value,
            threshold,
            timestamp,
        })
    }

    pub fn byzantine_spike(count: u64) -> ExternResult<Self> {
        Self::byzantine_spike_at(count, sys_time()?)
    }

    fn byzantine_spike_at(count: u64, timestamp: Timestamp) -> ExternResult<Self> {
        Self::new_at(
            AlertSeverity::Critical,
            format!(
                "Byzantine attempt spike detected: {} attempts in last hour",
                count
            ),
            MetricType::ByzantineAttempt,
            count as f64,
            100.0,
            timestamp,
        )
    }

    pub fn high_dispute_rate(rate: f64) -> ExternResult<Self> {
        Self::high_dispute_rate_at(rate, sys_time()?)
    }

    fn high_dispute_rate_at(rate: f64, timestamp: Timestamp) -> ExternResult<Self> {
        Self::new_at(
            AlertSeverity::Warning,
            format!("Dispute rate elevated: {:.1}%", rate * 100.0),
            MetricType::TransactionDisputed,
            rate,
            0.1, // 10% threshold
            timestamp,
        )
    }

    pub fn network_compromised(average_matl: f64) -> ExternResult<Self> {
        Self::network_compromised_at(average_matl, sys_time()?)
    }

    fn network_compromised_at(average_matl: f64, timestamp: Timestamp) -> ExternResult<Self> {
        Self::new_at(
            AlertSeverity::Critical,
            format!(
                "Network average MATL score critically low: {:.2}",
                average_matl
            ),
            MetricType::MatlScoreUpdated,
            average_matl,
            0.5,
            timestamp,
        )
    }
}

/// Marketplace metrics aggregator
pub struct MarketplaceMetrics {
    // Counters
    pub total_transactions: u64,
    pub completed_transactions: u64,
    pub disputed_transactions: u64,
    pub byzantine_attempts: u64,
    pub total_listings: u64,
    pub total_reviews: u64,

    // Aggregates
    pub average_matl_score: f64,
    pub matl_score_count: u64,

    // Recent events (for rate calculation)
    pub recent_events: VecDeque<MetricEvent>,
    pub max_recent_events: usize,

    // Alerts
    pub active_alerts: Vec<Alert>,
}

impl MarketplaceMetrics {
    pub fn new(max_recent_events: usize) -> Self {
        Self {
            total_transactions: 0,
            completed_transactions: 0,
            disputed_transactions: 0,
            byzantine_attempts: 0,
            total_listings: 0,
            total_reviews: 0,
            average_matl_score: 0.5,
            matl_score_count: 0,
            recent_events: VecDeque::new(),
            max_recent_events,
            active_alerts: Vec::new(),
        }
    }

    pub fn default() -> Self {
        Self::new(1000) // Keep last 1000 events
    }

    /// Record a metric event
    pub fn record_event(&mut self, event: MetricEvent) -> ExternResult<()> {
        // Update counters
        match event.metric_type {
            MetricType::TransactionCreated => {
                self.total_transactions += 1;
            }
            MetricType::TransactionCompleted => {
                self.completed_transactions += 1;
            }
            MetricType::TransactionDisputed => {
                self.disputed_transactions += 1;
            }
            MetricType::ByzantineAttempt => {
                self.byzantine_attempts += 1;
            }
            MetricType::ListingCreated => {
                self.total_listings += 1;
            }
            MetricType::ReviewSubmitted => {
                self.total_reviews += 1;
            }
            MetricType::MatlScoreUpdated => {
                // Update running average
                self.average_matl_score = (self.average_matl_score * self.matl_score_count as f64
                    + event.value)
                    / (self.matl_score_count + 1) as f64;
                self.matl_score_count += 1;
            }
            _ => {}
        }

        // Add to recent events
        let event_timestamp = event.timestamp.clone();
        self.recent_events.push_back(event);

        // Trim if over limit
        while self.recent_events.len() > self.max_recent_events {
            self.recent_events.pop_front();
        }

        // Check for anomalies
        self.check_anomalies_at(event_timestamp)?;

        Ok(())
    }

    /// Check for anomalous patterns and generate alerts
    pub fn check_anomalies(&mut self) -> ExternResult<()> {
        self.check_anomalies_at(sys_time()?)
    }

    fn check_anomalies_at(&mut self, timestamp: Timestamp) -> ExternResult<()> {
        // 1. Byzantine attempt spike
        let recent_byzantine = self
            .recent_events
            .iter()
            .filter(|e| e.metric_type == MetricType::ByzantineAttempt)
            .count() as u64;

        if recent_byzantine > 100 {
            let alert = Alert::byzantine_spike_at(recent_byzantine, timestamp.clone())?;
            self.active_alerts.push(alert);
        }

        // 2. High dispute rate
        if self.total_transactions > 0 {
            let dispute_rate = self.disputed_transactions as f64 / self.total_transactions as f64;
            if dispute_rate > 0.1 {
                // >10%
                let alert = Alert::high_dispute_rate_at(dispute_rate, timestamp.clone())?;
                self.active_alerts.push(alert);
            }
        }

        // 3. Network average MATL too low (under attack)
        if self.average_matl_score < 0.5 && self.matl_score_count > 100 {
            let alert = Alert::network_compromised_at(self.average_matl_score, timestamp.clone())?;
            self.active_alerts.push(alert);
        }

        Ok(())
    }

    /// Calculate success rate
    pub fn success_rate(&self) -> f64 {
        if self.total_transactions == 0 {
            return 1.0;
        }

        self.completed_transactions as f64 / self.total_transactions as f64
    }

    /// Calculate dispute rate
    pub fn dispute_rate(&self) -> f64 {
        if self.total_transactions == 0 {
            return 0.0;
        }

        self.disputed_transactions as f64 / self.total_transactions as f64
    }

    /// Get Byzantine attempt rate (per 1000 transactions)
    pub fn byzantine_attempt_rate(&self) -> f64 {
        if self.total_transactions == 0 {
            return 0.0;
        }

        (self.byzantine_attempts as f64 / self.total_transactions as f64) * 1000.0
    }

    /// Get dashboard summary
    pub fn get_dashboard(&self) -> MarketplaceDashboard {
        MarketplaceDashboard {
            total_transactions: self.total_transactions,
            success_rate: self.success_rate(),
            dispute_rate: self.dispute_rate(),
            average_matl_score: self.average_matl_score,
            byzantine_attempts: self.byzantine_attempts,
            byzantine_attempt_rate: self.byzantine_attempt_rate(),
            total_listings: self.total_listings,
            total_reviews: self.total_reviews,
            active_alerts: self.active_alerts.clone(),
        }
    }
}

/// Dashboard summary
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MarketplaceDashboard {
    pub total_transactions: u64,
    pub success_rate: f64,
    pub dispute_rate: f64,
    pub average_matl_score: f64,
    pub byzantine_attempts: u64,
    pub byzantine_attempt_rate: f64,
    pub total_listings: u64,
    pub total_reviews: u64,
    pub active_alerts: Vec<Alert>,
}

/// Performance metrics
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PerformanceMetrics {
    pub average_query_time_ms: f64,
    pub cache_hit_rate: f64,
    pub total_queries: u64,
    pub cache_hits: u64,
    pub cache_misses: u64,
}

impl PerformanceMetrics {
    pub fn new() -> Self {
        Self {
            average_query_time_ms: 0.0,
            cache_hit_rate: 0.0,
            total_queries: 0,
            cache_hits: 0,
            cache_misses: 0,
        }
    }

    pub fn record_cache_hit(&mut self) {
        self.cache_hits += 1;
        self.total_queries += 1;
        self.update_cache_hit_rate();
    }

    pub fn record_cache_miss(&mut self) {
        self.cache_misses += 1;
        self.total_queries += 1;
        self.update_cache_hit_rate();
    }

    fn update_cache_hit_rate(&mut self) {
        if self.total_queries > 0 {
            self.cache_hit_rate = self.cache_hits as f64 / self.total_queries as f64;
        }
    }
}

/// Global metrics instance
static mut GLOBAL_METRICS: Option<MarketplaceMetrics> = None;
static mut GLOBAL_PERF_METRICS: Option<PerformanceMetrics> = None;

/// Get or initialize global metrics
pub fn get_metrics() -> &'static mut MarketplaceMetrics {
    unsafe {
        let ptr = std::ptr::addr_of_mut!(GLOBAL_METRICS);
        if (*ptr).is_none() {
            *ptr = Some(MarketplaceMetrics::default());
        }
        (*ptr).as_mut().unwrap()
    }
}

/// Get or initialize performance metrics
pub fn get_perf_metrics() -> &'static mut PerformanceMetrics {
    unsafe {
        let ptr = std::ptr::addr_of_mut!(GLOBAL_PERF_METRICS);
        if (*ptr).is_none() {
            *ptr = Some(PerformanceMetrics::new());
        }
        (*ptr).as_mut().unwrap()
    }
}

/// Emit metric (public API)
pub fn emit_metric(
    metric_type: MetricType,
    value: f64,
    agent: Option<AgentPubKey>,
    metadata: Option<String>,
) -> ExternResult<()> {
    let event = MetricEvent::new(metric_type, value, agent, metadata)?;
    let metrics = get_metrics();
    metrics.record_event(event)?;
    Ok(())
}

/// Get dashboard (public API)
pub fn get_dashboard() -> MarketplaceDashboard {
    let metrics = get_metrics();
    metrics.get_dashboard()
}

/// Get active alerts
pub fn get_active_alerts() -> Vec<Alert> {
    let metrics = get_metrics();
    metrics.active_alerts.clone()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_timestamp() -> Timestamp {
        Timestamp::from_micros(1_000_000)
    }

    fn event(metric_type: MetricType, value: f64) -> MetricEvent {
        MetricEvent::new_at(metric_type, value, None, None, test_timestamp()).unwrap()
    }

    #[test]
    fn test_metric_event_creation() {
        let event = MetricEvent::new_at(
            MetricType::TransactionCreated,
            1.0,
            None,
            Some("test".to_string()),
            test_timestamp(),
        )
        .unwrap();

        assert_eq!(event.metric_type, MetricType::TransactionCreated);
        assert_eq!(event.value, 1.0);
        assert_eq!(event.timestamp, test_timestamp());
    }

    #[test]
    fn test_metrics_recording() {
        let mut metrics = MarketplaceMetrics::default();

        metrics.record_event(event(MetricType::TransactionCreated, 1.0)).unwrap();
        assert_eq!(metrics.total_transactions, 1);

        metrics
            .record_event(event(MetricType::TransactionCompleted, 1.0))
            .unwrap();
        assert_eq!(metrics.completed_transactions, 1);
    }

    #[test]
    fn test_success_rate_calculation() {
        let mut metrics = MarketplaceMetrics::default();

        for _ in 0..10 {
            metrics.record_event(event(MetricType::TransactionCreated, 1.0)).unwrap();
        }

        for _ in 0..7 {
            metrics
                .record_event(event(MetricType::TransactionCompleted, 1.0))
                .unwrap();
        }

        assert_eq!(metrics.success_rate(), 0.7);
    }

    #[test]
    fn test_dispute_rate_calculation() {
        let mut metrics = MarketplaceMetrics::default();

        for _ in 0..10 {
            metrics.record_event(event(MetricType::TransactionCreated, 1.0)).unwrap();
        }

        for _ in 0..2 {
            metrics
                .record_event(event(MetricType::TransactionDisputed, 1.0))
                .unwrap();
        }

        assert_eq!(metrics.dispute_rate(), 0.2);
    }

    #[test]
    fn test_matl_average_calculation() {
        let mut metrics = MarketplaceMetrics::default();

        for score in [0.8, 0.7, 0.9, 0.6] {
            metrics
                .record_event(event(MetricType::MatlScoreUpdated, score))
                .unwrap();
        }

        assert!((metrics.average_matl_score - 0.75).abs() < 0.01);
    }

    #[test]
    fn test_byzantine_spike_alert() {
        let alert = Alert::byzantine_spike_at(150, test_timestamp()).unwrap();

        assert_eq!(alert.severity, AlertSeverity::Critical);
        assert!(alert.message.contains("150"));
        assert_eq!(alert.timestamp, test_timestamp());
    }

    #[test]
    fn test_high_dispute_rate_alert() {
        let alert = Alert::high_dispute_rate_at(0.15, test_timestamp()).unwrap();

        assert_eq!(alert.severity, AlertSeverity::Warning);
        assert!(alert.message.contains("15"));
        assert_eq!(alert.timestamp, test_timestamp());
    }

    #[test]
    fn test_network_compromised_alert() {
        let alert = Alert::network_compromised_at(0.3, test_timestamp()).unwrap();

        assert_eq!(alert.severity, AlertSeverity::Critical);
        assert!(alert.message.contains("0.30"));
        assert_eq!(alert.timestamp, test_timestamp());
    }

    #[test]
    fn test_anomaly_detection() {
        let mut metrics = MarketplaceMetrics::default();

        for _ in 0..150 {
            metrics
                .record_event(event(MetricType::ByzantineAttempt, 1.0))
                .unwrap();
        }

        assert!(!metrics.active_alerts.is_empty());
        assert!(
            metrics
                .active_alerts
                .iter()
                .any(|alert| alert.severity == AlertSeverity::Critical)
        );
        assert_eq!(metrics.active_alerts[0].timestamp, test_timestamp());
    }

    #[test]
    fn test_dashboard_generation() {
        let mut metrics = MarketplaceMetrics::default();

        for _ in 0..10 {
            metrics
                .record_event(event(MetricType::TransactionCreated, 1.0))
                .unwrap();
        }

        let dashboard = metrics.get_dashboard();

        assert_eq!(dashboard.total_transactions, 10);
        assert!(dashboard.success_rate >= 0.0 && dashboard.success_rate <= 1.0);
    }

    #[test]
    fn test_performance_metrics() {
        let mut perf = PerformanceMetrics::new();

        perf.record_cache_hit();
        perf.record_cache_hit();
        perf.record_cache_miss();

        assert_eq!(perf.cache_hits, 2);
        assert_eq!(perf.cache_misses, 1);
        assert_eq!(perf.total_queries, 3);
        assert!((perf.cache_hit_rate - 0.666).abs() < 0.01);
    }

    #[test]
    fn test_recent_events_size_limit() {
        let mut metrics = MarketplaceMetrics::new(10);

        for i in 0..20 {
            metrics
                .record_event(event(MetricType::ListingCreated, i as f64))
                .unwrap();
        }

        assert_eq!(metrics.recent_events.len(), 10);
        assert_eq!(metrics.recent_events.front().unwrap().value, 10.0);
        assert_eq!(metrics.recent_events.back().unwrap().value, 19.0);
    }
}
