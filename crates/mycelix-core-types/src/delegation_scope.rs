//! Versioned finite-set scope semantics for delegated authority.
//!
//! This module is deliberately separate from ROS-005 exact-name v1/v2
//! ActionScope and ResourceScope. It does not change their equality semantics
//! or authorize an execution. It supplies a versioned profile for later
//! owner-controlled integration after canonical encoding and containment
//! rules are independently qualified.
//!
//! Resource identifiers are exact opaque identifiers in this version; no
//! hierarchy is inferred from string prefixes. Purpose and data class are
//! bound by commitments to owner-native ROS-004 consent semantics and are not
//! redefined here.
//!
//! Quantitative limits are static scope constraints only. This module does
//! not reserve or consume budgets, establish currentness, verify signatures,
//! authenticate clocks, or prove durable one-shot use.

use std::fmt;

const SCOPE_DOMAIN: &[u8] = b"mycelix.ros005.typed-scope.v1\0";
const SCHEMA_VERSION: u16 = 1;
const MAX_SET_ITEMS: usize = 1024;
const MAX_IDENTIFIER_BYTES: usize = 256;

/// Direction of a protected data/control flow.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum DataDirectionV1 {
    /// Data disclosed by the authority owner to the delegate.
    OwnerToDelegate,
    /// Data or control supplied by the delegate to the authority owner.
    DelegateToOwner,
    /// Effect local to the authority owner.
    OwnerLocal,
}

impl DataDirectionV1 {
    fn tag(self) -> u8 {
        match self {
            Self::OwnerToDelegate => 0,
            Self::DelegateToOwner => 1,
            Self::OwnerLocal => 2,
        }
    }
}

/// Commitment to existing consent semantics owned by ROS-004.
///
/// Equal commitments are required across a delegation edge. This type does
/// not decide whether consent is valid, current, or revocable.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ConsentBindingV1 {
    consent_profile_digest: [u8; 32],
    purpose_digest: [u8; 32],
    data_class_digest: [u8; 32],
}

impl ConsentBindingV1 {
    /// Bind a scope to exact ROS-004 consent, purpose and data-class profiles.
    pub fn new(
        consent_profile_digest: [u8; 32],
        purpose_digest: [u8; 32],
        data_class_digest: [u8; 32],
    ) -> Result<Self, TypedScopeError> {
        if consent_profile_digest == [0; 32]
            || purpose_digest == [0; 32]
            || data_class_digest == [0; 32]
        {
            return Err(TypedScopeError::ZeroDigest);
        }
        Ok(Self {
            consent_profile_digest,
            purpose_digest,
            data_class_digest,
        })
    }

    /// Digest of the owner-native consent profile.
    pub const fn consent_profile_digest(&self) -> &[u8; 32] {
        &self.consent_profile_digest
    }

    /// Digest of the exact purpose definition.
    pub const fn purpose_digest(&self) -> &[u8; 32] {
        &self.purpose_digest
    }

    /// Digest of the exact data-class definition.
    pub const fn data_class_digest(&self) -> &[u8; 32] {
        &self.data_class_digest
    }
}

/// Version-1 exact-operation binding for a typed delegation scope.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum OperationBindingV1 {
    /// Any operation within the other exact scope dimensions may be proposed.
    AnyWithinProfile,
    /// Only the immutable operation with this canonical digest is in scope.
    ExactDigest([u8; 32]),
}

impl OperationBindingV1 {
    fn is_no_wider_than(self, parent: Self) -> bool {
        match (self, parent) {
            (_, Self::AnyWithinProfile) => true,
            (Self::ExactDigest(child), Self::ExactDigest(parent)) => child == parent,
            (Self::AnyWithinProfile, Self::ExactDigest(_)) => false,
        }
    }

    fn tag(self) -> u8 {
        match self {
            Self::AnyWithinProfile => 0,
            Self::ExactDigest(_) => 1,
        }
    }
}

/// Rolling-window request-count limit.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct RateLimitV1 {
    max_events: u64,
    window_ms: u64,
}

impl RateLimitV1 {
    /// Construct a rolling-window limit with a positive duration.
    pub fn new(max_events: u64, window_ms: u64) -> Result<Self, TypedScopeError> {
        if window_ms == 0 {
            return Err(TypedScopeError::ZeroRateWindow);
        }
        Ok(Self {
            max_events,
            window_ms,
        })
    }

    /// Maximum admitted operations in any rolling window.
    pub const fn max_events(&self) -> u64 {
        self.max_events
    }

    /// Rolling-window duration in milliseconds.
    pub const fn window_ms(&self) -> u64 {
        self.window_ms
    }

    /// Whether every trace allowed by self also satisfies parent.
    ///
    /// The check is conservative. If the child's window is shorter, the
    /// parent window is partitioned into ceil(parent/child) subwindows and
    /// the worst-case event bound must fit the parent's limit. Overflow
    /// rejects containment rather than widening it.
    fn is_no_wider_than(self, parent: Self) -> bool {
        if self.window_ms >= parent.window_ms {
            return self.max_events <= parent.max_events;
        }

        let remainder = if parent.window_ms % self.window_ms == 0 {
            0
        } else {
            1
        };
        let segments = parent.window_ms / self.window_ms + remainder;
        segments
            .checked_mul(self.max_events)
            .is_some_and(|worst_case| worst_case <= parent.max_events)
    }
}

/// Static quantitative upper bounds for a delegation.
///
/// None means unbounded in that dimension. A child may add a bound where
/// the parent is unbounded, or reduce an existing bound; it may not remove a
/// parent bound or increase it. These limits do not implement durable
/// reservation or consumption.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct BudgetLimitsV1 {
    max_duration_ms: Option<u64>,
    max_operations: Option<u64>,
    max_bytes: Option<u64>,
    max_concurrent: Option<u32>,
    max_targets: Option<u32>,
    rate_limit: Option<RateLimitV1>,
}

impl BudgetLimitsV1 {
    /// Construct static upper bounds. A missing field is unbounded.
    pub const fn new(
        max_duration_ms: Option<u64>,
        max_operations: Option<u64>,
        max_bytes: Option<u64>,
        max_concurrent: Option<u32>,
        max_targets: Option<u32>,
        rate_limit: Option<RateLimitV1>,
    ) -> Self {
        Self {
            max_duration_ms,
            max_operations,
            max_bytes,
            max_concurrent,
            max_targets,
            rate_limit,
        }
    }

    /// Maximum lease duration in milliseconds.
    pub const fn max_duration_ms(&self) -> Option<u64> {
        self.max_duration_ms
    }

    /// Maximum total operation count.
    pub const fn max_operations(&self) -> Option<u64> {
        self.max_operations
    }

    /// Maximum total bytes.
    pub const fn max_bytes(&self) -> Option<u64> {
        self.max_bytes
    }

    /// Maximum concurrent operations.
    pub const fn max_concurrent(&self) -> Option<u32> {
        self.max_concurrent
    }

    /// Maximum distinct target count.
    pub const fn max_targets(&self) -> Option<u32> {
        self.max_targets
    }

    /// Optional rolling-window limit.
    pub const fn rate_limit(&self) -> Option<RateLimitV1> {
        self.rate_limit
    }

    const fn is_unbounded(self) -> bool {
        self.max_duration_ms.is_none()
            && self.max_operations.is_none()
            && self.max_bytes.is_none()
            && self.max_concurrent.is_none()
            && self.max_targets.is_none()
            && self.rate_limit.is_none()
    }

    /// Whether these limits are no wider than the parent's limits.
    pub fn is_no_wider_than(&self, parent: &Self) -> bool {
        optional_max_is_no_wider(self.max_duration_ms, parent.max_duration_ms)
            && optional_max_is_no_wider(self.max_operations, parent.max_operations)
            && optional_max_is_no_wider(self.max_bytes, parent.max_bytes)
            && optional_max_is_no_wider(self.max_concurrent, parent.max_concurrent)
            && optional_max_is_no_wider(self.max_targets, parent.max_targets)
            && optional_rate_is_no_wider(self.rate_limit, parent.rate_limit)
    }

    fn is_strictly_no_wider_than(&self, parent: &Self) -> bool {
        self.is_no_wider_than(parent) && self != parent
    }

}

fn optional_max_is_no_wider<T: Copy + Ord>(child: Option<T>, parent: Option<T>) -> bool {
    match (child, parent) {
        (_, None) => true,
        (Some(child), Some(parent)) => child <= parent,
        (None, Some(_)) => false,
    }
}

fn optional_rate_is_no_wider(
    child: Option<RateLimitV1>,
    parent: Option<RateLimitV1>,
) -> bool {
    match (child, parent) {
        (_, None) => true,
        (None, Some(_)) => false,
        (Some(child), Some(parent)) => child.is_no_wider_than(parent),
    }
}

/// Version-1 typed action/resource/audience and effect scope.
///
/// All string collections are finite, exact, sorted and duplicate-free.
/// Resource identifiers are opaque exact identifiers; this version does
/// not infer hierarchy from prefixes or path syntax.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TypedDelegationScopeV1 {
    actions: Vec<String>,
    resources: Vec<String>,
    audiences: Vec<String>,
    directions: Vec<DataDirectionV1>,
    consent: ConsentBindingV1,
    operation: OperationBindingV1,
    budgets: Option<BudgetLimitsV1>,
}

impl TypedDelegationScopeV1 {
    /// Construct and canonically normalize a typed scope.
    ///
    /// Identifiers are ASCII, non-empty, whitespace-free and at most 256
    /// bytes. Collections are limited to 1,024 entries. Duplicate identifiers
    /// are rejected; order is normalized before hashing.
    pub fn new(
        actions: Vec<String>,
        resources: Vec<String>,
        audiences: Vec<String>,
        directions: Vec<DataDirectionV1>,
        consent: ConsentBindingV1,
        operation: OperationBindingV1,
        budgets: Option<BudgetLimitsV1>,
    ) -> Result<Self, TypedScopeError> {
        if let OperationBindingV1::ExactDigest(digest) = operation
            && digest == [0; 32]
        {
            return Err(TypedScopeError::ZeroDigest);
        }

        let actions = canonicalize_identifiers(actions, "actions")?;
        let resources = canonicalize_identifiers(resources, "resources")?;
        let audiences = canonicalize_identifiers(audiences, "audiences")?;
        let directions = canonicalize_directions(directions)?;
        let budgets = budgets.filter(|budget| !budget.is_unbounded());

        Ok(Self {
            actions,
            resources,
            audiences,
            directions,
            consent,
            operation,
            budgets,
        })
    }

    /// Schema version encoded into canonical bytes.
    pub const fn schema_version(&self) -> u16 {
        SCHEMA_VERSION
    }

    /// Exact action identifiers.
    pub fn actions(&self) -> &[String] {
        &self.actions
    }

    /// Exact resource identifiers.
    pub fn resources(&self) -> &[String] {
        &self.resources
    }

    /// Exact relying-party/audience identifiers.
    pub fn audiences(&self) -> &[String] {
        &self.audiences
    }

    /// Permitted data/control directions.
    pub fn directions(&self) -> &[DataDirectionV1] {
        &self.directions
    }

    /// ROS-004 consent/purpose/data-class commitment.
    pub const fn consent_binding(&self) -> ConsentBindingV1 {
        self.consent
    }

    /// Exact operation binding.
    pub const fn operation_binding(&self) -> OperationBindingV1 {
        self.operation
    }

    /// Static upper bounds; absence means unbounded.
    pub const fn budgets(&self) -> Option<BudgetLimitsV1> {
        self.budgets
    }

    /// Whether this scope is contained by parent in every modeled dimension.
    ///
    /// A true result is a structural comparison only. It does not establish
    /// authenticity, active status, currentness, consent, or effect permission.
    pub fn is_no_wider_than(&self, parent: &Self) -> bool {
        is_string_subset(&self.actions, &parent.actions)
            && is_string_subset(&self.resources, &parent.resources)
            && is_string_subset(&self.audiences, &parent.audiences)
            && is_direction_subset(&self.directions, &parent.directions)
            && self.consent == parent.consent
            && self.operation.is_no_wider_than(parent.operation)
            && optional_budget_is_no_wider(self.budgets, parent.budgets)
    }

    /// Whether this is a proper subset of parent in at least one dimension.
    ///
    /// Equal-scope children are not strict attenuation. Incomparable or wider
    /// scopes are rejected.
    pub fn is_strictly_attenuated_by(&self, parent: &Self) -> bool {
        self.is_no_wider_than(parent)
            && (self.actions != parent.actions
                || self.resources != parent.resources
                || self.audiences != parent.audiences
                || self.directions != parent.directions
                || self.operation != parent.operation
                || optional_budget_is_strictly_no_wider(self.budgets, parent.budgets))
    }

    /// Versioned canonical encoding suitable for content-addressed identity.
    ///
    /// Collections are normalized by the constructor. Lengths and numeric
    /// values use unsigned big-endian encoding. The domain and schema version
    /// prevent cross-profile reinterpretation.
    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(SCOPE_DOMAIN);
        out.extend_from_slice(&SCHEMA_VERSION.to_be_bytes());
        append_identifiers(&mut out, &self.actions);
        append_identifiers(&mut out, &self.resources);
        append_identifiers(&mut out, &self.audiences);

        out.extend_from_slice(&(self.directions.len() as u32).to_be_bytes());
        for direction in &self.directions {
            out.push(direction.tag());
        }

        out.extend_from_slice(self.consent.consent_profile_digest());
        out.extend_from_slice(self.consent.purpose_digest());
        out.extend_from_slice(self.consent.data_class_digest());

        out.push(self.operation.tag());
        if let OperationBindingV1::ExactDigest(digest) = self.operation {
            out.extend_from_slice(&digest);
        }

        append_optional_budget(&mut out, self.budgets);
        out
    }

    /// BLAKE3 digest of the versioned canonical encoding.
    pub fn digest(&self) -> [u8; 32] {
        *blake3::hash(&self.canonical_bytes()).as_bytes()
    }
}

fn optional_budget_is_no_wider(
    child: Option<BudgetLimitsV1>,
    parent: Option<BudgetLimitsV1>,
) -> bool {
    match (child, parent) {
        (_, None) => true,
        (Some(child), Some(parent)) => child.is_no_wider_than(&parent),
        (None, Some(_)) => false,
    }
}

fn optional_budget_is_strictly_no_wider(
    child: Option<BudgetLimitsV1>,
    parent: Option<BudgetLimitsV1>,
) -> bool {
    match (child, parent) {
        (Some(_), None) => true,
        (Some(child), Some(parent)) => child.is_strictly_no_wider_than(&parent),
        _ => false,
    }
}

fn is_string_subset(child: &[String], parent: &[String]) -> bool {
    child.iter().all(|value| parent.binary_search(value).is_ok())
}

fn is_direction_subset(child: &[DataDirectionV1], parent: &[DataDirectionV1]) -> bool {
    child
        .iter()
        .all(|value| parent.binary_search(value).is_ok())
}

fn canonicalize_identifiers(
    mut values: Vec<String>,
    field: &'static str,
) -> Result<Vec<String>, TypedScopeError> {
    if values.is_empty() {
        return Err(TypedScopeError::EmptyDimension(field));
    }
    if values.len() > MAX_SET_ITEMS {
        return Err(TypedScopeError::TooManyItems(field));
    }

    for value in &values {
        if value.is_empty()
            || value.len() > MAX_IDENTIFIER_BYTES
            || !value.is_ascii()
            || value
                .bytes()
                .any(|byte| byte.is_ascii_whitespace() || byte.is_ascii_control())
        {
            return Err(TypedScopeError::InvalidIdentifier(field));
        }
    }

    values.sort_unstable();
    for pair in values.windows(2) {
        if pair[0] == pair[1] {
            return Err(TypedScopeError::DuplicateIdentifier(field));
        }
    }
    Ok(values)
}

fn canonicalize_directions(
    mut values: Vec<DataDirectionV1>,
) -> Result<Vec<DataDirectionV1>, TypedScopeError> {
    if values.is_empty() {
        return Err(TypedScopeError::EmptyDimension("directions"));
    }
    if values.len() > 3 {
        return Err(TypedScopeError::TooManyItems("directions"));
    }

    values.sort_unstable();
    for pair in values.windows(2) {
        if pair[0] == pair[1] {
            return Err(TypedScopeError::DuplicateIdentifier("directions"));
        }
    }
    Ok(values)
}

fn append_identifiers(out: &mut Vec<u8>, values: &[String]) {
    out.extend_from_slice(&(values.len() as u32).to_be_bytes());
    for value in values {
        out.extend_from_slice(&(value.len() as u32).to_be_bytes());
        out.extend_from_slice(value.as_bytes());
    }
}

fn append_optional_u64(out: &mut Vec<u8>, value: Option<u64>) {
    match value {
        Some(value) => {
            out.push(1);
            out.extend_from_slice(&value.to_be_bytes());
        }
        None => out.push(0),
    }
}

fn append_optional_u32(out: &mut Vec<u8>, value: Option<u32>) {
    match value {
        Some(value) => {
            out.push(1);
            out.extend_from_slice(&value.to_be_bytes());
        }
        None => out.push(0),
    }
}

fn append_optional_budget(out: &mut Vec<u8>, budget: Option<BudgetLimitsV1>) {
    let Some(budget) = budget else {
        out.push(0);
        return;
    };
    out.push(1);
    append_optional_u64(out, budget.max_duration_ms);
    append_optional_u64(out, budget.max_operations);
    append_optional_u64(out, budget.max_bytes);
    append_optional_u32(out, budget.max_concurrent);
    append_optional_u32(out, budget.max_targets);
    match budget.rate_limit {
        Some(rate) => {
            out.push(1);
            out.extend_from_slice(&rate.max_events.to_be_bytes());
            out.extend_from_slice(&rate.window_ms.to_be_bytes());
        }
        None => out.push(0),
    }
}

/// Errors constructing a typed-scope profile.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TypedScopeError {
    /// A required action/resource/audience/direction set is empty.
    EmptyDimension(&'static str),
    /// An identifier is empty, non-ASCII, contains whitespace/control bytes,
    /// or exceeds the profile's maximum length.
    InvalidIdentifier(&'static str),
    /// An exact identifier occurs more than once in a set.
    DuplicateIdentifier(&'static str),
    /// A collection exceeds the profile's fixed bound.
    TooManyItems(&'static str),
    /// A required cryptographic commitment is all zeroes.
    ZeroDigest,
    /// A rolling-window duration must be positive.
    ZeroRateWindow,
}

impl fmt::Display for TypedScopeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyDimension(field) => write!(f, "{field} must not be empty"),
            Self::InvalidIdentifier(field) => {
                write!(f, "{field} contains an invalid identifier")
            }
            Self::DuplicateIdentifier(field) => {
                write!(f, "{field} contains a duplicate identifier")
            }
            Self::TooManyItems(field) => write!(f, "{field} exceeds the profile limit"),
            Self::ZeroDigest => f.write_str("required digest must not be zero"),
            Self::ZeroRateWindow => f.write_str("rate-limit window must be positive"),
        }
    }
}

impl std::error::Error for TypedScopeError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn consent() -> ConsentBindingV1 {
        ConsentBindingV1::new([1; 32], [2; 32], [3; 32]).unwrap()
    }

    fn scope(
        actions: Vec<&str>,
        resources: Vec<&str>,
        audiences: Vec<&str>,
        directions: Vec<DataDirectionV1>,
        operation: OperationBindingV1,
        budgets: Option<BudgetLimitsV1>,
    ) -> TypedDelegationScopeV1 {
        TypedDelegationScopeV1::new(
            actions.into_iter().map(str::to_owned).collect(),
            resources.into_iter().map(str::to_owned).collect(),
            audiences.into_iter().map(str::to_owned).collect(),
            directions,
            consent(),
            operation,
            budgets,
        )
        .unwrap()
    }

    #[test]
    fn canonical_encoding_is_order_independent_and_domain_versioned() {
        let first = scope(
            vec!["xenia.file.send", "xenia.screen.view"],
            vec!["host/b", "host/a"],
            vec!["viewer/b", "viewer/a"],
            vec![DataDirectionV1::OwnerToDelegate, DataDirectionV1::DelegateToOwner],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        let second = scope(
            vec!["xenia.screen.view", "xenia.file.send"],
            vec!["host/a", "host/b"],
            vec!["viewer/a", "viewer/b"],
            vec![DataDirectionV1::DelegateToOwner, DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        assert_eq!(first.canonical_bytes(), second.canonical_bytes());
        assert_eq!(first.digest(), second.digest());
        assert!(first.canonical_bytes().starts_with(SCOPE_DOMAIN));
        assert_eq!(first.schema_version(), 1);
    }

    #[test]
    fn canonical_encoding_matches_v1_known_answer_vector() {
        let value = scope(
            vec!["a"],
            vec!["r"],
            vec!["v"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        let actual = value
            .canonical_bytes()
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        let expected = concat!(
            "6d7963656c69782e726f733030352e74797065642d73636f70652e7631000001",
            "0000000100000001610000000100000001720000000100000001760000000100",
            "0101010101010101010101010101010101010101010101010101010101010101",
            "0202020202020202020202020202020202020202020202020202020202020202",
            "0303030303030303030303030303030303030303030303030303030303030303",
            "0000"
        );
        assert_eq!(actual, expected);
        assert_eq!(value.canonical_bytes().len(), 162);
    }

    #[test]
    fn exact_resource_ids_do_not_use_string_prefix_containment() {
        let parent = scope(
            vec!["xenia.file.send"],
            vec!["host/a"],
            vec!["viewer/one"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        let child = scope(
            vec!["xenia.file.send"],
            vec!["host/ab"],
            vec!["viewer/one"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        assert!(!child.is_no_wider_than(&parent));
    }

    #[test]
    fn child_must_be_a_subset_in_every_set_valued_dimension() {
        let parent = scope(
            vec!["a", "b"],
            vec!["host/a", "host/b"],
            vec!["viewer/a", "viewer/b"],
            vec![DataDirectionV1::OwnerToDelegate, DataDirectionV1::DelegateToOwner],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        let child = scope(
            vec!["a"],
            vec!["host/a"],
            vec!["viewer/a"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        assert!(child.is_no_wider_than(&parent));
        assert!(child.is_strictly_attenuated_by(&parent));
        assert!(!parent.is_no_wider_than(&child));
    }

    #[test]
    fn exhaustive_three_action_set_containment_matches_bitset_oracle() {
        fn make(mask: u8) -> TypedDelegationScopeV1 {
            let actions = ["a", "b", "c"]
                .into_iter()
                .enumerate()
                .filter(|(bit, _)| mask & (1 << bit) != 0)
                .map(|(_, value)| value)
                .collect();
            scope(
                actions,
                vec!["host/one"],
                vec!["viewer/one"],
                vec![DataDirectionV1::OwnerToDelegate],
                OperationBindingV1::AnyWithinProfile,
                None,
            )
        }

        for child_mask in 1u8..8 {
            for parent_mask in 1u8..8 {
                let child = make(child_mask);
                let parent = make(parent_mask);
                let expected = child_mask & !parent_mask == 0;
                assert_eq!(child.is_no_wider_than(&parent), expected);
                assert_eq!(
                    child.is_strictly_attenuated_by(&parent),
                    expected && child_mask != parent_mask
                );
            }
        }
    }

    #[test]
    fn exact_operation_binding_can_only_narrow_any_to_one_digest() {
        let parent = scope(
            vec!["xenia.file.send"],
            vec!["host/a"],
            vec!["viewer/one"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        let exact = scope(
            vec!["xenia.file.send"],
            vec!["host/a"],
            vec!["viewer/one"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::ExactDigest([9; 32]),
            None,
        );
        assert!(exact.is_strictly_attenuated_by(&parent));
        assert!(!parent.is_no_wider_than(&exact));
    }

    #[test]
    fn exact_operation_digest_cannot_change_across_parent_edge() {
        let parent = scope(
            vec!["xenia.file.send"],
            vec!["host/a"],
            vec!["viewer/one"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::ExactDigest([9; 32]),
            None,
        );
        let changed = scope(
            vec!["xenia.file.send"],
            vec!["host/a"],
            vec!["viewer/one"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::ExactDigest([8; 32]),
            None,
        );
        assert!(!changed.is_no_wider_than(&parent));
    }

    #[test]
    fn purpose_and_data_class_commitments_are_exact_constraints() {
        let parent = scope(
            vec!["xenia.screen.view"],
            vec!["host/a"],
            vec!["viewer/one"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        let child = TypedDelegationScopeV1::new(
            vec!["xenia.screen.view".into()],
            vec!["host/a".into()],
            vec!["viewer/one".into()],
            vec![DataDirectionV1::OwnerToDelegate],
            ConsentBindingV1::new([1; 32], [7; 32], [3; 32]).unwrap(),
            OperationBindingV1::AnyWithinProfile,
            None,
        )
        .unwrap();
        assert!(!child.is_no_wider_than(&parent));
    }

    #[test]
    fn budgets_can_only_be_added_or_reduced_not_removed_or_increased() {
        let parent = BudgetLimitsV1::new(
            Some(60_000),
            Some(100),
            Some(1_000_000),
            Some(4),
            Some(2),
            None,
        );
        let child = BudgetLimitsV1::new(
            Some(30_000),
            Some(50),
            Some(500_000),
            Some(2),
            Some(1),
            None,
        );
        assert!(child.is_strictly_no_wider_than(&parent));
        assert!(!parent.is_no_wider_than(&child));
    }

    #[test]
    fn all_unbounded_budget_normalizes_to_absent_and_is_not_attenuation() {
        let parent = scope(
            vec!["xenia.file.send"],
            vec!["host/a"],
            vec!["viewer/one"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        let empty_budget = BudgetLimitsV1::new(None, None, None, None, None, None);
        let child = scope(
            vec!["xenia.file.send"],
            vec!["host/a"],
            vec!["viewer/one"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            Some(empty_budget),
        );

        assert_eq!(child.budgets(), None);
        assert!(child.is_no_wider_than(&parent));
        assert!(!child.is_strictly_attenuated_by(&parent));
        assert_eq!(child.canonical_bytes(), parent.canonical_bytes());
    }

    #[test]
    fn child_may_add_a_limit_when_parent_is_unbounded() {
        let child = BudgetLimitsV1::new(Some(10_000), None, None, None, None, None);
        let parent = BudgetLimitsV1::new(None, None, None, None, None, None);
        assert!(child.is_strictly_no_wider_than(&parent));
        assert!(!parent.is_no_wider_than(&child));
    }

    #[test]
    fn rolling_window_containment_rejects_rate_amplification() {
        let parent = RateLimitV1::new(10, 60_000).unwrap();
        let safe_child = RateLimitV1::new(5, 30_000).unwrap();
        let unsafe_child = RateLimitV1::new(6, 30_000).unwrap();
        let slower_child = RateLimitV1::new(9, 120_000).unwrap();

        assert!(safe_child.is_no_wider_than(parent));
        assert!(!unsafe_child.is_no_wider_than(parent));
        assert!(slower_child.is_no_wider_than(parent));
    }

    #[test]
    fn accepted_rate_containment_preserves_all_small_enumerated_traces() {
        fn enumerate_traces(
            traces: &mut Vec<Vec<u64>>,
            current: &mut Vec<u64>,
            next_time: u64,
            remaining: usize,
        ) {
            traces.push(current.clone());
            if remaining == 0 {
                return;
            }
            for time in next_time..=5 {
                current.push(time);
                enumerate_traces(traces, current, time, remaining - 1);
                current.pop();
            }
        }

        fn allows(limit: RateLimitV1, trace: &[u64]) -> bool {
            trace.iter().all(|start| {
                let end = *start + limit.window_ms;
                let count = trace
                    .iter()
                    .filter(|time| **time >= *start && **time < end)
                    .count() as u64;
                count <= limit.max_events
            })
        }

        let mut traces = Vec::new();
        enumerate_traces(&mut traces, &mut Vec::new(), 0, 4);

        for child_window in 1..=4 {
            for child_events in 0..=3 {
                let child = RateLimitV1::new(child_events, child_window).unwrap();
                for parent_window in 1..=4 {
                    for parent_events in 0..=3 {
                        let parent =
                            RateLimitV1::new(parent_events, parent_window).unwrap();
                        if child.is_no_wider_than(parent) {
                            for trace in &traces {
                                if allows(child, trace) {
                                    assert!(
                                        allows(parent, trace),
                                        "accepted containment failed: child=({child_events}, {child_window}), parent=({parent_events}, {parent_window}), trace={trace:?}"
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn rate_limit_overflow_fails_closed() {
        let parent = RateLimitV1::new(u64::MAX, u64::MAX).unwrap();
        let child = RateLimitV1::new(u64::MAX, 1).unwrap();
        assert!(!child.is_no_wider_than(parent));
    }

    #[test]
    fn malformed_empty_duplicate_and_noncanonical_identifiers_are_rejected() {
        assert_eq!(
            TypedDelegationScopeV1::new(
                vec![],
                vec!["host/a".into()],
                vec!["viewer/one".into()],
                vec![DataDirectionV1::OwnerToDelegate],
                consent(),
                OperationBindingV1::AnyWithinProfile,
                None,
            )
            .unwrap_err(),
            TypedScopeError::EmptyDimension("actions")
        );
        assert_eq!(
            TypedDelegationScopeV1::new(
                vec!["a".into(), "a".into()],
                vec!["host/a".into()],
                vec!["viewer/one".into()],
                vec![DataDirectionV1::OwnerToDelegate],
                consent(),
                OperationBindingV1::AnyWithinProfile,
                None,
            )
            .unwrap_err(),
            TypedScopeError::DuplicateIdentifier("actions")
        );
        assert_eq!(
            TypedDelegationScopeV1::new(
                vec!["a b".into()],
                vec!["host/a".into()],
                vec!["viewer/one".into()],
                vec![DataDirectionV1::OwnerToDelegate],
                consent(),
                OperationBindingV1::AnyWithinProfile,
                None,
            )
            .unwrap_err(),
            TypedScopeError::InvalidIdentifier("actions")
        );
    }

    #[test]
    fn zero_required_commitment_or_rate_window_is_rejected() {
        assert_eq!(
            ConsentBindingV1::new([0; 32], [2; 32], [3; 32]),
            Err(TypedScopeError::ZeroDigest)
        );
        assert_eq!(RateLimitV1::new(1, 0), Err(TypedScopeError::ZeroRateWindow));
        assert_eq!(
            TypedDelegationScopeV1::new(
                vec!["a".into()],
                vec!["host/a".into()],
                vec!["viewer/one".into()],
                vec![DataDirectionV1::OwnerToDelegate],
                consent(),
                OperationBindingV1::ExactDigest([0; 32]),
                None,
            )
            .unwrap_err(),
            TypedScopeError::ZeroDigest
        );
    }

    #[test]
    fn field_mutation_changes_the_scope_digest() {
        let original = scope(
            vec!["xenia.file.send"],
            vec!["host/a"],
            vec!["viewer/one"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        let changed = scope(
            vec!["xenia.file.send"],
            vec!["host/a"],
            vec!["viewer/two"],
            vec![DataDirectionV1::OwnerToDelegate],
            OperationBindingV1::AnyWithinProfile,
            None,
        );
        assert_ne!(original.digest(), changed.digest());
    }
}
