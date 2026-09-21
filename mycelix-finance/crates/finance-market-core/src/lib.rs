#![forbid(unsafe_code)]

//! Provider-neutral market-order intent semantics for Mycelix Finance.
//!
//! This crate defines only immutable candidate order identity. It does not
//! authenticate account ownership, establish financial authority or buying
//! power, call a broker, observe fills, or establish settlement.

use mycelix_finance_exact::{AssetAmount, AssetId};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use sha2::{Digest as _, Sha256};
use std::fmt;

pub const MAX_PROTOCOL_ID_BYTES: usize = 128;
pub const MAX_EXTERNAL_ID_BYTES: usize = 256;
pub const COMMITMENT_PROFILE_REVISION_V1: u32 = 1;
const ORDER_INTENT_DOMAIN_V1: &[u8] = b"MYCELIX_FIN_MKT_ORDER_INTENT_V1\0";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MarketIntentError {
    InvalidProtocolIdentifier,
    InvalidExternalIdentifier,
    InvalidDigest,
    ZeroQuantity,
    ZeroPrice,
    IncompatibleStopLimitPrices,
}

impl fmt::Display for MarketIntentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidProtocolIdentifier => {
                "protocol identifier is empty, too long, non-ASCII, or contains an unsupported byte"
            }
            Self::InvalidExternalIdentifier => {
                "external identifier is empty, too long, or contains a control character"
            }
            Self::InvalidDigest => "digest must be exactly 64 lowercase hexadecimal characters",
            Self::ZeroQuantity => "market-order quantity/notional must be non-zero",
            Self::ZeroPrice => "limit/stop price must be non-zero",
            Self::IncompatibleStopLimitPrices => {
                "stop-limit prices must use the same pricing profile and quote asset"
            }
        };
        f.write_str(message)
    }
}

impl std::error::Error for MarketIntentError {}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct MarketProtocolIdV1(String);

impl MarketProtocolIdV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, MarketIntentError> {
        let value = value.into();
        if value.is_empty()
            || value.len() > MAX_PROTOCOL_ID_BYTES
            || !value.bytes().all(is_protocol_id_byte)
        {
            return Err(MarketIntentError::InvalidProtocolIdentifier);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for MarketProtocolIdV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for MarketProtocolIdV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        Self::new(String::deserialize(deserializer)?).map_err(serde::de::Error::custom)
    }
}

const fn is_protocol_id_byte(value: u8) -> bool {
    value.is_ascii_alphanumeric() || matches!(value, b'.' | b'_' | b':' | b'/' | b'-')
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct MarketExternalIdV1(String);

impl MarketExternalIdV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, MarketIntentError> {
        let value = value.into();
        if value.is_empty()
            || value.len() > MAX_EXTERNAL_ID_BYTES
            || value.chars().any(char::is_control)
        {
            return Err(MarketIntentError::InvalidExternalIdentifier);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for MarketExternalIdV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for MarketExternalIdV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        Self::new(String::deserialize(deserializer)?).map_err(serde::de::Error::custom)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Digest32([u8; 32]);

impl Digest32 {
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    pub fn to_hex(self) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut output = String::with_capacity(64);
        for byte in self.0 {
            output.push(HEX[(byte >> 4) as usize] as char);
            output.push(HEX[(byte & 0x0f) as usize] as char);
        }
        output
    }

    pub fn from_lower_hex(value: &str) -> Result<Self, MarketIntentError> {
        if value.len() != 64 {
            return Err(MarketIntentError::InvalidDigest);
        }
        let bytes = value.as_bytes();
        let mut output = [0_u8; 32];
        for (index, pair) in bytes.chunks_exact(2).enumerate() {
            let high = decode_lower_hex_nibble(pair[0]).ok_or(MarketIntentError::InvalidDigest)?;
            let low = decode_lower_hex_nibble(pair[1]).ok_or(MarketIntentError::InvalidDigest)?;
            output[index] = (high << 4) | low;
        }
        Ok(Self(output))
    }
}

impl fmt::Display for Digest32 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

impl Serialize for Digest32 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(&self.to_hex())
    }
}

impl<'de> Deserialize<'de> for Digest32 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        Self::from_lower_hex(&String::deserialize(deserializer)?)
            .map_err(serde::de::Error::custom)
    }
}

fn decode_lower_hex_nibble(value: u8) -> Option<u8> {
    match value {
        b'0'..=b'9' => Some(value - b'0'),
        b'a'..=b'f' => Some(value - b'a' + 10),
        _ => None,
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MarketProfileRefV1 {
    pub profile_id: MarketProtocolIdV1,
    pub revision: u32,
    pub digest: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MarketSubjectRefV1 {
    pub subject_profile: MarketProfileRefV1,
    pub subject_id: MarketExternalIdV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MarketInstrumentRefV1 {
    pub instrument_profile: MarketProfileRefV1,
    pub instrument_id: MarketExternalIdV1,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MarketIdempotencyRefV1 {
    pub idempotency_profile: MarketProfileRefV1,
    pub semantic_id: MarketProtocolIdV1,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MarketAssetAmountWireV1 {
    atomic_units: u64,
    asset: AssetId,
}

fn deserialize_asset_amount_closed<'de, D>(deserializer: D) -> Result<AssetAmount, D::Error>
where
    D: Deserializer<'de>,
{
    let wire = MarketAssetAmountWireV1::deserialize(deserializer)?;
    Ok(AssetAmount::new(wire.atomic_units, wire.asset))
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MarketQuantityV1 {
    pub unit_profile: MarketProfileRefV1,
    #[serde(deserialize_with = "deserialize_asset_amount_closed")]
    pub amount: AssetAmount,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MarketPriceV1 {
    pub pricing_profile: MarketProfileRefV1,
    #[serde(deserialize_with = "deserialize_asset_amount_closed")]
    pub quote_amount: AssetAmount,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MarketSideV1 {
    AcquireLong,
    ReduceLong,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum QuantitySpecV1 {
    Units(MarketQuantityV1),
    Notional(MarketQuantityV1),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum OrderTermsV1 {
    Market,
    Limit { price: MarketPriceV1 },
    Stop { price: MarketPriceV1 },
    StopLimit {
        stop_price: MarketPriceV1,
        limit_price: MarketPriceV1,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TimeInForceV1 {
    Day,
    GoodTilCanceled,
    ImmediateOrCancel,
    FillOrKill,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MarketOrderIntentInputV1 {
    pub intent_subject: MarketSubjectRefV1,
    pub account_subject: MarketSubjectRefV1,
    pub instrument: MarketInstrumentRefV1,
    pub side: MarketSideV1,
    pub quantity: QuantitySpecV1,
    pub order_terms: OrderTermsV1,
    pub time_in_force: TimeInForceV1,
    pub execution_profile: MarketProfileRefV1,
    pub semantic_idempotency: MarketIdempotencyRefV1,
    pub upstream_economic_effect_commitment: Option<Digest32>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CanonicalMarketOrderIntentV1 {
    input: MarketOrderIntentInputV1,
    intent_commitment: Digest32,
}

impl CanonicalMarketOrderIntentV1 {
    pub fn input(&self) -> &MarketOrderIntentInputV1 {
        &self.input
    }

    pub const fn commitment(&self) -> Digest32 {
        self.intent_commitment
    }

    pub fn canonical_bytes(&self) -> Vec<u8> {
        canonical_order_intent_bytes_v1_unchecked(&self.input)
    }
}

pub fn canonicalize_order_intent_v1(
    input: MarketOrderIntentInputV1,
) -> Result<CanonicalMarketOrderIntentV1, MarketIntentError> {
    validate_quantity(&input.quantity)?;
    validate_order_terms(&input.order_terms)?;
    let bytes = canonical_order_intent_bytes_v1_unchecked(&input);
    Ok(CanonicalMarketOrderIntentV1 {
        input,
        intent_commitment: sha256_digest(&bytes),
    })
}

fn validate_quantity(quantity: &QuantitySpecV1) -> Result<(), MarketIntentError> {
    let amount = match quantity {
        QuantitySpecV1::Units(value) | QuantitySpecV1::Notional(value) => &value.amount,
    };
    if amount.atomic_units() == 0 {
        return Err(MarketIntentError::ZeroQuantity);
    }
    Ok(())
}

fn validate_order_terms(terms: &OrderTermsV1) -> Result<(), MarketIntentError> {
    match terms {
        OrderTermsV1::Market => Ok(()),
        OrderTermsV1::Limit { price } | OrderTermsV1::Stop { price } => validate_price(price),
        OrderTermsV1::StopLimit {
            stop_price,
            limit_price,
        } => {
            validate_price(stop_price)?;
            validate_price(limit_price)?;
            if stop_price.pricing_profile != limit_price.pricing_profile
                || stop_price.quote_amount.asset() != limit_price.quote_amount.asset()
            {
                return Err(MarketIntentError::IncompatibleStopLimitPrices);
            }
            Ok(())
        }
    }
}

fn validate_price(price: &MarketPriceV1) -> Result<(), MarketIntentError> {
    if price.quote_amount.atomic_units() == 0 {
        return Err(MarketIntentError::ZeroPrice);
    }
    Ok(())
}

fn canonical_order_intent_bytes_v1_unchecked(input: &MarketOrderIntentInputV1) -> Vec<u8> {
    let mut output = Vec::with_capacity(768);
    output.extend_from_slice(ORDER_INTENT_DOMAIN_V1);
    output.extend_from_slice(&COMMITMENT_PROFILE_REVISION_V1.to_be_bytes());
    push_subject_ref(&mut output, &input.intent_subject);
    push_subject_ref(&mut output, &input.account_subject);
    push_instrument_ref(&mut output, &input.instrument);
    output.push(side_tag(input.side));
    push_quantity(&mut output, &input.quantity);
    push_order_terms(&mut output, &input.order_terms);
    output.push(time_in_force_tag(input.time_in_force));
    push_profile_ref(&mut output, &input.execution_profile);
    push_idempotency_ref(&mut output, &input.semantic_idempotency);
    match input.upstream_economic_effect_commitment {
        None => output.push(0),
        Some(commitment) => {
            output.push(1);
            output.extend_from_slice(commitment.as_bytes());
        }
    }
    output
}

fn push_profile_ref(output: &mut Vec<u8>, value: &MarketProfileRefV1) {
    push_text(output, value.profile_id.as_str());
    output.extend_from_slice(&value.revision.to_be_bytes());
    output.extend_from_slice(value.digest.as_bytes());
}

fn push_subject_ref(output: &mut Vec<u8>, value: &MarketSubjectRefV1) {
    push_profile_ref(output, &value.subject_profile);
    push_text(output, value.subject_id.as_str());
}

fn push_instrument_ref(output: &mut Vec<u8>, value: &MarketInstrumentRefV1) {
    push_profile_ref(output, &value.instrument_profile);
    push_text(output, value.instrument_id.as_str());
}

fn push_idempotency_ref(output: &mut Vec<u8>, value: &MarketIdempotencyRefV1) {
    push_profile_ref(output, &value.idempotency_profile);
    push_text(output, value.semantic_id.as_str());
}

fn push_quantity(output: &mut Vec<u8>, value: &QuantitySpecV1) {
    match value {
        QuantitySpecV1::Units(quantity) => {
            output.push(0);
            push_market_quantity(output, quantity);
        }
        QuantitySpecV1::Notional(quantity) => {
            output.push(1);
            push_market_quantity(output, quantity);
        }
    }
}

fn push_market_quantity(output: &mut Vec<u8>, value: &MarketQuantityV1) {
    push_profile_ref(output, &value.unit_profile);
    push_asset_amount(output, &value.amount);
}

fn push_order_terms(output: &mut Vec<u8>, value: &OrderTermsV1) {
    match value {
        OrderTermsV1::Market => output.push(0),
        OrderTermsV1::Limit { price } => {
            output.push(1);
            push_market_price(output, price);
        }
        OrderTermsV1::Stop { price } => {
            output.push(2);
            push_market_price(output, price);
        }
        OrderTermsV1::StopLimit {
            stop_price,
            limit_price,
        } => {
            output.push(3);
            push_market_price(output, stop_price);
            push_market_price(output, limit_price);
        }
    }
}

fn push_market_price(output: &mut Vec<u8>, value: &MarketPriceV1) {
    push_profile_ref(output, &value.pricing_profile);
    push_asset_amount(output, &value.quote_amount);
}

fn push_asset_amount(output: &mut Vec<u8>, value: &AssetAmount) {
    output.extend_from_slice(&value.atomic_units().to_be_bytes());
    push_text(output, value.asset().as_str());
}

fn push_text(output: &mut Vec<u8>, value: &str) {
    let len = u32::try_from(value.len()).expect("bounded FIN-MKT identifiers fit u32");
    output.extend_from_slice(&len.to_be_bytes());
    output.extend_from_slice(value.as_bytes());
}

const fn side_tag(value: MarketSideV1) -> u8 {
    match value {
        MarketSideV1::AcquireLong => 0,
        MarketSideV1::ReduceLong => 1,
    }
}

const fn time_in_force_tag(value: TimeInForceV1) -> u8 {
    match value {
        TimeInForceV1::Day => 0,
        TimeInForceV1::GoodTilCanceled => 1,
        TimeInForceV1::ImmediateOrCancel => 2,
        TimeInForceV1::FillOrKill => 3,
    }
}

fn sha256_digest(bytes: &[u8]) -> Digest32 {
    let hash = Sha256::digest(bytes);
    let mut output = [0_u8; 32];
    output.copy_from_slice(&hash);
    Digest32::from_bytes(output)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXPECTED_VECTOR: &str =
        "37d7854bb7012da68bf12cbd76f9a8ca46d3e8364140a6d275168da2f0721f71";

    fn profile(id: &str, byte: u8) -> MarketProfileRefV1 {
        MarketProfileRefV1 {
            profile_id: MarketProtocolIdV1::new(id).unwrap(),
            revision: 1,
            digest: Digest32::from_bytes([byte; 32]),
        }
    }

    fn fixture() -> MarketOrderIntentInputV1 {
        MarketOrderIntentInputV1 {
            intent_subject: MarketSubjectRefV1 {
                subject_profile: profile("mycelix.market.intent", 0x10),
                subject_id: MarketExternalIdV1::new("intent:demo:001").unwrap(),
            },
            account_subject: MarketSubjectRefV1 {
                subject_profile: profile("mycelix.finance.account", 0x11),
                subject_id: MarketExternalIdV1::new("account:demo:001").unwrap(),
            },
            instrument: MarketInstrumentRefV1 {
                instrument_profile: profile("instrument.listed-equity.us", 0x22),
                instrument_id: MarketExternalIdV1::new("instrument:demo:AAPL").unwrap(),
            },
            side: MarketSideV1::AcquireLong,
            quantity: QuantitySpecV1::Units(MarketQuantityV1 {
                unit_profile: profile("quantity.micro-share-demo", 0x33),
                amount: AssetAmount::new(
                    10_000_000,
                    AssetId::new("AAPL.share.micro.demo").unwrap(),
                ),
            }),
            order_terms: OrderTermsV1::Limit {
                price: MarketPriceV1 {
                    pricing_profile: profile("price.usd-cents-per-share-demo", 0x44),
                    quote_amount: AssetAmount::new(
                        18_750,
                        AssetId::new("USD.cent.demo").unwrap(),
                    ),
                },
            },
            time_in_force: TimeInForceV1::Day,
            execution_profile: profile("execution.market-order-demo", 0x55),
            semantic_idempotency: MarketIdempotencyRefV1 {
                idempotency_profile: profile("idempotency.market-order-v1", 0x66),
                semantic_id: MarketProtocolIdV1::new("idem:demo:001").unwrap(),
            },
            upstream_economic_effect_commitment: Some(Digest32::from_bytes([0x77; 32])),
        }
    }

    fn fixture_json() -> serde_json::Value {
        serde_json::from_str(include_str!("../test-vectors/order-intent-v1.json")).unwrap()
    }

    #[test]
    fn frozen_vector_matches_independent_oracle() {
        let canonical = canonicalize_order_intent_v1(fixture()).unwrap();
        assert_eq!(canonical.canonical_bytes().len(), 671);
        assert_eq!(canonical.commitment().to_hex(), EXPECTED_VECTOR);
    }

    #[test]
    fn json_fixture_reconstructs_frozen_vector() {
        let input: MarketOrderIntentInputV1 = serde_json::from_value(fixture_json()).unwrap();
        let canonical = canonicalize_order_intent_v1(input).unwrap();
        assert_eq!(canonical.canonical_bytes().len(), 671);
        assert_eq!(canonical.commitment().to_hex(), EXPECTED_VECTOR);
    }

    #[test]
    fn protocol_and_external_identifier_roles_are_distinct() {
        assert!(MarketProtocolIdV1::new("profile.market-v1").is_ok());
        assert!(MarketProtocolIdV1::new("账户").is_err());
        assert!(MarketProtocolIdV1::new("profile with space").is_err());
        assert!(MarketExternalIdV1::new("账户:甲").is_ok());
        assert!(MarketExternalIdV1::new("bad\nsubject").is_err());
    }

    #[test]
    fn effect_significant_mutations_change_identity() {
        let base = canonicalize_order_intent_v1(fixture()).unwrap();

        let mut changed = fixture();
        changed.side = MarketSideV1::ReduceLong;
        assert_ne!(
            base.commitment(),
            canonicalize_order_intent_v1(changed).unwrap().commitment()
        );

        let mut changed = fixture();
        changed.time_in_force = TimeInForceV1::GoodTilCanceled;
        assert_ne!(
            base.commitment(),
            canonicalize_order_intent_v1(changed).unwrap().commitment()
        );

        let mut changed = fixture();
        changed.semantic_idempotency.semantic_id =
            MarketProtocolIdV1::new("idem:demo:002").unwrap();
        assert_ne!(
            base.commitment(),
            canonicalize_order_intent_v1(changed).unwrap().commitment()
        );
    }

    #[test]
    fn zero_quantity_and_zero_prices_fail_closed() {
        let mut zero_quantity = fixture();
        zero_quantity.quantity = QuantitySpecV1::Units(MarketQuantityV1 {
            unit_profile: profile("quantity.micro-share-demo", 0x33),
            amount: AssetAmount::new(0, AssetId::new("AAPL.share.micro.demo").unwrap()),
        });
        assert_eq!(
            canonicalize_order_intent_v1(zero_quantity),
            Err(MarketIntentError::ZeroQuantity)
        );

        let mut zero_price = fixture();
        zero_price.order_terms = OrderTermsV1::Limit {
            price: MarketPriceV1 {
                pricing_profile: profile("price.usd-cents-per-share-demo", 0x44),
                quote_amount: AssetAmount::new(0, AssetId::new("USD.cent.demo").unwrap()),
            },
        };
        assert_eq!(
            canonicalize_order_intent_v1(zero_price),
            Err(MarketIntentError::ZeroPrice)
        );
    }

    #[test]
    fn stop_limit_requires_comparable_price_semantics() {
        let mut changed = fixture();
        changed.order_terms = OrderTermsV1::StopLimit {
            stop_price: MarketPriceV1 {
                pricing_profile: profile("price.usd-cents-per-share-demo", 0x44),
                quote_amount: AssetAmount::new(
                    18_000,
                    AssetId::new("USD.cent.demo").unwrap(),
                ),
            },
            limit_price: MarketPriceV1 {
                pricing_profile: profile("price.eur-cents-per-share-demo", 0x45),
                quote_amount: AssetAmount::new(
                    17_900,
                    AssetId::new("EUR.cent.demo").unwrap(),
                ),
            },
        };
        assert_eq!(
            canonicalize_order_intent_v1(changed),
            Err(MarketIntentError::IncompatibleStopLimitPrices)
        );
    }

    #[test]
    fn digest_wire_rejects_noncanonical_uppercase() {
        assert_eq!(
            Digest32::from_lower_hex(&"AA".repeat(32)),
            Err(MarketIntentError::InvalidDigest)
        );
        assert!(Digest32::from_lower_hex(&"aa".repeat(32)).is_ok());
    }

    #[test]
    fn unknown_top_level_json_field_fails_closed() {
        let mut value = fixture_json();
        value["authorized"] = serde_json::Value::Bool(true);
        assert!(serde_json::from_value::<MarketOrderIntentInputV1>(value).is_err());
    }

    #[test]
    fn nested_order_variant_unknown_fields_fail_closed() {
        let mut limit = fixture_json();
        limit["order_terms"]["Limit"]["authorized"] = serde_json::Value::Bool(true);
        assert!(serde_json::from_value::<MarketOrderIntentInputV1>(limit).is_err());

        let mut stop_limit = fixture_json();
        let limit_body = stop_limit["order_terms"]["Limit"].clone();
        stop_limit["order_terms"] = serde_json::json!({
            "StopLimit": {
                "stop_price": limit_body["price"].clone(),
                "limit_price": limit_body["price"].clone(),
                "settled": true
            }
        });
        assert!(serde_json::from_value::<MarketOrderIntentInputV1>(stop_limit).is_err());
    }

    #[test]
    fn nested_asset_amount_unknown_fields_fail_closed() {
        let mut quantity = fixture_json();
        quantity["quantity"]["Units"]["amount"]["authorized"] =
            serde_json::Value::Bool(true);
        assert!(serde_json::from_value::<MarketOrderIntentInputV1>(quantity).is_err());

        let mut price = fixture_json();
        price["order_terms"]["Limit"]["price"]["quote_amount"]["settled"] =
            serde_json::Value::Bool(true);
        assert!(serde_json::from_value::<MarketOrderIntentInputV1>(price).is_err());
    }
}
