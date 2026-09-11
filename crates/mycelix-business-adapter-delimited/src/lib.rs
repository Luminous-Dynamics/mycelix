// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Strict, read-only CSV/TSV-style adapter for Mycelix Business ingress.
//!
//! The adapter converts a pinned delimited-file schema into evidence-bearing ingress records.
//! It contains no provider credentials, network client, mutation command, or execution authority.

use std::collections::{BTreeMap, BTreeSet};
use std::io::Read;

use csv::{ReaderBuilder, StringRecord};
use mycelix_business_core::{Digest32, ObservationRef, ReferenceId, ScopeRef};
use mycelix_business_ingress::{
    ExternalAdapterDescriptor, ExternalEventEnvelope, IngressBatch, IngressBatchError, IngressRecord,
    NormalizedInput,
};
use mycelix_business_shadow::{MetricObservation, ScaledValue, SourceWitness, WitnessRegistry};
use sha2::{Digest, Sha256};
use time::{OffsetDateTime, format_description::well_known::Rfc3339};

pub const DELIMITED_ADAPTER_IS_READ_ONLY: bool = true;
pub const MAX_EXPECTED_HEADERS: usize = 256;
pub const MAX_OUTPUT_MAPPINGS: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimestampEncoding {
    UnixMilliseconds,
    UnixSeconds,
    Rfc3339,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecimalPolicy {
    pub decimal_separator: char,
    pub grouping_separator: Option<char>,
    pub allow_negative: bool,
}

impl Default for DecimalPolicy {
    fn default() -> Self {
        Self {
            decimal_separator: '.',
            grouping_separator: None,
            allow_negative: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ScopeMapping {
    Constant(ScopeRef),
    Column {
        column: String,
        /// Prefix used to create a canonical opaque scope reference, e.g. `scope:location:`.
        prefix: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ValueMapping {
    Column {
        column: String,
        decimal: DecimalPolicy,
    },
    Constant {
        mantissa: i128,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutputMapping {
    pub input: ReferenceId,
    pub metric: ReferenceId,
    pub value: ValueMapping,
    pub unit: ReferenceId,
    pub scale: u32,
}

/// Complete deterministic mapping for one export shape.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DelimitedAdapterConfig {
    pub adapter_semantic_id: ReferenceId,
    pub source_system: ReferenceId,
    /// Release/build digest supplied by the adapter owner. It is not inferred from source code.
    pub adapter_digest: Digest32,
    pub source_schema: ReferenceId,
    pub delimiter: u8,
    /// Exact ordered header contract. Added/removed/reordered columns are schema drift.
    pub expected_headers: Vec<String>,
    pub source_event_id_column: String,
    pub observed_at_column: String,
    pub timestamp_encoding: TimestampEncoding,
    pub scope: ScopeMapping,
    pub outputs: Vec<OutputMapping>,
    pub maximum_batch_records: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConfigError {
    ZeroAdapterDigest,
    InvalidDelimiter,
    NoHeaders,
    TooManyHeaders,
    DuplicateHeader { header: String },
    EmptyHeader,
    MissingColumn { column: String },
    NoOutputs,
    TooManyOutputs,
    DuplicateOutputIdentity { input: ReferenceId, metric: ReferenceId },
    InvalidScale { metric: ReferenceId },
    InvalidDecimalPolicy { metric: ReferenceId },
    InvalidScopePrefix,
    ZeroMaximumBatchRecords,
}

#[derive(Debug)]
pub enum AdapterError {
    Config(ConfigError),
    Csv(csv::Error),
    HeaderMismatch,
    EmptyField { row: usize, column: String },
    InvalidReference { row: usize, column: String },
    InvalidTimestamp { row: usize, column: String },
    TimestampOutOfRange { row: usize, column: String },
    InvalidNumber { row: usize, column: String },
    InvalidGrouping { row: usize, column: String },
    PrecisionLoss { row: usize, column: String },
    NegativeNotAllowed { row: usize, column: String },
    ArithmeticOverflow { row: usize, column: String },
    InvalidScope { row: usize, column: String },
    TooManyRecords { actual: usize, maximum: u32 },
    EmptyFile,
    Ingress(IngressBatchError),
}

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

fn hash_len_prefixed(hasher: &mut Sha256, bytes: &[u8]) {
    hasher.update((bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hash_len_prefixed(hasher, value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    let bytes: [u8; 32] = hasher.finalize().into();
    Digest32(bytes)
}

fn hex_digest(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(64);
    for byte in digest.0 {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn invalid_numeric_separator(value: char) -> bool {
    value.is_ascii_digit() || value.is_control() || matches!(value, '+' | '-')
}

impl DelimitedAdapterConfig {
    pub fn validate(&self) -> Result<(), ConfigError> {
        if zero_digest(&self.adapter_digest) {
            return Err(ConfigError::ZeroAdapterDigest);
        }
        if self.delimiter == 0
            || matches!(self.delimiter, b'\n' | b'\r' | b'"')
            || !self.delimiter.is_ascii()
        {
            return Err(ConfigError::InvalidDelimiter);
        }
        if self.expected_headers.is_empty() {
            return Err(ConfigError::NoHeaders);
        }
        if self.expected_headers.len() > MAX_EXPECTED_HEADERS {
            return Err(ConfigError::TooManyHeaders);
        }

        let mut headers = BTreeSet::new();
        for header in &self.expected_headers {
            if header.is_empty() {
                return Err(ConfigError::EmptyHeader);
            }
            if !headers.insert(header.clone()) {
                return Err(ConfigError::DuplicateHeader {
                    header: header.clone(),
                });
            }
        }

        for column in [&self.source_event_id_column, &self.observed_at_column] {
            if !headers.contains(column) {
                return Err(ConfigError::MissingColumn {
                    column: column.clone(),
                });
            }
        }
        if let ScopeMapping::Column { column, prefix } = &self.scope {
            if !headers.contains(column) {
                return Err(ConfigError::MissingColumn {
                    column: column.clone(),
                });
            }
            if prefix.is_empty() || prefix != prefix.trim() {
                return Err(ConfigError::InvalidScopePrefix);
            }
        }

        if self.outputs.is_empty() {
            return Err(ConfigError::NoOutputs);
        }
        if self.outputs.len() > MAX_OUTPUT_MAPPINGS {
            return Err(ConfigError::TooManyOutputs);
        }
        let mut output_ids = BTreeSet::new();
        for output in &self.outputs {
            if output.scale > ScaledValue::MAX_SCALE {
                return Err(ConfigError::InvalidScale {
                    metric: output.metric.clone(),
                });
            }
            if !output_ids.insert((output.input.clone(), output.metric.clone())) {
                return Err(ConfigError::DuplicateOutputIdentity {
                    input: output.input.clone(),
                    metric: output.metric.clone(),
                });
            }
            if let ValueMapping::Column { column, decimal } = &output.value {
                if !headers.contains(column) {
                    return Err(ConfigError::MissingColumn {
                        column: column.clone(),
                    });
                }
                if invalid_numeric_separator(decimal.decimal_separator)
                    || decimal
                        .grouping_separator
                        .is_some_and(invalid_numeric_separator)
                    || decimal.grouping_separator == Some(decimal.decimal_separator)
                {
                    return Err(ConfigError::InvalidDecimalPolicy {
                        metric: output.metric.clone(),
                    });
                }
            }
        }
        if self.maximum_batch_records == 0 {
            return Err(ConfigError::ZeroMaximumBatchRecords);
        }
        Ok(())
    }

    /// Digest of the exact file shape. Header order is intentionally significant.
    pub fn source_schema_digest(&self) -> Result<Digest32, ConfigError> {
        self.validate()?;
        let mut hasher = Sha256::new();
        hash_str(&mut hasher, "mycelix:delimited-source-schema:v1");
        hasher.update([self.delimiter]);
        for header in &self.expected_headers {
            hash_str(&mut hasher, header);
        }
        Ok(finish_digest(hasher))
    }

    /// Digest of deterministic normalization semantics, bound to the source schema digest.
    pub fn mapping_digest(&self) -> Result<Digest32, ConfigError> {
        self.validate()?;
        let mut hasher = Sha256::new();
        hash_str(&mut hasher, "mycelix:delimited-mapping:v1");
        hasher.update(self.source_schema_digest()?.0);
        hash_str(&mut hasher, self.source_event_id_column.as_str());
        hash_str(&mut hasher, self.observed_at_column.as_str());
        hasher.update([match self.timestamp_encoding {
            TimestampEncoding::UnixMilliseconds => 1,
            TimestampEncoding::UnixSeconds => 2,
            TimestampEncoding::Rfc3339 => 3,
        }]);
        match &self.scope {
            ScopeMapping::Constant(scope) => {
                hasher.update([1]);
                hash_str(&mut hasher, scope.0.as_str());
            }
            ScopeMapping::Column { column, prefix } => {
                hasher.update([2]);
                hash_str(&mut hasher, column);
                hash_str(&mut hasher, prefix);
            }
        }
        for output in &self.outputs {
            hash_str(&mut hasher, output.input.as_str());
            hash_str(&mut hasher, output.metric.as_str());
            hash_str(&mut hasher, output.unit.as_str());
            hasher.update(output.scale.to_be_bytes());
            match &output.value {
                ValueMapping::Column { column, decimal } => {
                    hasher.update([1]);
                    hash_str(&mut hasher, column);
                    hasher.update((decimal.decimal_separator as u32).to_be_bytes());
                    match decimal.grouping_separator {
                        Some(value) => {
                            hasher.update([1]);
                            hasher.update((value as u32).to_be_bytes());
                        }
                        None => hasher.update([0]),
                    }
                    hasher.update([u8::from(decimal.allow_negative)]);
                }
                ValueMapping::Constant { mantissa } => {
                    hasher.update([2]);
                    hasher.update(mantissa.to_be_bytes());
                }
            }
        }
        Ok(finish_digest(hasher))
    }

    pub fn descriptor(&self) -> Result<ExternalAdapterDescriptor, ConfigError> {
        self.validate()?;
        Ok(ExternalAdapterDescriptor {
            adapter_semantic_id: self.adapter_semantic_id.clone(),
            source_system: self.source_system.clone(),
            adapter_digest: self.adapter_digest,
            source_schema: self.source_schema.clone(),
            source_schema_digest: self.source_schema_digest()?,
            mapping_digest: self.mapping_digest()?,
            supported_inputs: self.outputs.iter().map(|value| value.input.clone()).collect(),
            maximum_batch_records: self.maximum_batch_records,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DelimitedIngressAdapter {
    pub config: DelimitedAdapterConfig,
}

impl DelimitedIngressAdapter {
    pub fn new(config: DelimitedAdapterConfig) -> Result<Self, ConfigError> {
        config.validate()?;
        Ok(Self { config })
    }

    pub fn descriptor(&self) -> Result<ExternalAdapterDescriptor, ConfigError> {
        self.config.descriptor()
    }

    pub fn parse_batch<R: Read>(
        &self,
        reader: R,
        ingested_at_unix_ms: u64,
    ) -> Result<IngressBatch, AdapterError> {
        self.config.validate().map_err(AdapterError::Config)?;
        let descriptor = self.descriptor().map_err(AdapterError::Config)?;
        let mut csv = ReaderBuilder::new()
            .delimiter(self.config.delimiter)
            .has_headers(true)
            .flexible(false)
            .trim(csv::Trim::None)
            .from_reader(reader);

        let actual_headers = csv.headers().map_err(AdapterError::Csv)?.clone();
        let expected = StringRecord::from(self.config.expected_headers.clone());
        if actual_headers != expected {
            return Err(AdapterError::HeaderMismatch);
        }
        let header_index = actual_headers
            .iter()
            .enumerate()
            .map(|(index, name)| (name.to_owned(), index))
            .collect::<BTreeMap<_, _>>();

        let mut records = Vec::new();
        for (offset, result) in csv.records().enumerate() {
            let row_number = offset + 2; // header is row 1
            let record = result.map_err(AdapterError::Csv)?;
            records.push(self.parse_record(
                row_number,
                &actual_headers,
                &header_index,
                &record,
                ingested_at_unix_ms,
                &descriptor,
            )?);
            if records.len() > self.config.maximum_batch_records as usize {
                return Err(AdapterError::TooManyRecords {
                    actual: records.len(),
                    maximum: self.config.maximum_batch_records,
                });
            }
        }
        if records.is_empty() {
            return Err(AdapterError::EmptyFile);
        }

        let batch_digest = digest_batch(&descriptor, &records);
        let batch = IngressBatch {
            adapter_digest: descriptor.adapter_digest,
            records,
            batch_digest,
        };
        // Structural validation is performed against a temporary registry. The caller retains the
        // real registry and decides when the batch is admitted into durable witness state.
        batch
            .validate_and_register(&descriptor, &mut WitnessRegistry::default())
            .map_err(AdapterError::Ingress)?;
        Ok(batch)
    }

    fn parse_record(
        &self,
        row: usize,
        headers: &StringRecord,
        header_index: &BTreeMap<String, usize>,
        record: &StringRecord,
        ingested_at_unix_ms: u64,
        descriptor: &ExternalAdapterDescriptor,
    ) -> Result<IngressRecord, AdapterError> {
        let get = |column: &str| -> Result<&str, AdapterError> {
            let index = *header_index.get(column).expect("configuration was validated");
            let value = record.get(index).unwrap_or_default().trim();
            if value.is_empty() {
                return Err(AdapterError::EmptyField {
                    row,
                    column: column.to_owned(),
                });
            }
            Ok(value)
        };

        let event_raw = get(&self.config.source_event_id_column)?;
        let source_event_id = ReferenceId::new(event_raw).map_err(|_| AdapterError::InvalidReference {
            row,
            column: self.config.source_event_id_column.clone(),
        })?;
        let timestamp_raw = get(&self.config.observed_at_column)?;
        let observed_at_unix_ms = parse_timestamp(
            timestamp_raw,
            self.config.timestamp_encoding,
            row,
            &self.config.observed_at_column,
        )?;
        let scope = self.parse_scope(row, &get)?;
        let payload_digest = digest_record(headers, record);
        let mapping_digest = descriptor.mapping_digest;

        let witness = SourceWitness {
            source_system: descriptor.source_system.clone(),
            source_event_id: source_event_id.clone(),
            payload_digest,
            observed_at_unix_ms,
            ingested_at_unix_ms,
        };

        let mut normalized = Vec::with_capacity(self.config.outputs.len());
        for output in &self.config.outputs {
            let mantissa = match &output.value {
                ValueMapping::Constant { mantissa } => *mantissa,
                ValueMapping::Column { column, decimal } => {
                    let raw = get(column)?;
                    parse_scaled_decimal(raw, output.scale, decimal, row, column)?
                }
            };
            let observation_digest = digest_observation_identity(
                &descriptor.source_system,
                &source_event_id,
                &output.input,
                &output.metric,
                mapping_digest,
            );
            let observation = ObservationRef(
                ReferenceId::new(format!("observation:sha256:{}", hex_digest(observation_digest)))
                    .expect("generated observation id is canonical"),
            );
            normalized.push(NormalizedInput {
                input: output.input.clone(),
                observation: MetricObservation {
                    observation,
                    source_system: descriptor.source_system.clone(),
                    source_event_id: source_event_id.clone(),
                    source_payload_digest: payload_digest,
                    mapping_digest,
                    metric: output.metric.clone(),
                    scope: scope.clone(),
                    value: ScaledValue {
                        mantissa,
                        scale: output.scale,
                        unit: output.unit.clone(),
                    },
                    observed_at_unix_ms,
                },
            });
        }

        Ok(IngressRecord {
            event: ExternalEventEnvelope {
                witness,
                source_schema: descriptor.source_schema.clone(),
                source_schema_digest: descriptor.source_schema_digest,
            },
            mapping_digest,
            normalized,
        })
    }

    fn parse_scope<'a, F>(&self, row: usize, get: &F) -> Result<ScopeRef, AdapterError>
    where
        F: Fn(&str) -> Result<&'a str, AdapterError>,
    {
        match &self.config.scope {
            ScopeMapping::Constant(scope) => Ok(scope.clone()),
            ScopeMapping::Column { column, prefix } => {
                let value = get(column)?;
                ScopeRef::new(format!("{prefix}{value}")).map_err(|_| AdapterError::InvalidScope {
                    row,
                    column: column.clone(),
                })
            }
        }
    }
}

fn digest_record(headers: &StringRecord, record: &StringRecord) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:delimited-record:v1");
    for (header, value) in headers.iter().zip(record.iter()) {
        hash_str(&mut hasher, header);
        hash_str(&mut hasher, value);
    }
    finish_digest(hasher)
}

fn digest_observation_identity(
    source_system: &ReferenceId,
    source_event_id: &ReferenceId,
    input: &ReferenceId,
    metric: &ReferenceId,
    mapping_digest: Digest32,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:delimited-observation:v1");
    hash_str(&mut hasher, source_system.as_str());
    hash_str(&mut hasher, source_event_id.as_str());
    hash_str(&mut hasher, input.as_str());
    hash_str(&mut hasher, metric.as_str());
    hasher.update(mapping_digest.0);
    finish_digest(hasher)
}

fn digest_batch(descriptor: &ExternalAdapterDescriptor, records: &[IngressRecord]) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:delimited-batch:v2");
    hash_str(&mut hasher, descriptor.source_system.as_str());
    hash_str(&mut hasher, descriptor.source_schema.as_str());
    hasher.update(descriptor.adapter_digest.0);
    hasher.update(descriptor.source_schema_digest.0);
    hasher.update(descriptor.mapping_digest.0);
    for record in records {
        hash_str(&mut hasher, record.event.witness.source_event_id.as_str());
        hasher.update(record.event.witness.payload_digest.0);
    }
    finish_digest(hasher)
}

fn parse_timestamp(
    value: &str,
    encoding: TimestampEncoding,
    row: usize,
    column: &str,
) -> Result<u64, AdapterError> {
    let millis = match encoding {
        TimestampEncoding::UnixMilliseconds => value
            .parse::<u64>()
            .map_err(|_| AdapterError::InvalidTimestamp {
                row,
                column: column.to_owned(),
            })?,
        TimestampEncoding::UnixSeconds => value
            .parse::<u64>()
            .map_err(|_| AdapterError::InvalidTimestamp {
                row,
                column: column.to_owned(),
            })?
            .checked_mul(1_000)
            .ok_or_else(|| AdapterError::TimestampOutOfRange {
                row,
                column: column.to_owned(),
            })?,
        TimestampEncoding::Rfc3339 => {
            let parsed = OffsetDateTime::parse(value, &Rfc3339).map_err(|_| {
                AdapterError::InvalidTimestamp {
                    row,
                    column: column.to_owned(),
                }
            })?;
            let nanos = parsed.unix_timestamp_nanos();
            if nanos <= 0 {
                return Err(AdapterError::TimestampOutOfRange {
                    row,
                    column: column.to_owned(),
                });
            }
            u64::try_from(nanos / 1_000_000).map_err(|_| AdapterError::TimestampOutOfRange {
                row,
                column: column.to_owned(),
            })?
        }
    };
    if millis == 0 {
        return Err(AdapterError::TimestampOutOfRange {
            row,
            column: column.to_owned(),
        });
    }
    Ok(millis)
}

fn parse_scaled_decimal(
    value: &str,
    scale: u32,
    policy: &DecimalPolicy,
    row: usize,
    column: &str,
) -> Result<i128, AdapterError> {
    let mut text = value.trim();
    let negative = text.starts_with('-');
    if negative {
        if !policy.allow_negative {
            return Err(AdapterError::NegativeNotAllowed {
                row,
                column: column.to_owned(),
            });
        }
        text = &text[1..];
    } else if text.starts_with('+') {
        text = &text[1..];
    }
    if text.is_empty() {
        return Err(AdapterError::InvalidNumber {
            row,
            column: column.to_owned(),
        });
    }

    let parts = text.split(policy.decimal_separator).collect::<Vec<_>>();
    if parts.len() > 2 || parts[0].is_empty() {
        return Err(AdapterError::InvalidNumber {
            row,
            column: column.to_owned(),
        });
    }
    let whole_raw = parts[0];
    let fraction = parts.get(1).copied().unwrap_or("");
    if policy
        .grouping_separator
        .is_some_and(|group| fraction.contains(group))
    {
        return Err(AdapterError::InvalidGrouping {
            row,
            column: column.to_owned(),
        });
    }
    if !fraction.chars().all(|c| c.is_ascii_digit()) {
        return Err(AdapterError::InvalidNumber {
            row,
            column: column.to_owned(),
        });
    }

    let whole = if let Some(group) = policy.grouping_separator {
        if whole_raw.contains(group) {
            let groups = whole_raw.split(group).collect::<Vec<_>>();
            let Some(first) = groups.first() else {
                return Err(AdapterError::InvalidGrouping {
                    row,
                    column: column.to_owned(),
                });
            };
            if first.is_empty()
                || first.len() > 3
                || !first.chars().all(|c| c.is_ascii_digit())
                || groups
                    .iter()
                    .skip(1)
                    .any(|part| part.len() != 3 || !part.chars().all(|c| c.is_ascii_digit()))
            {
                return Err(AdapterError::InvalidGrouping {
                    row,
                    column: column.to_owned(),
                });
            }
            groups.concat()
        } else {
            whole_raw.to_owned()
        }
    } else {
        whole_raw.to_owned()
    };

    if !whole.chars().all(|c| c.is_ascii_digit()) {
        return Err(AdapterError::InvalidNumber {
            row,
            column: column.to_owned(),
        });
    }
    if fraction.len() > scale as usize {
        return Err(AdapterError::PrecisionLoss {
            row,
            column: column.to_owned(),
        });
    }

    let factor = 10_i128
        .checked_pow(scale)
        .ok_or_else(|| AdapterError::ArithmeticOverflow {
            row,
            column: column.to_owned(),
        })?;
    let whole_value = whole
        .parse::<i128>()
        .map_err(|_| AdapterError::InvalidNumber {
            row,
            column: column.to_owned(),
        })?
        .checked_mul(factor)
        .ok_or_else(|| AdapterError::ArithmeticOverflow {
            row,
            column: column.to_owned(),
        })?;
    let fraction_value = if fraction.is_empty() {
        0
    } else {
        let raw = fraction
            .parse::<i128>()
            .map_err(|_| AdapterError::InvalidNumber {
                row,
                column: column.to_owned(),
            })?;
        raw.checked_mul(10_i128.pow(scale - fraction.len() as u32))
            .ok_or_else(|| AdapterError::ArithmeticOverflow {
                row,
                column: column.to_owned(),
            })?
    };
    let magnitude = whole_value
        .checked_add(fraction_value)
        .ok_or_else(|| AdapterError::ArithmeticOverflow {
            row,
            column: column.to_owned(),
        })?;
    if negative {
        magnitude
            .checked_neg()
            .ok_or_else(|| AdapterError::ArithmeticOverflow {
                row,
                column: column.to_owned(),
            })
    } else {
        Ok(magnitude)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_business_core::{Digest32, ReferenceId};

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn config() -> DelimitedAdapterConfig {
        DelimitedAdapterConfig {
            adapter_semantic_id: id("adapter:delimited:hospitality-sales:v1"),
            source_system: id("source:restaurant-pos-export"),
            adapter_digest: Digest32::repeat(0x41),
            source_schema: id("schema:hospitality-sales-export:v1"),
            delimiter: b',',
            expected_headers: vec![
                "event_id".into(),
                "occurred_at".into(),
                "location".into(),
                "item".into(),
                "quantity".into(),
                "net_amount".into(),
            ],
            source_event_id_column: "event_id".into(),
            observed_at_column: "occurred_at".into(),
            timestamp_encoding: TimestampEncoding::Rfc3339,
            scope: ScopeMapping::Column {
                column: "location".into(),
                prefix: "scope:location:".into(),
            },
            outputs: vec![
                OutputMapping {
                    input: id("input:hospitality:sales-transactions:v1"),
                    metric: id("metric:hospitality:item-demand"),
                    value: ValueMapping::Column {
                        column: "quantity".into(),
                        decimal: DecimalPolicy::default(),
                    },
                    unit: id("unit:count"),
                    scale: 0,
                },
                OutputMapping {
                    input: id("input:hospitality:sales-transactions:v1"),
                    metric: id("metric:hospitality:net-sales"),
                    value: ValueMapping::Column {
                        column: "net_amount".into(),
                        decimal: DecimalPolicy::default(),
                    },
                    unit: id("unit:currency-minor-agnostic"),
                    scale: 2,
                },
            ],
            maximum_batch_records: 100,
        }
    }

    fn one_row_csv(amount: &str) -> String {
        format!(
            "event_id,occurred_at,location,item,quantity,net_amount\nsale:1,2026-09-10T08:00:00+02:00,rosebank,Coffee,1,{amount}\n"
        )
    }

    #[test]
    fn parses_realistic_csv_without_write_surface() {
        let adapter = DelimitedIngressAdapter::new(config()).unwrap();
        let csv = concat!(
            "event_id,occurred_at,location,item,quantity,net_amount\n",
            "sale:1,2026-09-10T08:00:00+02:00,rosebank,\"Breakfast, Large\",2,149.90\n",
            "sale:2,2026-09-10T08:05:00+02:00,rosebank,Coffee,1,34.50\n"
        );
        let batch = adapter
            .parse_batch(csv.as_bytes(), 1_789_000_000_000)
            .unwrap();
        assert!(DELIMITED_ADAPTER_IS_READ_ONLY);
        assert_eq!(batch.records.len(), 2);
        assert_eq!(batch.records[0].normalized.len(), 2);
        assert_eq!(batch.records[0].normalized[0].observation.value.mantissa, 2);
        assert_eq!(batch.records[0].normalized[1].observation.value.mantissa, 14_990);
        assert_eq!(
            batch.records[0].normalized[0].observation.scope.0.as_str(),
            "scope:location:rosebank"
        );
    }

    #[test]
    fn header_order_is_part_of_schema_contract() {
        let adapter = DelimitedIngressAdapter::new(config()).unwrap();
        let csv = concat!(
            "occurred_at,event_id,location,item,quantity,net_amount\n",
            "2026-09-10T08:00:00+02:00,sale:1,rosebank,Coffee,1,34.50\n"
        );
        assert!(matches!(
            adapter.parse_batch(csv.as_bytes(), 1_789_000_000_000),
            Err(AdapterError::HeaderMismatch)
        ));
    }

    #[test]
    fn row_change_changes_payload_witness() {
        let adapter = DelimitedIngressAdapter::new(config()).unwrap();
        let a = concat!(
            "event_id,occurred_at,location,item,quantity,net_amount\n",
            "sale:1,2026-09-10T08:00:00+02:00,rosebank,Coffee,1,34.50\n"
        );
        let b = concat!(
            "event_id,occurred_at,location,item,quantity,net_amount\n",
            "sale:1,2026-09-10T08:00:00+02:00,rosebank,Coffee,2,34.50\n"
        );
        let a = adapter.parse_batch(a.as_bytes(), 1_789_000_000_000).unwrap();
        let b = adapter.parse_batch(b.as_bytes(), 1_789_000_000_000).unwrap();
        assert_ne!(
            a.records[0].event.witness.payload_digest,
            b.records[0].event.witness.payload_digest
        );
    }

    #[test]
    fn mapping_change_changes_mapping_and_batch_digest() {
        let original = config();
        let original_mapping = original.mapping_digest().unwrap();
        let original_batch = DelimitedIngressAdapter::new(original.clone())
            .unwrap()
            .parse_batch(one_row_csv("34.50").as_bytes(), 1_789_000_000_000)
            .unwrap();

        let mut changed = original;
        changed.outputs[1].scale = 3;
        let changed_mapping = changed.mapping_digest().unwrap();
        let changed_batch = DelimitedIngressAdapter::new(changed)
            .unwrap()
            .parse_batch(one_row_csv("34.50").as_bytes(), 1_789_000_000_000)
            .unwrap();

        assert_ne!(original_mapping, changed_mapping);
        assert_ne!(original_batch.batch_digest, changed_batch.batch_digest);
    }

    #[test]
    fn exact_decimal_rejects_precision_loss() {
        let policy = DecimalPolicy::default();
        assert!(matches!(
            parse_scaled_decimal("1.234", 2, &policy, 2, "amount"),
            Err(AdapterError::PrecisionLoss { .. })
        ));
        assert_eq!(parse_scaled_decimal("1.23", 2, &policy, 2, "amount").unwrap(), 123);
    }

    #[test]
    fn comma_decimal_is_configurable_without_float_conversion() {
        let policy = DecimalPolicy {
            decimal_separator: ',',
            grouping_separator: Some('.'),
            allow_negative: true,
        };
        assert_eq!(
            parse_scaled_decimal("1.234,56", 2, &policy, 2, "amount").unwrap(),
            123_456
        );
        assert_eq!(
            parse_scaled_decimal("-2,50", 2, &policy, 2, "amount").unwrap(),
            -250
        );
    }

    #[test]
    fn malformed_grouping_fails_closed() {
        let policy = DecimalPolicy {
            decimal_separator: '.',
            grouping_separator: Some(','),
            allow_negative: false,
        };
        assert!(matches!(
            parse_scaled_decimal("1,2,34.50", 2, &policy, 2, "amount"),
            Err(AdapterError::InvalidGrouping { .. })
        ));
    }

    #[test]
    fn duplicate_source_event_fails_batch_validation() {
        let adapter = DelimitedIngressAdapter::new(config()).unwrap();
        let csv = concat!(
            "event_id,occurred_at,location,item,quantity,net_amount\n",
            "sale:1,2026-09-10T08:00:00+02:00,rosebank,Coffee,1,34.50\n",
            "sale:1,2026-09-10T08:05:00+02:00,rosebank,Tea,1,30.00\n"
        );
        assert!(matches!(
            adapter.parse_batch(csv.as_bytes(), 1_789_000_000_000),
            Err(AdapterError::Ingress(IngressBatchError::DuplicateSourceEvent { .. }))
        ));
    }

    #[test]
    fn unix_seconds_timestamp_is_exact() {
        assert_eq!(
            parse_timestamp("100", TimestampEncoding::UnixSeconds, 2, "time").unwrap(),
            100_000
        );
    }
}
