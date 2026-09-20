// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Server-neutral projection of verified Mycelix hardware component records
//! into KiCad's read-only HTTP Library response contract.
//!
//! This crate does not own HTTP transport, authentication, Holochain access, or
//! component authority. A transport adapter must enforce KiCad's token-auth
//! contract before calling this read-only catalog surface.

#![deny(unsafe_code)]

use mycelix_hardware_core::{ComponentIdentity, DigestRef, LifecycleState, SemanticId};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const KICAD_HTTP_PROFILE: &str = "mycelix.kicad-http-library.v1";
pub const MAX_CATEGORIES: usize = 512;
pub const MAX_PARTS_PER_CATEGORY: usize = 5_000;
pub const MAX_TEXT_BYTES: usize = 4096;
pub const MAX_FIELDS: usize = 256;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GatewayCategory {
    pub id: String,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

impl GatewayCategory {
    fn validate(&self) -> Result<(), GatewayError> {
        validate_token(&self.id, "category id")?;
        validate_text(&self.name, "category name")?;
        if let Some(description) = &self.description {
            validate_text(description, "category description")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GatewayComponentBinding {
    pub category_id: String,
    /// Exact digest supplied by the component source-of-record adapter.
    pub component_record_digest: DigestRef,
    /// Existing KiCad symbol library identifier, e.g. `Device:R`.
    pub symbol_id: String,
    /// Existing KiCad footprint library identifier.
    pub footprint: String,
    /// Symbol reference prefix/value are explicit; the gateway never guesses them.
    pub reference: String,
    pub value: String,
    pub display_name: Option<String>,
    pub description: Option<String>,
    pub keywords: Vec<String>,
    pub footprint_filters: Vec<String>,
    pub datasheet: Option<String>,
    pub exclude_from_bom: bool,
    pub exclude_from_board: bool,
    pub exclude_from_sim: bool,
    pub extra_fields: BTreeMap<String, KiCadBoundField>,
}

impl GatewayComponentBinding {
    fn validate(&self) -> Result<(), GatewayError> {
        validate_token(&self.category_id, "category id")?;
        self.component_record_digest
            .validate()
            .map_err(|_| GatewayError::InvalidDigest)?;
        validate_library_id(&self.symbol_id, "symbol id")?;
        validate_library_id(&self.footprint, "footprint")?;
        validate_token(&self.reference, "reference")?;
        validate_text(&self.value, "value")?;
        if let Some(name) = &self.display_name {
            validate_text(name, "display name")?;
        }
        if let Some(description) = &self.description {
            validate_text(description, "description")?;
        }
        if let Some(datasheet) = &self.datasheet {
            validate_text(datasheet, "datasheet")?;
        }
        if self.keywords.len() > MAX_FIELDS || self.footprint_filters.len() > MAX_FIELDS {
            return Err(GatewayError::TooManyValues);
        }
        ensure_unique(self.keywords.iter(), "keyword")?;
        ensure_unique(self.footprint_filters.iter(), "footprint filter")?;
        for keyword in &self.keywords {
            validate_text(keyword, "keyword")?;
        }
        for filter in &self.footprint_filters {
            validate_text(filter, "footprint filter")?;
        }
        if self.extra_fields.len() > MAX_FIELDS {
            return Err(GatewayError::TooManyFields);
        }
        for (name, field) in &self.extra_fields {
            validate_token(name, "extra field name")?;
            if reserved_field_names().contains(name.as_str()) {
                return Err(GatewayError::ReservedField(name.clone()));
            }
            field.validate()?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KiCadBoundField {
    pub value: String,
    pub visible: bool,
}

impl KiCadBoundField {
    fn validate(&self) -> Result<(), GatewayError> {
        validate_text(&self.value, "extra field value")
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GatewayCatalogEntry {
    pub component: ComponentIdentity,
    pub binding: GatewayComponentBinding,
}

pub trait ComponentRecordVerifier {
    /// Verify that `digest` is the source-of-record digest for exactly this
    /// component record. The gateway refuses to serve an entry if this fails.
    fn verify_component_record(
        &self,
        component: &ComponentIdentity,
        digest: &DigestRef,
    ) -> Result<bool, String>;
}

#[derive(Debug, Clone)]
pub struct KiCadHttpCatalog {
    categories: BTreeMap<String, GatewayCategory>,
    entries: BTreeMap<SemanticId, GatewayCatalogEntry>,
    category_members: BTreeMap<String, Vec<SemanticId>>,
}

impl KiCadHttpCatalog {
    pub fn new(
        categories: Vec<GatewayCategory>,
        entries: Vec<GatewayCatalogEntry>,
        verifier: &dyn ComponentRecordVerifier,
    ) -> Result<Self, GatewayError> {
        if categories.len() > MAX_CATEGORIES {
            return Err(GatewayError::TooManyCategories);
        }
        let mut category_map = BTreeMap::new();
        for category in categories {
            category.validate()?;
            if category_map.insert(category.id.clone(), category).is_some() {
                return Err(GatewayError::DuplicateCategory);
            }
        }

        let mut entry_map = BTreeMap::new();
        let mut category_members: BTreeMap<String, Vec<SemanticId>> = BTreeMap::new();
        for entry in entries {
            entry
                .component
                .validate()
                .map_err(|error| GatewayError::InvalidComponent(error.to_string()))?;
            entry.binding.validate()?;
            if !category_map.contains_key(&entry.binding.category_id) {
                return Err(GatewayError::UnknownCategory(
                    entry.binding.category_id.clone(),
                ));
            }
            match verifier.verify_component_record(
                &entry.component,
                &entry.binding.component_record_digest,
            ) {
                Ok(true) => {}
                Ok(false) => {
                    return Err(GatewayError::ComponentDigestNotVerified(
                        entry.component.id.to_string(),
                    ));
                }
                Err(reason) => return Err(GatewayError::Verifier(reason)),
            }
            let id = entry.component.id.clone();
            if entry_map.insert(id.clone(), entry).is_some() {
                return Err(GatewayError::DuplicateComponentId(id.to_string()));
            }
            let category_id = entry_map
                .get(&id)
                .expect("inserted entry")
                .binding
                .category_id
                .clone();
            category_members.entry(category_id).or_default().push(id);
        }

        for members in category_members.values_mut() {
            members.sort();
            if members.len() > MAX_PARTS_PER_CATEGORY {
                return Err(GatewayError::TooManyPartsInCategory);
            }
        }

        Ok(Self {
            categories: category_map,
            entries: entry_map,
            category_members,
        })
    }

    pub fn profile(&self) -> &'static str {
        KICAD_HTTP_PROFILE
    }

    /// KiCad endpoint-discovery response. Values may be blank per the protocol.
    pub fn endpoint_index(&self) -> BTreeMap<String, String> {
        BTreeMap::from([
            ("categories".to_string(), String::new()),
            ("parts".to_string(), String::new()),
        ])
    }

    pub fn categories(&self) -> Vec<KiCadCategoryResponse> {
        self.categories
            .values()
            .map(|category| KiCadCategoryResponse {
                id: category.id.clone(),
                name: category.name.clone(),
                description: category.description.clone(),
            })
            .collect()
    }

    pub fn parts_by_category(
        &self,
        category_id: &str,
    ) -> Result<Vec<KiCadPartSummary>, GatewayError> {
        if !self.categories.contains_key(category_id) {
            return Err(GatewayError::UnknownCategory(category_id.to_string()));
        }
        let mut response = Vec::new();
        if let Some(members) = self.category_members.get(category_id) {
            for id in members {
                let entry = self
                    .entries
                    .get(id)
                    .ok_or_else(|| GatewayError::InternalMissingEntry(id.to_string()))?;
                response.push(project_summary(entry)?);
            }
        }
        Ok(response)
    }

    pub fn part(&self, part_id: &str) -> Result<KiCadPartDetail, GatewayError> {
        let id = SemanticId::new(part_id.to_string())
            .map_err(|_| GatewayError::InvalidPartId(part_id.to_string()))?;
        let entry = self
            .entries
            .get(&id)
            .ok_or_else(|| GatewayError::UnknownPart(part_id.to_string()))?;
        project_detail(entry)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct KiCadCategoryResponse {
    pub id: String,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct KiCadFieldResponse {
    pub value: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visible: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct KiCadPartSummary {
    pub id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub keywords: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub footprint_filters: Vec<String>,
    pub fields: BTreeMap<String, KiCadFieldResponse>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct KiCadPartDetail {
    pub id: String,
    pub name: String,
    #[serde(rename = "symbolIdStr")]
    pub symbol_id_str: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub keywords: Option<String>,
    pub exclude_from_bom: String,
    pub exclude_from_board: String,
    pub exclude_from_sim: String,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub footprint_filters: Vec<String>,
    pub fields: BTreeMap<String, KiCadFieldResponse>,
}

fn project_summary(entry: &GatewayCatalogEntry) -> Result<KiCadPartSummary, GatewayError> {
    Ok(KiCadPartSummary {
        id: entry.component.id.to_string(),
        name: Some(
            entry
                .binding
                .display_name
                .clone()
                .unwrap_or_else(|| entry.component.name.clone()),
        ),
        description: entry.binding.description.clone(),
        keywords: keyword_string(&entry.binding.keywords),
        footprint_filters: entry.binding.footprint_filters.clone(),
        fields: fields_for(entry)?,
    })
}

fn project_detail(entry: &GatewayCatalogEntry) -> Result<KiCadPartDetail, GatewayError> {
    Ok(KiCadPartDetail {
        id: entry.component.id.to_string(),
        name: entry
            .binding
            .display_name
            .clone()
            .unwrap_or_else(|| entry.component.name.clone()),
        symbol_id_str: entry.binding.symbol_id.clone(),
        description: entry.binding.description.clone(),
        keywords: keyword_string(&entry.binding.keywords),
        exclude_from_bom: bool_string(entry.binding.exclude_from_bom),
        exclude_from_board: bool_string(entry.binding.exclude_from_board),
        exclude_from_sim: bool_string(entry.binding.exclude_from_sim),
        footprint_filters: entry.binding.footprint_filters.clone(),
        fields: fields_for(entry)?,
    })
}

fn fields_for(
    entry: &GatewayCatalogEntry,
) -> Result<BTreeMap<String, KiCadFieldResponse>, GatewayError> {
    let mut fields = BTreeMap::new();
    insert_field(&mut fields, "footprint", &entry.binding.footprint, false)?;
    insert_field(&mut fields, "value", &entry.binding.value, true)?;
    insert_field(&mut fields, "reference", &entry.binding.reference, true)?;
    insert_field(
        &mut fields,
        "mycelix_component_id",
        &entry.component.id.to_string(),
        false,
    )?;
    insert_field(
        &mut fields,
        "mycelix_component_digest",
        &digest_string(&entry.binding.component_record_digest),
        false,
    )?;
    insert_field(
        &mut fields,
        "mycelix_lifecycle",
        lifecycle_string(entry.component.lifecycle),
        false,
    )?;
    if let Some(manufacturer) = &entry.component.manufacturer {
        insert_field(&mut fields, "manufacturer", manufacturer, false)?;
    }
    if let Some(mpn) = &entry.component.manufacturer_part_number {
        insert_field(&mut fields, "manufacturer_part_number", mpn, false)?;
    }
    if let Some(datasheet) = &entry.binding.datasheet {
        insert_field(&mut fields, "datasheet", datasheet, false)?;
    }
    if let Some(description) = &entry.binding.description {
        insert_field(&mut fields, "description", description, false)?;
    }
    if let Some(keywords) = keyword_string(&entry.binding.keywords) {
        insert_field(&mut fields, "keywords", &keywords, false)?;
    }
    for (name, field) in &entry.binding.extra_fields {
        insert_field(&mut fields, name, &field.value, field.visible)?;
    }
    if fields.len() > MAX_FIELDS {
        return Err(GatewayError::TooManyFields);
    }
    Ok(fields)
}

fn insert_field(
    fields: &mut BTreeMap<String, KiCadFieldResponse>,
    name: &str,
    value: &str,
    visible: bool,
) -> Result<(), GatewayError> {
    validate_token(name, "field name")?;
    validate_text(value, "field value")?;
    if fields
        .insert(
            name.to_string(),
            KiCadFieldResponse {
                value: value.to_string(),
                visible: Some(bool_string(visible)),
            },
        )
        .is_some()
    {
        return Err(GatewayError::DuplicateProjectedField(name.to_string()));
    }
    Ok(())
}

fn bool_string(value: bool) -> String {
    if value { "True" } else { "False" }.to_string()
}

fn keyword_string(values: &[String]) -> Option<String> {
    if values.is_empty() {
        None
    } else {
        Some(values.join(" "))
    }
}

fn lifecycle_string(state: LifecycleState) -> &'static str {
    match state {
        LifecycleState::Unknown => "Unknown",
        LifecycleState::Active => "Active",
        LifecycleState::NotRecommendedForNewDesigns => "NotRecommendedForNewDesigns",
        LifecycleState::EndOfLife => "EndOfLife",
        LifecycleState::Obsolete => "Obsolete",
    }
}

fn digest_string(digest: &DigestRef) -> String {
    let algorithm = match digest.algorithm {
        mycelix_hardware_core::DigestAlgorithm::Sha256 => "sha256",
        mycelix_hardware_core::DigestAlgorithm::Blake3 => "blake3",
    };
    format!("{algorithm}:{}", digest.hex)
}

fn reserved_field_names() -> BTreeSet<&'static str> {
    BTreeSet::from([
        "footprint",
        "value",
        "reference",
        "datasheet",
        "description",
        "keywords",
        "manufacturer",
        "manufacturer_part_number",
        "mycelix_component_id",
        "mycelix_component_digest",
        "mycelix_lifecycle",
    ])
}

fn validate_library_id(value: &str, field: &'static str) -> Result<(), GatewayError> {
    validate_token(value, field)?;
    if !value.contains(':') {
        return Err(GatewayError::InvalidLibraryId(field));
    }
    Ok(())
}

fn validate_token(value: &str, field: &'static str) -> Result<(), GatewayError> {
    if value.trim().is_empty()
        || value != value.trim()
        || value.len() > MAX_TEXT_BYTES
        || value.chars().any(char::is_control)
        || value.chars().any(char::is_whitespace)
    {
        return Err(GatewayError::InvalidText(field));
    }
    Ok(())
}

fn validate_text(value: &str, field: &'static str) -> Result<(), GatewayError> {
    if value.trim().is_empty() || value != value.trim() || value.chars().any(char::is_control) {
        return Err(GatewayError::InvalidText(field));
    }
    if value.len() > MAX_TEXT_BYTES {
        return Err(GatewayError::FieldTooLong(field));
    }
    Ok(())
}

fn ensure_unique<'a, T, I>(values: I, kind: &'static str) -> Result<(), GatewayError>
where
    T: Ord + ?Sized + 'a,
    I: IntoIterator<Item = &'a T>,
{
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(value) {
            return Err(GatewayError::DuplicateValue(kind));
        }
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GatewayError {
    TooManyCategories,
    TooManyPartsInCategory,
    TooManyValues,
    TooManyFields,
    DuplicateCategory,
    DuplicateComponentId(String),
    DuplicateValue(&'static str),
    DuplicateProjectedField(String),
    UnknownCategory(String),
    UnknownPart(String),
    InvalidPartId(String),
    InternalMissingEntry(String),
    InvalidComponent(String),
    InvalidDigest,
    ComponentDigestNotVerified(String),
    Verifier(String),
    ReservedField(String),
    InvalidLibraryId(&'static str),
    InvalidText(&'static str),
    FieldTooLong(&'static str),
}

impl fmt::Display for GatewayError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooManyCategories => write!(f, "too many KiCad categories"),
            Self::TooManyPartsInCategory => write!(f, "too many parts in KiCad category"),
            Self::TooManyValues => write!(f, "too many values in KiCad binding"),
            Self::TooManyFields => write!(f, "too many KiCad fields"),
            Self::DuplicateCategory => write!(f, "duplicate KiCad category"),
            Self::DuplicateComponentId(id) => write!(f, "duplicate component id: {id}"),
            Self::DuplicateValue(kind) => write!(f, "duplicate {kind}"),
            Self::DuplicateProjectedField(name) => write!(f, "duplicate projected field: {name}"),
            Self::UnknownCategory(id) => write!(f, "unknown KiCad category: {id}"),
            Self::UnknownPart(id) => write!(f, "unknown KiCad part: {id}"),
            Self::InvalidPartId(id) => write!(f, "invalid KiCad part id: {id}"),
            Self::InternalMissingEntry(id) => write!(f, "catalog index refers to missing entry: {id}"),
            Self::InvalidComponent(reason) => write!(f, "invalid component: {reason}"),
            Self::InvalidDigest => write!(f, "invalid component record digest"),
            Self::ComponentDigestNotVerified(id) => write!(f, "component digest was not verified: {id}"),
            Self::Verifier(reason) => write!(f, "component verifier failed: {reason}"),
            Self::ReservedField(name) => write!(f, "extra field attempts to override reserved field: {name}"),
            Self::InvalidLibraryId(field) => write!(f, "{field} must identify an existing KiCad library item"),
            Self::InvalidText(field) => write!(f, "invalid {field}"),
            Self::FieldTooLong(field) => write!(f, "{field} exceeds maximum length"),
        }
    }
}

impl std::error::Error for GatewayError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_hardware_core::{ComponentKind, DigestRef, HARDWARE_SEMANTIC_SCHEMA};

    struct Verifier {
        accepted: String,
    }

    impl ComponentRecordVerifier for Verifier {
        fn verify_component_record(
            &self,
            component: &ComponentIdentity,
            digest: &DigestRef,
        ) -> Result<bool, String> {
            Ok(component.id.as_str() == self.accepted && digest.hex == "a".repeat(64))
        }
    }

    fn id(value: &str) -> SemanticId {
        SemanticId::new(value).unwrap()
    }

    fn component(id_value: &str, manufacturer: &str, mpn: &str) -> ComponentIdentity {
        ComponentIdentity {
            schema_version: HARDWARE_SEMANTIC_SCHEMA.into(),
            id: id(id_value),
            name: mpn.into(),
            kind: ComponentKind::Electronic,
            manufacturer: Some(manufacturer.into()),
            manufacturer_part_number: Some(mpn.into()),
            open_hardware_project: None,
            datasheet_refs: vec![],
            lifecycle: LifecycleState::Unknown,
        }
    }

    fn binding(category: &str) -> GatewayComponentBinding {
        GatewayComponentBinding {
            category_id: category.into(),
            component_record_digest: DigestRef::sha256("a".repeat(64)).unwrap(),
            symbol_id: "MCU_Microchip_ATmega:ATmega328P-AU".into(),
            footprint: "Package_QFP:TQFP-32_7x7mm_P0.8mm".into(),
            reference: "U".into(),
            value: "ATmega328P-AU".into(),
            display_name: None,
            description: Some("Example MCU".into()),
            keywords: vec!["mcu".into(), "avr".into()],
            footprint_filters: vec!["TQFP*".into()],
            datasheet: Some("https://example.invalid/datasheet.pdf".into()),
            exclude_from_bom: false,
            exclude_from_board: false,
            exclude_from_sim: true,
            extra_fields: BTreeMap::new(),
        }
    }

    fn category() -> GatewayCategory {
        GatewayCategory {
            id: "microcontrollers".into(),
            name: "Active Parts/Microcontrollers".into(),
            description: Some("Microcontrollers".into()),
        }
    }

    #[test]
    fn endpoint_index_matches_kicad_contract_keys() {
        let catalog = KiCadHttpCatalog::new(vec![], vec![], &Verifier { accepted: "none".into() }).unwrap();
        assert_eq!(
            catalog.endpoint_index(),
            BTreeMap::from([
                ("categories".to_string(), String::new()),
                ("parts".to_string(), String::new()),
            ])
        );
    }

    #[test]
    fn part_projection_uses_semantic_id_not_mpn_as_identity() {
        let component = component("component:atmega-a", "Microchip", "ATmega328P-AU");
        let catalog = KiCadHttpCatalog::new(
            vec![category()],
            vec![GatewayCatalogEntry {
                component,
                binding: binding("microcontrollers"),
            }],
            &Verifier {
                accepted: "component:atmega-a".into(),
            },
        )
        .unwrap();
        let part = catalog.part("component:atmega-a").unwrap();
        assert_eq!(part.id, "component:atmega-a");
        assert_eq!(part.exclude_from_bom, "False");
        assert_eq!(part.exclude_from_sim, "True");
        assert_eq!(part.fields["mycelix_lifecycle"].value, "Unknown");
        assert_eq!(part.fields["manufacturer_part_number"].value, "ATmega328P-AU");
    }

    #[test]
    fn duplicate_mpn_across_manufacturers_does_not_collide() {
        struct AcceptAll;
        impl ComponentRecordVerifier for AcceptAll {
            fn verify_component_record(
                &self,
                _component: &ComponentIdentity,
                _digest: &DigestRef,
            ) -> Result<bool, String> {
                Ok(true)
            }
        }
        let first = GatewayCatalogEntry {
            component: component("component:a", "VendorA", "SHARED-123"),
            binding: binding("microcontrollers"),
        };
        let second = GatewayCatalogEntry {
            component: component("component:b", "VendorB", "SHARED-123"),
            binding: binding("microcontrollers"),
        };
        let catalog = KiCadHttpCatalog::new(vec![category()], vec![first, second], &AcceptAll).unwrap();
        assert_eq!(catalog.parts_by_category("microcontrollers").unwrap().len(), 2);
    }

    #[test]
    fn unverified_component_digest_blocks_catalog_admission() {
        let entry = GatewayCatalogEntry {
            component: component("component:a", "VendorA", "A1"),
            binding: binding("microcontrollers"),
        };
        let error = KiCadHttpCatalog::new(
            vec![category()],
            vec![entry],
            &Verifier {
                accepted: "component:other".into(),
            },
        )
        .unwrap_err();
        assert!(matches!(error, GatewayError::ComponentDigestNotVerified(_)));
    }

    #[test]
    fn reserved_extra_fields_cannot_override_canonical_projection() {
        let mut binding = binding("microcontrollers");
        binding.extra_fields.insert(
            "footprint".into(),
            KiCadBoundField {
                value: "malicious:override".into(),
                visible: true,
            },
        );
        assert_eq!(
            binding.validate().unwrap_err(),
            GatewayError::ReservedField("footprint".into())
        );
    }

    #[test]
    fn detail_json_keeps_kicad_scalar_flags_as_strings() {
        let entry = GatewayCatalogEntry {
            component: component("component:a", "VendorA", "A1"),
            binding: binding("microcontrollers"),
        };
        let catalog = KiCadHttpCatalog::new(
            vec![category()],
            vec![entry],
            &Verifier {
                accepted: "component:a".into(),
            },
        )
        .unwrap();
        let json = serde_json::to_value(catalog.part("component:a").unwrap()).unwrap();
        assert_eq!(json["exclude_from_bom"], "False");
        assert_eq!(json["fields"]["footprint"]["visible"], "False");
        assert_eq!(json["symbolIdStr"], "MCU_Microchip_ATmega:ATmega328P-AU");
    }
}
