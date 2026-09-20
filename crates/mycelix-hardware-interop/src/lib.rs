// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Loss-aware interoperability adapters for Mycelix open hardware.
//!
//! External schemas are adapters, never the canonical authority. Conversions
//! always produce an [`InteropReport`] describing semantic preservation or loss.

#![deny(unsafe_code)]

use mycelix_hardware_core::{
    ComponentIdentity, ComponentKind, HardwareProject, LifecycleState, LicenseRef,
    SemanticId, HARDWARE_SEMANTIC_SCHEMA,
};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;
use std::fmt;

pub const INTEROP_SCHEMA: &str = "mycelix.hardware.interop.v1";
pub const OKH_24_VERSION: &str = "2.4";
pub const CYCLONEDX_17_VERSION: &str = "1.7";
pub const ADAPTER_IMPLEMENTATION_VERSION: &str = env!("CARGO_PKG_VERSION");

const CDX_MPN_PROPERTY: &str = "mycelix:hardware:manufacturer-part-number";
const CDX_LIFECYCLE_PROPERTY: &str = "mycelix:hardware:lifecycle";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InteropTarget {
    OpenKnowHow24,
    CycloneDx17,
    Spdx31DevHardware,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InteropDirection {
    Import,
    Export,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InteropDisposition {
    Preserved,
    PreservedOpaque,
    Approximated,
    Omitted,
    Unsupported,
    UnknownSourceValue,
    InventedValue,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdapterVersion {
    pub target_version: String,
    pub implementation_version: String,
    pub experimental: bool,
}

impl AdapterVersion {
    fn stable(target_version: impl Into<String>) -> Self {
        Self {
            target_version: target_version.into(),
            implementation_version: ADAPTER_IMPLEMENTATION_VERSION.into(),
            experimental: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InteropFinding {
    pub source_path: String,
    pub target_path: Option<String>,
    pub disposition: InteropDisposition,
    pub detail: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InteropReport {
    pub schema_version: String,
    pub target: InteropTarget,
    pub direction: InteropDirection,
    pub adapter: AdapterVersion,
    pub findings: Vec<InteropFinding>,
}

impl InteropReport {
    pub fn new(
        target: InteropTarget,
        direction: InteropDirection,
        adapter: AdapterVersion,
    ) -> Self {
        Self {
            schema_version: INTEROP_SCHEMA.into(),
            target,
            direction,
            adapter,
            findings: Vec::new(),
        }
    }

    pub fn record(
        &mut self,
        source_path: impl Into<String>,
        target_path: Option<impl Into<String>>,
        disposition: InteropDisposition,
        detail: impl Into<String>,
    ) {
        self.findings.push(InteropFinding {
            source_path: source_path.into(),
            target_path: target_path.map(Into::into),
            disposition,
            detail: detail.into(),
        });
    }

    pub fn has_hard_error(&self) -> bool {
        self.findings
            .iter()
            .any(|finding| finding.disposition == InteropDisposition::InventedValue)
    }

    pub fn has_loss(&self) -> bool {
        self.findings.iter().any(|finding| {
            matches!(
                finding.disposition,
                InteropDisposition::Approximated
                    | InteropDisposition::Omitted
                    | InteropDisposition::Unsupported
                    | InteropDisposition::UnknownSourceValue
                    | InteropDisposition::InventedValue
            )
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InteropError {
    UnsupportedTargetVersion(String),
    InvalidRequiredField(&'static str),
    MissingRequiredField(&'static str),
    AmbiguousLicense,
    ReservedPassthroughKey(String),
    InvalidSemanticId(String),
    InvalidCoreModel(String),
    InvalidCycloneDxFormat,
}

impl fmt::Display for InteropError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedTargetVersion(version) => {
                write!(f, "unsupported target version: {version}")
            }
            Self::InvalidRequiredField(field) => write!(f, "invalid required field: {field}"),
            Self::MissingRequiredField(field) => write!(f, "missing required field: {field}"),
            Self::AmbiguousLicense => write!(f, "target requires one unambiguous license expression"),
            Self::ReservedPassthroughKey(key) => {
                write!(f, "passthrough contains reserved target key: {key}")
            }
            Self::InvalidSemanticId(reason) => write!(f, "invalid semantic id: {reason}"),
            Self::InvalidCoreModel(reason) => write!(f, "invalid core model: {reason}"),
            Self::InvalidCycloneDxFormat => write!(f, "invalid CycloneDX format marker"),
        }
    }
}

impl std::error::Error for InteropError {}

// -----------------------------------------------------------------------------
// Open Know-How 2.4
// -----------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OpenKnowHow24Manifest {
    pub okhv: String,
    pub name: String,
    pub repo: String,
    pub license: String,
    pub licensor: Value,
    #[serde(rename = "function")]
    pub function_description: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    #[serde(flatten)]
    pub extra: BTreeMap<String, Value>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OpenKnowHowExternalContext {
    pub repo: String,
    pub licensor: Value,
    pub function_description: String,
    pub version: Option<String>,
}

impl OpenKnowHowExternalContext {
    pub fn validate(&self) -> Result<(), InteropError> {
        validate_http_url(&self.repo, "repo")?;
        validate_nonempty(&self.function_description, "function")?;
        validate_licensor(&self.licensor)?;
        if let Some(version) = &self.version {
            validate_nonempty(version, "version")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ImportedOpenKnowHowProject {
    pub project: HardwareProject,
    /// Required OKH values that do not have an equivalent canonical field yet.
    pub external_context: OpenKnowHowExternalContext,
    /// Additional source fields are preserved opaquely rather than discarded.
    pub passthrough: BTreeMap<String, Value>,
    pub report: InteropReport,
}

pub fn import_open_know_how_24(
    manifest: OpenKnowHow24Manifest,
    project_id: SemanticId,
) -> Result<ImportedOpenKnowHowProject, InteropError> {
    if manifest.okhv != OKH_24_VERSION {
        return Err(InteropError::UnsupportedTargetVersion(manifest.okhv));
    }
    validate_nonempty(&manifest.name, "name")?;
    validate_http_url(&manifest.repo, "repo")?;
    validate_nonempty(&manifest.license, "license")?;
    validate_licensor(&manifest.licensor)?;
    validate_nonempty(&manifest.function_description, "function")?;
    if let Some(version) = &manifest.version {
        validate_nonempty(version, "version")?;
    }

    let project = HardwareProject {
        schema_version: HARDWARE_SEMANTIC_SCHEMA.into(),
        id: project_id,
        name: manifest.name.clone(),
        description: None,
        maintainers: Vec::new(),
        license_refs: vec![LicenseRef {
            identifier: manifest.license.clone(),
            text_or_url: None,
        }],
    };
    project
        .validate()
        .map_err(|error| InteropError::InvalidCoreModel(error.to_string()))?;

    let external_context = OpenKnowHowExternalContext {
        repo: manifest.repo,
        licensor: manifest.licensor,
        function_description: manifest.function_description,
        version: manifest.version,
    };

    let mut report = InteropReport::new(
        InteropTarget::OpenKnowHow24,
        InteropDirection::Import,
        AdapterVersion::stable(OKH_24_VERSION),
    );
    report.record(
        "name",
        Some("HardwareProject.name"),
        InteropDisposition::Preserved,
        "project name maps directly",
    );
    report.record(
        "license",
        Some("HardwareProject.license_refs[0]"),
        InteropDisposition::Preserved,
        "single SPDX/custom license expression retained",
    );
    report.record(
        "repo",
        Some("OpenKnowHowExternalContext.repo"),
        InteropDisposition::PreservedOpaque,
        "development repository is preserved without inventing a canonical Mycelix field",
    );
    report.record(
        "licensor",
        Some("OpenKnowHowExternalContext.licensor"),
        InteropDisposition::PreservedOpaque,
        "licensor is not treated as equivalent to maintainer",
    );
    report.record(
        "function",
        Some("OpenKnowHowExternalContext.function_description"),
        InteropDisposition::PreservedOpaque,
        "functional claim is not collapsed into a generic project description",
    );
    if external_context.version.is_some() {
        report.record(
            "version",
            Some("OpenKnowHowExternalContext.version"),
            InteropDisposition::PreservedOpaque,
            "OKH module version retained until a revision-binding adapter is applied",
        );
    }
    for key in manifest.extra.keys() {
        report.record(
            key,
            Some(format!("passthrough.{key}")),
            InteropDisposition::PreservedOpaque,
            "field preserved opaquely because the first adapter slice does not interpret it",
        );
    }

    Ok(ImportedOpenKnowHowProject {
        project,
        external_context,
        passthrough: manifest.extra,
        report,
    })
}

pub fn export_open_know_how_24(
    project: &HardwareProject,
    context: OpenKnowHowExternalContext,
    passthrough: BTreeMap<String, Value>,
) -> Result<(OpenKnowHow24Manifest, InteropReport), InteropError> {
    project
        .validate()
        .map_err(|error| InteropError::InvalidCoreModel(error.to_string()))?;
    context.validate()?;
    validate_okh_passthrough(&passthrough)?;

    let [license] = project.license_refs.as_slice() else {
        return Err(InteropError::AmbiguousLicense);
    };

    let manifest = OpenKnowHow24Manifest {
        okhv: OKH_24_VERSION.into(),
        name: project.name.clone(),
        repo: context.repo,
        license: license.identifier.clone(),
        licensor: context.licensor,
        function_description: context.function_description,
        version: context.version,
        extra: passthrough,
    };

    let mut report = InteropReport::new(
        InteropTarget::OpenKnowHow24,
        InteropDirection::Export,
        AdapterVersion::stable(OKH_24_VERSION),
    );
    report.record(
        "HardwareProject.name",
        Some("name"),
        InteropDisposition::Preserved,
        "project name maps directly",
    );
    report.record(
        "HardwareProject.license_refs[0]",
        Some("license"),
        InteropDisposition::Preserved,
        "single license expression maps directly",
    );
    report.record(
        "OpenKnowHowExternalContext.repo",
        Some("repo"),
        InteropDisposition::Preserved,
        "caller supplied the target-required development repository",
    );
    report.record(
        "OpenKnowHowExternalContext.licensor",
        Some("licensor"),
        InteropDisposition::Preserved,
        "caller supplied licensor without conflating it with maintainer identity",
    );
    report.record(
        "OpenKnowHowExternalContext.function_description",
        Some("function"),
        InteropDisposition::Preserved,
        "caller supplied the target-required functional description",
    );
    for key in manifest.extra.keys() {
        report.record(
            format!("passthrough.{key}"),
            Some(key.clone()),
            InteropDisposition::PreservedOpaque,
            "opaque source field re-emitted unchanged",
        );
    }

    Ok((manifest, report))
}

// -----------------------------------------------------------------------------
// CycloneDX 1.7 component subset
// -----------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CycloneDx17Bom {
    #[serde(rename = "bomFormat")]
    pub bom_format: String,
    #[serde(rename = "specVersion")]
    pub spec_version: String,
    #[serde(default = "default_bom_version")]
    pub version: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub metadata: Option<Value>,
    #[serde(default)]
    pub components: Vec<CycloneDx17Component>,
    #[serde(flatten)]
    pub extra: BTreeMap<String, Value>,
}

fn default_bom_version() -> u32 {
    1
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CycloneDx17Component {
    #[serde(rename = "type")]
    pub component_type: String,
    #[serde(rename = "bom-ref")]
    pub bom_ref: String,
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<CycloneDxOrganization>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub properties: Vec<CycloneDxProperty>,
    #[serde(flatten)]
    pub extra: BTreeMap<String, Value>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CycloneDxOrganization {
    pub name: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CycloneDxProperty {
    pub name: String,
    pub value: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ImportedCycloneDxComponents {
    pub components: Vec<ComponentIdentity>,
    /// Unsupported component classes are kept intact rather than discarded.
    pub opaque_components: Vec<CycloneDx17Component>,
    pub metadata: Option<Value>,
    pub passthrough: BTreeMap<String, Value>,
    pub report: InteropReport,
}

pub fn export_cyclonedx_17_components(
    components: &[ComponentIdentity],
) -> Result<(CycloneDx17Bom, InteropReport), InteropError> {
    let mut report = InteropReport::new(
        InteropTarget::CycloneDx17,
        InteropDirection::Export,
        AdapterVersion::stable(CYCLONEDX_17_VERSION),
    );
    let mut exported = Vec::new();

    for component in components {
        component
            .validate()
            .map_err(|error| InteropError::InvalidCoreModel(error.to_string()))?;

        let Some(component_type) = cyclonedx_type_for_component(&component.kind) else {
            report.record(
                format!("ComponentIdentity[{}].kind", component.id),
                None::<String>,
                InteropDisposition::Unsupported,
                "the stable 1.7 adapter does not coerce this physical component class into CycloneDX 'device'",
            );
            continue;
        };

        let mut properties = Vec::new();
        if let Some(mpn) = &component.manufacturer_part_number {
            properties.push(CycloneDxProperty {
                name: CDX_MPN_PROPERTY.into(),
                value: mpn.clone(),
            });
            report.record(
                format!("ComponentIdentity[{}].manufacturer_part_number", component.id),
                Some(format!("components[{}].properties[{CDX_MPN_PROPERTY}]", component.id)),
                InteropDisposition::Approximated,
                "manufacturer part number is preserved in a namespaced CycloneDX property",
            );
        }

        properties.push(CycloneDxProperty {
            name: CDX_LIFECYCLE_PROPERTY.into(),
            value: lifecycle_to_string(component.lifecycle).into(),
        });
        report.record(
            format!("ComponentIdentity[{}].lifecycle", component.id),
            Some(format!("components[{}].properties[{CDX_LIFECYCLE_PROPERTY}]", component.id)),
            InteropDisposition::Approximated,
            "Mycelix lifecycle state is carried as a namespaced property",
        );

        if !component.datasheet_refs.is_empty() {
            report.record(
                format!("ComponentIdentity[{}].datasheet_refs", component.id),
                None::<String>,
                InteropDisposition::Omitted,
                "the first CycloneDX slice does not convert digest-only datasheet references into URLs",
            );
        }

        exported.push(CycloneDx17Component {
            component_type: component_type.into(),
            bom_ref: component.id.to_string(),
            name: component.name.clone(),
            manufacturer: component
                .manufacturer
                .as_ref()
                .map(|name| CycloneDxOrganization { name: name.clone() }),
            properties,
            extra: BTreeMap::new(),
        });

        report.record(
            format!("ComponentIdentity[{}].name", component.id),
            Some(format!("components[{}].name", component.id)),
            InteropDisposition::Preserved,
            "component name maps directly",
        );
        report.record(
            format!("ComponentIdentity[{}].kind", component.id),
            Some(format!("components[{}].type", component.id)),
            InteropDisposition::Approximated,
            "supported physical classes map to CycloneDX 1.7 'device'",
        );
        if component.manufacturer.is_some() {
            report.record(
                format!("ComponentIdentity[{}].manufacturer", component.id),
                Some(format!("components[{}].manufacturer.name", component.id)),
                InteropDisposition::Preserved,
                "manufacturer name maps directly",
            );
        }
    }

    Ok((
        CycloneDx17Bom {
            bom_format: "CycloneDX".into(),
            spec_version: CYCLONEDX_17_VERSION.into(),
            version: 1,
            metadata: Some(serde_json::json!({
                "lifecycles": [{ "phase": "design" }]
            })),
            components: exported,
            extra: BTreeMap::new(),
        },
        report,
    ))
}

pub fn import_cyclonedx_17_components(
    bom: CycloneDx17Bom,
) -> Result<ImportedCycloneDxComponents, InteropError> {
    if bom.bom_format != "CycloneDX" {
        return Err(InteropError::InvalidCycloneDxFormat);
    }
    if bom.spec_version != CYCLONEDX_17_VERSION {
        return Err(InteropError::UnsupportedTargetVersion(bom.spec_version));
    }

    let mut report = InteropReport::new(
        InteropTarget::CycloneDx17,
        InteropDirection::Import,
        AdapterVersion::stable(CYCLONEDX_17_VERSION),
    );
    let mut components = Vec::new();
    let mut opaque_components = Vec::new();

    for source in bom.components {
        if source.component_type != "device" {
            report.record(
                format!("components[{}].type", source.bom_ref),
                None::<String>,
                InteropDisposition::Unsupported,
                "non-device CycloneDX component preserved opaquely by the first hardware adapter slice",
            );
            opaque_components.push(source);
            continue;
        }

        let id = SemanticId::new(source.bom_ref.clone())
            .map_err(|error| InteropError::InvalidSemanticId(error.to_string()))?;
        validate_nonempty(&source.name, "component.name")?;

        let mpn = property_value(&source.properties, CDX_MPN_PROPERTY).map(ToOwned::to_owned);
        let lifecycle = match property_value(&source.properties, CDX_LIFECYCLE_PROPERTY) {
            Some(value) => parse_lifecycle(value, &mut report, &source.bom_ref),
            None => {
                report.record(
                    format!("components[{}].properties", source.bom_ref),
                    Some(format!("ComponentIdentity[{}].lifecycle", source.bom_ref)),
                    InteropDisposition::UnknownSourceValue,
                    "no Mycelix lifecycle property was present; imported lifecycle remains Unknown",
                );
                LifecycleState::Unknown
            }
        };

        let component = ComponentIdentity {
            schema_version: HARDWARE_SEMANTIC_SCHEMA.into(),
            id,
            name: source.name.clone(),
            kind: ComponentKind::Other(format!("CycloneDX:{}", source.component_type)),
            manufacturer: source.manufacturer.as_ref().map(|org| org.name.clone()),
            manufacturer_part_number: mpn,
            open_hardware_project: None,
            datasheet_refs: Vec::new(),
            lifecycle,
        };
        component
            .validate()
            .map_err(|error| InteropError::InvalidCoreModel(error.to_string()))?;

        report.record(
            format!("components[{}].name", source.bom_ref),
            Some(format!("ComponentIdentity[{}].name", source.bom_ref)),
            InteropDisposition::Preserved,
            "component name maps directly",
        );
        report.record(
            format!("components[{}].type", source.bom_ref),
            Some(format!("ComponentIdentity[{}].kind", source.bom_ref)),
            InteropDisposition::Approximated,
            "CycloneDX 'device' is retained as an explicit source-namespaced kind instead of guessing an electrical/mechanical subtype",
        );
        if source.manufacturer.is_some() {
            report.record(
                format!("components[{}].manufacturer.name", source.bom_ref),
                Some(format!("ComponentIdentity[{}].manufacturer", source.bom_ref)),
                InteropDisposition::Preserved,
                "manufacturer name maps directly",
            );
        }
        if !source.extra.is_empty() {
            report.record(
                format!("components[{}].*", source.bom_ref),
                None::<String>,
                InteropDisposition::PreservedOpaque,
                "unmodeled component fields remain inside the opaque source component representation only when the whole component is unsupported",
            );
        }

        components.push(component);
    }

    Ok(ImportedCycloneDxComponents {
        components,
        opaque_components,
        metadata: bom.metadata,
        passthrough: bom.extra,
        report,
    })
}

fn cyclonedx_type_for_component(kind: &ComponentKind) -> Option<&'static str> {
    match kind {
        ComponentKind::Electronic | ComponentKind::Electromechanical | ComponentKind::Module => {
            Some("device")
        }
        ComponentKind::Mechanical
        | ComponentKind::Optical
        | ComponentKind::Fluidic
        | ComponentKind::Material
        | ComponentKind::Fastener
        | ComponentKind::Cable
        | ComponentKind::Other(_) => None,
    }
}

fn lifecycle_to_string(state: LifecycleState) -> &'static str {
    match state {
        LifecycleState::Unknown => "unknown",
        LifecycleState::Active => "active",
        LifecycleState::NotRecommendedForNewDesigns => "not-recommended-for-new-designs",
        LifecycleState::EndOfLife => "end-of-life",
        LifecycleState::Obsolete => "obsolete",
    }
}

fn parse_lifecycle(value: &str, report: &mut InteropReport, bom_ref: &str) -> LifecycleState {
    match value {
        "unknown" => LifecycleState::Unknown,
        "active" => LifecycleState::Active,
        "not-recommended-for-new-designs" => LifecycleState::NotRecommendedForNewDesigns,
        "end-of-life" => LifecycleState::EndOfLife,
        "obsolete" => LifecycleState::Obsolete,
        other => {
            report.record(
                format!("components[{bom_ref}].properties[{CDX_LIFECYCLE_PROPERTY}]"),
                Some(format!("ComponentIdentity[{bom_ref}].lifecycle")),
                InteropDisposition::UnknownSourceValue,
                format!("unrecognized lifecycle value '{other}' retained as Unknown"),
            );
            LifecycleState::Unknown
        }
    }
}

fn property_value<'a>(properties: &'a [CycloneDxProperty], name: &str) -> Option<&'a str> {
    properties
        .iter()
        .find(|property| property.name == name)
        .map(|property| property.value.as_str())
}

fn validate_okh_passthrough(values: &BTreeMap<String, Value>) -> Result<(), InteropError> {
    const RESERVED: [&str; 7] = [
        "okhv",
        "name",
        "repo",
        "license",
        "licensor",
        "function",
        "version",
    ];
    if let Some(key) = values.keys().find(|key| RESERVED.contains(&key.as_str())) {
        return Err(InteropError::ReservedPassthroughKey(key.clone()));
    }
    Ok(())
}

fn validate_nonempty(value: &str, field: &'static str) -> Result<(), InteropError> {
    if value.trim().is_empty() || value != value.trim() || value.chars().any(char::is_control) {
        return Err(InteropError::InvalidRequiredField(field));
    }
    Ok(())
}

fn validate_http_url(value: &str, field: &'static str) -> Result<(), InteropError> {
    validate_nonempty(value, field)?;
    if !(value.starts_with("https://") || value.starts_with("http://"))
        || value.chars().any(char::is_whitespace)
    {
        return Err(InteropError::InvalidRequiredField(field));
    }
    Ok(())
}

fn validate_licensor(value: &Value) -> Result<(), InteropError> {
    match value {
        Value::String(text) => validate_nonempty(text, "licensor"),
        Value::Array(values) if !values.is_empty() => {
            if values.iter().any(Value::is_null) {
                return Err(InteropError::InvalidRequiredField("licensor"));
            }
            Ok(())
        }
        Value::Object(map) if !map.is_empty() => Ok(()),
        _ => Err(InteropError::InvalidRequiredField("licensor")),
    }
}

#[cfg(feature = "experimental-spdx-3-1-dev")]
pub mod spdx31_dev {
    use super::*;

    /// Minimal marker emitted only to make draft status explicit. This is not a
    /// claim of SPDX 3.1 conformance and deliberately does not expose a stable
    /// adapter API until the specification is released.
    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    pub struct ExperimentalProfileDeclaration {
        pub specification: String,
        pub profile_conformance: Vec<String>,
        pub experimental: bool,
    }

    pub fn profile_declaration() -> ExperimentalProfileDeclaration {
        ExperimentalProfileDeclaration {
            specification: "SPDX-3.1-dev".into(),
            profile_conformance: vec!["core".into(), "hardware".into()],
            experimental: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> SemanticId {
        SemanticId::new(value).unwrap()
    }

    fn project() -> HardwareProject {
        HardwareProject {
            schema_version: HARDWARE_SEMANTIC_SCHEMA.into(),
            id: id("project:microscope"),
            name: "Open Microscope".into(),
            description: Some("Project overview".into()),
            maintainers: Vec::new(),
            license_refs: vec![LicenseRef {
                identifier: "CERN-OHL-S-2.0".into(),
                text_or_url: None,
            }],
        }
    }

    fn component(kind: ComponentKind) -> ComponentIdentity {
        ComponentIdentity {
            schema_version: HARDWARE_SEMANTIC_SCHEMA.into(),
            id: id("component:controller"),
            name: "Controller".into(),
            kind,
            manufacturer: Some("Example Semiconductor".into()),
            manufacturer_part_number: Some("EX-42".into()),
            open_hardware_project: None,
            datasheet_refs: Vec::new(),
            lifecycle: LifecycleState::Unknown,
        }
    }

    fn okh_context() -> OpenKnowHowExternalContext {
        OpenKnowHowExternalContext {
            repo: "https://example.org/open-microscope".into(),
            licensor: Value::String("Example Foundation".into()),
            function_description: "Provides an open optical microscopy platform".into(),
            version: Some("1.2.0".into()),
        }
    }

    #[test]
    fn report_treats_invented_values_as_hard_errors() {
        let mut report = InteropReport::new(
            InteropTarget::OpenKnowHow24,
            InteropDirection::Export,
            AdapterVersion::stable(OKH_24_VERSION),
        );
        report.record(
            "missing",
            Some("required"),
            InteropDisposition::InventedValue,
            "test",
        );
        assert!(report.has_hard_error());
        assert!(report.has_loss());
    }

    #[test]
    fn okh_import_does_not_conflate_function_with_description() {
        let manifest = OpenKnowHow24Manifest {
            okhv: OKH_24_VERSION.into(),
            name: "Open Microscope".into(),
            repo: "https://example.org/open-microscope".into(),
            license: "CERN-OHL-S-2.0".into(),
            licensor: Value::String("Example Foundation".into()),
            function_description: "Provides microscopy".into(),
            version: Some("1.0.0".into()),
            extra: BTreeMap::new(),
        };
        let imported = import_open_know_how_24(manifest, id("project:microscope")).unwrap();
        assert_eq!(imported.project.description, None);
        assert_eq!(
            imported.external_context.function_description,
            "Provides microscopy"
        );
        assert!(imported.report.findings.iter().any(|finding| {
            finding.source_path == "function"
                && finding.disposition == InteropDisposition::PreservedOpaque
        }));
    }

    #[test]
    fn okh_export_requires_one_unambiguous_license() {
        let mut value = project();
        value.license_refs.clear();
        assert_eq!(
            export_open_know_how_24(&value, okh_context(), BTreeMap::new()).unwrap_err(),
            InteropError::AmbiguousLicense
        );

        value.license_refs.push(LicenseRef {
            identifier: "MIT".into(),
            text_or_url: None,
        });
        value.license_refs.push(LicenseRef {
            identifier: "Apache-2.0".into(),
            text_or_url: None,
        });
        assert_eq!(
            export_open_know_how_24(&value, okh_context(), BTreeMap::new()).unwrap_err(),
            InteropError::AmbiguousLicense
        );
    }

    #[test]
    fn okh_passthrough_cannot_override_normative_fields() {
        let mut passthrough = BTreeMap::new();
        passthrough.insert("license".into(), Value::String("MIT".into()));
        assert_eq!(
            export_open_know_how_24(&project(), okh_context(), passthrough).unwrap_err(),
            InteropError::ReservedPassthroughKey("license".into())
        );
    }

    #[test]
    fn cyclonedx_export_refuses_to_call_fastener_a_device() {
        let (bom, report) = export_cyclonedx_17_components(&[component(ComponentKind::Fastener)])
            .unwrap();
        assert!(bom.components.is_empty());
        assert!(report.findings.iter().any(|finding| {
            finding.disposition == InteropDisposition::Unsupported
                && finding.source_path.ends_with(".kind")
        }));
    }

    #[test]
    fn cyclonedx_device_import_does_not_guess_mycelix_subtype() {
        let source = CycloneDx17Bom {
            bom_format: "CycloneDX".into(),
            spec_version: CYCLONEDX_17_VERSION.into(),
            version: 1,
            metadata: None,
            components: vec![CycloneDx17Component {
                component_type: "device".into(),
                bom_ref: "component:external".into(),
                name: "External Device".into(),
                manufacturer: None,
                properties: Vec::new(),
                extra: BTreeMap::new(),
            }],
            extra: BTreeMap::new(),
        };
        let imported = import_cyclonedx_17_components(source).unwrap();
        assert_eq!(imported.components.len(), 1);
        assert_eq!(
            imported.components[0].kind,
            ComponentKind::Other("CycloneDX:device".into())
        );
        assert_eq!(imported.components[0].lifecycle, LifecycleState::Unknown);
    }

    #[test]
    fn unknown_lifecycle_value_stays_unknown() {
        let source = CycloneDx17Bom {
            bom_format: "CycloneDX".into(),
            spec_version: CYCLONEDX_17_VERSION.into(),
            version: 1,
            metadata: None,
            components: vec![CycloneDx17Component {
                component_type: "device".into(),
                bom_ref: "component:external".into(),
                name: "External Device".into(),
                manufacturer: None,
                properties: vec![CycloneDxProperty {
                    name: CDX_LIFECYCLE_PROPERTY.into(),
                    value: "mysterious-state".into(),
                }],
                extra: BTreeMap::new(),
            }],
            extra: BTreeMap::new(),
        };
        let imported = import_cyclonedx_17_components(source).unwrap();
        assert_eq!(imported.components[0].lifecycle, LifecycleState::Unknown);
        assert!(imported.report.findings.iter().any(|finding| {
            finding.disposition == InteropDisposition::UnknownSourceValue
        }));
    }

    #[test]
    fn cyclonedx_round_trip_keeps_mpn_in_namespaced_property() {
        let original = component(ComponentKind::Electronic);
        let (bom, export_report) = export_cyclonedx_17_components(&[original]).unwrap();
        assert_eq!(bom.components.len(), 1);
        assert!(export_report.has_loss());

        let imported = import_cyclonedx_17_components(bom).unwrap();
        assert_eq!(
            imported.components[0].manufacturer_part_number.as_deref(),
            Some("EX-42")
        );
    }
}
