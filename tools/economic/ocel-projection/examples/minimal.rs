use std::collections::BTreeMap;

use mycelix_economic_ocel_projection::{
    AttributeDeclaration, EventRecord, EventType, ObjectRecord, ObjectType, OcelValue,
    ProjectionInput, QualifiedRelation, QualifiedTime, TimeAssertionMetadata, TimePrecision,
    TimedObjectAttribute, ValueKind,
};

fn declaration(name: &str, kind: ValueKind) -> AttributeDeclaration {
    AttributeDeclaration { name: name.into(), kind }
}

fn main() {
    let mut event_attributes = BTreeMap::new();
    event_attributes.insert("quantity".into(), OcelValue::Integer(250));
    event_attributes.insert(
        "unit_profile".into(),
        OcelValue::String("filter_cartridge".into()),
    );

    let input = ProjectionInput {
        mapping_profile_id: "economic-os-ocel-crosswalk-v0.1".into(),
        event_types: vec![EventType {
            name: "goods_received".into(),
            attributes: vec![
                declaration("time_basis", ValueKind::String),
                declaration("time_evidence_ref", ValueKind::String),
                declaration("time_precision", ValueKind::String),
                declaration("time_quality", ValueKind::String),
                declaration("source_event_ref", ValueKind::String),
                declaration("quantity", ValueKind::Integer),
                declaration("unit_profile", ValueKind::String),
            ],
        }],
        object_types: vec![ObjectType {
            name: "DeliveryLot".into(),
            attributes: vec![
                declaration("quantity", ValueKind::Integer),
                declaration("unit_profile", ValueKind::String),
            ],
        }],
        events: vec![EventRecord {
            id: "e-receipt".into(),
            event_type: "goods_received".into(),
            time: QualifiedTime::source_event_time(
                "2026-11-21T13:00:00Z",
                "source:receipt:time",
                TimeAssertionMetadata::source_asserted(
                    "receiver-device-utc-v1",
                    "warehouse:receiver-1:goods-receipt-authority",
                    TimePrecision::Second,
                ),
            ),
            attributes: event_attributes,
            source_event_ref: "source:goods-receipt:1".into(),
            relationships: vec![QualifiedRelation {
                object_id: "LOT-001".into(),
                qualifier: "received_lot".into(),
            }],
        }],
        objects: vec![ObjectRecord {
            id: "LOT-001".into(),
            object_type: "DeliveryLot".into(),
            attributes: vec![
                TimedObjectAttribute {
                    name: "quantity".into(),
                    time: QualifiedTime::source_attribute_time(
                        "2026-11-21T10:00:00Z",
                        "source:lot:quantity-time",
                        TimeAssertionMetadata::source_asserted(
                            "supplier-device-utc-v1",
                            "supplier:sup-001:delivery-lot-authority",
                            TimePrecision::Second,
                        ),
                    ),
                    value: OcelValue::Integer(250),
                },
                TimedObjectAttribute {
                    name: "unit_profile".into(),
                    time: QualifiedTime::source_attribute_time(
                        "2026-11-21T10:00:00Z",
                        "source:lot:unit-time",
                        TimeAssertionMetadata::source_asserted(
                            "supplier-device-utc-v1",
                            "supplier:sup-001:delivery-lot-authority",
                            TimePrecision::Second,
                        ),
                    ),
                    value: OcelValue::String("filter_cartridge".into()),
                },
            ],
            relationships: vec![],
        }],
    };
    let artifacts = input.export().expect("synthetic source-evidence fixture is valid");
    if std::env::args().any(|argument| argument == "--provenance") {
        println!("{}", artifacts.time_provenance_json);
    } else {
        println!("{}", artifacts.ocel_json);
    }
}
