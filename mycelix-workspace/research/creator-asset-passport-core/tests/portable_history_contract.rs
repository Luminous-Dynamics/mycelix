use creator_asset_passport_core::history::{GrantEvent, GrantHistory};
use creator_asset_passport_core::portable::{GrantHistoryEnvelope, PortableHistoryError};
use creator_asset_passport_core::{AssetId, EventId, GrantId, PartyId, UsageGrant, UseAction};

fn party(value: &str) -> PartyId {
    PartyId::new(value).unwrap()
}

fn history_with_issue_evidence(issue_evidence: &str) -> GrantHistory {
    let grant = UsageGrant::new(
        GrantId::new("grant-portable").unwrap(),
        AssetId::new("recording-portable").unwrap(),
        party("studio"),
        UseAction::TrainModel,
        "private-model-v1",
        party("creator"),
        "consent://training",
        10,
        Some(100),
    )
    .unwrap();
    let event = GrantEvent::issue(
        EventId::new("event-issue").unwrap(),
        5,
        party("creator"),
        issue_evidence,
        grant,
    )
    .unwrap();
    let revoke = GrantEvent::revoke(
        EventId::new("event-revoke").unwrap(),
        50,
        party("creator"),
        "evidence://revoke",
        GrantId::new("grant-portable").unwrap(),
    )
    .unwrap();
    GrantHistory::replay(&[event, revoke]).unwrap()
}

fn history_with_grant() -> GrantHistory {
    history_with_issue_evidence("evidence://issue")
}

#[test]
fn portable_envelope_round_trip_preserves_effective_revocation() {
    let envelope = GrantHistoryEnvelope::from_history(&history_with_grant()).unwrap();
    let digest = envelope.digest_hex().unwrap();
    let bytes = envelope.canonical_bytes().unwrap();

    let imported = GrantHistoryEnvelope::from_bytes(&bytes).unwrap();
    imported.verify_digest(&digest).unwrap();
    let imported_history = imported.replay().unwrap();
    let imported_grant = imported_history
        .find_grant(&GrantId::new("grant-portable").unwrap())
        .unwrap();

    assert_eq!(imported.event_count(), 2);
    assert_eq!(imported_grant.revocation_timestamp(), Some(50));
    assert_eq!(imported.canonical_bytes().unwrap(), bytes);
}

#[test]
fn different_event_evidence_changes_the_content_digest() {
    let original =
        GrantHistoryEnvelope::from_history(&history_with_issue_evidence("evidence://issue"))
            .unwrap();
    let changed_source = GrantHistoryEnvelope::from_history(&history_with_issue_evidence(
        "evidence://changed-source",
    ))
    .unwrap();

    assert_ne!(
        original.digest_hex().unwrap(),
        changed_source.digest_hex().unwrap()
    );
}

#[test]
fn envelope_rejects_unknown_use_action_tag() {
    let envelope = GrantHistoryEnvelope::from_history(&history_with_grant()).unwrap();
    let mut bytes = envelope.canonical_bytes().unwrap();
    // Locate the stable TrainModel action tag in the canonical issue payload.
    // The action tag follows grant id, asset id, and grantee id length-prefixed strings.
    let magic_len = b"MYCELIX-CSCP-GRANT-HISTORY\0".len();
    let mut cursor = magic_len + 2 + 8;
    let advance_string = |cursor: &mut usize| {
        let len = u32::from_be_bytes(bytes[*cursor..*cursor + 4].try_into().unwrap()) as usize;
        *cursor += 4 + len;
    };
    advance_string(&mut cursor); // event ID
    cursor += 8; // event timestamp
    advance_string(&mut cursor); // actor
    advance_string(&mut cursor); // event evidence reference
    cursor += 1; // issued-event tag
    advance_string(&mut cursor); // grant ID
    advance_string(&mut cursor); // asset ID
    advance_string(&mut cursor); // grantee ID
    let action_tag = cursor;
    assert_eq!(
        GrantHistoryEnvelope::from_bytes(&bytes),
        Ok(envelope.clone())
    );
    bytes[action_tag] = 255;
    assert!(matches!(
        GrantHistoryEnvelope::from_bytes(&bytes),
        Err(PortableHistoryError::InvalidEventKind)
            | Err(PortableHistoryError::InvalidUseAction)
            | Err(PortableHistoryError::InvalidHistory(_))
    ));
}

#[test]
fn trailing_bytes_are_not_ignored() {
    let envelope = GrantHistoryEnvelope::from_history(&history_with_grant()).unwrap();
    let mut bytes = envelope.canonical_bytes().unwrap();
    bytes.extend_from_slice(b"extra");
    assert_eq!(
        GrantHistoryEnvelope::from_bytes(&bytes),
        Err(PortableHistoryError::TrailingBytes)
    );
}
