// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Keys Integrity Zome
//!
//! Pre-key bundles for E2E encryption key exchange.
//!
//! Entry/link type registration and validation live here; the plain data
//! types themselves live in `keys_types` (re-exported below for existing
//! callers) so other zomes can depend on the types without pulling in this
//! crate's `#[hdk_extern]` zome surface — see that crate's doc comment for
//! why sharing an integrity crate directly breaks WASM linking.

use hdi::prelude::*;
pub use keys_types::{
    HYBRID_KEY_BUNDLE_V2, HYBRID_SUITE_V2, HybridKeyBundleV2, HybridKeyStateV2, KeyRotation,
    ML_DSA_65_PUBLIC_KEY_BYTES, ML_KEM_768_PUBLIC_KEY_BYTES, OneTimePreKey, PreKeyBundle,
    RotationReason, UsedPreKey, hybrid_key_id, hybrid_key_signing_content, pre_key_signing_content,
};

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    PreKeyBundle(PreKeyBundle),
    HybridKeyBundleV2(HybridKeyBundleV2),
    UsedPreKey(UsedPreKey),
    KeyRotation(KeyRotation),
}

fn validate_create_hybrid_key_bundle_v2(
    action: Create,
    bundle: HybridKeyBundleV2,
) -> ExternResult<ValidateCallbackResult> {
    if bundle.version != HYBRID_KEY_BUNDLE_V2 || bundle.suite != HYBRID_SUITE_V2 {
        return Ok(ValidateCallbackResult::Invalid(
            "Unsupported hybrid key bundle version or suite".into(),
        ));
    }
    if bundle.key_id != hybrid_key_id(&bundle) {
        return Ok(ValidateCallbackResult::Invalid(
            "Hybrid key ID does not match its public-key transcript".into(),
        ));
    }
    if bundle.ml_kem_768_public_key.len() != ML_KEM_768_PUBLIC_KEY_BYTES {
        return Ok(ValidateCallbackResult::Invalid(
            "ML-KEM-768 public key must be 1184 bytes".into(),
        ));
    }
    if bundle.ml_dsa_65_public_key.len() != ML_DSA_65_PUBLIC_KEY_BYTES {
        return Ok(ValidateCallbackResult::Invalid(
            "ML-DSA-65 public key must be 1952 bytes".into(),
        ));
    }
    if bundle.expires_at <= bundle.created_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Expiration must be after creation time".into(),
        ));
    }
    if bundle.agent_signature.len() != 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "Agent signature must be 64 bytes".into(),
        ));
    }
    let mut signature = [0; 64];
    signature.copy_from_slice(&bundle.agent_signature);
    if !verify_signature_raw(
        action.author,
        Signature(signature),
        hybrid_key_signing_content(&bundle),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "Hybrid key bundle agent signature verification failed".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_link_types]
pub enum LinkTypes {
    AgentToBundle,
    BundleToUsedKeys,
    KeyRotations,
}

/// Re-derives consume_pre_key's own real invariant at the DHT level: exactly one
/// previously-unused one_time_pre_key may flip to used=true (its key_id/public_key
/// unchanged), no pre-keys may be added/removed, no already-used key may be un-used, and
/// every other field must stay byte-identical to the original.
fn validate_update_pre_key_bundle(
    bundle: PreKeyBundle,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    if bundle.identity_key.len() != 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "Identity key must be 32 bytes".to_string(),
        ));
    }

    let original_record = must_get_valid_record(original_action_hash)?;
    let Some(original): Option<PreKeyBundle> = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
    else {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid original PreKeyBundle entry".to_string(),
        ));
    };

    if bundle.identity_key != original.identity_key
        || bundle.signed_pre_key != original.signed_pre_key
        || bundle.signed_pre_key_id != original.signed_pre_key_id
        || bundle.signed_pre_key_signature != original.signed_pre_key_signature
        || bundle.created_at != original.created_at
        || bundle.expires_at != original.expires_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "PreKeyBundle updates may only mark a one-time pre-key as used -- all other \
             fields must be unchanged"
                .to_string(),
        ));
    }

    if bundle.one_time_pre_keys.len() != original.one_time_pre_keys.len() {
        return Ok(ValidateCallbackResult::Invalid(
            "PreKeyBundle updates cannot add or remove one-time pre-keys".to_string(),
        ));
    }

    let mut newly_used_count = 0u32;
    for (new_key, old_key) in bundle
        .one_time_pre_keys
        .iter()
        .zip(original.one_time_pre_keys.iter())
    {
        if new_key.key_id != old_key.key_id || new_key.public_key != old_key.public_key {
            return Ok(ValidateCallbackResult::Invalid(
                "One-time pre-key id/public_key cannot change".to_string(),
            ));
        }
        match (old_key.used, new_key.used) {
            (false, false) | (true, true) => {}
            (false, true) => newly_used_count += 1,
            (true, false) => {
                return Ok(ValidateCallbackResult::Invalid(
                    "A one-time pre-key cannot be marked unused once consumed".to_string(),
                ));
            }
        }
    }

    if newly_used_count != 1 {
        return Ok(ValidateCallbackResult::Invalid(
            "PreKeyBundle updates must mark exactly one previously-unused one-time \
             pre-key as used"
                .to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate pre-key bundle
fn validate_create_pre_key_bundle(
    action: Create,
    bundle: PreKeyBundle,
) -> ExternResult<ValidateCallbackResult> {
    // Validate identity key length (32 bytes for X25519)
    if bundle.identity_key.len() != 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "Identity key must be 32 bytes".to_string(),
        ));
    }

    // Validate signed pre-key length
    if bundle.signed_pre_key.len() != 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "Signed pre-key must be 32 bytes".to_string(),
        ));
    }

    // Validate signature length (64 bytes for Ed25519)
    if bundle.signed_pre_key_signature.len() != 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "Signature must be 64 bytes".to_string(),
        ));
    }
    let mut signature = [0u8; 64];
    signature.copy_from_slice(&bundle.signed_pre_key_signature);
    if !verify_signature_raw(
        action.author,
        Signature(signature),
        pre_key_signing_content(&bundle),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "Signed pre-key signature verification failed".to_string(),
        ));
    }

    // Validate at least some one-time pre-keys
    if bundle.one_time_pre_keys.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Bundle must have at least one one-time pre-key".to_string(),
        ));
    }

    // Validate one-time pre-key lengths
    for otpk in &bundle.one_time_pre_keys {
        if otpk.public_key.len() != 32 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "One-time pre-key {} must be 32 bytes",
                otpk.key_id
            )));
        }
    }

    // Validate expiration is after creation
    if bundle.expires_at <= bundle.created_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Expiration must be after creation time".to_string(),
        ));
    }

    // Note: Expiration check against current time is done in coordinator zome
    // since sys_time() is not available in integrity zomes

    Ok(ValidateCallbackResult::Valid)
}

/// Validate used pre-key record -- bind to its committer. consume_pre_key already derives
/// used_by from agent_info() coordinator-side with zero user input (P0 author-binding gap).
/// Note: consume_pre_key legitimately calls update_entry on the OTHER agent's PreKeyBundle
/// (X3DH protocol -- the consumer marks the bundle owner's one-time key used) -- that
/// cross-agent update path is a real, deliberate exception and is NOT touched here.
fn validate_create_used_pre_key(
    action: Create,
    used: UsedPreKey,
) -> ExternResult<ValidateCallbackResult> {
    // Basic validation
    if used.key_id == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Key ID cannot be 0".to_string(),
        ));
    }

    if used.used_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "UsedPreKey must be recorded by the consuming agent (used_by forgery)".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate key rotation
fn validate_create_key_rotation(
    _action: Create,
    rotation: KeyRotation,
) -> ExternResult<ValidateCallbackResult> {
    // Old and new bundles must be different
    if rotation.old_bundle_hash == rotation.new_bundle_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Old and new bundle hashes must be different".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Main validation dispatcher
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::PreKeyBundle(bundle) => validate_create_pre_key_bundle(action, bundle),
                EntryTypes::HybridKeyBundleV2(bundle) => {
                    validate_create_hybrid_key_bundle_v2(action, bundle)
                }
                EntryTypes::UsedPreKey(used) => validate_create_used_pre_key(action, used),
                EntryTypes::KeyRotation(rotation) => validate_create_key_rotation(action, rotation),
            },
            OpEntry::UpdateEntry {
                app_entry,
                original_action_hash,
                ..
            } => match app_entry {
                // PreKeyBundle updates are a genuine, deliberate cross-agent exception
                // (X3DH protocol -- consume_pre_key's caller is the key CONSUMER, marking
                // the bundle OWNER's one-time key used), so this is NOT an author-binding
                // check. Instead it re-derives consume_pre_key's own real invariant: only
                // one previously-unused one_time_pre_key may flip to used=true, its
                // key_id/public_key must be unchanged, and every other field (including
                // every other pre-key) must stay byte-identical. Previously any modified
                // coordinator could rewrite the whole bundle -- add/remove keys, un-use a
                // consumed key, or swap key material -- since only identity_key's length
                // was checked (P0 author-binding Turn B, keys.PreKeyBundle).
                EntryTypes::PreKeyBundle(bundle) => {
                    validate_update_pre_key_bundle(bundle, original_action_hash)
                }
                EntryTypes::HybridKeyBundleV2(_) => Ok(ValidateCallbackResult::Invalid(
                    "Hybrid V2 bundles are immutable; publish a successor bundle".into(),
                )),
                _ => Ok(ValidateCallbackResult::Valid),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

/// Proves `validate_update_pre_key_bundle`'s real invariant: a modified coordinator can no
/// longer rewrite the whole bundle -- only marking exactly one previously-unused
/// one-time pre-key as used is accepted; adding/removing keys, un-using a consumed key, or
/// swapping key material is rejected. Previously only `identity_key`'s length was checked.
/// Mocks the HDI host's `must_get_valid_record` so this runs as a plain `cargo test`, no
/// live conductor needed.
#[cfg(test)]
mod tests {
    use super::*;

    struct MockRecordHdi {
        records: std::collections::HashMap<ActionHash, Record>,
    }

    impl hdi::hdi::HdiT for MockRecordHdi {
        fn must_get_valid_record(&self, input: MustGetValidRecordInput) -> ExternResult<Record> {
            self.records
                .get(&input.0)
                .cloned()
                .ok_or_else(|| wasm_error!(WasmErrorInner::Guest("no such record in mock".into())))
        }
        fn verify_signature(&self, _: VerifySignature) -> ExternResult<bool> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_entry(&self, _: MustGetEntryInput) -> ExternResult<EntryHashed> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_action(&self, _: MustGetActionInput) -> ExternResult<SignedActionHashed> {
            unimplemented!("not exercised by this fix")
        }
        fn must_get_agent_activity(
            &self,
            _: MustGetAgentActivityInput,
        ) -> ExternResult<Vec<RegisterAgentActivity>> {
            unimplemented!("not exercised by this fix")
        }
        fn dna_info(&self, _: ()) -> ExternResult<DnaInfo> {
            unimplemented!("not exercised by this fix")
        }
        fn zome_info(&self, _: ()) -> ExternResult<ZomeInfo> {
            unimplemented!("not exercised by this fix")
        }
        fn trace(&self, _: TraceMsg) -> ExternResult<()> {
            unimplemented!("not exercised by this fix")
        }
        fn x_salsa20_poly1305_decrypt(
            &self,
            _: XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by this fix")
        }
        fn x_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: X25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<Option<XSalsa20Poly1305Data>> {
            unimplemented!("not exercised by this fix")
        }
        fn ed_25519_x_salsa20_poly1305_decrypt(
            &self,
            _: Ed25519XSalsa20Poly1305Decrypt,
        ) -> ExternResult<XSalsa20Poly1305Data> {
            unimplemented!("not exercised by this fix")
        }
    }

    fn wrap_entry_record<T>(author: AgentPubKey, value: T) -> Record
    where
        T: TryInto<SerializedBytes>,
        <T as TryInto<SerializedBytes>>::Error: std::fmt::Debug,
    {
        let entry = Entry::App(AppEntryBytes::try_from(value.try_into().unwrap()).unwrap());
        let action = Action::Create(Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![1; 36]),
            weight: Default::default(),
        });
        let hashed = HoloHashed::from_content_sync(action);
        let signed_action = SignedActionHashed::with_presigned(hashed, Signature([0; 64]));
        Record::new(signed_action, Some(entry))
    }

    fn test_bundle() -> PreKeyBundle {
        PreKeyBundle {
            identity_key: vec![1; 32],
            signed_pre_key: vec![2; 32],
            signed_pre_key_id: 1,
            signed_pre_key_signature: vec![3; 64],
            one_time_pre_keys: vec![
                OneTimePreKey {
                    key_id: 1,
                    public_key: vec![4; 32],
                    used: false,
                },
                OneTimePreKey {
                    key_id: 2,
                    public_key: vec![5; 32],
                    used: false,
                },
            ],
            created_at: 0,
            expires_at: u64::MAX,
        }
    }

    fn setup_mock(author: AgentPubKey, original: &PreKeyBundle, original_hash: ActionHash) {
        hdi::hdi::set_hdi(MockRecordHdi {
            records: std::collections::HashMap::from([(
                original_hash,
                wrap_entry_record(author, original.clone()),
            )]),
        });
    }

    #[test]
    fn marking_exactly_one_key_used_is_accepted() {
        let original = test_bundle();
        let original_hash = ActionHash::from_raw_36(vec![30; 36]);
        setup_mock(
            AgentPubKey::from_raw_36(vec![1; 36]),
            &original,
            original_hash.clone(),
        );

        let mut updated = original.clone();
        updated.one_time_pre_keys[0].used = true;

        let result = validate_update_pre_key_bundle(updated, original_hash).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn marking_two_keys_used_at_once_is_rejected() {
        let original = test_bundle();
        let original_hash = ActionHash::from_raw_36(vec![30; 36]);
        setup_mock(
            AgentPubKey::from_raw_36(vec![1; 36]),
            &original,
            original_hash.clone(),
        );

        let mut updated = original.clone();
        updated.one_time_pre_keys[0].used = true;
        updated.one_time_pre_keys[1].used = true;

        let result = validate_update_pre_key_bundle(updated, original_hash).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(_)),
            "exactly one previously-unused key may flip to used per update"
        );
    }

    #[test]
    fn unusing_a_consumed_key_is_rejected() {
        let mut original = test_bundle();
        original.one_time_pre_keys[0].used = true;
        let original_hash = ActionHash::from_raw_36(vec![30; 36]);
        setup_mock(
            AgentPubKey::from_raw_36(vec![1; 36]),
            &original,
            original_hash.clone(),
        );

        let mut updated = original.clone();
        updated.one_time_pre_keys[0].used = false;
        updated.one_time_pre_keys[1].used = true;

        let result = validate_update_pre_key_bundle(updated, original_hash).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(_)),
            "a consumed key cannot be marked unused once used -- previously a modified \
             coordinator could rewrite the whole bundle"
        );
    }

    #[test]
    fn swapping_key_material_is_rejected() {
        let original = test_bundle();
        let original_hash = ActionHash::from_raw_36(vec![30; 36]);
        setup_mock(
            AgentPubKey::from_raw_36(vec![1; 36]),
            &original,
            original_hash.clone(),
        );

        let mut updated = original.clone();
        updated.one_time_pre_keys[0].used = true;
        updated.one_time_pre_keys[1].public_key = vec![99; 32];

        let result = validate_update_pre_key_bundle(updated, original_hash).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn adding_a_new_one_time_key_is_rejected() {
        let original = test_bundle();
        let original_hash = ActionHash::from_raw_36(vec![30; 36]);
        setup_mock(
            AgentPubKey::from_raw_36(vec![1; 36]),
            &original,
            original_hash.clone(),
        );

        let mut updated = original.clone();
        updated.one_time_pre_keys[0].used = true;
        updated.one_time_pre_keys.push(OneTimePreKey {
            key_id: 3,
            public_key: vec![6; 32],
            used: false,
        });

        let result = validate_update_pre_key_bundle(updated, original_hash).unwrap();
        assert!(
            matches!(result, ValidateCallbackResult::Invalid(_)),
            "PreKeyBundle updates cannot add or remove one-time pre-keys"
        );
    }
}
