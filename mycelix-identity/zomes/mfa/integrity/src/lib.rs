
            match original.action().entry_type() {
                Some(EntryType::App(entry_def))
                    if entry_def.entry_index() == EntryDefIndex::from(0)
                        || entry_def.entry_index() == EntryDefIndex::from(1)
                        || entry_def.entry_index() == EntryDefIndex::from(2)
                        || entry_def.entry_index() == EntryDefIndex::from(3) =>
                {
                    // These indices are the declaration order in EntryTypes:
                    // MfaState, FactorEnrollment, FactorVerification,
                    // EncryptedEntry. All four are security-sensitive state
                    // or audit history and are append-only.
                    Ok(ValidateCallbackResult::Invalid(
                        "MFA security and audit entries cannot be deleted".into(),
                    ))
                }
                _ => Ok(ValidateCallbackResult::Valid),
            }