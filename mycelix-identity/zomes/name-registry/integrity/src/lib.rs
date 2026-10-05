        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete entries".into(),
                ));
            }
            match original.action().entry_type() {
                Some(EntryType::App(def))
                    if def.entry_index() == EntryDefIndex::from(0)
                        || def.entry_index() == EntryDefIndex::from(1) =>
                {
                    // EntryTypes declaration order: MeshNameEntry,
                    // NameTransfer, Anchor. Name bindings and transfers are
                    // security-sensitive state; Anchor remains deletable.
                    Ok(ValidateCallbackResult::Invalid(
                        "Name bindings and transfers cannot be deleted".into(),
                    ))
                }
                _ => Ok(ValidateCallbackResult::Valid),
            }
        }