const RECORD_MAGIC: &[u8] = b"MYCELIX-CONTENT-RECORD\0";

pub(crate) fn record_id(domain: &str, schema_version: u16, body: &[u8]) -> [u8; 32] {
    let mut hasher = blake3::Hasher::new();
    hasher.update(RECORD_MAGIC);
    put_field_hash(&mut hasher, domain.as_bytes());
    hasher.update(&schema_version.to_be_bytes());
    put_field_hash(&mut hasher, body);
    *hasher.finalize().as_bytes()
}

pub(crate) fn append_field(out: &mut Vec<u8>, bytes: &[u8]) {
    out.extend_from_slice(&(bytes.len() as u64).to_be_bytes());
    out.extend_from_slice(bytes);
}

pub(crate) fn append_option_bytes(out: &mut Vec<u8>, bytes: Option<&[u8]>) {
    match bytes {
        Some(bytes) => {
            out.push(1);
            append_field(out, bytes);
        }
        None => out.push(0),
    }
}

fn put_field_hash(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}
