//! D6S-CANON-1 canonical JSON subset for the COS reference crate.
//!
//! Lexical input ambiguity (duplicate keys, exponent notation, negative zero,
//! lone surrogates) is rejected by the separate d6s_raw_json module.

use serde::Serialize;

pub const D6S_REFERENCE_CANONICALIZATION_VERSION: &str = "D6S-CANON-1";
pub const D6S_HASH_DOMAIN: &[u8] = b"MYCELIX-INTEGRAL-D6S-RECEIPT-V1\0";

pub fn canonical_bytes<T: Serialize>(value: &T) -> Result<Vec<u8>, String> {
    let value = serde_json::to_value(value).map_err(|e| e.to_string())?;
    let mut out = Vec::new();
    write_canonical_json(&value, &mut out)?;
    Ok(out)
}

fn utf16_sort_key(value: &str) -> Vec<u16> {
    value.encode_utf16().collect()
}

fn write_canonical_json(value: &serde_json::Value, out: &mut Vec<u8>) -> Result<(), String> {
    match value {
        serde_json::Value::Null => out.extend_from_slice(b"null"),
        serde_json::Value::Bool(v) => out.extend_from_slice(if *v { b"true" } else { b"false" }),
        serde_json::Value::Number(number) => {
            // D6S-CANON-1 deliberately admits integers only. All current D6S
            // schema numerics are integral, so this removes float/runtime
            // differences instead of pretending to canonicalize them.
            if let Some(v) = number.as_i64() {
                out.extend_from_slice(v.to_string().as_bytes());
            } else if let Some(v) = number.as_u64() {
                out.extend_from_slice(v.to_string().as_bytes());
            } else {
                return Err("D6S-CANON-1 rejects non-integral numeric values".into());
            }
        }
        serde_json::Value::String(value) => write_canonical_string(value, out),
        serde_json::Value::Array(values) => {
            out.push(b'[');
            for (index, value) in values.iter().enumerate() {
                if index != 0 {
                    out.push(b',');
                }
                write_canonical_json(value, out)?;
            }
            out.push(b']');
        }
        serde_json::Value::Object(map) => {
            let mut entries: Vec<_> = map.iter().collect();
            entries.sort_by(|(left, _), (right, _)| utf16_sort_key(left).cmp(&utf16_sort_key(right)));
            out.push(b'{');
            for (index, (key, value)) in entries.iter().enumerate() {
                if index != 0 {
                    out.push(b',');
                }
                write_canonical_string(key, out);
                out.push(b':');
                write_canonical_json(value, out)?;
            }
            out.push(b'}');
        }
    }
    Ok(())
}

fn write_canonical_string(value: &str, out: &mut Vec<u8>) {
    out.push(b'"');
    for ch in value.chars() {
        match ch {
            '"' => out.extend_from_slice(br#"\""#),
            '\\' => out.extend_from_slice(br#"\\"#),
            '\u{0008}' => out.extend_from_slice(br#"\b"#),
            '\t' => out.extend_from_slice(br#"\t"#),
            '\n' => out.extend_from_slice(br#"\n"#),
            '\u{000C}' => out.extend_from_slice(br#"\f"#),
            '\r' => out.extend_from_slice(br#"\r"#),
            ch if (ch as u32) <= 0x1F => {
                let escaped = format!("\\u{:04x}", ch as u32);
                out.extend_from_slice(escaped.as_bytes());
            }
            ch => {
                let mut buf = [0u8; 4];
                out.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
            }
        }
    }
    out.push(b'"');
}
