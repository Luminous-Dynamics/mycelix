//! Strict raw-input gate for D6S-CANON-1.
use serde_json::Value;
use std::collections::BTreeSet;
use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum D6SRawJsonError {
    Syntax(String),
    DuplicateProperty(String),
    NegativeZero,
    NonIntegralNumber(String),
    IntegerOutOfRange(String),
    LoneSurrogate(u16),
    InvalidUtf8,
}
impl fmt::Display for D6SRawJsonError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Syntax(e) => write!(f, "invalid JSON syntax: {e}"),
            Self::DuplicateProperty(k) => write!(f, "duplicate object property: {k:?}"),
            Self::NegativeZero => write!(f, "D6S-CANON-1 rejects negative zero"),
            Self::NonIntegralNumber(n) => write!(f, "D6S-CANON-1 rejects non-integral numeric form: {n}"),
            Self::IntegerOutOfRange(n) => write!(f, "D6S-CANON-1 integer out of range: {n}"),
            Self::LoneSurrogate(v) => write!(f, "D6S-CANON-1 rejects lone surrogate: U+{v:04X}"),
            Self::InvalidUtf8 => write!(f, "D6S-CANON-1 input is not valid UTF-8"),
        }
    }
}
impl std::error::Error for D6SRawJsonError {}

pub fn parse_d6s_canon_json(input: &[u8]) -> Result<Value, D6SRawJsonError> {
    std::str::from_utf8(input).map_err(|_| D6SRawJsonError::InvalidUtf8)?;
    let mut p = Scanner { input, pos: 0 };
    p.parse_value()?;
    p.ws();
    if p.pos != input.len() {
        return Err(D6SRawJsonError::Syntax("trailing bytes after JSON value".into()));
    }
    serde_json::from_slice(input).map_err(|e| D6SRawJsonError::Syntax(e.to_string()))
}

struct Scanner<'a> { input: &'a [u8], pos: usize }
impl<'a> Scanner<'a> {
    fn ws(&mut self) { while matches!(self.input.get(self.pos), Some(b' ' | b'\n' | b'\r' | b'\t')) { self.pos += 1; } }

    fn parse_value(&mut self) -> Result<(), D6SRawJsonError> {
        self.ws();
        match self.input.get(self.pos).copied() {
            Some(b'n') => self.literal(b"null"),
            Some(b't') => self.literal(b"true"),
            Some(b'f') => self.literal(b"false"),
            Some(b'"') => { self.parse_string_value()?; Ok(()) },
            Some(b'[') => self.parse_array(),
            Some(b'{') => self.parse_object(),
            Some(b'-' | b'0'..=b'9') => self.parse_number(),
            _ => Err(D6SRawJsonError::Syntax(format!("unexpected byte at {}", self.pos))),
        }
    }
    fn literal(&mut self, expected: &[u8]) -> Result<(), D6SRawJsonError> {
        if self.input.get(self.pos..self.pos + expected.len()) == Some(expected) { self.pos += expected.len(); Ok(()) }
        else { Err(D6SRawJsonError::Syntax(format!("invalid literal at {}", self.pos))) }
    }
    fn parse_array(&mut self) -> Result<(), D6SRawJsonError> {
        self.pos += 1; self.ws();
        if self.input.get(self.pos) == Some(&b']') { self.pos += 1; return Ok(()); }
        loop {
            self.parse_value()?; self.ws();
            match self.input.get(self.pos) {
                Some(b',') => self.pos += 1,
                Some(b']') => { self.pos += 1; return Ok(()); }
                _ => return Err(D6SRawJsonError::Syntax(format!("expected ',' or ']' at {}", self.pos))),
            }
        }
    }
    fn parse_object(&mut self) -> Result<(), D6SRawJsonError> {
        self.pos += 1; self.ws();
        let mut keys = BTreeSet::new();
        if self.input.get(self.pos) == Some(&b'}') { self.pos += 1; return Ok(()); }
        loop {
            self.ws();
            if self.input.get(self.pos) != Some(&b'"') {
                return Err(D6SRawJsonError::Syntax(format!("expected object key at {}", self.pos)));
            }
            let key = self.parse_string_value()?;
            if !keys.insert(key.clone()) { return Err(D6SRawJsonError::DuplicateProperty(key)); }
            self.ws();
            if self.input.get(self.pos) != Some(&b':') {
                return Err(D6SRawJsonError::Syntax(format!("expected ':' at {}", self.pos)));
            }
            self.pos += 1; self.parse_value()?; self.ws();
            match self.input.get(self.pos) {
                Some(b',') => self.pos += 1,
                Some(b'}') => { self.pos += 1; return Ok(()); }
                _ => return Err(D6SRawJsonError::Syntax(format!("expected ',' or '}}' at {}", self.pos))),
            }
        }
    }
    fn parse_string_value(&mut self) -> Result<String, D6SRawJsonError> {
        let start = self.pos; self.pos += 1;
        while self.pos < self.input.len() {
            match self.input[self.pos] {
                b'"' => {
                    self.pos += 1;
                    let raw = std::str::from_utf8(&self.input[start..self.pos]).map_err(|_| D6SRawJsonError::InvalidUtf8)?;
                    return serde_json::from_str(raw).map_err(|e| D6SRawJsonError::Syntax(e.to_string()));
                }
                b'\\' => {
                    self.pos += 1;
                    let escape = *self.input.get(self.pos).ok_or_else(|| D6SRawJsonError::Syntax("unterminated escape".into()))?;
                    if escape == b'u' {
                        let first = self.hex4(self.pos + 1)?;
                        if (0xD800..=0xDBFF).contains(&first) {
                            if self.input.get(self.pos + 5) != Some(&b'\\') || self.input.get(self.pos + 6) != Some(&b'u') {
                                return Err(D6SRawJsonError::LoneSurrogate(first));
                            }
                            let second = self.hex4(self.pos + 7)?;
                            if !(0xDC00..=0xDFFF).contains(&second) { return Err(D6SRawJsonError::LoneSurrogate(first)); }
                            self.pos += 11;
                        } else if (0xDC00..=0xDFFF).contains(&first) {
                            return Err(D6SRawJsonError::LoneSurrogate(first));
                        } else { self.pos += 5; }
                    } else if matches!(escape, b'"' | b'\\' | b'/' | b'b' | b'f' | b'n' | b'r' | b't') {
                        self.pos += 1;
                    } else { return Err(D6SRawJsonError::Syntax(format!("invalid escape at {}", self.pos))); }
                }
                0x00..=0x1F => return Err(D6SRawJsonError::Syntax(format!("unescaped control byte at {}", self.pos))),
                _ => self.pos += 1,
            }
        }
        Err(D6SRawJsonError::Syntax("unterminated string".into()))
    }
    fn hex4(&self, start: usize) -> Result<u16, D6SRawJsonError> {
        let bytes = self.input.get(start..start + 4).ok_or_else(|| D6SRawJsonError::Syntax("short unicode escape".into()))?;
        let mut v = 0u16;
        for &b in bytes {
            let d = match b { b'0'..=b'9' => b-b'0', b'a'..=b'f' => b-b'a'+10, b'A'..=b'F' => b-b'A'+10, _ => return Err(D6SRawJsonError::Syntax("invalid unicode escape".into())) };
            v = (v << 4) | u16::from(d);
        }
        Ok(v)
    }
    fn parse_number(&mut self) -> Result<(), D6SRawJsonError> {
        let start = self.pos; let negative = self.input.get(self.pos) == Some(&b'-');
        if negative { self.pos += 1; }
        match self.input.get(self.pos) {
            Some(b'0') => {
                self.pos += 1;
                if negative { return Err(D6SRawJsonError::NegativeZero); }
                if matches!(self.input.get(self.pos), Some(b'0'..=b'9')) { return Err(D6SRawJsonError::Syntax("leading zero".into())); }
            }
            Some(b'1'..=b'9') => while matches!(self.input.get(self.pos), Some(b'0'..=b'9')) { self.pos += 1; },
            _ => return Err(D6SRawJsonError::Syntax(format!("invalid number at {start}"))),
        }
        if matches!(self.input.get(self.pos), Some(b'.' | b'e' | b'E')) {
            let end = self.number_end();
            return Err(D6SRawJsonError::NonIntegralNumber(String::from_utf8_lossy(&self.input[start..end]).into_owned()));
        }
        let raw = std::str::from_utf8(&self.input[start..self.pos]).unwrap_or_default();
        if negative {
            let magnitude = raw[1..].parse::<u64>().map_err(|_| D6SRawJsonError::IntegerOutOfRange(raw.into()))?;
            if magnitude > (i64::MAX as u64) + 1 { return Err(D6SRawJsonError::IntegerOutOfRange(raw.into())); }
        } else if raw.parse::<u64>().is_err() {
            return Err(D6SRawJsonError::IntegerOutOfRange(raw.into()));
        }
        Ok(())
    }
    fn number_end(&self) -> usize {
        let mut i = self.pos;
        while matches!(self.input.get(i), Some(b'0'..=b'9' | b'.' | b'e' | b'E' | b'+' | b'-')) { i += 1; }
        i
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rejects_frozen_raw_boundaries() {
        for (name, input) in [
            ("negative-zero", b"-0" as &[u8]),
            ("fractional", b"1.5"),
            ("exponent", b"1e0"),
            ("overflow", b"18446744073709551616"),
            ("underflow", b"-9223372036854775809"),
            ("duplicate", br#"{"a":1,"a":2}"#),
            ("lone-high", br#""\uD800""#),
            ("lone-low", br#""\uDC00""#),
        ] { assert!(parse_d6s_canon_json(input).is_err(), "{name} unexpectedly accepted"); }
    }
    #[test]
    fn accepts_paired_surrogate() {
        assert!(parse_d6s_canon_json(br#""\uD834\uDD1E""#).is_ok());
    }
    #[test]
    fn rejects_trailing_input() { assert!(parse_d6s_canon_json(b"{}{}").is_err()); }
}
