// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Shared SSRF policy for Prism's legacy HTTP bridge.
//!
//! This module is only a destination policy layer. Domain names still require
//! DNS-time admission in prism-net::SafeFetchClient. A URL passing this
//! function is therefore not itself network authority.

pub fn is_private_ipv4(ip: &std::net::Ipv4Addr) -> bool {
    let [a, b, c, _] = ip.octets();

    ip.is_loopback()
        || ip.is_private()
        || ip.is_link_local()
        || ip.is_broadcast()
        || ip.is_unspecified()
        || ip.is_multicast()
        || (a == 100 && (64..=127).contains(&b))
        || (a == 192 && b == 0 && c == 0)
        || (a == 192 && b == 0 && c == 2)
        || (a == 192 && b == 88 && c == 99)
        || (a == 198 && (18..=19).contains(&b))
        || (a == 198 && b == 51 && c == 100)
        || (a == 203 && b == 0 && c == 113)
        || a >= 240
}

pub fn is_private_ipv6(ip: &std::net::Ipv6Addr) -> bool {
    let seg = ip.segments();

    ip.is_loopback()
        || ip.is_unspecified()
        || ip.is_multicast()
        || (seg[0] & 0xffc0) == 0xfe80
        || (seg[0] & 0xfe00) == 0xfc00
        || (seg[0..5] == [0, 0, 0, 0, 0])
            && seg[5] == 0xffff
            && is_private_ipv4(&std::net::Ipv4Addr::new(
                (seg[6] >> 8) as u8,
                seg[6] as u8,
                (seg[7] >> 8) as u8,
                seg[7] as u8,
            ))
        || (seg[0] == 0x2001 && seg[1] == 0x0db8)
        || (seg[0] == 0x2001 && seg[1] == 0x0002)
}

pub fn validate_proxy_url(raw: &str) -> Result<url::Url, &'static str> {
    let parsed = url::Url::parse(raw).map_err(|_| "Invalid URL")?;

    match parsed.scheme() {
        "http" | "https" => {}
        _ => return Err("Only http and https URLs are allowed"),
    }

    if !parsed.username().is_empty() || parsed.password().is_some() {
        return Err("URL userinfo is forbidden");
    }

    let host = parsed.host_str().unwrap_or("");
    let host_lower = host.to_ascii_lowercase();
    if host_lower == "localhost"
        || host_lower == "metadata.google.internal"
        || host_lower.ends_with(".localhost")
        || host_lower.ends_with(".local")
    {
        return Err("Access to internal/special-use hostnames is forbidden");
    }

    match parsed.host() {
        Some(url::Host::Ipv4(ip)) if is_private_ipv4(&ip) => {
            Err("Access to private/reserved IP addresses is forbidden")
        }
        Some(url::Host::Ipv6(ip)) if is_private_ipv6(&ip) => {
            Err("Access to private/reserved IP addresses is forbidden")
        }
        _ => Ok(parsed),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_localhost_and_metadata() {
        for url in [
            "http://localhost:8080",
            "http://foo.localhost",
            "http://127.0.0.1:5432",
            "http://[::1]:80",
            "http://metadata.google.internal",
        ] {
            assert!(validate_proxy_url(url).is_err(), "{url}");
        }
    }

    #[test]
    fn rejects_private_shared_and_special_ipv4() {
        for url in [
            "http://10.0.0.1",
            "http://172.16.0.1",
            "http://192.168.1.1",
            "http://100.64.0.1",
            "http://192.0.0.1",
            "http://192.0.2.1",
            "http://198.18.0.1",
            "http://203.0.113.1",
            "http://224.0.0.1",
            "http://255.255.255.255",
        ] {
            assert!(validate_proxy_url(url).is_err(), "{url}");
        }
    }

    #[test]
    fn rejects_ipv6_special_classes_and_mapped_private() {
        for url in [
            "http://[fc00::1]",
            "http://[fe80::1]",
            "http://[ff02::1]",
            "http://[::ffff:127.0.0.1]",
            "http://[::ffff:192.168.1.1]",
            "http://[2001:db8::1]",
        ] {
            assert!(validate_proxy_url(url).is_err(), "{url}");
        }
    }

    #[test]
    fn rejects_userinfo() {
        assert!(validate_proxy_url("https://user:password@example.com").is_err());
        assert!(validate_proxy_url("https://user@example.com").is_err());
    }

    #[test]
    fn allows_public_literals_and_domains() {
        assert!(validate_proxy_url("https://example.com").is_ok());
        assert!(validate_proxy_url("https://1.1.1.1").is_ok());
        assert!(validate_proxy_url("https://[2606:4700:4700::1111]").is_ok());
    }

    #[test]
    fn is_private_ipv4_covers_multicast() {
        assert!(is_private_ipv4(&"224.0.0.1".parse().unwrap()));
    }
}
