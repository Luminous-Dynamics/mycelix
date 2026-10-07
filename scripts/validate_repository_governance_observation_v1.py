def validate_bound_raw_payload(observation: Any, name: str) -> bytes:
    raw_field = f"{name}_payload_base64"
    digest_field = f"{name}_payload_sha256"
    encoded = observation.get(raw_field)
    expected_digest = sha256(observation.get(digest_field), digest_field)
    require(isinstance(encoded, str) and encoded != "", f"{raw_field} missing")
    try:
        raw = base64.b64decode(encoded, validate=True)
    except (ValueError, TypeError) as exc:
        raise EvidenceError(f"{raw_field} is not valid base64") from exc
    actual_digest = hashlib.sha256(raw).hexdigest()
    require(actual_digest == expected_digest, f"{digest_field} does not match {raw_field}")
    return raw
