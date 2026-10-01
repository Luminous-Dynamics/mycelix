#!/usr/bin/env python3
"""Independent semantic evaluator for mobility temporal applicability."""
def valid_interval(start, end):
    return end is None or start is None or end >= start

def overlap(a_start, a_end, b_start, b_end):
    if None in (a_start, a_end, b_start, b_end):
        return None
    return a_start <= b_end and b_start <= a_end

def validate_assertion(event_time, start, end, publication_time, provenance):
    if not valid_interval(start, end):
        return False
    if provenance == "protocol_publication":
        return False
    # Event/publication values are deliberately distinct fields.
    return True

def self_test():
    assert validate_assertion(100, 100, None, 200, "engineering_claim")
    assert not validate_assertion(100, 20, 10, 200, "engineering_claim")
    assert overlap(10, 20, 21, 30) is False
    assert overlap(10, None, 20, 30) is None
    assert overlap(10, 20, 20, 30) is True
    assert not validate_assertion(100, 100, 200, 300, "protocol_publication")

if __name__ == "__main__":
    self_test()
    print("mobility temporal applicability reference qualification: PASS")
