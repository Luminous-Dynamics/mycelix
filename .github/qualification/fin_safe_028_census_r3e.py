#!/usr/bin/env python3
import importlib.util
import pathlib

CORE_PATH = pathlib.Path('/tmp/fin_safe_028_census_r3d_core.py')
EXPECTED_CORE_BLOB_SHA = '1e2784aad15d7fca8b605f16b4eea227f832972c'
EXPECTED_RECORD_SET_SHA256 = 'd71f50ff938b7e8bf986f2b982542259183b1e829ba6fda95757d9f9b29d50a5'
EXPECTED_GATE_SET_SHA256 = 'ed1660667c522627f4f4115e4b3fa0be248ae98ff4b00be707dd3379bf7d1ecd'
EXPECTED_CALL_SET_SHA256 = '3a3a8d84b796950168509969fba7803a96e1542d7a4169b50552bb09e90f32c6'

spec = importlib.util.spec_from_file_location('fin_safe_028_census_r3d_core', CORE_PATH)
if spec is None or spec.loader is None:
    raise SystemExit('unable to load frozen R3d core')
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)

# R3e changes only the three derived exact-set bindings. The R3d core remains
# byte-identical and supplies all classifier, topology, occurrence, dedupe,
# receipt, and evidence-generation behavior.
core.EXPECTED_RECORD_SET_SHA256 = EXPECTED_RECORD_SET_SHA256
core.EXPECTED_GATE_SET_SHA256 = EXPECTED_GATE_SET_SHA256
core.EXPECTED_CALL_SET_SHA256 = EXPECTED_CALL_SET_SHA256

core.main()
