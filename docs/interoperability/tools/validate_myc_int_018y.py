#!/usr/bin/env python3
import argparse, json
from pathlib import Path

EXPECTED_PARENT='98a37538ef2cd36407e63f7c0904b012012bced8'
V_IDS={'rijk-zwaan-sandalina-rz','rijk-zwaan-kireve-rz','rijk-zwaan-extranet-rz','rijk-zwaan-klee-rz','rijk-zwaan-station-rz'}
W_IDS={'apogee-sq520-usb','apogee-sq522-ss-modbus','licor-li190r','atlas-ezo-do-kit','dfrobot-sen0237-a'}


def load_json(path):
    return json.loads(Path(path).read_text())


def all_false(d):
    return isinstance(d, dict) and all(v is False for v in d.values())


def validate(v, w):
    errors=[]
    if (v.get('profile_id'),v.get('profile_version'),v.get('status'),v.get('parent_018u_subject')) != ('myc-int-018v-h2-lettuce-cultivar-admission-v1','1.0.0','research-fixture',EXPECTED_PARENT):
        errors.append('018V identity drift')
    if v.get('exact_cultivar_selected') is not False:
        errors.append('cultivar selected')
    if v.get('weighted_overall_score') is not False:
        errors.append('018V weighted score prohibited')
    source_policy=v.get('source_policy',{})
    for key in ['supplier_claims_are_source_claims_not_local_measurements','catalogue_listing_is_not_stock_evidence','hydroponic_claim_is_not_dwc_qualification','resistance_code_is_not_immunity']:
        if source_policy.get(key) is not True:
            errors.append('018V source policy missing: '+key)
    vc={c.get('candidate_id'):c for c in v.get('candidates',[])}
    if set(vc)!=V_IDS:
        errors.append('018V candidate set drift')
    for cid,c in vc.items():
        if c.get('admission_state')!='MoreEvidenceRequired':
            errors.append(cid+': admission promoted')
        if not c.get('source_url') or not c.get('known_unknowns'):
            errors.append(cid+': source/unknowns missing')
        if c.get('dwc_specific_evidence') is not None or c.get('exact_cycle_evidence') is not None or c.get('exact_seed_lot') is not None:
            errors.append(cid+': evidence unexpectedly bound')
        if c.get('geometry_fit')!='Unbound':
            errors.append(cid+': geometry fit unexpectedly established')
    if not all_false(v.get('claim_ceiling',{})):
        errors.append('018V claim ceiling upgraded')

    if (w.get('profile_id'),w.get('profile_version'),w.get('status'),w.get('parent_018u_subject')) != ('myc-int-018w-h2-sensor-candidates-v1','1.0.0','research-fixture',EXPECTED_PARENT):
        errors.append('018W identity drift')
    if w.get('weighted_overall_score') is not False:
        errors.append('018W weighted score prohibited')
    rules=w.get('measurement_rules',{})
    for key in ['lux_is_not_ppfd','fixture_power_is_not_ppfd','fixture_command_is_not_ppfd','spot_ppfd_is_not_canopy_dli','dli_requires_window_and_coverage','do_reading_is_not_root_health','sensor_output_has_no_actuation_authority']:
        if rules.get(key) is not True:
            errors.append('018W measurement rule missing: '+key)
    wc={c.get('candidate_id'):c for c in w.get('candidates',[])}
    if set(wc)!=W_IDS:
        errors.append('018W candidate set drift')
    for cid,c in wc.items():
        if c.get('admission_state')!='MoreEvidenceRequired':
            errors.append(cid+': admission promoted')
        if not c.get('source_url') or not c.get('known_unknowns'):
            errors.append(cid+': source/unknowns missing')
    if wc.get('apogee-sq520-usb',{}).get('raspberry_pi_linux_acquisition')!='Unverified':
        errors.append('SQ-520 Linux/Pi state must remain Unverified')
    sq522=wc.get('apogee-sq522-ss-modbus',{})
    if 'Modbus' not in sq522.get('interface','') or not sq522.get('conditioning_required'):
        errors.append('SQ-522 interface/profile obligations drift')
    if not wc.get('licor-li190r',{}).get('conditioning_required'):
        errors.append('LI-190R conditioning requirement missing')
    atlas=wc.get('atlas-ezo-do-kit',{})
    if set(atlas.get('compensation',[]))!={'temperature','salinity','pressure'} or 'electrically isolated carrier' not in atlas.get('kit_features',[]):
        errors.append('Atlas D.O. profile drift')
    df=wc.get('dfrobot-sen0237-a',{})
    if 'analog' not in df.get('interface','').lower() or not any('NaOH' in x for x in df.get('service',[])) or 'ADC' not in df.get('conditioning_required',''):
        errors.append('DFRobot D.O. acquisition/service profile drift')
    if 'coverage-completeness' not in w.get('dli_evidence_requirements',[]):
        errors.append('DLI coverage requirement missing')
    if not w.get('do_evidence_requirements'):
        errors.append('D.O. evidence requirements missing')
    if not all_false(w.get('claim_ceiling',{})):
        errors.append('018W claim ceiling upgraded')
    return errors


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('cultivar_fixture')
    parser.add_argument('sensor_fixture')
    args=parser.parse_args()
    errors=validate(load_json(args.cultivar_fixture),load_json(args.sensor_fixture))
    if errors:
        for error in errors:
            print('ERROR:',error)
        raise SystemExit(1)
    print('MYC-INT-018Y: PASS')


if __name__=='__main__':
    main()
