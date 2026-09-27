import unittest
from validate_myc_int_018y import validate, EXPECTED_PARENT


def cultivar_fixture():
    ids=['rijk-zwaan-sandalina-rz','rijk-zwaan-kireve-rz','rijk-zwaan-extranet-rz','rijk-zwaan-klee-rz','rijk-zwaan-station-rz']
    return {
        'profile_id':'myc-int-018v-h2-lettuce-cultivar-admission-v1','profile_version':'1.0.0','status':'research-fixture','parent_018u_subject':EXPECTED_PARENT,
        'exact_cultivar_selected':False,'weighted_overall_score':False,
        'source_policy':{'supplier_claims_are_source_claims_not_local_measurements':True,'catalogue_listing_is_not_stock_evidence':True,'hydroponic_claim_is_not_dwc_qualification':True,'resistance_code_is_not_immunity':True},
        'candidates':[{'candidate_id':x,'source_url':'https://example.invalid/'+x,'known_unknowns':['x'],'dwc_specific_evidence':None,'exact_cycle_evidence':None,'exact_seed_lot':None,'geometry_fit':'Unbound','admission_state':'MoreEvidenceRequired'} for x in ids],
        'claim_ceiling':{'cultivar_selected':False,'seed_in_stock':False,'dwc_qualified':False}
    }


def sensor_fixture():
    return {
        'profile_id':'myc-int-018w-h2-sensor-candidates-v1','profile_version':'1.0.0','status':'research-fixture','parent_018u_subject':EXPECTED_PARENT,'weighted_overall_score':False,
        'measurement_rules':{'lux_is_not_ppfd':True,'fixture_power_is_not_ppfd':True,'fixture_command_is_not_ppfd':True,'spot_ppfd_is_not_canopy_dli':True,'dli_requires_window_and_coverage':True,'do_reading_is_not_root_health':True,'sensor_output_has_no_actuation_authority':True},
        'candidates':[
            {'candidate_id':'apogee-sq520-usb','source_url':'u','known_unknowns':['x'],'admission_state':'MoreEvidenceRequired','raspberry_pi_linux_acquisition':'Unverified'},
            {'candidate_id':'apogee-sq522-ss-modbus','source_url':'u','known_unknowns':['x'],'admission_state':'MoreEvidenceRequired','interface':'Modbus RTU','conditioning_required':'serial profile'},
            {'candidate_id':'licor-li190r','source_url':'u','known_unknowns':['x'],'admission_state':'MoreEvidenceRequired','conditioning_required':'logger'},
            {'candidate_id':'atlas-ezo-do-kit','source_url':'u','known_unknowns':['x'],'admission_state':'MoreEvidenceRequired','compensation':['temperature','salinity','pressure'],'kit_features':['electrically isolated carrier']},
            {'candidate_id':'dfrobot-sen0237-a','source_url':'u','known_unknowns':['x'],'admission_state':'MoreEvidenceRequired','interface':'analog','service':['NaOH filling solution'],'conditioning_required':'ADC path'}
        ],
        'dli_evidence_requirements':['coverage-completeness'],'do_evidence_requirements':['calibration-evidence-ref'],
        'claim_ceiling':{'purchase_selected':False,'calibration_established':False}
    }


class Tests(unittest.TestCase):
    def reject(self, mutate):
        v,w=cultivar_fixture(),sensor_fixture()
        mutate(v,w)
        self.assertTrue(validate(v,w))

    def test_00_pristine(self): self.assertEqual(validate(cultivar_fixture(),sensor_fixture()),[])
    def test_01_cultivar_selected(self): self.reject(lambda v,w:v.__setitem__('exact_cultivar_selected',True))
    def test_02_scalar_score(self): self.reject(lambda v,w:v.__setitem__('weighted_overall_score',True))
    def test_03_v_admission_promoted(self): self.reject(lambda v,w:v['candidates'][0].__setitem__('admission_state','AdmittedForH2ReferenceProfile'))
    def test_04_catalogue_as_stock(self): self.reject(lambda v,w:v['source_policy'].__setitem__('catalogue_listing_is_not_stock_evidence',False))
    def test_05_hydroponic_as_dwc(self): self.reject(lambda v,w:v['source_policy'].__setitem__('hydroponic_claim_is_not_dwc_qualification',False))
    def test_06_seedlot_silent(self): self.reject(lambda v,w:v['candidates'][0].__setitem__('exact_seed_lot','lot'))
    def test_07_geometry_promoted(self): self.reject(lambda v,w:v['candidates'][0].__setitem__('geometry_fit','Qualified'))
    def test_08_lux_as_ppfd(self): self.reject(lambda v,w:w['measurement_rules'].__setitem__('lux_is_not_ppfd',False))
    def test_09_dli_no_coverage(self): self.reject(lambda v,w:w['dli_evidence_requirements'].remove('coverage-completeness'))
    def test_10_sq520_linux_asserted(self): self.reject(lambda v,w:w['candidates'][0].__setitem__('raspberry_pi_linux_acquisition','Qualified'))
    def test_11_sq522_interface_removed(self): self.reject(lambda v,w:w['candidates'][1].__setitem__('interface','USB'))
    def test_12_licor_conditioning_removed(self): self.reject(lambda v,w:w['candidates'][2].__setitem__('conditioning_required',''))
    def test_13_atlas_comp_removed(self): self.reject(lambda v,w:w['candidates'][3].__setitem__('compensation',['temperature']))
    def test_14_df_not_analog(self): self.reject(lambda v,w:w['candidates'][4].__setitem__('interface','digital'))
    def test_15_df_naoh_removed(self): self.reject(lambda v,w:w['candidates'][4].__setitem__('service',[]))
    def test_16_sensor_authority(self): self.reject(lambda v,w:w['measurement_rules'].__setitem__('sensor_output_has_no_actuation_authority',False))
    def test_17_w_admission_promoted(self): self.reject(lambda v,w:w['candidates'][0].__setitem__('admission_state','AdmittedForH2ReferenceProfile'))
    def test_18_claim_upgraded_v(self): self.reject(lambda v,w:v['claim_ceiling'].__setitem__('cultivar_selected',True))
    def test_19_claim_upgraded_w(self): self.reject(lambda v,w:w['claim_ceiling'].__setitem__('purchase_selected',True))
    def test_20_candidate_drift(self): self.reject(lambda v,w:v['candidates'].pop())


if __name__=='__main__':
    unittest.main()
