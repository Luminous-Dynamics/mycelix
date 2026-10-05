#!/usr/bin/env python3
"""Bind TCG EK SAN device attributes to independently captured TPM properties."""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
import re
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-cert-san-device-identity.v0.1"
TPM_MFG_OID = "tcg-at-tpmManufacturer"
TPM_MODEL_OID = "tcg-at-tpmModel"
TPM_VERSION_OID = "tcg-at-tpmVersion"
REFERENCE_PROPERTIES_SHA256 = "e0b80bf063a21abbf0729996a75f3233ac640e69a08bcce09dbc0f0fd8e93731"
REFERENCE_PROPERTIES_SOURCE_TAG = "synthetic-tpm-properties-fixture-v0.1"
REFERENCE_PROPERTIES_SOURCE_SHA256 = hashlib.sha256(
    json.dumps(
        {"source_tag": REFERENCE_PROPERTIES_SOURCE_TAG, "properties_sha256": REFERENCE_PROPERTIES_SHA256},
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
).hexdigest()

PROPS_FIXTURE = """TPM2_PT_MANUFACTURER:
  raw: 0x1014
  value: "TPM2"
TPM2_PT_VENDOR_STRING_1:
  raw: 0x534C4239
  value: "SLB9"
TPM2_PT_VENDOR_STRING_2:
  raw: 0x36373000
  value: "670"
TPM2_PT_VENDOR_STRING_3:
  raw: 0x00000000
  value: ""
TPM2_PT_VENDOR_STRING_4:
  raw: 0x00000000
  value: ""
TPM2_PT_FIRMWARE_VERSION_1:
  raw: 0x00070055
"""

FIXTURES = {
  "valid": "MIIDbzCCAlegAwIBAgIBAjANBgkqhkiG9w0BAQsFADAhMR8wHQYDVQQDDBZNeWNlbGl4IFNBTiBGaXh0dXJlIENBMB4XDTI2MTAwMTAwMDAwMFoXDTM2MTAwMTAwMDAwMFowHTEbMBkGA1UEAwwSTXljZWxpeCBFSyBGaXh0dXJlMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAgblRQIeGpnDIyDMJHyafnoH3owFucWIai5ElCVbiUBzUR5OQ24BjmDLFUrxTP9BMyWXwTfhqve+m3jHJx5EUJJk5rr/IkwvAd5ueVgxaXIk5bXi5RJZbdL7XQnXc+TTILQEVOJYzdO2gVIudY/HE+L/zkoRk4r+Pm3TFEQxVYNidXm0+VCTsTGTyX4BmU26xW3VGR+7e9TW2B9UQIRSpBYK9O/2te98lJvqgfrpcHrNNNimfXzE2v9yYM+WfB8uIQ6sTEd/DKFlBCOdZJwHdNcuInsrPUr8fRUjJA0KIVEbdl5TYu4BMewFgml1oN61ps//nnr+n7YLb18fpgeP+AwIDAQABo4G1MIGyMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgUgMFIGA1UdEQRLMEmkRzBFMRYwFAYFZ4EFAgEMC2lkOjAwMDAxMDE0MRMwEQYFZ4EFAgIMCFNMQjkgNjcwMRYwFAYFZ4EFAgMMC2lkOjAwMDcwMDU1MB0GA1UdDgQWBBQay3Yfjfwu8m7xJwi1E/AgMqzR4DAfBgNVHSMEGDAWgBQay3Yfjfwu8m7xJwi1E/AgMqzR4DANBgkqhkiG9w0BAQsFAAOCAQEAEjxWqllcmRoWh82ynCNzYWfaQ0ZxD//yxT+J8E3FLqi4uL4awtiKqhmxymWKwc8rOBsPseBOm/vXeehZCrFUZK/wioQw5+31/NqLty7f8LGYX3nCVYu0MV5z8xgPqg49jzx7xV3YIEqteMWAtqky22B0LVAjvnY2WNOcpCnLVEqptEcVN2DY/NYZPh0R8eQh/NLXH4PBk/7qtuHF5CqT6E18FTTmJoUVHUyFvSNsOF5TlrPglZL1ZQNj1UDD/VqwgzJfc6JseFn5UHuI240I6crPTwm0afcIbNdDysi1K1LDOp+3MNhyibXFJBhPVQIZHo8ARPz+Eoiq67e0Gf0xCg==",
  "missing-manufacturer": "MIIDVzCCAj+gAwIBAgIBAzANBgkqhkiG9w0BAQsFADAhMR8wHQYDVQQDDBZNeWNlbGl4IFNBTiBGaXh0dXJlIENBMB4XDTI2MTAwMTAwMDAwMFoXDTM2MTAwMTAwMDAwMFowHTEbMBkGA1UEAwwSTXljZWxpeCBFSyBGaXh0dXJlMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAgblRQIeGpnDIyDMJHyafnoH3owFucWIai5ElCVbiUBzUR5OQ24BjmDLFUrxTP9BMyWXwTfhqve+m3jHJx5EUJJk5rr/IkwvAd5ueVgxaXIk5bXi5RJZbdL7XQnXc+TTILQEVOJYzdO2gVIudY/HE+L/zkoRk4r+Pm3TFEQxVYNidXm0+VCTsTGTyX4BmU26xW3VGR+7e9TW2B9UQIRSpBYK9O/2te98lJvqgfrpcHrNNNimfXzE2v9yYM+WfB8uIQ6sTEd/DKFlBCOdZJwHdNcuInsrPUr8fRUjJA0KIVEbdl5TYu4BMewFgml1oN61ps//nnr+n7YLb18fpgeP+AwIDAQABo4GdMIGaMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgUgMDoGA1UdEQQzMDGkLzAtMRMwEQYFZ4EFAgIMCFNMQjkgNjcwMRYwFAYFZ4EFAgMMC2lkOjAwMDcwMDU1MB0GA1UdDgQWBBQay3Yfjfwu8m7xJwi1E/AgMqzR4DAfBgNVHSMEGDAWgBQay3Yfjfwu8m7xJwi1E/AgMqzR4DANBgkqhkiG9w0BAQsFAAOCAQEAfoRDAub42SjrmgJdo4/ER1RB/0FMjyxDdDS/CF+3tgNp4vdyVK4F+tMN2lNs2nOaUdBjMCvkA9oefeB2zuA1r+M0dLUfFQSJvXUKo61dnC/xrLdqvW7Jq9GWDjs2HFyZWx/RPd5FaYpfyx+MFYrryFG3FYuNmy8Ysfz3hW7/EMDU7mOAW8XLyjt4L5caP6aIoG9uvg3VTp8U/PtenAPdm0nXDlLc/OUVG/IfGxjSspcfDjftRRjcAk1Xj5Bnux99f/nTYr+j9VKuQ8UPNXCkCjIgDkWu90pHU4nQDGzTWIw+24AJymRi+1NBPSMW8GvNgfEi+Hq5WM/JdcVBR4CHEQ==",
  "manufacturer-mismatch": "MIIDbzCCAlegAwIBAgIBBDANBgkqhkiG9w0BAQsFADAhMR8wHQYDVQQDDBZNeWNlbGl4IFNBTiBGaXh0dXJlIENBMB4XDTI2MTAwMTAwMDAwMFoXDTM2MTAwMTAwMDAwMFowHTEbMBkGA1UEAwwSTXljZWxpeCBFSyBGaXh0dXJlMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAgblRQIeGpnDIyDMJHyafnoH3owFucWIai5ElCVbiUBzUR5OQ24BjmDLFUrxTP9BMyWXwTfhqve+m3jHJx5EUJJk5rr/IkwvAd5ueVgxaXIk5bXi5RJZbdL7XQnXc+TTILQEVOJYzdO2gVIudY/HE+L/zkoRk4r+Pm3TFEQxVYNidXm0+VCTsTGTyX4BmU26xW3VGR+7e9TW2B9UQIRSpBYK9O/2te98lJvqgfrpcHrNNNimfXzE2v9yYM+WfB8uIQ6sTEd/DKFlBCOdZJwHdNcuInsrPUr8fRUjJA0KIVEbdl5TYu4BMewFgml1oN61ps//nnr+n7YLb18fpgeP+AwIDAQABo4G1MIGyMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgUgMFIGA1UdEQRLMEmkRzBFMRYwFAYFZ4EFAgEMC2lkOjAwMDAxMDE1MRMwEQYFZ4EFAgIMCFNMQjkgNjcwMRYwFAYFZ4EFAgMMC2lkOjAwMDcwMDU1MB0GA1UdDgQWBBQay3Yfjfwu8m7xJwi1E/AgMqzR4DAfBgNVHSMEGDAWgBQay3Yfjfwu8m7xJwi1E/AgMqzR4DANBgkqhkiG9w0BAQsFAAOCAQEAbjJyb0ozW1ApzaakgjRqSBvuUTIKYlRrZv5XYv/MWQwqJ9ckeZBG1AwNvEzU/JgWHXnxGnO1EcfthSAroPZNnC/nGJzVBnETETSZLm+E/+YYJ6FuctKd92W5ZeqaotyCXSBixkJ00qCF4vidvfh0t9nq6yCP/QDGDzKXVb/WwU14T5+zGHOvUMaUNq5EGH7FLlN8sQuqzCr0VBDL+WHIdJd8uY09lFWUNsc59Cug4bJk0PgIeGfzvWK75ai+F7br4TH5heY4nqc7t5fJCoKH6vVsSKBvUaBF3H0r63vjAt+gFJvz/Mwal5O1oCnHdNHcznSucaLRC951UotVotAlzQ==",
  "model-mismatch": "MIIDbzCCAlegAwIBAgIBBTANBgkqhkiG9w0BAQsFADAhMR8wHQYDVQQDDBZNeWNlbGl4IFNBTiBGaXh0dXJlIENBMB4XDTI2MTAwMTAwMDAwMFoXDTM2MTAwMTAwMDAwMFowHTEbMBkGA1UEAwwSTXljZWxpeCBFSyBGaXh0dXJlMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAgblRQIeGpnDIyDMJHyafnoH3owFucWIai5ElCVbiUBzUR5OQ24BjmDLFUrxTP9BMyWXwTfhqve+m3jHJx5EUJJk5rr/IkwvAd5ueVgxaXIk5bXi5RJZbdL7XQnXc+TTILQEVOJYzdO2gVIudY/HE+L/zkoRk4r+Pm3TFEQxVYNidXm0+VCTsTGTyX4BmU26xW3VGR+7e9TW2B9UQIRSpBYK9O/2te98lJvqgfrpcHrNNNimfXzE2v9yYM+WfB8uIQ6sTEd/DKFlBCOdZJwHdNcuInsrPUr8fRUjJA0KIVEbdl5TYu4BMewFgml1oN61ps//nnr+n7YLb18fpgeP+AwIDAQABo4G1MIGyMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgUgMFIGA1UdEQRLMEmkRzBFMRYwFAYFZ4EFAgIMCFNMQjkgNjcxMRYwFAYFZ4EFAgEMC2lkOjAwMDAxMDE0MRMwEQYFZ4EFAgMMC2lkOjAwMDcwMDU1MB0GA1UdDgQWBBQay3Yfjfwu8m7xJwi1E/AgMqzR4DAfBgNVHSMEGDAWgBQay3Yfjfwu8m7xJwi1E/AgMqzR4DANBgkqhkiG9w0BAQsFAAOCAQEABAlSRMJyLTklwOr4q1clNJSlxSpLGAbUADf5hoxRGsLsTm9O/G/xZlukIsAmX9mh8KKgI1ISoGEua/pHOvJooXSkxFA8EVQdrWgAtCEElKcHPmvPtgw/fqS8sB9Pm2uKysj7P8LsrYUCdaGzSrdX8sYrYyvYhgV++juxu/lehAjzv7OnUsmnwEgQpwyfNnq5BAjGGpV+7AxSnZUbcmGPubXsywFxHUDJA0vLrzl7FXkFW5+fPfOTLeWYfdOPp4a/TqiXhUFn5Q3+DNSjJCYebx6J9ROsGiuGry422GvboDpleNdiv1JSOqBig9qHfSzxVTxZchh2qCYo6C+W0GTuVA==",
  "duplicate-manufacturer": "MIIDhzCCAm+gAwIBAgIBBjANBgkqhkiG9w0BAQsFADAhMR8wHQYDVQQDDBZNeWNlbGl4IFNBTiBGaXh0dXJlIENBMB4XDTI2MTAwMTAwMDAwMFoXDTM2MTAwMTAwMDAwMFowHTEbMBkGA1UEAwwSTXljZWxpeCBFSyBGaXh0dXJlMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAgblRQIeGpnDIyDMJHyafnoH3owFucWIai5ElCVbiUBzUR5OQ24BjmDLFUrxTP9BMyWXwTfhqve+m3jHJx5EUJJk5rr/IkwvAd5ueVgxaXIk5bXi5RJZbdL7XQnXc+TTILQEVOJYzdO2gVIudY/HE+L/zkoRk4r+Pm3TFEQxVYNidXm0+VCTsTGTyX4BmU26xW3VGR+7e9TW2B9UQIRSpBYK9O/2te98lJvqgfrpcHrNNNimfXzE2v9yYM+WfB8uIQ6sTEd/DKFlBCOdZJwHdNcuInsrPUr8fRUjJA0KIVEbdl5TYu4BMewFgml1oN61ps//nnr+n7YLb18fpgeP+AwIDAQABo4HNMIHKMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgUgMGoGA1UdEQRjMGGkXzBdMRYwFAYFZ4EFAgEMC2lkOjAwMDAxMDE0MRYwFAYFZ4EFAgEMC2lkOjAwMDAxMDE1MRMwEQYFZ4EFAgIMCFNMQjkgNjcwMRYwFAYFZ4EFAgMMC2lkOjAwMDcwMDU1MB0GA1UdDgQWBBQay3Yfjfwu8m7xJwi1E/AgMqzR4DAfBgNVHSMEGDAWgBQay3Yfjfwu8m7xJwi1E/AgMqzR4DANBgkqhkiG9w0BAQsFAAOCAQEAF8+ep7kl6Cjp9U0Lcn6jLpj5wj289dD0YB67vWDFEW4Lqi3mDcrSY0GqDvrm5tqJC8KTl+jRoYNMkT9zjiMAEpASAhAeFVhfIU2MOk0snc+TImGztU+IEpnNnxLmM200T1bIJSsYRy5YFHdah8iwuxh+Ri8XsgZ1WLqZd8pazZqTdU6H4IYhklpB7cqMqBaBzJBve25Uf7LJsbEySZdZ7ZHlbDDSllX75VMw11qSGvKufv8vyhjTcN38x+rarPyBWOqQZrnU/94VDeUaNCbjALTgTI3lcQJbQGGGKe0P1d0vCuhfNxk1cr1NB08WsVpdvzweIE0UPseyrkp5Mh2KCw==",
  "missing-model": "MIIDWjCCAkKgAwIBAgIBBzANBgkqhkiG9w0BAQsFADAhMR8wHQYDVQQDDBZNeWNlbGl4IFNBTiBGaXh0dXJlIENBMB4XDTI2MTAwMTAwMDAwMFoXDTM2MTAwMTAwMDAwMFowHTEbMBkGA1UEAwwSTXljZWxpeCBFSyBGaXh0dXJlMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAgblRQIeGpnDIyDMJHyafnoH3owFucWIai5ElCVbiUBzUR5OQ24BjmDLFUrxTP9BMyWXwTfhqve+m3jHJx5EUJJk5rr/IkwvAd5ueVgxaXIk5bXi5RJZbdL7XQnXc+TTILQEVOJYzdO2gVIudY/HE+L/zkoRk4r+Pm3TFEQxVYNidXm0+VCTsTGTyX4BmU26xW3VGR+7e9TW2B9UQIRSpBYK9O/2te98lJvqgfrpcHrNNNimfXzE2v9yYM+WfB8uIQ6sTEd/DKFlBCOdZJwHdNcuInsrPUr8fRUjJA0KIVEbdl5TYu4BMewFgml1oN61ps//nnr+n7YLb18fpgeP+AwIDAQABo4GgMIGdMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgUgMD0GA1UdEQQ2MDSkMjAwMRYwFAYFZ4EFAgEMC2lkOjAwMDAxMDE0MRYwFAYFZ4EFAgMMC2lkOjAwMDcwMDU1MB0GA1UdDgQWBBQay3Yfjfwu8m7xJwi1E/AgMqzR4DAfBgNVHSMEGDAWgBQay3Yfjfwu8m7xJwi1E/AgMqzR4DANBgkqhkiG9w0BAQsFAAOCAQEAFUuFPwJmF8BmwCz3hW0c6UandELv8jJvP113KtKXbR7SMk1Cwa56x7vGnVc724/Sct5xlYSU1odJD6ftKbo16m56VJ0AUaXXylpSVddBw7dri5RTgJl24pa3WLxmZx14WC6iRWI+dH13fv0wfpDmKc3EA+PkDqLMknUH1mZHcdkbk3cHLPEeo1I00whnrLJ51UV7B3YA1S+esEds6gOjVxjTJBSc8XLKzor3cX5aej10Y5XRVoxTWpdebkEL7yJkEaXNGkheUb5WhfM7QlZiM1fxGvpF6NfZaSX6yx2zuUwUZAPBtjzTaAhh9Yk+beQQP3QjwVyCqMWoHGshKHTsfA==",
  "missing-version": "MIIDVzCCAj+gAwIBAgIBCDANBgkqhkiG9w0BAQsFADAhMR8wHQYDVQQDDBZNeWNlbGl4IFNBTiBGaXh0dXJlIENBMB4XDTI2MTAwMTAwMDAwMFoXDTM2MTAwMTAwMDAwMFowHTEbMBkGA1UEAwwSTXljZWxpeCBFSyBGaXh0dXJlMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAgblRQIeGpnDIyDMJHyafnoH3owFucWIai5ElCVbiUBzUR5OQ24BjmDLFUrxTP9BMyWXwTfhqve+m3jHJx5EUJJk5rr/IkwvAd5ueVgxaXIk5bXi5RJZbdL7XQnXc+TTILQEVOJYzdO2gVIudY/HE+L/zkoRk4r+Pm3TFEQxVYNidXm0+VCTsTGTyX4BmU26xW3VGR+7e9TW2B9UQIRSpBYK9O/2te98lJvqgfrpcHrNNNimfXzE2v9yYM+WfB8uIQ6sTEd/DKFlBCOdZJwHdNcuInsrPUr8fRUjJA0KIVEbdl5TYu4BMewFgml1oN61ps//nnr+n7YLb18fpgeP+AwIDAQABo4GdMIGaMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgUgMDoGA1UdEQQzMDGkLzAtMRYwFAYFZ4EFAgEMC2lkOjAwMDAxMDE0MRMwEQYFZ4EFAgIMCFNMQjkgNjcwMB0GA1UdDgQWBBQay3Yfjfwu8m7xJwi1E/AgMqzR4DAfBgNVHSMEGDAWgBQay3Yfjfwu8m7xJwi1E/AgMqzR4DANBgkqhkiG9w0BAQsFAAOCAQEAIcuORYDdWxxmbP4U1N1McWsom4cy7uujV6ui81LNmigDtiAecZYBsVTBXVEVoZs6Vw3K8Brvmhe5CDRQiDFKPwx3b/sC0CjR0/TSH7s6hHCuWSDRNuuD8EIVEacPxnayfEHPMeqGBqMHPixZb5D+KIoE88+rPXWX5ibKRVsWL3tVZXR1QRrq3ptSpuynanBhBG0pG4DFskgTIH7oQ0Bg6Cq6OPdWBaAbMv0PVFymvR/dg5frU6kL0MSXRHRAzMDD6+0GdFlPcAQkqRTRiIbG57Rs0mUW1BrRaYPnNP/ZMLYYJt7iZCeqEjPdE00ixK6P8bIVggzHgGjzUTbAMtje3A==",
  "firmware-mismatch": "MIIDbzCCAlegAwIBAgIBCTANBgkqhkiG9w0BAQsFADAhMR8wHQYDVQQDDBZNeWNlbGl4IFNBTiBGaXh0dXJlIENBMB4XDTI2MTAwMTAwMDAwMFoXDTM2MTAwMTAwMDAwMFowHTEbMBkGA1UEAwwSTXljZWxpeCBFSyBGaXh0dXJlMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAgblRQIeGpnDIyDMJHyafnoH3owFucWIai5ElCVbiUBzUR5OQ24BjmDLFUrxTP9BMyWXwTfhqve+m3jHJx5EUJJk5rr/IkwvAd5ueVgxaXIk5bXi5RJZbdL7XQnXc+TTILQEVOJYzdO2gVIudY/HE+L/zkoRk4r+Pm3TFEQxVYNidXm0+VCTsTGTyX4BmU26xW3VGR+7e9TW2B9UQIRSpBYK9O/2te98lJvqgfrpcHrNNNimfXzE2v9yYM+WfB8uIQ6sTEd/DKFlBCOdZJwHdNcuInsrPUr8fRUjJA0KIVEbdl5TYu4BMewFgml1oN61ps//nnr+n7YLb18fpgeP+AwIDAQABo4G1MIGyMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgUgMFIGA1UdEQRLMEmkRzBFMRYwFAYFZ4EFAgEMC2lkOjAwMDAxMDE0MRMwEQYFZ4EFAgIMCFNMQjkgNjcwMRYwFAYFZ4EFAgMMC2lkOjAwMDcwMDU2MB0GA1UdDgQWBBQay3Yfjfwu8m7xJwi1E/AgMqzR4DAfBgNVHSMEGDAWgBQay3Yfjfwu8m7xJwi1E/AgMqzR4DANBgkqhkiG9w0BAQsFAAOCAQEAS4RQKGsUKvNdDsTLGe4KrJ5L9DdmlJ3uA9S0Q4wZzifPqAg1jwDfyHcuBSgj/l6Lc/jQ9f4JikXwPtI2ydmlskHohoLjy932fWrZ2dsZLiDtZrqIEQ4u3uDfz7STcLoq255E4xd8JEO3xZ70Y7sfdnza+nB9p3ty3swxlPClKX8jHrZZkAHLaei/S89edrJ4Nfv3rp0TIoH7ouTv6P5Cic4i42AjeqyypE84ighCrNMosxjsmGGYU2afBkbs6iqBOiI8hM7GUr4MAKipneyBo+QwSyt0irfQ5TiuKNbQG9g0PxC60XOuzUyGpNnq3cQt4yNnMCzsl54jn/YypMF3DA==",
  "firmware-malformed": "9da4e1ccdd0355cbffa743552bafe1228fe95bd18e15abf48d5714ae0eec2a39"
}

def canonical_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()).hexdigest()

def valid_hash(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)

def run(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)

def x509_san_text(cert: bytes, work: Path) -> str:
    path = work / "ek.der"
    path.write_bytes(cert)
    if shutil.which("openssl") is None:
        raise RuntimeError("openssl unavailable")
    proc = run(["openssl","x509","-inform","DER","-in",str(path),"-noout","-ext","subjectAltName"], work)
    if proc.returncode != 0:
        raise ValueError("openssl subjectAltName parse failed: " + proc.stderr.strip())
    return proc.stdout

def parse_properties(text: str) -> dict[str, Any]:
    def one(name: str) -> int:
        pattern = re.compile(
            rf"(?ms)^TPM2_PT_{re.escape(name)}:\\s*\\n\\s+raw:\\s*0x([0-9A-Fa-f]+)\\s*\\n"
        )
        matches = pattern.findall(text)
        if len(matches) != 1:
            raise ValueError(f"{name} raw property count={len(matches)}")
        return int(matches[0],16)
    manufacturer = one("MANUFACTURER")
    vendor = [one(f"VENDOR_STRING_{i}") for i in range(1,5)]
    firmware = one("FIRMWARE_VERSION_1")
    model_bytes = b"".join(x.to_bytes(4,"big") for x in vendor)
    model_bytes = model_bytes.split(b"\\x00",1)[0]
    try:
        model = model_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("TPMModel vendor-string bytes are not UTF-8") from exc
    return {"manufacturer":manufacturer,"model":model,"firmware_version":firmware,"vendor_string_raw":[f"0x{x:08X}" for x in vendor]}

def split_rdn_path(path: str) -> list[str]:
    parts=[];current=[];escaped=False
    for char in path:
        if char=="\\" and not escaped:
            escaped=True
            current.append(char)
            continue
        if char=="/" and not escaped:
            parts.append("".join(current)); current=[]
            continue
        current.append(char)
        escaped=False
    if current: parts.append("".join(current))
    return [p for p in parts if p]

def parse_directory_name(san_text: str) -> dict[str, Any]:
    dirs=[]
    for line in san_text.splitlines():
        stripped=line.strip()
        if stripped.startswith("DirName:"):
            dirs.append(stripped[len("DirName:"):])
    if len(dirs)!=1:
        raise ValueError(f"expected exactly one SAN directoryName, found {len(dirs)}")
    attrs={}
    duplicates=[]
    for part in split_rdn_path(dirs[0]):
        if "=" not in part: continue
        key,value=part.split("=",1)
        if key in attrs: duplicates.append(key)
        attrs.setdefault(key, value)
    if duplicates:
        raise ValueError("duplicate SAN directoryName attributes: " + ",".join(sorted(set(duplicates))))
    required={TPM_MFG_OID,TPM_MODEL_OID,TPM_VERSION_OID}
    if required-set(attrs):
        raise ValueError("missing SAN attributes: " + ",".join(sorted(required-set(attrs))))
    return attrs

def parse_id_hex(value: str, field: str) -> int:
    m=re.fullmatch(r"id:([0-9A-Fa-f]{8})",value.strip())
    if not m: raise ValueError(f"{field} must be id: followed by eight hex digits")
    return int(m.group(1),16)

def firmware_status(cert_value: str, current: int, require_current: bool) -> tuple[str,bool]:
    cert=parse_id_hex(cert_value,"TPMVersion")
    if cert==current: return "MATCH_CURRENT", True
    if require_current: return "MISMATCH_POLICY_DENY", False
    return "MISMATCH_POSSIBLE_FIELD_UPGRADE", True

def result(state: str, reason: str, details: dict[str,Any]|None=None)->dict[str,Any]:
    out={"verifier_id":VERIFIER_ID,"state":state,"reason":reason}
    if details: out["details"]=details
    return out

def verify(m: dict[str,Any])->dict[str,Any]:
    required={
        "profile_id","profile_version","verification_mode","claim_ceiling","session_id",
        "tpm_identity_digest","ek_public_wire_sha256","certificate_der_base64",
        "certificate_der_sha256","tpm_properties_text","tpm_properties_sha256",
        "tpm_properties_source_sha256","require_current_firmware_match","verifier_source_sha256",
    }
    missing=sorted(required-set(m))
    if missing:return result("DENY","missing-required-fields",{"fields":missing})
    if m["profile_id"]!="mycelix.security.tpm.ek-cert-san-device-identity":return result("DENY","profile-id-mismatch")
    if m["profile_version"]!="0.1.0":return result("DENY","profile-version-mismatch")
    if m["claim_ceiling"]!="ReferenceModelOnly":return result("DENY","claim-ceiling-mismatch")
    if m["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:return result("DENY","verification-mode-invalid")
    if not valid_hash(m["tpm_identity_digest"]) or not valid_hash(m["ek_public_wire_sha256"]) or not valid_hash(m["certificate_der_sha256"]) or not valid_hash(m["tpm_properties_sha256"]) or not valid_hash(m["tpm_properties_source_sha256"]) or not valid_hash(m["verifier_source_sha256"]):
        return result("DENY","digest-invalid")
    if m["verifier_source_sha256"] != hashlib.sha256(Path(__file__).read_bytes()).hexdigest():
        return result("DENY","verifier-source-mismatch")
    props=m["tpm_properties_text"]
    if not isinstance(props,str):return result("DENY","tpm-properties-text-invalid")
    props_bytes=props.encode()
    if hashlib.sha256(props_bytes).hexdigest()!=m["tpm_properties_sha256"]:
        return result("DENY","tpm-properties-digest-mismatch")
    if m["verification_mode"]=="ReferenceModelOnly" and m["tpm_properties_sha256"]!=REFERENCE_PROPERTIES_SHA256:
        return result("DENY","reference-properties-not-approved")
    if m["tpm_properties_source_sha256"] != REFERENCE_PROPERTIES_SOURCE_SHA256 and m["verification_mode"]=="ReferenceModelOnly":
        return result("DENY","reference-properties-source-not-approved")
    cert=base64.b64decode(m["certificate_der_base64"],validate=True)
    if hashlib.sha256(cert).hexdigest()!=m["certificate_der_sha256"]:
        return result("DENY","certificate-digest-mismatch")
    try:
        props_data=parse_properties(props)
    except ValueError as exc:return result("DENY","tpm-properties-parse-failed",{"error":str(exc)})
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-san-") as td:
        try:san_text=x509_san_text(cert,Path(td))
        except (RuntimeError,ValueError) as exc:return result("DENY","san-parse-failed",{"error":str(exc)})
    try:
        attrs=parse_directory_name(san_text)
        mfg=parse_id_hex(attrs[TPM_MFG_OID],"TPMManufacturer")
        fw,fw_ok=firmware_status(attrs[TPM_VERSION_OID],props_data["firmware_version"],bool(m["require_current_firmware_match"]))
    except ValueError as exc:return result("DENY","san-attribute-parse-failed",{"error":str(exc)})
    if mfg != props_data["manufacturer"]:
        return result("DENY","tpm-manufacturer-mismatch",{"certificate":attrs[TPM_MFG_OID],"current_raw":f"0x{props_data['manufacturer']:08X}"})
    if attrs[TPM_MODEL_OID] != props_data["model"]:
        return result("DENY","tpm-model-mismatch",{"certificate":attrs[TPM_MODEL_OID],"current":props_data["model"]})
    if not fw_ok:return result("DENY","tpm-firmware-mismatch-policy",{"certificate":attrs[TPM_VERSION_OID],"current_raw":f"0x{props_data['firmware_version']:08X}"})
    details={
      "certificate_der_sha256":m["certificate_der_sha256"],
      "tpm_properties_sha256":m["tpm_properties_sha256"],
      "tpm_manufacturer_raw":f"0x{props_data['manufacturer']:08X}",
      "tpm_model":props_data["model"],
      "tpm_vendor_string_raw":props_data["vendor_string_raw"],
      "certificate_tpm_manufacturer":attrs[TPM_MFG_OID],
      "certificate_tpm_model":attrs[TPM_MODEL_OID],
      "certificate_tpm_version":attrs[TPM_VERSION_OID],
      "firmware_status":fw,
      "firmware_issuance_time_status":"MATCH_CURRENT" if fw=="MATCH_CURRENT" else "UNPROVABLE_ISSUANCE_TIME",
      "tpm_identity_digest":m["tpm_identity_digest"],
      "ek_public_wire_sha256":m["ek_public_wire_sha256"],
    }
    if m["verification_mode"]!="ReferenceModelOnly":
        return result("INDETERMINATE","live-origin-not-authorized-by-reference-model",details)
    return result("PASS","ek-san-device-identity-coherent",details)

def fixture()->dict[str,Any]:
    cert=base64.b64decode(FIXTURES["valid"])
    cd=hashlib.sha256(cert).hexdigest()
    pd="bc"*32
    props_sha=hashlib.sha256(PROPS_FIXTURE.encode()).hexdigest()
    session="ek-san-self-test"; tpm="44"*32
    return {
      "profile_id":"mycelix.security.tpm.ek-cert-san-device-identity",
      "profile_version":"0.1.0","verification_mode":"ReferenceModelOnly","claim_ceiling":"ReferenceModelOnly",
      "session_id":session,"tpm_identity_digest":tpm,"ek_public_wire_sha256":pd,
      "certificate_der_base64":FIXTURES["valid"],"certificate_der_sha256":cd,
      "tpm_properties_text":PROPS_FIXTURE,"tpm_properties_sha256":props_sha,
      "tpm_properties_source_sha256":REFERENCE_PROPERTIES_SOURCE_SHA256,
      "require_current_firmware_match":False,
      "verifier_source_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }

def mutate_cert(v:dict[str,Any],fixture_name:str)->None:
    b=base64.b64decode(FIXTURES[fixture_name]);v["certificate_der_base64"]=base64.b64encode(b).decode();v["certificate_der_sha256"]=hashlib.sha256(b).hexdigest()

def mutate_props(v:dict[str,Any],text_value:str)->None:
    v["tpm_properties_text"]=text_value;v["tpm_properties_sha256"]=hashlib.sha256(text_value.encode()).hexdigest()

def self_test()->int:
    base=fixture()
    cases=[
      ("canonical-valid","PASS",lambda x:x),
      ("missing-san-manufacturer","DENY",lambda x:mutate_cert(x,"missing-manufacturer")),
      ("manufacturer-mismatch","DENY",lambda x:mutate_cert(x,"manufacturer-mismatch")),
      ("model-mismatch","DENY",lambda x:mutate_cert(x,"model-mismatch")),
      ("duplicate-manufacturer","DENY",lambda x:mutate_cert(x,"duplicate-manufacturer")),
      ("missing-model","DENY",lambda x:mutate_cert(x,"missing-model")),
      ("missing-version","DENY",lambda x:mutate_cert(x,"missing-version")),
      ("firmware-mismatch-possible-upgrade","PASS",lambda x:mutate_cert(x,"firmware-mismatch")),
      ("firmware-mismatch-strict-policy","DENY",lambda x:(mutate_cert(x,"firmware-mismatch"),x.update({"require_current_firmware_match":True}))),
      ("firmware-malformed","DENY",lambda x:mutate_cert(x,"firmware-malformed")),
      ("properties-manufacturer-substitution","DENY",lambda x:mutate_props(x,PROPS_FIXTURE.replace("raw: 0x1014","raw: 0x1015"))),
      ("properties-model-substitution","DENY",lambda x:mutate_props(x,PROPS_FIXTURE.replace("value: \\"670\\"","value: \\"671\\"").replace("raw: 0x36373000","raw: 0x36373100"))),
      ("properties-source-substitution","DENY",lambda x:x.update({"tpm_properties_source_sha256":"11"*32})),
      ("certificate-digest-substitution","DENY",lambda x:x.update({"certificate_der_sha256":"22"*32})),
      ("ek-public-digest-substitution","DENY",lambda x:x.update({"ek_public_wire_sha256":"33"*32})),
      ("tpm-identity-substitution","DENY",lambda x:x.update({"tpm_identity_digest":"55"*32})),
      ("verifier-source-substitution","DENY",lambda x:x.update({"verifier_source_sha256":"66"*32})),
      ("offline-bundle","INDETERMINATE",lambda x:x.update({"verification_mode":"OfflineBundle"})),
      ("live-verifier","INDETERMINATE",lambda x:x.update({"verification_mode":"LiveVerifierSession"})),
    ]
    for name,expected,mut in cases:
        candidate=copy.deepcopy(base);mut(candidate);observed=verify(candidate)
        if observed["state"]!=expected:
            print(f"{name}: FAIL expected={expected} got={observed['state']} reason={observed['reason']}")
            return 1
    perm=json.loads(json.dumps(base,sort_keys=True))
    if verify(perm)["state"]!="PASS":
        print("key-order-permutation: FAIL");return 1
    print("EK SAN device identity semantic corpus: PASS")
    print("19 adversarial mutations plus canonical and key-order control: PASS")
    return 0

def main()->int:
    parser=argparse.ArgumentParser()
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test",action="store_true")
    group.add_argument("--verify",metavar="MANIFEST")
    parser.add_argument("--output")
    args=parser.parse_args()
    if args.self_test:return self_test()
    path=Path(args.verify).resolve();m=json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(m,dict):raise SystemExit("manifest must be an object")
    v=verify(m);out={"profile_id":"mycelix.security.tpm.ek-cert-san-device-identity","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),**v}
    out["content_sha256"]=canonical_hash({k:v for k,v in out.items() if k!="content_sha256"})
    rendered=json.dumps(out,indent=2,sort_keys=True)+"\\n"
    if args.output:Path(args.output).write_text(rendered,encoding="utf-8")
    else:print(rendered,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[v["state"]]

if __name__=="__main__":raise SystemExit(main())
