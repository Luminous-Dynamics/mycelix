{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://luminous-dynamics.org/schemas/mycelix/interoperability/myc-int-006c-water-i0-corpus.schema.json",
  "title": "MYC-INT-006C Water I0 Corpus",
  "type": "object",
  "additionalProperties": false,
  "required": ["corpus_id", "corpus_version", "profile", "source_refs", "subjects", "cases", "nonclaims"],
  "properties": {
    "corpus_id": {"type": "string", "const": "myc-int-006c-water-i0"},
    "corpus_version": {"type": "string", "pattern": "^[0-9]+\\.[0-9]+\\.[0-9]+$"},
    "profile": {"type": "string", "const": "runtime-neutral-semantic-conformance-v1"},
    "source_refs": {
      "type": "object",
      "additionalProperties": false,
      "required": ["scenario_issue", "evaluation_issue", "integral_source_registry_pr"],
      "properties": {
        "scenario_issue": {"type": "integer", "const": 3119},
        "evaluation_issue": {"type": "integer", "const": 3147},
        "integral_source_registry_pr": {"type": "integer", "const": 3144}
      }
    },
    "subjects": {
      "type": "object",
      "minProperties": 1,
      "propertyNames": {"pattern": "^[a-z0-9][a-z0-9._-]{2,63}$"},
      "additionalProperties": {"$ref": "#/$defs/subject"}
    },
    "cases": {
      "type": "object",
      "minProperties": 1,
      "propertyNames": {"pattern": "^I0-[A-Z]+-[0-9]{3}$"},
      "additionalProperties": {"$ref": "#/$defs/case"}
    },
    "nonclaims": {
      "type": "array",
      "minItems": 1,
      "uniqueItems": true,
      "items": {"type": "string", "minLength": 1}
    }
  },
  "$defs": {
    "semanticRef": {
      "type": "object",
      "additionalProperties": false,
      "required": ["namespace", "name", "version"],
      "properties": {
        "namespace": {"type": "string", "pattern": "^[a-z0-9][a-z0-9._-]*$"},
        "name": {"type": "string", "pattern": "^[a-z0-9][a-z0-9._-]*$"},
        "version": {"type": "string", "minLength": 1, "maxLength": 128}
      }
    },
    "subject": {
      "type": "object",
      "additionalProperties": false,
      "required": ["kind", "semantic_ref", "description"],
      "properties": {
        "kind": {
          "type": "string",
          "enum": [
            "Resource", "Issue", "Observation", "ActorReport", "EvidenceBundle",
            "Alternative", "Objection", "Recommendation", "Decision", "Authorization",
            "ImplementationAttempt", "ImplementationReceipt", "OutcomeObservation",
            "ReviewCandidate", "SupersedingDecisionCandidate", "Credential",
            "Certification", "DerivedSummary", "ForeignAuthority", "DeliveryReceipt"
          ]
        },
        "semantic_ref": {"$ref": "#/$defs/semanticRef"},
        "description": {"type": "string", "minLength": 1},
        "runtime_identity_semantics": {
          "type": "string",
          "const": "runtime identifiers are provenance only unless explicitly promoted by a source profile"
        }
      }
    },
    "disposition": {
      "type": "object",
      "additionalProperties": false,
      "required": ["class"],
      "properties": {
        "class": {"type": "string", "enum": ["Accepted", "Rejected", "Indeterminate", "Unsupported"]},
        "reason_code": {"type": "string", "pattern": "^[A-Z][A-Z0-9_]{2,63}$"}
      }
    },
    "assertion": {
      "type": "object",
      "additionalProperties": false,
      "required": ["predicate"],
      "properties": {
        "predicate": {
          "type": "string",
          "enum": [
            "DistinctSemanticSubjects", "NoEffectAuthority", "BindsExactSubject",
            "PreservesSourceSchema", "PreservesProvenance", "PreservesUnknownState",
            "PreservesConflict", "SingleLogicalEffect", "NoHistoricalMutation",
            "CreatesReviewCandidate", "NoLocalAuthority", "NoObservationPromotion",
            "NoReceiptPromotion", "NoOutcomePromotion", "DeclaresTranslationLoss",
            "IdempotentReplay", "RejectsExpiredAuthority", "RejectsStaleSchema"
          ]
        },
        "subjects": {
          "type": "array",
          "minItems": 1,
          "items": {"type": "string", "pattern": "^[a-z0-9][a-z0-9._-]{2,63}$"}
        },
        "note": {"type": "string", "minLength": 1}
      }
    },
    "case": {
      "type": "object",
      "additionalProperties": false,
      "required": ["kind", "summary", "inputs", "expected_disposition", "assertions"],
      "properties": {
        "kind": {"type": "string", "enum": ["Positive", "Hostile", "Partition", "Migration"]},
        "summary": {"type": "string", "minLength": 1},
        "inputs": {
          "type": "array",
          "minItems": 1,
          "uniqueItems": true,
          "items": {"type": "string", "pattern": "^[a-z0-9][a-z0-9._-]{2,63}$"}
        },
        "expected_disposition": {"$ref": "#/$defs/disposition"},
        "assertions": {
          "type": "array",
          "minItems": 1,
          "items": {"$ref": "#/$defs/assertion"}
        }
      }
    }
  }
}
