# Hearth Planner Evidence v4

Additive planner-history adapter backed exclusively by Care Digest v4.

Current member facts still contain no historical-work field. The adapter derives recent care minutes only from Digest-v4 `known_actual_minutes`, performs checked conversion into the planner's current `u32` field, and carries assignment/time/completion exclusion provenance forward with the resulting plan.

The older Digest-v3 planner adapter remains unchanged for compatibility.
