# LLM Operator Rules

## Working preference from the notes
- keep replies short and manageable
- start from a roadmap or task list
- after approval, execute one task at a time

## Editing rule for future note maintenance
Do not overwrite the entire file when updating notes.
Only add or change the specific lines needed for the update, unless a full rewrite is explicitly requested.

## Keeping notes up to date
When we complete tasks, change behavior, fix bugs, or add features that affect the robot (e.g. line following, calibration, sensors, firmware, workflow), update the relevant `notes/*.md` files so the next session has current context. Examples: `overview.md` (priorities, dates), `change_log.md` (what was done and why), `commands_and_workflow.md`, `line_following.md`, `sensor_reference.md`, `system_architecture.md`, `test_plan_and_analysis.md`. Do this as part of the same session; do not leave the notes stale.

## What this means in practice
- avoid making broad destructive edits
- keep session handoff material modular
- prefer incremental updates unless the user asks for a full restructure
- after significant work, update the notes bundle so the next chat can rely on it
