---
name: Orval + zod v3 integer pitfall
description: OpenAPI `type: integer` breaks the api-zod codegen in this workspace
---
Rule: in `lib/api-spec/openapi.yaml`, use `type: number` (not `type: integer`) for numeric fields.

**Why:** orval v8 emits `zod.int()` for `type: integer`, but the workspace pins zod v3 (catalog), which has no `z.int` — `pnpm run codegen` then fails typecheck in the generated `lib/api-zod/src/generated/api.ts`.

**How to apply:** whenever extending the OpenAPI spec, declare integers as `type: number` (optionally with min/max) until zod is upgraded to v4.
