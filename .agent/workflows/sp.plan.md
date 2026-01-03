# /sp.plan - Create Technical Implementation Plan

---
description: Create a technical implementation plan with your tech stack and architecture choices.
---

## Purpose

This command creates a detailed technical implementation plan that defines:
- Technology stack decisions
- Architecture patterns
- Project structure
- Technical constraints
- Data models and contracts

## Usage

```
/sp.plan [tech stack and architecture preferences]
```

## Example

```
/sp.plan The application uses Vite with minimal number of libraries. Use vanilla HTML, CSS, and JavaScript as much as possible. Images are not uploaded anywhere and metadata is stored in a local SQLite database.
```

## Prerequisites

- Constitution must exist at `.specify/memory/constitution.md`
- Specification must exist at `specs/[feature-name]/spec.md`

## Workflow Steps

### Step 1: Parse Technical Preferences
Extract from user input:
- Framework/build tool preferences
- Language preferences
- Library restrictions
- Storage requirements
- Deployment considerations

### Step 2: Constitution Check
Read `.specify/memory/constitution.md` and verify the technical choices align with:
- Allowed technologies
- Performance requirements
- Quality standards
- Constraints

### Step 3: Review Feature Specification
Read `specs/[feature-name]/spec.md` to understand:
- User stories to implement
- Functional requirements
- Data entities
- Success criteria

### Step 4: Generate Implementation Plan
Create `specs/[feature-name]/plan.md` using template at `.specify/templates/plan-template.md`:

```markdown
# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link to spec.md]
**Input**: Feature specification from `specs/[feature-name]/spec.md`

## Summary

[Primary requirement + technical approach]

## Technical Context

**Language/Version**: [e.g., JavaScript ES2022, Python 3.11]
**Primary Dependencies**: [e.g., Vite, SQLite]
**Storage**: [e.g., Local SQLite database]
**Testing**: [e.g., Vitest, manual testing]
**Target Platform**: [e.g., Web browser, Desktop]
**Project Type**: [single/web/mobile]
**Performance Goals**: [e.g., 60fps interactions, <100ms load]
**Constraints**: [e.g., No external uploads, offline-capable]
**Scale/Scope**: [e.g., Single user, local files only]

## Constitution Check

| Principle | Status | Notes |
|-----------|--------|-------|
| [Principle 1] | ✅ Pass | [How it's met] |
| [Principle 2] | ✅ Pass | [How it's met] |

## Project Structure

### Documentation (this feature)

```text
specs/[feature-name]/
├── spec.md              # Feature specification
├── plan.md              # This file
├── data-model.md        # Data entities
├── contracts/           # API contracts (if applicable)
└── tasks.md             # Task list (created by /sp.tasks)
```

### Source Code

```text
[Project structure based on technical choices]
```

**Structure Decision**: [Rationale for chosen structure]

## Data Model

### Entities

[Define entities from spec with technical details]

## API Contracts (if applicable)

[Define interfaces/contracts]

## Complexity Tracking

| Decision | Justification | Simpler Alternative Rejected |
|----------|---------------|------------------------------|
| [Choice] | [Why needed] | [What was considered] |
```

### Step 5: Create Supporting Documents
If needed, create additional files:
- `specs/[feature-name]/data-model.md` - Detailed entity definitions
- `specs/[feature-name]/contracts/` - API contracts

### Step 6: Confirm Creation
Report to the user:
- Location of the plan
- Summary of technical decisions
- Constitution check results
- Next steps (run /sp.tasks)

## Output Location

- `specs/[feature-name]/plan.md`
- `specs/[feature-name]/data-model.md` (if applicable)
- `specs/[feature-name]/contracts/` (if applicable)

## Related Commands

- `/sp.constitution` - Project principles (checked against)
- `/sp.specify` - Feature specification (input)
- `/sp.tasks` - Generate tasks from plan (next step)
- `/sp.implement` - Execute implementation
