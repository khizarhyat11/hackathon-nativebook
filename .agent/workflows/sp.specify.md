# /sp.specify - Create Feature Specification

---
description: Create a feature specification describing what you want to build, focusing on the what and why, not the tech stack.
---

## Purpose

This command creates a technology-agnostic feature specification that defines:
- User stories and scenarios
- Functional requirements
- Success criteria
- Edge cases

## Usage

```
/sp.specify [description of what you want to build]
```

## Example

```
/sp.specify Build an application that can help me organize my photos in separate photo albums. Albums are grouped by date and can be re-organized by dragging and dropping on the main page. Albums are never in other nested albums. Within each album, photos are previewed in a tile-like interface.
```

## Workflow Steps

### Step 1: Parse User Description
Extract key concepts from the user's description:
- Primary functionality
- User interactions
- Data entities
- Relationships between entities

### Step 2: Reference Constitution
Read `.specify/memory/constitution.md` to ensure the specification aligns with project principles.

### Step 3: Create Feature Directory
Create a new directory under `specs/[feature-name]/` for this feature.

### Step 4: Generate Specification
Create `specs/[feature-name]/spec.md` using the template at `.specify/templates/spec-template.md`:

```markdown
# Feature Specification: [FEATURE NAME]

**Feature Branch**: `[###-feature-name]`
**Created**: [DATE]
**Status**: Draft
**Input**: User description: "[original user input]"

## User Scenarios & Testing

### User Story 1 - [Brief Title] (Priority: P1)

[Describe this user journey in plain language]

**Why this priority**: [Explain the value and why it has this priority level]

**Independent Test**: [Describe how this can be tested independently]

**Acceptance Scenarios**:

1. **Given** [initial state], **When** [action], **Then** [expected outcome]
2. **Given** [initial state], **When** [action], **Then** [expected outcome]

---

### User Story 2 - [Brief Title] (Priority: P2)

[Continue for additional stories...]

---

### Edge Cases

- What happens when [boundary condition]?
- How does system handle [error scenario]?

## Requirements

### Functional Requirements

- **FR-001**: System MUST [specific capability]
- **FR-002**: System MUST [specific capability]
- **FR-003**: Users MUST be able to [key interaction]

### Key Entities

- **[Entity 1]**: [What it represents, key attributes]
- **[Entity 2]**: [What it represents, relationships]

## Success Criteria

### Measurable Outcomes

- **SC-001**: [Measurable metric]
- **SC-002**: [Measurable metric]
- **SC-003**: [User satisfaction metric]
```

### Step 5: Confirm Creation
Report to the user:
- Location of the specification file
- Summary of user stories identified
- Any clarifications needed

## Output Location

`specs/[feature-name]/spec.md`

## Important Notes

- Focus on WHAT and WHY, not HOW
- No technology choices in this phase
- User stories should be independently testable
- Requirements should be measurable
- Mark unclear requirements with [NEEDS CLARIFICATION]

## Related Commands

- `/sp.constitution` - Project principles (referenced by spec)
- `/sp.plan` - Create technical implementation plan (uses spec as input)
- `/sp.tasks` - Generate tasks from plan
- `/sp.implement` - Execute implementation
