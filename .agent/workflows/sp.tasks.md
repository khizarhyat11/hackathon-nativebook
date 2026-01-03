# /sp.tasks - Create Actionable Task List

---
description: Create an actionable task list from your implementation plan, organized by user story for incremental delivery.
---

## Purpose

This command creates a detailed task list that:
- Breaks down the implementation plan into actionable tasks
- Organizes tasks by user story priority
- Identifies parallel execution opportunities
- Defines clear dependencies and checkpoints

## Usage

```
/sp.tasks
```

Note: No arguments needed - this command reads from existing spec and plan documents.

## Prerequisites

- Constitution at `.specify/memory/constitution.md`
- Specification at `specs/[feature-name]/spec.md`
- Implementation plan at `specs/[feature-name]/plan.md`

## Workflow Steps

### Step 1: Read Input Documents
Load and parse:
- `specs/[feature-name]/spec.md` - User stories with priorities
- `specs/[feature-name]/plan.md` - Technical decisions and structure
- `specs/[feature-name]/data-model.md` - Entity definitions (if exists)

### Step 2: Extract User Stories
From spec.md, identify:
- User story priorities (P1, P2, P3...)
- Acceptance scenarios
- Independent test criteria

### Step 3: Map to Technical Components
From plan.md, determine:
- Files to create per story
- Dependencies between components
- Shared infrastructure needs

### Step 4: Generate Task List
Create `specs/[feature-name]/tasks.md` using template at `.specify/templates/tasks-template.md`:

```markdown
# Tasks: [FEATURE NAME]

**Input**: Design documents from `specs/[feature-name]/`
**Prerequisites**: plan.md, spec.md, data-model.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1, US2, US3...)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project structure per implementation plan
- [ ] T002 Initialize project with dependencies from plan.md
- [ ] T003 [P] Configure development tools (linting, formatting)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before user stories

⚠️ CRITICAL: No user story work can begin until this phase is complete

- [ ] T004 [Setup shared components/utilities]
- [ ] T005 [P] [Database schema/storage setup]
- [ ] T006 [P] [Core services that all stories need]

**Checkpoint**: Foundation ready - user stories can begin

---

## Phase 3: User Story 1 - [Title] (Priority: P1) 🎯 MVP

**Goal**: [From spec.md user story]
**Independent Test**: [From spec.md]

### Implementation for User Story 1

- [ ] T007 [US1] [Component/file for story 1]
- [ ] T008 [US1] [Service for story 1]
- [ ] T009 [US1] [UI for story 1]
- [ ] T010 [US1] [Integration/wiring]

**Checkpoint**: User Story 1 fully functional

---

## Phase 4: User Story 2 - [Title] (Priority: P2)

**Goal**: [From spec.md user story]
**Independent Test**: [From spec.md]

### Implementation for User Story 2

- [ ] T011 [US2] [Components for story 2]
- [ ] T012 [US2] [Services for story 2]

**Checkpoint**: User Story 2 fully functional

---

[Continue for additional user stories...]

---

## Phase N: Polish & Cross-Cutting Concerns

- [ ] TXXX [P] Documentation updates
- [ ] TXXX Code cleanup and optimization
- [ ] TXXX Final validation against spec

---

## Dependencies & Execution Order

### Phase Dependencies
- Setup → Foundational → User Stories → Polish
- User stories can run in parallel after Foundational

### Parallel Opportunities
- Tasks marked [P] can run concurrently
- Different user stories can be parallelized

---

## Implementation Strategy

### MVP First
1. Complete Setup + Foundational
2. Complete User Story 1 (P1)
3. STOP and VALIDATE
4. Deploy/demo as MVP

### Incremental Delivery
- Each user story adds value independently
- Can stop after any story checkpoint
```

### Step 5: Validate Task Coverage
Ensure:
- Every user story from spec has a phase
- Every component from plan has tasks
- Dependencies are clearly marked
- Parallel opportunities identified

### Step 6: Confirm Creation
Report to the user:
- Location of tasks file
- Total number of tasks
- Estimated phases
- Recommended execution order
- Next step: run /sp.implement

## Output Location

`specs/[feature-name]/tasks.md`

## Task Naming Conventions

- **T001-T099**: Setup and foundational tasks
- **T100-T199**: User Story 1 tasks
- **T200-T299**: User Story 2 tasks
- **T300+**: Additional stories and polish

## Related Commands

- `/sp.constitution` - Project principles
- `/sp.specify` - Feature specification (source of user stories)
- `/sp.plan` - Implementation plan (source of technical structure)
- `/sp.implement` - Execute the generated tasks (next step)
