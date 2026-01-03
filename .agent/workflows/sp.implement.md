# /sp.implement - Execute Implementation

---
description: Execute all tasks and build your feature according to the implementation plan.
---

## Purpose

This command executes the task list, implementing the feature by:
- Following the task order from tasks.md
- Creating files according to plan.md structure
- Implementing functionality per spec.md requirements
- Validating against success criteria

## Usage

```
/sp.implement
```

Or to implement a specific phase/story:

```
/sp.implement [phase or user story]
```

## Examples

```
/sp.implement                    # Implement all tasks
/sp.implement Phase 1           # Setup only
/sp.implement User Story 1      # MVP implementation
```

## Prerequisites

- Constitution at `.specify/memory/constitution.md`
- Specification at `specs/[feature-name]/spec.md`
- Implementation plan at `specs/[feature-name]/plan.md`
- Task list at `specs/[feature-name]/tasks.md`

## Workflow Steps

### Step 1: Load All Documents
Read and parse:
- `specs/[feature-name]/tasks.md` - Ordered task list
- `specs/[feature-name]/plan.md` - Technical structure
- `specs/[feature-name]/spec.md` - Requirements and acceptance criteria
- `.specify/memory/constitution.md` - Quality standards

### Step 2: Determine Scope
Based on user input:
- **No arguments**: Execute all uncompleted tasks
- **Phase specified**: Execute only that phase
- **User Story specified**: Execute that story's tasks

### Step 3: Execute Tasks in Order

For each task in sequence:

1. **Read task description** from tasks.md
2. **Identify target files** from plan.md structure
3. **Implement the task**:
   - Create new files as needed
   - Modify existing files
   - Follow coding standards from constitution
4. **Mark task complete** in tasks.md:
   - Change `- [ ] T001` to `- [x] T001`
5. **Validate at checkpoints**:
   - Run any tests if specified
   - Verify acceptance scenarios
   - Check constitution compliance

### Step 4: Handle Parallel Tasks

For tasks marked with [P]:
- Group them together
- Execute concurrently when possible
- Ensure no file conflicts

### Step 5: Checkpoint Validation

At each phase checkpoint:
1. Verify all phase tasks are complete
2. Run validation steps
3. Report status to user
4. Confirm before proceeding to next phase

### Step 6: Final Validation

After all tasks complete:
1. Review against spec.md acceptance scenarios
2. Check constitution compliance
3. Verify success criteria are met
4. Generate completion report

## Task Execution Format

When executing each task, follow this pattern:

```markdown
### Executing Task T001

**Task**: Create project structure per implementation plan
**Status**: In Progress

**Actions**:
1. Creating directory structure...
2. Initializing files...
3. [Actual implementation work]

**Result**: ✅ Complete
```

## Handling Issues

If a task cannot be completed:
1. Document the blocker in tasks.md
2. Mark task with ⚠️ warning
3. Report to user with options:
   - Skip and continue
   - Resolve blocker and retry
   - Abort implementation

## Progress Tracking

Update tasks.md after each task:

```markdown
- [x] T001 Create project structure per implementation plan ✅
- [x] T002 Initialize project with dependencies ✅
- [ ] T003 [P] Configure development tools (in progress)
- [ ] T004 Setup shared components
```

## Output

### During Implementation
- Real-time progress updates
- Files created/modified
- Checkpoint validations

### After Completion
- Summary of completed tasks
- Any skipped/blocked tasks
- Validation results
- Links to created files

## Quality Gates

Before marking implementation complete:

1. **All required tasks done** - No unchecked P1 story tasks
2. **Constitution check passed** - Meets all quality standards
3. **Acceptance scenarios verified** - Core user journeys work
4. **No critical issues** - Build succeeds, no blocking errors

## Related Commands

- `/sp.constitution` - Project principles (quality standards)
- `/sp.specify` - Feature specification (requirements source)
- `/sp.plan` - Implementation plan (structure guide)
- `/sp.tasks` - Task list (execution input)

## Tips

- Start with `/sp.implement User Story 1` for MVP
- Validate at each checkpoint before proceeding
- Use parallel execution for [P] tasks when possible
- Keep tasks.md updated as single source of truth
