# /sp.constitution - Establish Project Principles

---
description: Create or update the project's governing principles and development guidelines that will guide all subsequent development.
---

## Purpose

This command creates the project's constitution - a foundational document that defines:
- Core principles and values
- Development standards
- Quality requirements
- Constraints and boundaries

## Usage

```
/sp.constitution [description of principles to establish]
```

## Example

```
/sp.constitution Create principles focused on code quality, testing standards, user experience consistency, and performance requirements
```

## Workflow Steps

### Step 1: Analyze Requirements
Read the user's description to understand what principles they want to establish. Consider:
- Code quality standards
- Testing requirements
- User experience guidelines
- Performance requirements
- Security considerations
- Accessibility standards

### Step 2: Read Existing Constitution (if any)
Check if `.specify/memory/constitution.md` already exists:
- If yes, review current principles and merge with new requirements
- If no, create from scratch

### Step 3: Generate Constitution
Create/update the constitution file at `.specify/memory/constitution.md` with:

```markdown
# [Project Name] Constitution

## Project Identity

**Name**: [Project Name]
**Type**: [Application Type]
**Framework**: [Primary Framework(s)]

## Core Principles

1. **[Principle 1]**: [Description]
2. **[Principle 2]**: [Description]
3. **[Principle 3]**: [Description]
4. **[Principle 4]**: [Description]

## Standards

### Code Quality
- [Coding standards]
- [Documentation requirements]
- [Review processes]

### Testing Standards
- [Test coverage requirements]
- [Testing methodologies]
- [Quality gates]

### User Experience
- [UX principles]
- [Accessibility requirements]
- [Responsiveness standards]

### Performance Requirements
- [Load time requirements]
- [Memory constraints]
- [Scalability goals]

## Constraints

### Technical Constraints
- [Framework limitations]
- [Dependency restrictions]
- [Platform requirements]

### Process Constraints
- [Development workflow]
- [Review requirements]
- [Deployment limitations]

## Success Criteria

### Quality Success
- [Quality metrics]
- [Code health indicators]

### User Success
- [User satisfaction metrics]
- [Usability benchmarks]

### Performance Success
- [Performance benchmarks]
- [Reliability metrics]

## Governance

This constitution supersedes all other development practices for this project. Any amendments require:
1. Documentation of the proposed change
2. Review of impact on existing code
3. Migration plan for affected components

**Version**: 1.0.0 | **Created**: [DATE] | **Last Amended**: [DATE]
```

### Step 4: Confirm Creation
Report to the user that the constitution has been created and summarize the key principles established.

## Output Location

`.specify/memory/constitution.md`

## Related Commands

- `/sp.specify` - Create feature specifications (references constitution)
- `/sp.plan` - Create implementation plans (must pass constitution check)
- `/sp.tasks` - Generate tasks (follows constitution standards)
- `/sp.implement` - Execute implementation (governed by constitution)
