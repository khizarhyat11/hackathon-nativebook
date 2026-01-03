# Data Model: ROS 2 Book Module

**Feature**: ROS 2 Book Module
**Storage**: Markdown/MDX files
**Last Updated**: 2025-12-16

## Entity Definitions

### Chapter

A section of the module covering a specific ROS 2 topic.

| Attribute | Type | Description |
|-----------|------|-------------|
| id | string | Unique chapter identifier (e.g., "01-architecture") |
| title | string | Chapter title |
| sidebar_label | string | Short label for sidebar navigation |
| sidebar_position | integer | Order in sidebar |
| learning_objectives | array | List of learning objectives |
| content | MDX | Main chapter content |
| code_examples | array | Associated code examples |

### Code Example

Runnable Python code demonstrating a ROS 2 concept.

| Attribute | Type | Description |
|-----------|------|-------------|
| id | string | Unique example identifier |
| filename | string | Python file name |
| chapter_id | string | Associated chapter |
| description | string | What the example demonstrates |
| source_code | string | Python source code |
| expected_output | string | Expected console output |
| ros2_version | string | Minimum ROS 2 version required |

### URDF Model

XML-based robot description.

| Attribute | Type | Description |
|-----------|------|-------------|
| id | string | Model identifier |
| filename | string | URDF file name |
| robot_name | string | Robot name in URDF |
| links | array | List of link definitions |
| joints | array | List of joint definitions |
| materials | array | Visual material definitions |

### Learning Objective

A specific skill or knowledge the reader will gain.

| Attribute | Type | Description |
|-----------|------|-------------|
| id | string | Objective identifier |
| chapter_id | string | Associated chapter |
| description | string | What the learner will achieve |
| verification | string | How to verify the objective is met |

## Entity Relationships

```
┌─────────────────────┐       ┌─────────────────────┐
│      Chapter        │       │    Code Example     │
├─────────────────────┤       ├─────────────────────┤
│ id (PK)             │──────<│ id (PK)             │
│ title               │       │ chapter_id (FK)     │
│ sidebar_label       │       │ filename            │
│ sidebar_position    │       │ description         │
│ learning_objectives │       │ source_code         │
│ content             │       │ expected_output     │
└─────────────────────┘       └─────────────────────┘
         │
         │
         ▼
┌─────────────────────┐
│ Learning Objective  │
├─────────────────────┤
│ id (PK)             │
│ chapter_id (FK)     │
│ description         │
│ verification        │
└─────────────────────┘
```

## Content Structure

### Chapter Files

```text
my-website/docs/module-01-ros2/
├── _category_.json       # Sidebar category config
├── architecture.mdx      # Chapter 1
├── communication.mdx     # Chapter 2
├── ai-integration.mdx    # Chapter 3
└── urdf-modeling.mdx     # Chapter 4
```

### Code Example Files

```text
examples/ros2/
├── publisher_subscriber/
│   ├── minimal_publisher.py
│   └── minimal_subscriber.py
├── service_client/
│   ├── minimal_service.py
│   └── minimal_client.py
├── ai_agent_bridge/
│   ├── ai_agent.py
│   └── controller_bridge.py
└── urdf/
    └── humanoid_robot.urdf
```

## Frontmatter Schema

Each MDX file must include the following frontmatter:

```yaml
---
sidebar_label: "Chapter Title"
sidebar_position: 1
---
```
