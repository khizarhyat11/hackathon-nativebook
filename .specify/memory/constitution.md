# Physical AI & Humanoid Robotics Textbook Constitution

## Project Identity

**Name**: AI-Native Textbook: Physical AI & Humanoid Robotics
**Type**: Educational Documentation Site
**Framework**: Docusaurus + Spec-Kit Plus

## Core Principles

1. **Human-Agent-Robot Symbiosis**: Content designed for collaborative learning between humans, AI assistants, and robotic systems
2. **AI-Native Pedagogy**: Structured for both human comprehension and AI-assisted learning
3. **Practical Rigor**: Every concept paired with executable code examples
4. **Future-Ready Skills**: Curriculum aligned with emerging industry demands

## Curriculum Structure (4 Modules)

### Module 01: The Robotic Nervous System (ROS 2)
- **Objective**: Establish the middleware foundation for robot control
- **Topics**: ROS 2 Architecture, Python Bridging (rclpy), URDF
- **Deliverable**: "Hello Robot" node + basic URDF bipedal model

### Module 02: The Digital Twin (Gazebo & Unity)
- **Objective**: Master physics simulation and high-fidelity environments
- **Topics**: Physics Engines, Rendering, Sensor Simulation
- **Deliverable**: Simulation environment with obstacle sensing

### Module 03: The AI-Robot Brain (NVIDIA Isaac™)
- **Objective**: Implement advanced perception and VSLAM
- **Topics**: Isaac Sim, Visual SLAM, Nav2 Navigation
- **Deliverable**: Robot that can map a room and navigate A to B

### Module 04: Vision-Language-Action (VLA)
- **Objective**: Convergence of LLMs and Physical Robotics (Capstone)
- **Topics**: Voice Pipeline (Whisper), LLM Task Parsing, Action Sequences
- **Deliverable**: "The Autonomous Humanoid" - voice-commanded robot

## Formatting Standards

- **Frontmatter**: Each file must include `sidebar_label` and `sidebar_position`
- **Code Blocks**: Python/C++ with syntax highlighting
- **Visuals**: Mermaid.js diagrams for ROS node graphs
- **Callouts**: Docusaurus admonitions (:::tip, :::danger) for safety warnings

## Standards

### Technology Stack
- **Framework**: Docusaurus for static site generation
- **Scaffolding**: Spec-Kit Plus for spec-driven development
- **Content Generation**: Claude Code for AI-assisted authoring
- **Output Format**: Markdown-based documentation
- **Deployment**: GitHub Pages for public accessibility

### Content Structure
- Modular chapters with clear learning objectives
- Interactive code blocks with copy functionality
- Visual diagrams for complex concepts
- Progressive difficulty from foundations to advanced topics
- Cross-references between related topics

### Target Audience
- O/A Level students with programming fundamentals
- Engineering students and professionals
- Medical professionals transitioning to healthcare robotics
- Technical learners seeking AI + robotics skills

## Constraints

### Technical Constraints
- Must use Spec-Kit Plus for all specification work
- Must use Claude Code for content generation
- Static site hosting via GitHub Pages only
- No server-side processing requirements

### Content Constraints
- Focus on intersection of physical hardware control and AI agent logic
- Code examples must be executable (Python, ROS2 preferred)
- All diagrams must be accessible (alt text required)
- Content must support both dark and light reading modes

## Success Criteria

### Deployment Success
- Zero-error build accessible via public GitHub Pages URL
- All pages load in under 3 seconds
- Mobile-responsive design works on all device sizes

### Completeness Success
- Full course curriculum covered in Docusaurus structure
- All 5 parts with learning objectives met
- Glossary and references complete

### Scalability Success
- Architecture allows for future AI-agent integration
- Content structured for machine readability
- Modular design enables easy updates and additions

## Governance

This constitution supersedes all other development practices for this project. Any amendments require:
1. Documentation of the proposed change
2. Review of impact on existing content
3. Migration plan for affected sections

**Version**: 1.0.0 | **Ratified**: 2025-12-21 | **Last Amended**: 2025-12-21
