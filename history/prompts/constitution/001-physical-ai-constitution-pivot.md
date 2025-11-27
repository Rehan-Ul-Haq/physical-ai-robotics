---
id: 001
title: Physical AI Constitution Pivot
stage: constitution
date: 2025-11-27
surface: claude-code
model: claude-opus-4-5-20250129
feature: physical-ai-book
branch: main
user: system
command: /sp.constitution
labels: [constitution, domain-pivot, physical-ai, v7.0.0]
links:
  spec: N/A
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: .specify/memory/constitution.md
    action: updated
    description: Major version bump v6.0.1 → v7.0.0 for Physical AI domain
  - path: CLAUDE.md
    action: updated
    description: Updated Active Technologies and constitution reference
  - path: context/physical_ai_book.md
    action: read
    description: Source document for Physical AI course context
tests:
  - N/A (documentation change)
---

## Prompt

Analyze the current constitution.md and ensure that it aligns well with our book for the course @context\physical_ai_book.md

## Response snapshot

Analyzed constitution v6.0.1 (AI Native Software Development) against Physical AI & Humanoid Robotics course requirements. Identified critical gaps:

1. **Preamble**: Wrong book title/purpose
2. **Core Thesis**: "Reusable intelligence" vs "Embodied intelligence + Sim-to-Real"
3. **Target Audience**: Generic developers vs robotics learners with Python basics
4. **Layer Examples**: Markdown/Docker vs ROS 2/URDF/Isaac
5. **Hardware Context**: Missing entirely (RTX GPUs, Jetson, cloud alternatives)
6. **Technology Stack**: Generic vs ROS 2, Gazebo, Isaac specific

Updated constitution to v7.0.0 with:
- New book title: "Physical AI & Humanoid Robotics"
- Section Ia: Physical AI Domain Context (technology stack, hardware tiers)
- Section IIb: Physical AI-Specific Pedagogical Conflicts
- Hardware tier decision framework integrated throughout
- All layer examples updated to ROS 2/robotics domain
- Sim-to-Real considerations added to Layer 4 capstone

Preserved all pedagogical frameworks (4-Layer Method, 7 Principles, Agent Coordination).

## Outcome

- ✅ Impact: MAJOR - Complete domain pivot for new book project
- 🧪 Tests: N/A (documentation)
- 📁 Files: 2 modified (constitution.md, CLAUDE.md), 1 read (physical_ai_book.md)
- 🔁 Next prompts: Update spec-template.md and plan-template.md for hardware context
- 🧠 Reflection: The pedagogical framework (4-Layer, 7 Principles) is domain-agnostic and transfers cleanly. Only examples and domain context needed updating.

## Evaluation notes (flywheel)

- Failure modes observed: None - clean domain pivot
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate that chapter-planner agent correctly applies hardware tier requirements when planning Physical AI modules
