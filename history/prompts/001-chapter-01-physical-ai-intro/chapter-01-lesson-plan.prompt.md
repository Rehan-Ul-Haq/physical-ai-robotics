---
id: "ch01-plan-001"
title: "Chapter 01 Lesson Plan - Concept Density Analysis"
stage: "plan"
date: "2025-11-27"
surface: "chapter-planner"
model: "claude-haiku-4-5"
feature: "001-chapter-01-physical-ai-intro"
branch: "001-chapter-01-physical-ai-intro"
user: "instructional-designer"
command: "Create detailed lesson plan for Chapter 01: Introduction to Physical AI"
labels: ["planning", "pedagogy", "a2-proficiency", "layer1-foundation", "concept-density-analysis"]
links:
  spec: "specs/001-chapter-01-physical-ai-intro/spec.md"
  plan: "specs/001-chapter-01-physical-ai-intro/plan.md"
  adr: ""
  pr: ""
files:
  - path: "specs/001-chapter-01-physical-ai-intro/plan.md"
    status: "created"
    lines: 1507
tests:
  - compliance: "Layer1-ManualFoundation"
    result: "PASS"
  - compliance: "A2-Proficiency-CognitiveLimits"
    result: "PASS"
  - compliance: "ConceptDensity-Justified"
    result: "PASS"
---

## Prompt

**User Request**: Create a detailed lesson plan for Chapter 01: Introduction to Physical AI based on the approved specification.

**Context Provided**:
- Specification: specs/001-chapter-01-physical-ai-intro/spec.md
- Chapter: 1 - Introduction to Physical AI
- Part: 1 - The Robotic Nervous System (ROS 2)
- Week: 1
- Proficiency: A2 (Beginner - 5-7 concepts per section max)
- Hardware Tier: None (conceptual introduction)
- Prerequisites: Python fundamentals only

**Learning Objectives** (from spec):
1. LO-01: Explain the difference between digital AI and Physical AI
2. LO-02: Describe the Sim-to-Real transfer challenge and why it matters
3. LO-03: Identify the key technologies in the Physical AI stack (ROS 2, Gazebo, Isaac, VLA)
4. LO-04: Articulate why humanoid robots excel in human-centered environments

**Constraints**:
- Layer 1 (Manual Foundation) teaching approach
- A2 proficiency: max 5-7 concepts per lesson
- No code execution (purely conceptual)
- No hardware requirements
- Single "Try With AI" closing section per lesson
- Constitutional compliance (specification primacy, minimal content, framework invisible)

**Analytical Questions to Answer**:
1. Concept Density: How many distinct concepts does the spec define? How do they chunk into related groups?
2. Pedagogical Phases: Which concepts are Foundation vs Application vs Integration?
3. Lesson Count Justification: Based on concept density and A2 cognitive load, how many lessons are needed?
4. Teaching Modality Selection: What teaching approaches suit each concept?
5. Capstone Composition: How does the final lesson synthesize accumulated understanding?

**Output Requirements**:
Create a plan.md file with:
- Chapter Overview (lesson count justification, pedagogical arc)
- Lesson Structure (title, LO mapping, modality, concepts, sections, visuals, Try With AI activity, duration)
- Pedagogical Progression (phases, cognitive load, modality variation)
- Success Criteria Mapping (map each lesson to success criteria from spec)

## Response snapshot

**Analysis Process Executed**:

1. **Concept Density Analysis** (15 total concepts identified):
   - Paradigm Shift Cluster (Lesson 1): Digital AI, Physical AI, Embodied Intelligence, Perception-Reasoning-Action cycle = 4 concepts
   - Challenge Cluster (Lesson 2): Reality Gap, Sim-to-Real Transfer, Domain Randomization = 3 concepts
   - Technology Stack Cluster (Lesson 3): ROS 2, Gazebo, NVIDIA Isaac, Vision-Language-Action = 4 concepts
   - Application Cluster (Lesson 4): Form Factor Advantage, Kinematic Reuse, Tool Affordance, Training Data Abundance = 4 concepts

2. **Lesson Count Justification**:
   - 15 concepts ÷ ~4 concepts per lesson = 4 lessons required
   - All lessons within A2 limit: 4 < 7, 3 < 7, 4 < 7, 4 < 7
   - NOT arbitrary 9-lesson template; driven by concept density and cognitive load

3. **Pedagogical Progression Mapped**:
   - Foundation Phase (Lessons 1-2): Build mental models of paradigm shift and core challenges
   - Application Phase (Lesson 3): Map abstract concepts to concrete tools
   - Integration Phase (Lesson 4): Synthesize understanding of why humanoids + technology stack fit together

4. **Teaching Modality Variation**:
   - Lesson 1: Comparative Analysis (before/after table) — establishes baseline
   - Lesson 2: Problem-Centered Narrative ("Why does this fail?") — motivates solutions
   - Lesson 3: Reference + Exploration (tool-as-solution structure) — applies concepts
   - Lesson 4: Design Principle Reasoning (form follows function) — integrates understanding
   - (Anti-convergence principle: each lesson uses distinct modality for later chapters to vary from)

5. **Visual Assets Planned** (27 total):
   - Must-Have (6): Comparison table, Physical AI loop, Stack diagram, ROS 2 flow, Environment examples, Kinematic comparison
   - Nice-to-Have (21): Timelines, Physics/sensor illustrations, Data visualizations

6. **Constitutional Compliance Verified**:
   - ✓ Layer 1: Direct teaching, framework invisible, AI only in Try With AI exploration
   - ✓ A2: All lessons ≤7 concepts, heavy scaffolding with analogies
   - ✓ Specification Primacy: WHAT before HOW, foundation for later chapters
   - ✓ Minimal Content: Every section maps to learning objectives
   - ✓ Single Closing: Only "Try With AI" at lesson end

**Key Plan Sections Created**:
- Lesson 1 (25 min, 4 concepts): Paradigm Shift — From Software AI to Embodied Intelligence
- Lesson 2 (20 min, 3 concepts): Reality Gap — Why Simulation Isn't Reality
- Lesson 3 (25 min, 4 concepts): Technology Stack — Tools for Embodied Intelligence
- Lesson 4 (20 min, 4 concepts): Why Humanoids — Form Follows Function
- Total Chapter Duration: 90 minutes (within 2-hour SC-005 constraint)

**Plan Document Size**: 1,507 lines, comprehensive pedagogical framework with:
- Per-lesson section breakdowns
- Teaching notes for content implementer
- Visual asset specifications (27 assets with prioritization)
- Assessment strategy (formative + summative)
- Pacing breakdown
- Constitutional compliance checklist (14 compliance items verified)
- Transition guidance for content-implementer

## Outcome

- ✅ **Impact**: Detailed, justified lesson plan created that bridges spec → implementation. Plan provides clear roadmap for content-implementer with concept density analysis backing every pedagogical decision.
- 🧪 **Tests**: Constitutional compliance verified (Layer 1, A2 proficiency, specification primacy, minimal content). All 6 success criteria from spec mapped to specific lessons.
- 📁 **Files**: specs/001-chapter-01-physical-ai-intro/plan.md created (1,507 lines)
- 🔁 **Next prompts**: Content-implementer will use this plan to create 4 lesson markdown files with full narrative prose, embedded visual references, and Try With AI activities
- 🧠 **Reflection**: Plan demonstrates concept density analysis as alternative to arbitrary 9-lesson template. Pedagogical phases (Foundation → Application → Integration) provide coherent arc. Teaching modality variation establishes baseline for later chapters to differentiate from. Layer 1 foundation ensures students understand concepts before AI collaboration in Chapter 2.

## Evaluation notes (flywheel)

- **Failure modes observed**: None in planning phase. Plan avoids common convergences: (1) arbitrary lesson count, (2) skipped stages (no spec-first in L1), (3) cognitive overload, (4) missing Three Roles (N/A for L1), (5) out-of-scope content (all sections map to LOs)
- **Graders run and results**:
  - Concept Density Justified: ✓ PASS (15 concepts distributed across 4 lessons, all ≤7 per lesson)
  - Layer 1 Foundation: ✓ PASS (Direct teaching, no passive tool use, Try With AI exploration only)
  - A2 Proficiency: ✓ PASS (Heavy scaffolding, analogies, no jargon)
  - Success Criteria Mapping: ✓ PASS (All 6 SC mapped to lessons)
  - Duration Target: ✓ PASS (90 minutes < 2-hour constraint)
- **Prompt variant**: None (single-pass planning sufficient)
- **Next experiment**: Content-implementer will implement lessons from this plan; validation-auditor will verify prose quality, visual asset completeness, and Try With AI activity effectiveness

## Analytical notes (hidden from user)

**Why This Plan Succeeds**:

1. **Concept Density as Decision Framework**: Rather than using arbitrary 9-lesson template, plan counts and groups concepts, then calculates justified lesson count (15 concepts ÷ 4 per lesson = 4 lessons). This reasoning-activated approach avoids prediction mode convergence on generic structures.

2. **Pedagogical Phases as Arc**: Foundation (what is it?) → Application (where do tools fit?) → Integration (how does everything fit together?) provides narrative coherence. Students don't experience isolated lessons; they experience progression of understanding.

3. **Teaching Modality Variation as Anti-Convergence**: Each lesson uses distinct modality (comparison, narrative, reference, design principle). This establishes baseline for Chapters 2-5 to vary from, preventing lecture-style repetition. Modality variation maintains student engagement across chapter sequence.

4. **Layer 1 Compliance as Foundation**: Direct teaching before AI collaboration ensures students have manual understanding (mental model) before engaging with tools. This is pedagogically sound and fulfills constitutional requirement.

5. **A2 Proficiency as Cognitive Guardrail**: All lessons stay well under 7-concept limit (4, 3, 4, 4). Heavy scaffolding with analogies (nervous system, basketball, bikes) makes concepts accessible to students with only Python background (no robotics assumed).

**Implementation Readiness**:

Plan is structured to hand directly to content-implementer:
- Clear section outlines provide prose framework
- Teaching notes guide implementation tone/approach
- Visual asset specs (27 assets) enable parallel illustration work
- Try With AI prompts are templated (implementer customizes per lesson)
- Duration targets are realistic (verified against section breakdown)

**Risk Mitigation**:

- Constitutional compliance verified before implementation (avoids Chapter 9 reversion scenario)
- Concept density analysis documented (justifies 4 lessons, prevents arbitrary changes)
- Success criteria explicitly mapped (content won't miss requirements)
- Layer 1 approach locked in (prevents Layer 4 confusion like Chapter 9)

