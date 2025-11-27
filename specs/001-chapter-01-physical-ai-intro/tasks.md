# Tasks: Chapter 01 - Introduction to Physical AI

**Input**: Design documents from `/specs/001-chapter-01-physical-ai-intro/`
**Prerequisites**: spec.md (user stories), plan.md (lesson structure)

**Content Type**: Educational book chapter (conceptual, no code execution)
**Target**: 4 lessons, 90 minutes total, A2 proficiency

**Organization**: Tasks are grouped by user story (lesson) to enable independent implementation and validation.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story/lesson this task belongs to (US1=Lesson 1, US2=Lesson 2, etc.)
- Include exact file paths in descriptions

## Path Conventions

- **Lesson files**: `book-source/docs/01-robotic-nervous-system/01-introduction-to-physical-ai/`
- **Spec directory**: `specs/001-chapter-01-physical-ai-intro/`

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Chapter directory structure and foundational files

- [ ] T001 Verify chapter directory exists at book-source/docs/01-robotic-nervous-system/01-introduction-to-physical-ai/
- [ ] T002 Verify _category_.json exists with correct metadata (Chapter 1, Week 1, A2 proficiency)
- [ ] T003 [P] Create chapter README.md at book-source/docs/01-robotic-nervous-system/01-introduction-to-physical-ai/README.md with chapter overview

**Checkpoint**: Chapter structure ready for lesson implementation

---

## Phase 2: Foundational (Visual Assets Planning)

**Purpose**: Identify and plan visual assets needed across all lessons

**⚠️ NOTE**: Visual assets can be created in parallel with lesson content or added iteratively

- [ ] T004 Create visual-assets-checklist.md in specs/001-chapter-01-physical-ai-intro/ listing all 27 planned assets
- [ ] T005 [P] Identify must-have assets (6 critical diagrams) vs nice-to-have (21 supplementary)
- [ ] T006 [P] Determine asset creation strategy (AI-generated, diagrams-as-code, stock images)

**Checkpoint**: Visual asset plan documented, lesson implementation can proceed

---

## Phase 3: User Story 1 - Understanding the Paradigm Shift (Priority: P1) 🎯

**Goal**: Students understand the fundamental difference between Digital AI and Physical AI

**Independent Test**: Student can explain to a peer why "AI stepping into the physical world" is different from "AI generating text/images" and identify at least 3 key differences

**Maps to**: LO-01, SC-001 (90%+ accuracy on Digital vs Physical AI)

### Implementation for User Story 1

- [ ] T007 [US1] Create lesson file at book-source/docs/01-robotic-nervous-system/01-introduction-to-physical-ai/01-paradigm-shift.md
- [ ] T008 [US1] Write Section 1.1: What is Digital AI? (5 min, familiar foundation using ChatGPT/Midjourney examples)
- [ ] T009 [US1] Write Section 1.2: What is Physical AI? with comparison table (12 min, 3 key differences)
- [ ] T010 [US1] Write Section 1.3: Why This Matters (8 min, motivation and career relevance)
- [ ] T011 [US1] Write "Try With AI" closing activity with 3-question prompt template
- [ ] T012 [US1] Add placeholder references for visual assets (comparison table, flowcharts, robot diagram)
- [ ] T013 [US1] Verify A2 compliance: 4 concepts max (Digital AI, Physical AI, Embodied Intelligence, Perception-Reasoning-Action)
- [ ] T014 [US1] Verify lesson ends ONLY with "Try With AI" section (no "What's Next" or "Summary")

**Checkpoint**: Lesson 1 complete and independently readable. Student can explain Digital vs Physical AI difference.

---

## Phase 4: User Story 2 - Grasping the Sim-to-Real Challenge (Priority: P1)

**Goal**: Students understand why Sim-to-Real transfer is THE central challenge of Physical AI

**Independent Test**: Student can explain why a robot trained perfectly in simulation might fail in the real world and name at least 2 causes of the "reality gap"

**Maps to**: LO-02, SC-002 (explain Sim-to-Real demonstrating understanding)

### Implementation for User Story 2

- [ ] T015 [US2] Create lesson file at book-source/docs/01-robotic-nervous-system/01-introduction-to-physical-ai/02-reality-gap.md
- [ ] T016 [US2] Write Section 2.1: The Perfect Simulation Problem (7 min, narrative "why does it fail?")
- [ ] T017 [US2] Write Section 2.2: Why This Matters - The Sim-to-Real Challenge (8 min, core challenge framing)
- [ ] T018 [US2] Write Section 2.3: Domain Randomization - One Solution (5 min, solution framework)
- [ ] T019 [US2] Write "Try With AI" closing activity with 3-question prompt template
- [ ] T020 [US2] Add placeholder references for visual assets (reality gap diagram, physics/sensor/latency illustrations)
- [ ] T021 [US2] Verify A2 compliance: 3 concepts max (Reality Gap, Sim-to-Real Transfer, Domain Randomization)
- [ ] T022 [US2] Verify lesson ends ONLY with "Try With AI" section (no "What's Next" or "Summary")

**Checkpoint**: Lesson 2 complete. Student can explain reality gap and domain randomization.

---

## Phase 5: User Story 3 - Mapping the Technology Stack (Priority: P2)

**Goal**: Students gain high-level understanding of Physical AI technology stack without using tools yet

**Independent Test**: Student can match technologies to their purposes (ROS 2 → middleware, Gazebo → simulation, Isaac → AI platform, VLA → language+vision+action)

**Maps to**: LO-03, SC-003 (match 4/4 technologies to roles)

### Implementation for User Story 3

- [ ] T023 [US3] Create lesson file at book-source/docs/01-robotic-nervous-system/01-introduction-to-physical-ai/03-technology-stack.md
- [ ] T024 [US3] Write Section 3.1: The Stack Concept (3 min, layered tools architecture)
- [ ] T025 [US3] Write Section 3.2: ROS 2 - The Nervous System (5 min, middleware role)
- [ ] T026 [US3] Write Section 3.3: Gazebo - The Simulation Engine (5 min, physics simulation)
- [ ] T027 [US3] Write Section 3.4: NVIDIA Isaac - The AI Brain (5 min, perception+reasoning+control)
- [ ] T028 [US3] Write Section 3.5: Vision-Language-Action (VLA) (4 min, frontier unified model)
- [ ] T029 [US3] Write Section 3.6: How They Work Together (3 min, integration narrative)
- [ ] T030 [US3] Write "Try With AI" closing activity with 3-question prompt template
- [ ] T031 [US3] Add placeholder references for visual assets (stack diagram, ROS 2 data flow, Gazebo screenshot)
- [ ] T032 [US3] Verify A2 compliance: 4 concepts max (ROS 2, Gazebo, Isaac, VLA)
- [ ] T033 [US3] Verify lesson ends ONLY with "Try With AI" section (no "What's Next" or "Summary")

**Checkpoint**: Lesson 3 complete. Student can identify role of each technology in stack.

---

## Phase 6: User Story 4 - Understanding Humanoid Form Factor (Priority: P2)

**Goal**: Students understand why humanoid robots are uniquely suited for human environments

**Independent Test**: Student can articulate at least 3 reasons why humanoid form factors are advantageous in human-designed spaces

**Maps to**: LO-04, SC-004 (≥3 humanoid advantages with examples), SC-006 (big picture understanding)

### Implementation for User Story 4

- [ ] T034 [US4] Create lesson file at book-source/docs/01-robotic-nervous-system/01-introduction-to-physical-ai/04-why-humanoids.md
- [ ] T035 [US4] Write Section 4.1: Why Humanoids, Not Other Robots? (3 min, problem framing)
- [ ] T036 [US4] Write Section 4.2: Form Factor Advantage (4 min, environment fit - stairs, doors, tools)
- [ ] T037 [US4] Write Section 4.3: Kinematic Reuse (4 min, human training data morphology match)
- [ ] T038 [US4] Write Section 4.4: Tool Affordance (3 min, using human-designed tools)
- [ ] T039 [US4] Write Section 4.5: Training Data Abundance (3 min, billions of hours of human video)
- [ ] T040 [US4] Write Section 4.6: Synthesis - Why Humanoids? (2 min, complete argument)
- [ ] T041 [US4] Write "Try With AI" CAPSTONE closing activity (chapter synthesis, 3-question prompt)
- [ ] T042 [US4] Add placeholder references for visual assets (morphology comparison, stairs/doors examples, kinematic comparison)
- [ ] T043 [US4] Verify A2 compliance: 4 concepts max (Form Factor, Kinematic Reuse, Tool Affordance, Training Data)
- [ ] T044 [US4] Verify lesson ends ONLY with "Try With AI" section (no "What's Next" or "Summary")

**Checkpoint**: Lesson 4 complete. Student can articulate humanoid advantages and synthesize chapter learning.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Chapter-wide validation and quality assurance

### Content Validation

- [ ] T045 [P] Verify all 4 lessons follow Layer 1 teaching approach (direct explanation before AI collaboration)
- [ ] T046 [P] Verify all lessons end with single "Try With AI" section (no "What's Next", "Summary", or "Key Takeaways")
- [ ] T047 [P] Verify cognitive load: Lesson 1 (4 concepts), Lesson 2 (3 concepts), Lesson 3 (4 concepts), Lesson 4 (4 concepts)
- [ ] T048 [P] Verify teaching modality variation: Comparative Analysis (L1), Problem Narrative (L2), Reference (L3), Design Principle (L4)

### Constitutional Compliance

- [ ] T049 [P] Run anti-pattern check: grep for "What's Next|Key Takeaways|Summary|Stage [0-9]|Layer [0-9]" in all lesson files
- [ ] T050 [P] Verify no code examples requiring execution (conceptual chapter)
- [ ] T051 [P] Verify no hardware requirements mentioned beyond "None required"

### Learning Objective Mapping

- [ ] T052 Verify LO-01 → Lesson 1 (Digital vs Physical AI)
- [ ] T053 Verify LO-02 → Lesson 2 (Sim-to-Real challenge)
- [ ] T054 Verify LO-03 → Lesson 3 (Technology stack)
- [ ] T055 Verify LO-04 → Lesson 4 (Humanoid advantages)

### Final Validation

- [ ] T056 Read through all 4 lessons sequentially to verify narrative flow
- [ ] T057 Verify chapter completion time estimate (~90 minutes for A2 learner)
- [ ] T058 Update chapter-index.md status from "📋 Planned" to "✅ Complete" for Chapter 1

**Checkpoint**: Chapter 01 complete, validated, and ready for publication.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (Visual Assets Planning)**: Can run parallel to Phase 1
- **Phases 3-6 (Lessons 1-4)**: Depend on Phase 1 completion, can run in parallel or sequentially
- **Phase 7 (Polish)**: Depends on all lesson phases (3-6) completion

### Lesson Dependencies

- **Lesson 1 (US1)**: Can start after Setup - No dependencies on other lessons
- **Lesson 2 (US2)**: Can start after Setup - References Lesson 1 concepts but independently readable
- **Lesson 3 (US3)**: Can start after Setup - References Lessons 1-2 but independently readable
- **Lesson 4 (US4)**: Can start after Setup - Synthesizes all lessons, best done last

### Within Each Lesson

- Create file → Write sections sequentially → Add "Try With AI" → Add visual placeholders → Verify compliance

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Visual Asset Planning tasks marked [P] can run in parallel
- Lessons 1-3 can be written in parallel (different files)
- Lesson 4 benefits from Lessons 1-3 being complete (synthesis content)
- All Polish tasks marked [P] can run in parallel

---

## Parallel Execution Example: Lessons 1-3

```bash
# These lessons can be implemented in parallel (different files):
Task: "Create lesson file at .../01-paradigm-shift.md" [US1]
Task: "Create lesson file at .../02-reality-gap.md" [US2]
Task: "Create lesson file at .../03-technology-stack.md" [US3]

# Lesson 4 should be last (capstone synthesis):
Task: "Create lesson file at .../04-why-humanoids.md" [US4]
```

---

## Implementation Strategy

### MVP First (Lesson 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 3: Lesson 1 (Paradigm Shift)
3. **STOP and VALIDATE**: Read Lesson 1, verify A2 compliance, check "Try With AI"
4. Proceed if quality acceptable

### Incremental Delivery

1. Complete Setup → Structure ready
2. Add Lesson 1 → Validate independently → Chapter has one complete lesson
3. Add Lesson 2 → Validate independently → Two lessons complete
4. Add Lesson 3 → Validate independently → Three lessons complete
5. Add Lesson 4 → Validate independently → Chapter complete
6. Polish phase → Full validation → Ready for publication

### Sequential Implementation (Recommended for Single Author)

1. Setup (T001-T003)
2. Lesson 1 (T007-T014) → Validate
3. Lesson 2 (T015-T022) → Validate
4. Lesson 3 (T023-T033) → Validate
5. Lesson 4 (T034-T044) → Validate
6. Polish (T045-T058)

---

## Policy Note: "Try With AI" Section

Within this chapter (Part 1, before AI tools onboarding), each lesson must end with a single final section titled **"Try With AI"**.

**For Part 1 lessons (before tool onboarding)**:
- Instruct students to use **ChatGPT web interface** (universally accessible)
- Provide specific prompt templates students can copy-paste
- Focus on conceptual exploration, not tool mastery

**After tool onboarding (Parts 2+)**:
- Instruct learners to use their preferred AI companion tool (Gemini CLI, Claude CLI, etc.)
- Optionally provide both CLI and web variants of prompts

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Tasks** | 58 |
| **Setup Tasks** | 3 |
| **Visual Planning Tasks** | 3 |
| **Lesson 1 Tasks (US1)** | 8 |
| **Lesson 2 Tasks (US2)** | 8 |
| **Lesson 3 Tasks (US3)** | 11 |
| **Lesson 4 Tasks (US4)** | 11 |
| **Polish Tasks** | 14 |
| **Parallel Opportunities** | 18 tasks marked [P] |
| **Suggested MVP** | Lesson 1 only (T001-T003, T007-T014) |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific lesson for traceability
- Each lesson should be independently readable and valuable
- Verify "Try With AI" is the ONLY closing section (constitutional requirement)
- No code execution required - purely conceptual content
- Visual assets can be added iteratively after lesson prose is complete
- Commit after each lesson completion for incremental progress
