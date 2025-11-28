# Tasks: Chapter 02 - ROS 2 Fundamentals

**Input**: Design documents from `/specs/002-chapter-02-ros2-fundamentals/`
**Prerequisites**: plan.md, spec.md
**Organization**: Tasks are grouped by user story (lessons) to enable independent implementation

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which lesson/user story this task belongs to (L1, L2, L3, L4, L5, L6)
- Include exact file paths in descriptions

## Path Conventions

- Lesson files: `book-source/docs/01-robotic-nervous-system/02-ros2-fundamentals/`
- Each lesson: `lesson-N-[slug].md`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Chapter structure and shared assets

- [ ] T001 Create chapter directory structure at `book-source/docs/01-robotic-nervous-system/02-ros2-fundamentals/`
- [ ] T002 [P] Create chapter README.md with overview and lesson links
- [ ] T003 [P] Prepare shared assets directory for diagrams and images

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Research and verification that must be complete before lesson content creation

**⚠️ CRITICAL**: No lesson content can be written until these are verified

- [ ] T004 Verify all ROS 2 Humble commands on Ubuntu 22.04 (create verification log)
- [ ] T005 [P] Test installation procedure on clean Ubuntu 22.04 system (document steps)
- [ ] T006 [P] Verify turtlesim examples work as expected (capture screenshots)
- [ ] T007 [P] Test all rclpy code examples for Lesson 4-6 (ensure they execute without errors)
- [ ] T008 Create ROS 2 command reference sheet for students (common commands + expected outputs)

**Checkpoint**: Foundation ready - lesson content creation can now begin in parallel

---

## Phase 3: Lesson 1 - What is ROS 2 and Why Robots Need Middleware (Priority: P1) 🎯 MVP

**Goal**: Students understand what ROS 2 is, why robots need middleware, and the problems ROS 2 solves

**Independent Test**: Student can explain to a peer why ROS 2 exists and identify 3 problems it solves

**Maps to**: LO-01, User Story 1 (Understanding ROS 2's Role)

### Implementation for Lesson 1

- [ ] T009 [P] [L1] Create lesson file `book-source/docs/01-robotic-nervous-system/02-ros2-fundamentals/lesson-1-what-is-ros2.md`
- [ ] T010 [P] [L1] Write "What is Middleware?" section explaining inter-process communication challenges
- [ ] T011 [P] [L1] Create robot system diagram showing components needing communication (sensors, actuators, planning)
- [ ] T012 [L1] Write "Problems ROS 2 Solves" section (3 key problems with examples)
- [ ] T013 [L1] Write "ROS vs Operating System" clarification section
- [ ] T014 [L1] Create "Check Your Understanding" section with 3 conceptual questions
- [ ] T015 [L1] Add "Try With AI" section (Layer 1, minimal AI use—conceptual exploration only)
- [ ] T016 [L1] Verify cognitive load: 3 concepts total (within A2 limit)
- [ ] T017 [L1] Verify hardware tier stated: "No hardware required for this lesson"

**Checkpoint**: Lesson 1 should be readable, conceptually clear, and establish ROS 2 mental model

---

## Phase 4: Lesson 2 - Installing ROS 2 Humble (Priority: P1)

**Goal**: Students successfully install ROS 2 Humble on Ubuntu 22.04 and verify it works

**Independent Test**: Student runs `ros2 --version` and launches turtlesim successfully

**Maps to**: LO-02, User Story 2 (Installing and Configuring ROS 2)

### Implementation for Lesson 2

- [ ] T018 [P] [L2] Create lesson file `book-source/docs/01-robotic-nervous-system/02-ros2-fundamentals/lesson-2-installing-ros2-humble.md`
- [ ] T019 [P] [L2] Write "Prerequisites Check" section (Ubuntu 22.04, Python 3.10+, disk space)
- [ ] T020 [L2] Write step-by-step installation instructions using apt packages (based on T005 verification)
- [ ] T021 [L2] Add "Verification Steps" section (version check, environment sourcing, turtlesim launch)
- [ ] T022 [L2] Write "Troubleshooting Common Issues" section (5+ common problems with solutions)
- [ ] T023 [L2] Add "Cloud Alternative" section for students without Ubuntu (AWS RoboMaker or GitHub Codespaces)
- [ ] T024 [L2] Create "Check Your Understanding" section verifying installation success
- [ ] T025 [L2] Add "Try With AI" section (Layer 1, minimal—students can ask AI about installation errors)
- [ ] T026 [L2] Verify cognitive load: 2 concepts (Humble distribution, environment setup)
- [ ] T027 [L2] Verify hardware tier stated: "Simulation Only—any computer + Ubuntu 22.04"

**Checkpoint**: Students should have working ROS 2 Humble installation before proceeding

---

## Phase 5: Lesson 3 - Exploring ROS 2 Graph with CLI (Priority: P2)

**Goal**: Students use CLI tools to explore nodes, topics, services and build intuitive understanding of ROS 2 graph

**Independent Test**: Student can use `ros2 node/topic/service` commands to analyze turtlesim and explain what they see

**Maps to**: LO-04, User Story 3 (Exploring ROS 2 Graph with CLI Tools)

### Implementation for Lesson 3

- [ ] T028 [P] [L3] Create lesson file `book-source/docs/01-robotic-nervous-system/02-ros2-fundamentals/lesson-3-exploring-ros2-graph-cli.md`
- [ ] T029 [P] [L3] Write "Launch Turtlesim" section with exact commands
- [ ] T030 [L3] Write "Discovering Nodes" section (`ros2 node list`, `ros2 node info`) with expected outputs
- [ ] T031 [L3] Write "Exploring Topics" section (`ros2 topic list`, `ros2 topic echo`, `ros2 topic info`)
- [ ] T032 [L3] Write "Understanding Topic Direction" section (cmd_vel as input, pose as output)
- [ ] T033 [L3] Write "Discovering Services" section (`ros2 service list`, `ros2 service call /spawn`)
- [ ] T034 [L3] Create "ROS 2 Graph Concept" section explaining how nodes/topics/services form the graph
- [ ] T035 [L3] Add hands-on exercise: "Spawn a second turtle and observe the graph changes"
- [ ] T036 [L3] Create "Check Your Understanding" section with CLI command questions
- [ ] T037 [L3] Add "Try With AI" section (Layer 1, minimal—students explore commands with AI guidance)
- [ ] T038 [L3] Verify cognitive load: 5 concepts (nodes, topics, topic direction, services, graph)
- [ ] T039 [L3] Verify hands-on discovery modality implemented (execute → observe → understand)

**Checkpoint**: Students should confidently use ROS CLI tools and understand the graph structure

---

## Phase 6: Lesson 4 - Create Your First Python Node (Priority: P2)

**Goal**: Students write their first ROS 2 Python node using rclpy with proper initialization and callbacks

**Independent Test**: Student creates a "Hello ROS 2" node that runs without errors

**Maps to**: LO-03, User Story 4 (Creating First ROS 2 Node with Python)

### Implementation for Lesson 4

- [ ] T040 [P] [L4] Create lesson file `book-source/docs/01-robotic-nervous-system/02-ros2-fundamentals/lesson-4-first-python-node.md`
- [ ] T041 [P] [L4] Write "Node Anatomy" section explaining rclpy.init(), Node class, rclpy.spin()
- [ ] T042 [L4] Create minimal "Hello ROS 2" node example (verified in T007)
- [ ] T043 [L4] Write "Understanding Callbacks" section with timer callback example
- [ ] T044 [L4] Write "Running Your Node" section with execution instructions
- [ ] T045 [L4] Add "Debugging Common Errors" section (environment not sourced, import errors, node name conflicts)
- [ ] T046 [L4] Create hands-on exercise: "Modify timer interval and observe behavior"
- [ ] T047 [L4] Add "Try With AI" section (Layer 2) demonstrating Three Roles:
  - AI as Teacher: Explains callback patterns (student didn't know)
  - AI as Student: Student corrects AI's logging approach (ROS-native practices)
  - Convergence: Iterative refinement on error handling
- [ ] T048 [L4] Verify Three Roles framework is INVISIBLE (no role labels, shown through action)
- [ ] T049 [L4] Verify cognitive load: 4 concepts (rclpy init, Node class, spinning, callbacks)
- [ ] T050 [L4] Verify "Try With AI" is ONLY closing section (no "What's Next" or "Key Takeaways")

**Checkpoint**: Students should be able to create and run basic ROS 2 Python nodes

---

## Phase 7: Lesson 5 - Publisher-Subscriber Pattern (Priority: P1)

**Goal**: Students create publisher and subscriber nodes that successfully exchange messages

**Independent Test**: Student runs two separate nodes (publisher + subscriber) and observes message flow

**Maps to**: LO-05, User Story 5 (Building Publisher-Subscriber Communication)

### Implementation for Lesson 5

- [ ] T051 [P] [L5] Create lesson file `book-source/docs/01-robotic-nervous-system/02-ros2-fundamentals/lesson-5-publisher-subscriber.md`
- [ ] T052 [P] [L5] Write "Publisher Concept" section explaining one-way message streaming
- [ ] T053 [L5] Create publisher node example (`talker.py`) publishing to `/chatter` topic (verified in T007)
- [ ] T054 [L5] Write "Understanding create_publisher()" section (message type, topic name, queue size)
- [ ] T055 [L5] Write "Subscriber Concept" section explaining callback-based message reception
- [ ] T056 [L5] Create subscriber node example (`listener.py`) subscribing to `/chatter` (verified in T007)
- [ ] T057 [L5] Write "Message Types" section (`std_msgs/String`, where to find message definitions)
- [ ] T058 [L5] Add "Running Pub-Sub System" section (two terminals, observing communication)
- [ ] T059 [L5] Write "Debugging Topic Communication" section (topic name mismatches, message type errors)
- [ ] T060 [L5] Create hands-on exercise: "Create custom topic and modify message content"
- [ ] T061 [L5] Add "Try With AI" section (Layer 2) demonstrating Three Roles:
  - AI as Teacher: Explains queue size parameter implications
  - AI as Student: Student corrects timing optimization advice
  - Convergence: Debugging why subscriber doesn't receive messages
- [ ] T062 [L5] Verify Three Roles framework is INVISIBLE
- [ ] T063 [L5] Verify cognitive load: 5 concepts (publisher, subscriber, message types, topic naming, queue size)
- [ ] T064 [L5] Verify "Try With AI" is ONLY closing section

**Checkpoint**: Students should understand pub-sub pattern and be able to create communicating nodes

---

## Phase 8: Lesson 6 - Services (Request-Response) (Priority: P3)

**Goal**: Students understand when to use services vs topics and create a working service server-client pair

**Independent Test**: Student creates an addition service (inputs: 2 numbers, output: sum) with working client

**Maps to**: LO-06, User Story 6 (Understanding Services for Request-Response)

### Implementation for Lesson 6

- [ ] T065 [P] [L6] Create lesson file `book-source/docs/01-robotic-nervous-system/02-ros2-fundamentals/lesson-6-services.md`
- [ ] T066 [P] [L6] Write "Topics vs Services" section (when to use continuous streaming vs on-demand requests)
- [ ] T067 [L6] Create service server example (`add_two_ints_server.py`) using `example_interfaces/srv/AddTwoInts` (verified in T007)
- [ ] T068 [L6] Write "Understanding create_service()" section (service type, service name, callback)
- [ ] T069 [L6] Create service client example (`add_two_ints_client.py`) (verified in T007)
- [ ] T070 [L6] Write "Service Types" section (where to find service definitions, request/response structure)
- [ ] T071 [L6] Add "Running Service System" section (server terminal + client terminal)
- [ ] T072 [L6] Write "Debugging Services" section (service not found, timeout issues, type mismatches)
- [ ] T073 [L6] Create hands-on exercise: "Create a service that returns current timestamp"
- [ ] T074 [L6] Add "Try With AI" section (Layer 2) demonstrating Three Roles:
  - AI as Teacher: Explains async service patterns for defensive programming
  - AI as Student: Student chooses between async/sync approaches based on use case
  - Convergence: Debugging service timing and availability issues
- [ ] T075 [L6] Verify Three Roles framework is INVISIBLE
- [ ] T076 [L6] Verify cognitive load: 3 concepts (service concept, server, client)
- [ ] T077 [L6] Verify "Try With AI" is ONLY closing section

**Checkpoint**: Students should understand when to use services and be able to create service-based interactions

---

## Phase 9: Chapter Integration & Polish

**Purpose**: Cross-cutting concerns and chapter-level assets

- [ ] T078 [P] Update chapter README with all 6 lesson links and descriptions
- [ ] T079 [P] Create chapter summary diagram showing progression (Concepts → CLI → Nodes → Pub-Sub → Services)
- [ ] T080 [P] Verify all code examples use consistent style and ROS 2 best practices
- [ ] T081 [P] Verify all lessons state hardware tier: "Simulation Only"
- [ ] T082 [P] Verify Layer 1 lessons (1-3) have minimal/no AI collaboration sections
- [ ] T083 [P] Verify Layer 2 lessons (4-6) demonstrate Three Roles WITHOUT exposing framework
- [ ] T084 [P] Verify NO lessons have "What's Next" or "Key Takeaways" sections (only "Try With AI")
- [ ] T085 [P] Create "Chapter 2 Completion Checklist" for students (6 skills they should have)
- [ ] T086 Run grammar/spelling check on all lessons
- [ ] T087 Update `book-source/docs/chapter-index.md` to mark Chapter 2 status as "Complete"

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all lesson content
- **Lessons (Phases 3-8)**: All depend on Foundational phase completion
  - Lessons 1-3 (L1 Manual) can proceed in sequence or parallel
  - Lessons 4-6 (L2 AI Collab) depend on Lessons 1-3 conceptual foundation
- **Integration & Polish (Phase 9)**: Depends on all lessons being complete

### Lesson Dependencies

- **Lesson 1**: Can start after Foundational - No dependencies
- **Lesson 2**: Can start after Foundational - No dependencies (but logically after Lesson 1)
- **Lesson 3**: Depends on Lesson 2 (needs working ROS installation)
- **Lesson 4**: Depends on Lessons 1-3 (needs ROS concepts and installation)
- **Lesson 5**: Depends on Lesson 4 (builds on node creation)
- **Lesson 6**: Depends on Lesson 4 (builds on node creation)

### Parallel Opportunities

- All Setup tasks (T001-T003) can run in parallel
- All Foundational tasks (T004-T008) can run in parallel
- Within each lesson, tasks marked [P] can run in parallel:
  - Lesson 1: T009-T011 (file creation + writing + diagram)
  - Lesson 2: T018-T019 (file creation + prerequisites)
  - Lesson 3: T028-T030 (file creation + initial sections)
  - Lesson 4: T040-T042 (file creation + anatomy + example)
  - Lesson 5: T051-T053 (file creation + concept + publisher example)
  - Lesson 6: T065-T067 (file creation + comparison + server example)
- All Integration & Polish tasks (T078-T087) can run in parallel after lessons complete

---

## Parallel Example: Lesson 5 (Publisher-Subscriber)

```bash
# Launch all parallel file creation and concept writing together:
Task: "Create lesson file book-source/docs/.../lesson-5-publisher-subscriber.md"
Task: "Write Publisher Concept section"
Task: "Create publisher node example (talker.py)"

# Then sequential tasks that depend on examples existing:
Task: "Write Understanding create_publisher() section"
Task: "Create subscriber node example (listener.py)"
# ...continue with debugging and Try With AI sections
```

---

## Implementation Strategy

### MVP First (Lessons 1-2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - verify all commands work)
3. Complete Phase 3: Lesson 1 (Conceptual foundation)
4. Complete Phase 4: Lesson 2 (Installation)
5. **STOP and VALIDATE**: Students can understand ROS and have it installed
6. Deploy/review if ready

### Incremental Delivery

1. Complete Setup + Foundational → Commands verified, ready to write
2. Add Lesson 1 → Test readability → Publish
3. Add Lesson 2 → Test installation procedure → Publish
4. Add Lesson 3 → Test CLI exercises → Publish
5. Add Lesson 4 → Test Python node creation → Publish
6. Add Lesson 5 → Test pub-sub communication → Publish
7. Add Lesson 6 → Test services → Publish
8. Each lesson adds value and builds on previous lessons sequentially

### Parallel Team Strategy

With multiple content creators:

1. Team completes Setup + Foundational together (verify commands)
2. Once Foundational done:
   - Writer A: Lessons 1-2 (conceptual + installation)
   - Writer B: Lesson 3 (CLI exploration)
   - Writer C: Lessons 4-5 (Python nodes + pub-sub)
   - Writer D: Lesson 6 (services)
3. Lessons integrate sequentially but can be drafted in parallel

---

## Notes

### Educational Content Specifics

- Each lesson is an independent learning unit (like a user story for traditional features)
- Layer 1 lessons (1-3): Manual foundation, minimal AI use
- Layer 2 lessons (4-6): AI collaboration with Three Roles framework (invisible to students)
- "Try With AI" is the ONLY permitted final section (per constitution)
- All code examples MUST be verified on ROS 2 Humble with Ubuntu 22.04 before inclusion
- Cognitive load strictly managed: A2 tier = max 5-7 concepts per lesson

### Quality Gates

- [ ] All ROS commands verified (T004-T008)
- [ ] All code examples tested (T007)
- [ ] Three Roles demonstrated in L2 lessons but framework invisible (T048, T062, T075)
- [ ] No meta-commentary exposing pedagogical scaffolding (T084)
- [ ] Hardware tier stated in every lesson (T081)
- [ ] Specification Primacy maintained (WHAT before HOW in each lesson)
- [ ] Cognitive load within limits (verified in T016, T026, T038, T049, T063, T076)

### Constitutional Compliance

- ✅ Concept-density driven: 6 lessons justified by 20 concepts and 6 user scenarios
- ✅ Layer progression: L1 (Lessons 1-3) → L2 (Lessons 4-6)
- ✅ Hands-on discovery: Execute → observe → understand modality
- ✅ A2 proficiency: All lessons within 5-7 concept limit
- ✅ Anti-convergence: Differs from Chapter 1 direct teaching approach
- ✅ Three Roles invisible: Framework demonstrated through action, not exposition
