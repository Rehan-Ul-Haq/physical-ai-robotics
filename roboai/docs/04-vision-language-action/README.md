---
title: "Part 4: Vision-Language-Action"
sidebar_position: 0
description: "The convergence of LLMs and robotics. Build humanoid robots that understand voice commands, reason about tasks, and execute physical actions."
---

# Part 4: Vision-Language-Action (VLA)

**Weeks 11-13 | Hardware: Full Stack (Jetson + optional physical robot)**

---

## What You'll Learn

**VLA represents the frontier of Physical AI**—the convergence of large language models, computer vision, and robotic action. Your robot will understand natural language, reason about complex tasks, and execute physical actions autonomously.

By the end of Part 4, you will:

- **Integrate voice interaction** — Speech-to-text with OpenAI Whisper
- **Implement cognitive planning** — LLMs (GPT/Claude) for task decomposition
- **Build end-to-end autonomy** — Voice → Reasoning → Action
- **Deploy autonomous humanoid** — Complete capstone project

---

## Chapter Overview

| Chapter | Title | Week | Key Topics |
|---------|-------|------|------------|
| 13 | Voice-to-Action | 11 | Whisper, speech processing, action mapping |
| 14 | LLM Cognitive Planning | 12 | GPT/Claude, task decomposition, safety |
| 15 | Autonomous Humanoid Capstone | 13 | Full integration, deployment |

---

## The VLA Stack

```
┌─────────────────────────────────────┐
│     LLM Cognitive Planning          │  ← Chapter 14: GPT/Claude reasoning
├─────────────────────────────────────┤
│     Voice-to-Action                  │  ← Chapter 13: Whisper + NLU
├─────────────────────────────────────┤
│     Perception + Navigation          │  ← From Part 3
├─────────────────────────────────────┤
│     ROS 2 + Simulation               │  ← From Parts 1-2
└─────────────────────────────────────┘
```

---

## Hardware Requirements

Part 4 brings everything together:

### Edge Deployment Kit
- **Brain:** NVIDIA Jetson Orin Nano/NX (~$249-$499)
- **Vision:** Intel RealSense D435i (~$349)
- **Audio:** ReSpeaker USB Mic Array (~$69)
- **Total:** ~$700

### Cloud API Access
- OpenAI API (GPT-4) or Anthropic API (Claude)
- For cognitive planning (Chapter 14)

### Optional Physical Robot
- Unitree Go2 quadruped (~$1,800-$3,000) — excellent ROS 2 support
- Miniature humanoid (Unitree G1 ~$16k, Robotis OP3 ~$12k)
- Learning kit (Hiwonder TonyPi Pro ~$600)

**Note:** The capstone works in high-fidelity simulation if physical robot is unavailable.

---

## The VLA Paradigm

Traditional robotics programmed specific behaviors. VLA enables **emergent behavior** from natural language:

| Traditional | VLA |
|-------------|-----|
| "Execute pick_object(red_cup)" | "Pick up the red cup" |
| Hardcoded motion planning | LLM-generated action sequences |
| Fixed vocabulary | Natural language understanding |
| Brittle to variation | Robust to phrasing differences |

**The transformation:** From programming robots to *conversing* with them.

---

## Learning Path

```
Week 11: Voice Interface
  └── Chapter 13: Whisper integration, command mapping

Week 12: Cognitive Layer
  └── Chapter 14: LLM planning, safety constraints

Week 13: Capstone Integration
  └── Chapter 15: Full autonomous humanoid deployment
```

---

## The Capstone: Autonomous Humanoid

Chapter 15 brings together everything from the entire book:

1. **Voice Input** — "Bring me the blue cup from the kitchen"
2. **LLM Planning** — Decompose into navigation + perception + manipulation
3. **Perception** — Find kitchen, identify blue cup
4. **Navigation** — Path planning to kitchen, obstacle avoidance
5. **Manipulation** — Grasp cup, return to user
6. **Feedback** — "Here is your blue cup"

This is the culmination of 13 weeks—an autonomous system that understands, reasons, and acts.
