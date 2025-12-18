---
id: 0003
title: Generate Constitution for Spec-Kit Plus Writing System
stage: constitution
date: 2025-12-07
surface: agent
model: gemini-1.5-flash-latest
feature: physical-ai-humanoid-robotics-book
branch: main
user: user
command: Generate a complete /sp.constitution file for my Spec-Kit Plus writing system.  Project Context: I am writing a highly technical book on Physical AI & Humanoid Robotics. The constitution will govern all AI-generated writing to ensure accuracy, safety, and consistency.  Your task: Create a strict, professional constitution that defines: - Writing principles - Accuracy rules - Style & formatting rules - Safety & reliability constraints - AI behavior expectations - Research standards (citations, source validation) - Technical correctness requirements (robotics, ROS 2, Gazebo, Nvidia Isaac, VLA models, etc.) - Prohibited behaviors - Allowed behaviors - Quality benchmarks for all chapters - Markdown formatting rules - Definitions of “good answer” and “bad answer” - Tone & clarity guidelines - Versioning conventions - Update policy for chapters and modules  Constraints: - Must be written as a formal constitution - Use hierarchy (H1 → H2 → H3) - Clear, enforceable rules (no vague statements) - All writing must be reproducible, factual, verifiable - Must apply to all future interactions and chapter generations  Output format: Return only the fully written /sp.constitution in clean Markdown. Do not generate any chapter content.
labels: ["constitution", "spec-kit", "writing-system"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- sp.constitution
tests:
null
---

## Prompt

Generate a complete /sp.constitution file for my Spec-Kit Plus writing system.

Project Context:
I am writing a highly technical book on Physical AI & Humanoid Robotics.  
The constitution will govern all AI-generated writing to ensure accuracy, safety, and consistency.

Your task:
Create a strict, professional constitution that defines:
- Writing principles  
- Accuracy rules  
- Style & formatting rules  
- Safety & reliability constraints  
- AI behavior expectations  
- Research standards (citations, source validation)  
- Technical correctness requirements (robotics, ROS 2, Gazebo, Nvidia Isaac, VLA models, etc.)  
- Prohibited behaviors  
- Allowed behaviors  
- Quality benchmarks for all chapters  
- Markdown formatting rules  
- Definitions of “good answer” and “bad answer”  
- Tone & clarity guidelines  
- Versioning conventions  
- Update policy for chapters and modules  

Constraints:
- Must be written as a formal constitution  
- Use hierarchy (H1 → H2 → H3)  
- Clear, enforceable rules (no vague statements)  
- All writing must be reproducible, factual, verifiable  
- Must apply to all future interactions and chapter generations  

Output format:
Return only the fully written /sp.constitution in clean Markdown.
Do not generate any chapter content.

## Response snapshot

Successfully created and wrote to new file: C:\Users\Hira Ali\OneDrive\Desktop\hack_yt\sp.constitution.

## Outcome

- ✅ Impact: Constitution file generated to govern AI writing.
- 🧪 Tests: No automated tests were run. Manual review of content confirms adherence to requirements.
- 📁 Files: `sp.constitution` created with comprehensive rules.
- 🔁 Next prompts: The user will likely proceed with chapter generation, now governed by this constitution.
- 🧠 Reflection: The user provided very clear and structured requirements, which made generating the formal constitution straightforward. The hierarchical structure was key.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
