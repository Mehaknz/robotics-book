---
id: 0007
title: Generate Detailed Chapter Template for Book Project
stage: spec
date: 2025-12-07
surface: agent
model: gemini-1.5-flash-latest
feature: physical-ai-humanoid-robotics-book
branch: main
user: user
command: |-
  Generate a complete /sp.chapter.template file for my Spec-Kit Plus project.

  Project Context:
  Book title: "Physical AI & Humanoid Robotics"
  This chapter template will be used to generate every chapter in the book.
  It must follow all rules defined in /sp.constitution, /sp.vision, and /sp.architecture.

  Your Task:
  Produce a professional, strict, reusable chapter template that includes the following mandatory fields:

  1. Frontmatter
     - title
     - sidebar_position
     - description
     - tags
     - version
     - last_reviewed
     - authors
     - difficulty_level (Beginner / Intermediate / Advanced)

  2. Chapter Header
     - Summary (3–5 sentences)
     - Learning Objectives (4–8 bullet points)
     - Prerequisites
     - Estimated Reading Time
     - Required Tools & Software (e.g., ROS 2 Humble, Isaac Sim, Gazebo, Unity)

  3. Core Body Sections (must be consistent across ALL chapters)
     - H1: Introduction
     - H1: Theory & Concepts
     - H1: Technical Foundations
       - Subsections for ROS 2, URDF, Gazebo, Isaac Sim, VLA, or relevant technologies
     - H1: Step-by-Step Implementation
       - Detailed instructions
       - Code blocks (Python, ROS 2 launch files, URDF samples)
     - H1: Diagrams & System Architecture
       - Mermaid diagrams
       - Hardware/software flow diagrams
     - H1: Example Lab / Hands-On Exercise
       - Clear steps
       - Expected outputs
     - H1: Troubleshooting & Common Mistakes
     - H1: Summary & Key Takeaways

  4. Technical Accuracy Requirements
     - All claims must be fact-checked and referenced
     - Provide APA-style citations
     - Include cross-links to related chapters

  5. Code Standards
     - Provide clean, minimal reproducible examples
     - Use ROS 2 conventions (nodes, topics, services, actions)
     - Use Python 3.10+
     - Code must run without modification unless noted

  6. Acceptance Criteria for Each Chapter
     - At least 1 diagram
     - Minimum 3 references (APA)
     - All steps reproducible
     - No hallucinations or unverified claims
     - Formatting must strictly follow the template

  7. Versioning Rules
     - Semantic versioning per chapter (v1.0.0)
     - "last_reviewed" must be present
     - Change-log field at bottom

  8. Final Required Sections (end of file)
     - References (APA style only)
     - Change Log
     - Reviewer Notes

  Constraints:
  - Must output only the final /sp.chapter.template in Markdown
  - Must be strict, clean, and reusable for all chapters
  - Must not generate any chapter content

  Output:
  Return only the complete Markdown template, ready to save as:
  /sp.chapter.template
labels: ["spec-kit", "template", "chapter", "documentation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- sp.chapter.template
tests:
null
---

## Prompt

Generate a complete /sp.chapter.template file for my Spec-Kit Plus project.

Project Context:
Book title: "Physical AI & Humanoid Robotics"
This chapter template will be used to generate every chapter in the book.  
It must follow all rules defined in /sp.constitution, /sp.vision, and /sp.architecture.

Your Task:
Produce a professional, strict, reusable chapter template that includes the following mandatory fields:

1. Frontmatter
   - title  
   - sidebar_position  
   - description  
   - tags  
   - version  
   - last_reviewed  
   - authors  
   - difficulty_level (Beginner / Intermediate / Advanced)  

2. Chapter Header
   - Summary (3–5 sentences)
   - Learning Objectives (4–8 bullet points)
   - Prerequisites
   - Estimated Reading Time
   - Required Tools & Software (e.g., ROS 2 Humble, Isaac Sim, Gazebo, Unity)

3. Core Body Sections (must be consistent across ALL chapters)
   - H1: Introduction  
   - H1: Theory & Concepts  
   - H1: Technical Foundations  
     - Subsections for ROS 2, URDF, Gazebo, Isaac Sim, VLA, or relevant technologies  
   - H1: Step-by-Step Implementation  
     - Detailed instructions  
     - Code blocks (Python, ROS 2 launch files, URDF samples)  
   - H1: Diagrams & System Architecture  
     - Mermaid diagrams  
     - Hardware/software flow diagrams  
   - H1: Example Lab / Hands-On Exercise  
     - Clear steps  
     - Expected outputs  
   - H1: Troubleshooting & Common Mistakes  
   - H1: Summary & Key Takeaways  

4. Technical Accuracy Requirements
   - All claims must be fact-checked and referenced  
   - Provide APA-style citations  
   - Include cross-links to related chapters  

5. Code Standards
   - Provide clean, minimal reproducible examples  
   - Use ROS 2 conventions (nodes, topics, services, actions)  
   - Use Python 3.10+  
   - Code must run without modification unless noted  

6. Acceptance Criteria for Each Chapter
   - At least 1 diagram  
   - Minimum 3 references (APA)  
   - All steps reproducible  
   - No hallucinations or unverified claims  
   - Formatting must strictly follow the template  

7. Versioning Rules
   - Semantic versioning per chapter (v1.0.0)  
   - "last_reviewed" must be present  
   - Change-log field at bottom  

8. Final Required Sections (end of file)
   - References (APA style only)  
   - Change Log  
   - Reviewer Notes  

Constraints:
- Must output only the final /sp.chapter.template in Markdown  
- Must be strict, clean, and reusable for all chapters  
- Must not generate any chapter content  

Output:
Return only the complete Markdown template, ready to save as:  
/sp.chapter.template

## Response snapshot

Successfully created and wrote to new file: C:\Users\Hira Ali\OneDrive\Desktop\hack_yt\sp.chapter.template.

## Outcome

- ✅ Impact: A comprehensive, reusable chapter template was created, ensuring consistency and quality across all book chapters.
- 🧪 Tests: No automated tests were run. Manual review of the generated file confirms it matches the user's comprehensive and strict requirements.
- 📁 Files: `sp.chapter.template` was created.
- 🔁 Next prompts: The user is now fully equipped to start writing chapters using this template.
- 🧠 Reflection: This request provided the most specific requirements for a template yet. I successfully incorporated all the mandatory fields, body sections, and author guidelines into a single, clean Markdown file. The use of a commented-out checklist at the top is a good pattern for embedding rules directly into a template.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
