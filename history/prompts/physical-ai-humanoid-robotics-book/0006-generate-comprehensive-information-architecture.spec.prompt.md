---
id: 0006
title: Generate Comprehensive Information Architecture for Book Project
stage: spec
date: 2025-12-07
surface: agent
model: gemini-1.5-flash-latest
feature: physical-ai-humanoid-robotics-book
branch: main
user: user
command: |-
  Generate a complete /sp.architecture file for my Spec-Kit Plus project.

  Project context:
  Book title: "Physical AI & Humanoid Robotics"
  This book follows the /sp.constitution and /sp.vision and must map the high-level layout into a concrete architecture for chapters, files, Docusaurus structure, and writing workflow.

  Your task:
  Produce a detailed architecture document that specifies, for the entire book:

  1. Top-level structure
     - Module → Chapter → Section mapping (for the 4 course modules + Intro, Hardware, Assessments, Appendix)
     - Exact filenames for each chapter (e.g., sp.chapter.01.md or docs/chapter-01.md)

  2. Chapter templates
     - Provide a reusable Markdown template for every chapter with fields: Title, Summary, Learning Objectives, Prerequisites, Estimated length, Required tools, Step-by-step labs, Code examples, Diagrams, References (APA), Acceptance criteria, Version.

  3. File & folder layout (Docusaurus-ready)
     - Recommended repo tree (docs/, sidebars.js, docusaurus.config.js, static/, assets/, sp.* files)
     - Where to store images, datasets, USD/URDF assets, and example code
     - Naming conventions and slug rules for routes

  4. Metadata & frontmatter
     - Standard frontmatter for every doc (title, sidebar_position, description, tags, last_reviewed, version)
     - Convention for versioning and review dates

  5. Sidebar & navigation design
     - Exact sidebar groupings for Docusaurus (Intro, Module 1, Module 2, Module 3, Module 4, Hardware, Capstone, Assessments, Appendix)
     - Breadcrumb strategy and cross-links between modules

  6. Code & example management
     - Where to place runnable code (examples/, notebooks/, docker/)
     - Recommended languages and runtimes (Python 3.10+, ROS 2 Humble/Iron, Docker images, Jetson deploy folder)
     - CI steps to test code examples (unit tests, linters, colcon build checks)

  7. Asset handling
     - How to reference binary assets (USD scenes, models) without inflating repo (git-lfs or external storage)
     - Recommended folder for synthetic dataset samples and how to cite size/format

  8. Diagrams & visualizations
     - Text-based architecture diagrams to include (system flow, dataflow, hardware topology)
     - Recommendations for tools to produce visuals (draw.io, Diagrams.net, Inkscape, Mermaid for Markdown)

  9. Citation & source workflow
     - Where to place APA references, bibliographies, and a centralized references.md
     - Process for verifying sources per /sp.constitution

  10. Review, versioning & release process
      - Branching model for chapters (feature/chapter-xx), review checklist, acceptance criteria
      - How to tag releases and produce GitHub Pages snapshots (release/2025-12-07 style)
      - Automated checks: plagiarism, link validations, build checks

  11. CI/CD & Deployment
      - Minimal GitHub Actions workflow to build Docusaurus, run tests, and deploy to GitHub Pages
      - Secrets and environment management recommendations

  12. Timeline & milestones
      - Suggested milestone breakdown mapped to hackathon 3-day schedule and full 13-week course timeline
      - Deliverables per milestone

  Constraints:
  - Must conform to /sp.constitution and /sp.vision
  - Use clear, prescriptive conventions (no vague suggestions)
  - Output must be in Markdown, ready to save as /sp.architecture
  - Keep language professional and concise

  Output:
  Return only the full /sp.architecture content in clean Markdown. Do not generate chapter content.
labels: ["spec-kit", "architecture", "docusaurus", "documentation", "workflow"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- sp.architecture
tests:
null
---

## Prompt

Generate a complete /sp.architecture file for my Spec-Kit Plus project.

Project context:
Book title: "Physical AI & Humanoid Robotics"
This book follows the /sp.constitution and /sp.vision and must map the high-level layout into a concrete architecture for chapters, files, Docusaurus structure, and writing workflow.

Your task:
Produce a detailed architecture document that specifies, for the entire book:

1. Top-level structure
   - Module → Chapter → Section mapping (for the 4 course modules + Intro, Hardware, Assessments, Appendix)
   - Exact filenames for each chapter (e.g., sp.chapter.01.md or docs/chapter-01.md)

2. Chapter templates
   - Provide a reusable Markdown template for every chapter with fields: Title, Summary, Learning Objectives, Prerequisites, Estimated length, Required tools, Step-by-step labs, Code examples, Diagrams, References (APA), Acceptance criteria, Version.

3. File & folder layout (Docusaurus-ready)
   - Recommended repo tree (docs/, sidebars.js, docusaurus.config.js, static/, assets/, sp.* files)
   - Where to store images, datasets, USD/URDF assets, and example code
   - Naming conventions and slug rules for routes

4. Metadata & frontmatter
   - Standard frontmatter for every doc (title, sidebar_position, description, tags, last_reviewed, version)
   - Convention for versioning and review dates

5. Sidebar & navigation design
   - Exact sidebar groupings for Docusaurus (Intro, Module 1, Module 2, Module 3, Module 4, Hardware, Capstone, Assessments, Appendix)
   - Breadcrumb strategy and cross-links between modules

6. Code & example management
   - Where to place runnable code (examples/, notebooks/, docker/)
   - Recommended languages and runtimes (Python 3.10+, ROS 2 Humble/Iron, Docker images, Jetson deploy folder)
   - CI steps to test code examples (unit tests, linters, colcon build checks)

7. Asset handling
   - How to reference binary assets (USD scenes, models) without inflating repo (git-lfs or external storage)
   - Recommended folder for synthetic dataset samples and how to cite size/format

8. Diagrams & visualizations
   - Text-based architecture diagrams to include (system flow, dataflow, hardware topology)
   - Recommendations for tools to produce visuals (draw.io, Diagrams.net, Inkscape, Mermaid for Markdown)

9. Citation & source workflow
   - Where to place APA references, bibliographies, and a centralized references.md
   - Process for verifying sources per /sp.constitution

10. Review, versioning & release process
    - Branching model for chapters (feature/chapter-xx), review checklist, acceptance criteria
    - How to tag releases and produce GitHub Pages snapshots (release/2025-12-07 style)
    - Automated checks: plagiarism, link validations, build checks

11. CI/CD & Deployment
    - Minimal GitHub Actions workflow to build Docusaurus, run tests, and deploy to GitHub Pages
    - Secrets and environment management recommendations

12. Timeline & milestones
    - Suggested milestone breakdown mapped to hackathon 3-day schedule and full 13-week course timeline
    - Deliverables per milestone

Constraints:
- Must conform to /sp.constitution and /sp.vision
- Use clear, prescriptive conventions (no vague suggestions)
- Output must be in Markdown, ready to save as /sp.architecture
- Keep language professional and concise

Output:
Return only the full /sp.architecture content in clean Markdown. Do not generate chapter content.

## Response snapshot

Successfully created and wrote to new file: C:\Users\Hira Ali\OneDrive\Desktop\hack_yt\sp.architecture.

## Outcome

- ✅ Impact: A detailed information architecture and development workflow was created, providing a concrete plan for building the book on Docusaurus.
- 🧪 Tests: No automated tests were run. Manual review of the generated file confirms it matches the user's comprehensive requirements.
- 📁 Files: `sp.architecture` was overwritten with a new, more detailed version focusing on information architecture and workflow.
- 🔁 Next prompts: The user is likely to proceed with setting up the Docusaurus project or start writing the first chapter according to this new architecture.
- 🧠 Reflection: This request significantly changed the meaning of `/sp.architecture` from a technical hardware/software stack to the information architecture of the book's website. I correctly interpreted this as an overwrite request. The detailed prompt allowed for the generation of a very specific and actionable document that will govern the entire project's structure and workflow.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
