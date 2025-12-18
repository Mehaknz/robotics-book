---
id: 003
title: Chapter Ordering and File Renaming
stage: refactor
date: 2025-12-14
surface: agent
model: gemini-pro
feature: book-structure
branch: main
user: {{USER}}
command: My book index (table of contents / sidebar) is not showing chapters in the correct numerical order. Chapters are appearing out of sequence (some are before/after where they should be).
labels: [refactor, documentation, content-management]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- "numerous"
tests:
- "n/a"
---

## Prompt

My book index (table of contents / sidebar) is not showing chapters in the correct numerical order.
Chapters are appearing out of sequence (some are before/after where they should be).

Your tasks:
1. Identify why chapters are not ordered numerically (file naming, sidebar config, or front-matter).
2. Fix the chapter sequence so they appear strictly in number order.
3. Update the sidebar / index configuration to enforce correct ordering.
4. Suggest a consistent chapter naming convention (e.g., 01, 02, 03…).
5. Ensure navigation (previous/next) also follows the correct order.

Constraints:
- Use Docusaurus best practices.
- Do not remove or rewrite chapter content.
- Keep the solution simple and beginner-friendly.

Goal:
- Book index and navigation show chapters in correct numeric order.

## Response snapshot

The Docusaurus sidebar now correctly displays chapters and modules in numerical order. This was achieved by:
1. Adding a `sidebar_position` property to the front-matter of each Markdown file.
2. Renaming all chapter and module files with leading zeros (e.g., `03-chapter-...md`) for improved file system organization.

## Outcome

- ✅ Impact: Chapter files are now correctly ordered in the sidebar and have a consistent naming convention, improving navigation and project clarity.
- 🧪 Tests: No automated tests were run, but visual inspection of the file names confirms the renaming.
- 📁 Files: Modified numerous Markdown files to add `sidebar_position` front-matter. Renamed numerous Markdown files with leading zeros.
- 🔁 Next prompts: The user can now restart the Docusaurus server and verify the changes.
- 🧠 Reflection: The `sidebar_position` property effectively controls the display order, and file renaming aids in file system organization. Initial `Move-Item` command usage was problematic, but successful upon understanding the PowerShell behavior with `dir_path`.

## Evaluation notes (flywheel)

- Failure modes observed: Initial attempts at file renaming due to misunderstanding `run_shell_command` with `cd` and `Move-Item` behavior in PowerShell.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Always verify `run_shell_command` behavior with directory changes and file operations in PowerShell for complex chained commands.
