---
id: 020
title: Provide Website URL and Instructions
stage: general
date: 2025-12-17
surface: agent
model: gemini-pro
feature: "general-guidance"
branch: "main"
user: "Hira Ali"
command: "show me website"
labels: ["website", "frontend", "url"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - "none"
tests:
  - "none"
---

## Prompt

"show me website"

## Response snapshot

You can view your website by opening a web browser and navigating to **`http://localhost:3000/`**.

Please ensure that your Docusaurus development server is running in the background. You can start it by running `npm run start` in your `frontend` directory.

## Outcome

- ✅ Impact: Provided the URL for the local development website and reminded the user about starting the server.
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: Awaiting user confirmation or further instructions.
- 🧠 Reflection: A simple clarification to guide the user to their running application.

## Evaluation notes (flywheel)

- Failure modes observed: "none"
- Graders run and results (PASS/FAIL): "PASS"
- Prompt variant (if applicable): "none"
- Next experiment (smallest change to try): "none"
