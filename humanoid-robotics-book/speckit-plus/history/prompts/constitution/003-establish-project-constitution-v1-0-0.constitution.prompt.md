---
id: 003
title: Establish Project Constitution v1.0.0
stage: constitution
date: 2025-12-12
surface: "agent"
model: "gemini"
feature: "none"
branch: "main"
user: "Mehaknz"
command: "/sp.constitution"
labels: ["constitution", "principles", "governance"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
 - none
---

## Prompt

/sp.constitution

Project: Unified AI-Driven Book + Embedded RAG Chatbot
(using Claude Code, Spec-Kit Plus, Docusaurus, GitHub Pages, OpenAI Agents/ChatKit, FastAPI, Neon Serverless Postgres, Qdrant Cloud)

Core principles:

Spec-driven development across book, backend, chatbot, and infra

Technical clarity for readers with web or AI development experience

Consistency between written content, code, APIs, and chatbot behavior

Full traceability from requirements → implementation → deployment

Maintainability and extensibility as a primary design goal

Key standards:

Book (Docusaurus)

Written using Spec-Kit Plus + Claude Code

Chapter-level specs with goals, constraints, and acceptance criteria

Deployment to GitHub Pages required

Must document workflows, architecture, and implementation steps

RAG Chatbot

Stack: OpenAI Agents/ChatKit SDKs, FastAPI backend, Qdrant Cloud Free Tier, Neon Serverless Postgres

Capabilities:

Answer questions strictly from book content

Cite source locations within the book

Support answering based on user-selected text

Log interactions (privacy-safe) in Neon Postgres

All routing, retrieval, and embedding logic must be spec-verified

Prompt templates fully documented

Documentation & Code Quality

Every code unit mapped to a specific requirement

Architecture diagrams for RAG pipeline, API structure, and infra

Complete README (install, run, deploy)

Robust error handling + clear module boundaries

Version control with clean, structured commits

Constraints:

Book length: minimum 8 chapters

Mandatory coverage of:

Docusaurus workflow

Spec-Kit Plus methodology

Claude Code processes

RAG architecture & design

Agents/ChatKit usage

FastAPI backend implementation

Qdrant + Neon database integration

Chatbot latency target: <1.5s for retrieval + response (cloud tier permitting)

No exposed secrets; GitHub Secrets required

Backend must avoid insecure or unauthenticated endpoints

Success criteria:

Book builds and deploys correctly to GitHub Pages

RAG chatbot loads inside the book and functions end-to-end

Chatbot reliably answers questions from book content or user-selected text only

All modules pass Spec-Kit Plus validation

Repo includes:

Full specifications

Working, documented FastAPI backend

Qdrant + Neon integration

Deployment scripts and instructions

Architecture diagrams

Final system is cohesive: book + chatbot + backend + infra all aligned with the constitution above

## Response snapshot

I have updated the project constitution to version 1.0.0 based on your input. This involved defining the core principles, standards, constraints, and success criteria. I also updated the `plan-template.md` to align with the new constitution.

## Outcome

- ✅ Impact: Established the foundational principles and standards for the project.
- 🧪 Tests: none
- 📁 Files: 
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
- 🔁 Next prompts: none
- 🧠 Reflection: The constitution provides a solid framework for future development.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
