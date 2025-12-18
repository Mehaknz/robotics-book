# Unified AI-Driven Book + Embedded RAG Chatbot Constitution
<!-- 
Sync Impact Report:
- Version change: none -> 1.0.0
- List of modified principles: all principles updated
- Added sections: Key Standards, Constraints, Success Criteria
- Removed sections: none
- Templates requiring updates: 
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: none
-->

## Core Principles

### I. Spec-Driven Development
Spec-driven development is mandatory across all components, including the book, backend services, chatbot, and infrastructure. Every feature or chapter must begin with a clear specification that outlines goals, constraints, and acceptance criteria.

### II. Technical Clarity
All content, code, and documentation must be written with technical clarity, targeting an audience with existing experience in web or AI development. Complex topics should be broken down into understandable parts, and architectural decisions must be clearly justified.

### III. Consistency
Strict consistency must be maintained between the written book content, code examples, API contracts, and the live chatbot's behavior. The chatbot's responses must align with the information presented in the book.

### IV. Full Traceability
There must be full traceability from requirements to implementation and deployment. Every code unit, API endpoint, and infrastructure component must be mapped back to a specific requirement in a specification document.

### V. Maintainability and Extensibility
The system must be designed for maintainability and extensibility. This includes using clear module boundaries, robust error handling, comprehensive documentation, and a clean, structured version control history.

## Key Standards

### Book (Docusaurus)
- Written using Spec-Kit Plus + Claude Code.
- Each chapter must have a corresponding specification with goals, constraints, and acceptance criteria.
- The final book must be deployable to GitHub Pages.
- Workflows, architecture, and implementation steps must be documented within the book.

### RAG Chatbot
- **Stack:** OpenAI Agents/ChatKit SDKs, FastAPI backend, Qdrant Cloud (Free Tier), Neon Serverless Postgres.
- **Capabilities:**
  - Answer questions strictly from the book's content.
  - Cite source locations within the book.
  - Support answering based on user-selected text.
  - Log interactions (in a privacy-safe manner) to Neon Postgres.
- All routing, retrieval, and embedding logic must be spec-verified.
- Prompt templates must be fully documented.

### Documentation & Code Quality
- Every code unit must be mapped to a specific requirement.
- Architecture diagrams must be created for the RAG pipeline, API structure, and infrastructure.
- A complete `README.md` must be provided, detailing installation, execution, and deployment steps.
- The codebase must feature robust error handling and clear module boundaries.
- Version control must be maintained with clean, structured commits.

## Constraints

- **Book Length:** The book must contain a minimum of 8 chapters.
- **Mandatory Coverage:** The content must cover:
  - Docusaurus workflow
  - Spec-Kit Plus methodology
  - Claude Code processes
  - RAG architecture & design
  - Agents/ChatKit usage
  - FastAPI backend implementation
  - Qdrant + Neon database integration
- **Chatbot Latency:** The target latency for the RAG chatbot is under 1.5 seconds for retrieval and response (cloud tier permitting).
- **Security:**
  - No secrets shall be exposed in the codebase; GitHub Secrets must be used.
  - The backend must not contain insecure or unauthenticated endpoints.

## Success Criteria

- The book must build and deploy correctly to GitHub Pages.
- The RAG chatbot must load inside the book and function end-to-end.
- The chatbot must reliably answer questions derived only from book content or user-selected text.
- All modules must pass Spec-Kit Plus validation.
- The repository must include full specifications, a working and documented FastAPI backend, Qdrant and Neon integration, deployment scripts, and architecture diagrams.

## Governance

This Constitution is the source of truth for all project principles and standards. All development, code reviews, and architectural decisions must align with it. Amendments to this document require a formal proposal, review, and an update to the version number.

**Version**: 1.0.0 | **Ratified**: 2025-12-12 | **Last Amended**: 2025-12-12