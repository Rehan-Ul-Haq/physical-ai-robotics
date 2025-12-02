# Project Overview

This is a two-part hackathon project focused on creating an AI-powered book about Physical AI & Humanoid Robotics and an intelligent chatbot interface.

## Project Components

### Part 1: Book Creation and Publishing
**Objective**: Create and publish a comprehensive book about Physical AI & Humanoid Robotics — teaching ROS 2, simulation, and Sim-to-Real transfer.

**Technology Stack**:
- **Static Site Generator**: Docusaurus
- **Hosting**: GitHub Pages
- **Content Focus**: Physical AI & Humanoid Robotics (ROS 2, Gazebo, NVIDIA Isaac)

**Key Requirements**:
- Generate book content following AI-driven and spec-driven methodologies
- Structure content in a well-organized, navigable format
- Deploy as a static website accessible via GitHub Pages
- Ensure professional presentation and user-friendly navigation

### Part 2: RAG-Based Chatbot Integration
**Objective**: Develop and embed an intelligent chatbot into the published book that can answer questions about the book's content.

**Technology Stack**:
- **AI Framework**: OpenAI Agents SDK
- **Chat Interface**: OpenAI ChatKit SDK
- **Backend API**: FastAPI
- **Vector Database**: Qdrant Cloud (Free Tier)
- **RAG Architecture**: Retrieval-Augmented Generation for context-aware responses

**Core Features**:
1. **General Book Q&A**: Users can ask any question about the book content, and the chatbot retrieves relevant information using RAG
2. **Context-Specific Queries**: Users can select/highlight specific text portions from the book and ask questions exclusively about that selected context
3. **Embedded Integration**: Chatbot seamlessly embedded within the published Docusaurus site

**Technical Requirements**:
- Implement vector embeddings of book content for semantic search
- Store embeddings in Qdrant Cloud vector database
- Use OpenAI Agents SDK for orchestrating retrieval and generation
- Build FastAPI backend to handle chat requests and vector database queries
- Integrate OpenAI ChatKit SDK for frontend chat interface
- Support two query modes:
  - Full-book context: RAG searches entire book content
  - Selected-text context: RAG limited to user-highlighted text

## Development Approach

Both components should be developed following **AI-Driven and Spec-Driven Development** principles:
- Start with clear specifications and requirements
- Leverage AI tools for code generation and content creation
- Maintain high code quality and documentation standards
- Implement iterative refinement based on testing and feedback

## Success Criteria

- [ ] Book is published and accessible on GitHub Pages
- [ ] Book content is comprehensive and well-structured
- [ ] Chatbot successfully answers questions about book content
- [ ] Context-selection feature works correctly
- [ ] RAG retrieval provides accurate and relevant responses
- [ ] User interface is intuitive and responsive
- [ ] All components integrate seamlessly