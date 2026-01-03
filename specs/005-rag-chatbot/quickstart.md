# Quick Start: RAG Chatbot Module

## Prerequisites

- Python 3.10+
- OpenAI API key
- Node.js (for frontend)

## Backend Setup

```bash
# Navigate to backend
cd chatbot-backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
# or
.\venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export OPENAI_API_KEY=your_key_here

# Run the server
uvicorn app.main:app --reload
```

## Ingest Documents

```bash
# Ingest textbook content
python scripts/ingest.py --path ../my-website/docs/
```

## Test the API

```bash
# Health check
curl http://localhost:8000/api/health

# Send a chat message
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?", "session_id": "test123"}'
```

## Frontend Integration

The chatbot component is integrated into the Docusaurus site. Access it by clicking the chat icon in the bottom right corner of any documentation page.

## Key Concepts

1. Documents are chunked and embedded for semantic search
2. User queries retrieve relevant chunks from ChromaDB
3. LLM generates responses using retrieved context
4. Sources are cited for transparency
