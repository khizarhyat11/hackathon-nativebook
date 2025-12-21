# RAG Chatbot Backend

A FastAPI-powered RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics textbook.

## Features

- 🔍 **Semantic Search**: Uses Google Gemini embeddings (`text-embedding-004`) with Qdrant vector database
- 🧠 **AI-Powered Responses**: Google Gemini 1.5 Flash with textbook-aware context
- 💬 **Conversation History**: Persisted in Neon Postgres
- 📖 **Context-Aware**: Responds only based on textbook content
- 🎯 **Selected Text Support**: Users can select text and ask about it

## Setup

### 1. Install Dependencies

```bash
cd chatbot-backend
python -m venv venv
venv\Scripts\activate
# If you get a policy error on Windows PowerShell:
# Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
pip install -r requirements.txt
```

### 2. Configure Environment

Copy `.env.example` to `.env` and fill in your credentials:

```bash
copy .env.example .env
```

Required credentials:
- **Google Gemini API Key**: Get from Google AI Studio
- **Qdrant Cloud**: Create free cluster at https://cloud.qdrant.io
- **Neon Postgres**: Create free database at https://neon.tech

### 3. Index the Textbook

```bash
python scripts/index_textbook.py
```

### 4. Run the Server

```bash
python -m uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```
