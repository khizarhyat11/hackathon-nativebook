"""
FastAPI application for the RAG Chatbot.
Main entry point and API routes.
"""

# Trigger reload
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from contextlib import asynccontextmanager
import json

from .config import settings
from .database import init_db, get_db, Conversation, Message
from .vector_store import vector_store
from .agent import rag_agent
from .schemas import (
    ChatRequest, ChatResponse, ChatMessage,
    ConversationHistory, HealthResponse,
    IndexRequest, IndexResponse, SourceReference
)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize services on startup."""
    print("Starting RAG Chatbot Backend...")
    
    # Initialize database
    await init_db()
    print("Database initialized")
    
    # Initialize vector store collection
    await vector_store.init_collection()
    print("Vector store ready")
    
    yield
    
    print("Shutting down...")


app = FastAPI(
    title="Physical AI Textbook Chatbot",
    description="RAG-powered chatbot for the Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    lifespan=lifespan
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Check API health and service status."""
    try:
        stats = vector_store.get_collection_stats()
        return HealthResponse(
            status="healthy",
            vector_store=stats,
            database="connected"
        )
    except Exception as e:
        raise HTTPException(status_code=503, detail=str(e))


@app.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Send a message to the chatbot and get a response.
    
    - Creates new conversation if conversation_id not provided
    - Retrieves relevant context from the textbook
    - Generates a human-like response based on book content
    - Stores conversation history
    """
    try:
        # Get or create conversation
        if request.conversation_id:
            result = await db.execute(
                select(Conversation).where(Conversation.id == request.conversation_id)
            )
            conversation = result.scalar_one_or_none()
            if not conversation:
                raise HTTPException(status_code=404, detail="Conversation not found")
        else:
            conversation = Conversation()
            db.add(conversation)
            await db.flush()
        
        # Get conversation history
        result = await db.execute(
            select(Message)
            .where(Message.conversation_id == conversation.id)
            .order_by(Message.created_at)
        )
        messages = result.scalars().all()
        
        history = [
            {"role": msg.role, "content": msg.content}
            for msg in messages
        ]
        
        # Generate response using RAG
        response_data = await rag_agent.generate_response(
            query=request.message,
            conversation_history=history,
            selected_text=request.selected_text
        )
        
        # Store user message
        user_msg = Message(
            conversation_id=conversation.id,
            role="user",
            content=request.message
        )
        db.add(user_msg)
        
        # Store assistant response
        assistant_msg = Message(
            conversation_id=conversation.id,
            role="assistant",
            content=response_data["response"],
            context_chunks=[s["source"] for s in response_data["sources"]]
        )
        db.add(assistant_msg)
        
        await db.commit()
        
        return ChatResponse(
            response=response_data["response"],
            conversation_id=conversation.id,
            sources=[
                SourceReference(**s) for s in response_data["sources"]
            ],
            context_used=response_data["context_used"]
        )
        
    except HTTPException:
        raise
    except Exception as e:
        await db.rollback()
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/chat/stream")
async def chat_stream(
    request: ChatRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Stream chat response for real-time display.
    Returns Server-Sent Events (SSE).
    """
    try:
        # Get conversation history
        history = []
        if request.conversation_id:
            result = await db.execute(
                select(Message)
                .where(Message.conversation_id == request.conversation_id)
                .order_by(Message.created_at)
            )
            messages = result.scalars().all()
            history = [{"role": msg.role, "content": msg.content} for msg in messages]
        
        async def generate():
            async for chunk in rag_agent.generate_response_stream(
                query=request.message,
                conversation_history=history,
                selected_text=request.selected_text
            ):
                yield f"data: {json.dumps({'chunk': chunk})}\n\n"
            yield "data: [DONE]\n\n"
        
        return StreamingResponse(
            generate(),
            media_type="text/event-stream"
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/conversation/{conversation_id}", response_model=ConversationHistory)
async def get_conversation(
    conversation_id: str,
    db: AsyncSession = Depends(get_db)
):
    """Get full conversation history."""
    result = await db.execute(
        select(Conversation).where(Conversation.id == conversation_id)
    )
    conversation = result.scalar_one_or_none()
    
    if not conversation:
        raise HTTPException(status_code=404, detail="Conversation not found")
    
    result = await db.execute(
        select(Message)
        .where(Message.conversation_id == conversation_id)
        .order_by(Message.created_at)
    )
    messages = result.scalars().all()
    
    return ConversationHistory(
        conversation_id=conversation.id,
        messages=[
            ChatMessage(role=msg.role, content=msg.content)
            for msg in messages
        ],
        created_at=conversation.created_at,
        updated_at=conversation.updated_at
    )


@app.delete("/conversation/{conversation_id}")
async def delete_conversation(
    conversation_id: str,
    db: AsyncSession = Depends(get_db)
):
    """Delete a conversation and all its messages."""
    result = await db.execute(
        select(Conversation).where(Conversation.id == conversation_id)
    )
    conversation = result.scalar_one_or_none()
    
    if not conversation:
        raise HTTPException(status_code=404, detail="Conversation not found")
    
    await db.delete(conversation)
    await db.commit()
    
    return {"message": "Conversation deleted"}


@app.post("/index", response_model=IndexResponse)
async def index_documents(request: IndexRequest):
    """
    Index documents into the vector store.
    Used to add textbook content for retrieval.
    """
    try:
        ids = vector_store.index_documents(request.documents)
        return IndexResponse(
            indexed_count=len(ids),
            message=f"Successfully indexed {len(ids)} documents"
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/stats")
async def get_stats():
    """Get vector store statistics."""
    return vector_store.get_collection_stats()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host=settings.host,
        port=settings.port,
        reload=True
    )
