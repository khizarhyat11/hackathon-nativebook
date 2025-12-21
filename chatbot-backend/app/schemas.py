"""
Pydantic schemas for API request/response models.
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Dict
from datetime import datetime


class ChatMessage(BaseModel):
    """A single chat message."""
    role: str = Field(..., description="'user' or 'assistant'")
    content: str = Field(..., description="Message content")
    

class ChatRequest(BaseModel):
    """Request to send a chat message."""
    message: str = Field(..., description="User's question or message")
    conversation_id: Optional[str] = Field(None, description="Existing conversation ID")
    selected_text: Optional[str] = Field(None, description="Text selected by user from the page")
    
    class Config:
        json_schema_extra = {
            "example": {
                "message": "What is ROS 2 and how does it work?",
                "conversation_id": None,
                "selected_text": None
            }
        }


class SourceReference(BaseModel):
    """A reference to source material."""
    title: str
    source: str
    score: float


class ChatResponse(BaseModel):
    """Response from the chatbot."""
    response: str = Field(..., description="Assistant's response")
    conversation_id: str = Field(..., description="Conversation ID for continuity")
    sources: List[SourceReference] = Field(default=[], description="Source references")
    context_used: bool = Field(..., description="Whether context was found")


class ConversationHistory(BaseModel):
    """Full conversation history."""
    conversation_id: str
    messages: List[ChatMessage]
    created_at: datetime
    updated_at: datetime


class IndexRequest(BaseModel):
    """Request to index documents."""
    documents: List[Dict] = Field(..., description="List of documents to index")
    

class IndexResponse(BaseModel):
    """Response after indexing."""
    indexed_count: int
    message: str


class HealthResponse(BaseModel):
    """Health check response."""
    status: str
    vector_store: Dict
    database: str
