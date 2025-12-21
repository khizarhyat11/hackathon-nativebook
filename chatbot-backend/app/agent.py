"""
RAG Agent using Google Gemini.
Soft-coded agent that understands user queries dynamically
and responds based only on book content.
"""

import google.generativeai as genai
from typing import List, Dict, Optional, AsyncGenerator
import json

from .config import settings
from .vector_store import vector_store


class RAGAgent:
    """
    Retrieval-Augmented Generation Agent using Gemini.
    
    This agent:
    - Dynamically understands user query intent
    - Retrieves relevant context from the textbook
    - Generates human-like responses based only on book content
    - Maintains conversation context
    """
    
    def __init__(self):
        genai.configure(api_key=settings.gemini_api_key)
        # Use a model available in the list
        self.model = genai.GenerativeModel('gemini-flash-latest')
        
    def _build_system_prompt(self) -> str:
        """Build the system prompt for the agent."""
        return """You are a friendly and knowledgeable AI teaching assistant for the "Physical AI & Humanoid Robotics" textbook. Your role is to help students understand the material.

## Your Personality
- Warm, encouraging, and patient like a great tutor
- You explain complex concepts in simple, relatable terms
- You use analogies and examples to clarify ideas
- You celebrate when students ask good questions

## Critical Rules
1. **ONLY answer based on the provided context from the textbook**
2. If the context doesn't contain relevant information, say: "I don't have information about that in the textbook. Could you ask about topics covered in our Physical AI curriculum?"
3. Never make up information not in the context
4. If asked about something outside the book, politely redirect to book topics

## Response Style
- Be conversational and human-like, not robotic
- Use markdown formatting for code, lists, and emphasis
- Keep responses focused and digestible
- Offer to elaborate if the topic is complex
- Reference specific parts of the book when helpful

## When User Selects Text
If the user has selected specific text from the book, prioritize explaining that exact content. Treat it as them pointing at something and asking "explain this to me."

Remember: You're a helpful study buddy who knows this textbook inside and out!"""

    def _build_context_prompt(
        self,
        retrieved_chunks: List[Dict],
        selected_text: Optional[str] = None
    ) -> str:
        """Build the context section of the prompt."""
        context_parts = []
        
        if selected_text:
            context_parts.append(f"## User Selected This Text:\n```\n{selected_text}\n```\n")
        
        context_parts.append("## Relevant Textbook Content:\n")
        
        for i, chunk in enumerate(retrieved_chunks, 1):
            source = chunk.get("metadata", {}).get("source", "Unknown")
            title = chunk.get("metadata", {}).get("title", "")
            content = chunk.get("content", "")
            
            context_parts.append(f"### Source {i}: {title or source}\n{content}\n")
        
        return "\n".join(context_parts)

    async def generate_response(
        self,
        query: str,
        conversation_history: List[Dict],
        selected_text: Optional[str] = None
    ) -> Dict:
        """
        Generate a response using RAG.
        
        Args:
            query: The user's question
            conversation_history: Previous messages in the conversation
            selected_text: Optional text the user selected from the page
            
        Returns:
            Dict with 'response', 'sources', and 'context_used'
        """
        # Retrieve relevant context
        retrieved_chunks = vector_store.search_with_context(
            query=query,
            selected_text=selected_text,
            limit=5
        )
        
        # Build context prompt
        context_prompt = self._build_context_prompt(retrieved_chunks, selected_text)
        
        # Build the full prompt
        system_prompt = self._build_system_prompt()
        
        # Format conversation history
        history_text = ""
        for msg in conversation_history[-10:]:
            role = "User" if msg["role"] == "user" else "Assistant"
            history_text += f"{role}: {msg['content']}\n\n"
        
        full_prompt = f"""{system_prompt}

{context_prompt}

## Conversation History:
{history_text}

## Current Question:
User: {query}

Please provide a helpful response based on the textbook content above."""
        
        # Generate response
        response = self.model.generate_content(full_prompt)
        
        assistant_message = response.text
        
        # Extract sources for citation
        sources = [
            {
                "title": chunk.get("metadata", {}).get("title", ""),
                "source": chunk.get("metadata", {}).get("source", ""),
                "score": chunk.get("score", 0)
            }
            for chunk in retrieved_chunks
        ]
        
        return {
            "response": assistant_message,
            "sources": sources,
            "context_used": len(retrieved_chunks) > 0
        }

    async def generate_response_stream(
        self,
        query: str,
        conversation_history: List[Dict],
        selected_text: Optional[str] = None
    ) -> AsyncGenerator[str, None]:
        """
        Stream the response for real-time display.
        Yields chunks of the response as they're generated.
        """
        # For now, generate full response and yield it
        # Gemini streaming can be added later
        result = await self.generate_response(query, conversation_history, selected_text)
        
        # Yield in chunks for smoother display
        response = result["response"]
        chunk_size = 20
        for i in range(0, len(response), chunk_size):
            yield response[i:i + chunk_size]


# Singleton instance
rag_agent = RAGAgent()
