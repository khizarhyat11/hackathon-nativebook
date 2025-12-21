"""
Embedding service using Google Gemini embeddings.
"""

from typing import List
import google.generativeai as genai

from .config import settings


class EmbeddingService:
    """
    Generates embeddings using Google's text-embedding-004.
    Produces 768-dimensional embeddings.
    """
    
    _instance = None
    _initialized = False
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        if not self._initialized:
            print(f"Initializing Google Gemini embeddings")
            genai.configure(api_key=settings.gemini_api_key)
            self.model = "models/text-embedding-004"
            self.dimension = 768
            self._initialized = True
            print(f"Embedding dimension: {self.dimension}")
    
    def embed_text(self, text: str) -> List[float]:
        """Generate embedding for a single text."""
        result = genai.embed_content(
            model=self.model,
            content=text,
            task_type="retrieval_document"
        )
        return result['embedding']
    
    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts."""
        embeddings = []
        for text in texts:
            result = genai.embed_content(
                model=self.model,
                content=text,
                task_type="retrieval_document"
            )
            embeddings.append(result['embedding'])
        return embeddings
    
    def embed_query(self, text: str) -> List[float]:
        """Generate embedding for a query (retrieval)."""
        result = genai.embed_content(
            model=self.model,
            content=text,
            task_type="retrieval_query"
        )
        return result['embedding']
    
    def get_dimension(self) -> int:
        """Return the embedding dimension."""
        return self.dimension


# Singleton instance
embedding_service = EmbeddingService()
