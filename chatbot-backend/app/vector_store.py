"""
Qdrant vector store integration.
Handles document indexing and semantic search.
"""

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance, VectorParams, PointStruct,
    Filter, FieldCondition, MatchValue
)
from typing import List, Dict, Optional
import uuid

from .config import settings
from .embeddings import embedding_service


class VectorStore:
    """
    Qdrant vector store for semantic search.
    Stores document chunks and retrieves relevant context.
    """
    
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        self.collection_name = settings.qdrant_collection_name
        self.dimension = embedding_service.get_dimension()
    
    async def init_collection(self):
        """Initialize or verify the collection exists."""
        collections = self.client.get_collections().collections
        exists = any(c.name == self.collection_name for c in collections)
        
        if not exists:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.dimension,
                    distance=Distance.COSINE
                )
            )
            print(f"Created collection: {self.collection_name}")
        else:
            print(f"Collection exists: {self.collection_name}")
    
    def index_document(
        self,
        content: str,
        metadata: Dict,
        doc_id: Optional[str] = None
    ) -> str:
        """Index a single document chunk."""
        doc_id = doc_id or str(uuid.uuid4())
        embedding = embedding_service.embed_text(content)
        
        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                PointStruct(
                    id=doc_id,
                    vector=embedding,
                    payload={
                        "content": content,
                        **metadata
                    }
                )
            ]
        )
        return doc_id
    
    def index_documents(
        self,
        documents: List[Dict]
    ) -> List[str]:
        """
        Index multiple document chunks.
        Each document should have 'content' and 'metadata' keys.
        """
        points = []
        ids = []
        
        contents = [doc["content"] for doc in documents]
        embeddings = embedding_service.embed_texts(contents)
        
        for doc, embedding in zip(documents, embeddings):
            doc_id = doc.get("id", str(uuid.uuid4()))
            ids.append(doc_id)
            
            points.append(
                PointStruct(
                    id=doc_id,
                    vector=embedding,
                    payload={
                        "content": doc["content"],
                        **doc.get("metadata", {})
                    }
                )
            )
        
        # Batch upsert
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
        
        return ids
    
    def search(
        self,
        query: str,
        limit: int = 5,
        score_threshold: float = 0.3,
        filter_metadata: Optional[Dict] = None
    ) -> List[Dict]:
        """
        Semantic search for relevant document chunks.
        Returns list of chunks with content, metadata, and score.
        """
        query_embedding = embedding_service.embed_text(query)
        
        # Build filter if provided
        query_filter = None
        if filter_metadata:
            conditions = [
                FieldCondition(
                    key=key,
                    match=MatchValue(value=value)
                )
                for key, value in filter_metadata.items()
            ]
            query_filter = Filter(must=conditions)
        
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=limit,
            score_threshold=score_threshold,
            query_filter=query_filter
        ).points
        
        return [
            {
                "id": str(result.id),
                "content": result.payload.get("content", ""),
                "metadata": {
                    k: v for k, v in result.payload.items()
                    if k != "content"
                },
                "score": result.score
            }
            for result in results
        ]
    
    def search_with_context(
        self,
        query: str,
        selected_text: Optional[str] = None,
        limit: int = 5
    ) -> List[Dict]:
        """
        Enhanced search that considers user-selected text.
        If selected_text is provided, uses it to refine the search.
        """
        # If user selected specific text, search for that first
        if selected_text and len(selected_text.strip()) > 10:
            # Combine selected text with query for better context
            enhanced_query = f"{selected_text}\n\nUser question: {query}"
            results = self.search(enhanced_query, limit=limit)
        else:
            results = self.search(query, limit=limit)
        
        return results
    
    def get_collection_stats(self) -> Dict:
        """Get statistics about the collection."""
        info = self.client.get_collection(self.collection_name)
        return {
            "points_count": info.points_count,
            "status": info.status,
            "indexed_vectors_count": info.indexed_vectors_count if hasattr(info, 'indexed_vectors_count') else None
        }


# Singleton instance
vector_store = VectorStore()
