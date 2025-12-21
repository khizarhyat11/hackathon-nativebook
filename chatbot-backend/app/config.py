"""
Configuration settings for the RAG Chatbot.
Loads from environment variables.
"""

from pydantic_settings import BaseSettings
from typing import List
import os


class Settings(BaseSettings):
    """Application settings loaded from environment."""
    
    # Google Gemini API
    gemini_api_key: str
    
    # Qdrant Vector Database
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "physical_ai_textbook"
    
    # Neon Postgres
    database_url: str
    
    # Server
    host: str = "0.0.0.0"
    port: int = 8000
    cors_origins: str = "http://localhost:3000,http://localhost:3001"
    
    @property
    def cors_origins_list(self) -> List[str]:
        return [origin.strip() for origin in self.cors_origins.split(",")]
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


settings = Settings()
