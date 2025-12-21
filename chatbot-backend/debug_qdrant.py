from qdrant_client import QdrantClient
from app.config import settings

client = QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key)
print("Client attributes:")
print([attr for attr in dir(client) if not attr.startswith('_')])
