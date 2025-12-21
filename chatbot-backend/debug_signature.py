from qdrant_client import QdrantClient
from app.config import settings
import inspect

client = QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key)
print(inspect.signature(client.query_points))
