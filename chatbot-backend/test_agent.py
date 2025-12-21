import asyncio
import sys
import os

from dotenv import load_dotenv
load_dotenv()

# Mock settings if needed or verify they load
from app.config import settings
print(f"API Key present: {bool(settings.gemini_api_key)}")

# Import agent
from app.agent import rag_agent

async def test():
    print("Testing Agent Generation...")
    try:
        response = await rag_agent.generate_response(
            query="What is Physical AI?",
            conversation_history=[]
        )
        print("Response received:")
        print(response["response"][:100] + "...")
        print("Sources:", response["sources"])
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test())
