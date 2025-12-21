import asyncio
from app.database import init_db, get_db, Conversation
from sqlalchemy import select

async def test_insert():
    print("Initializing DB...")
    await init_db()
    
    print("Testing Insert...")
    async for session in get_db():
        try:
            conv = Conversation()
            session.add(conv)
            await session.commit()
            print(f"Created conversation: {conv.id}")
            
            # Verify fetch
            result = await session.execute(select(Conversation).where(Conversation.id == conv.id))
            fetched = result.scalar_one_or_none()
            print(f"Fetched conversation: {fetched.id}")
            
        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_insert())
