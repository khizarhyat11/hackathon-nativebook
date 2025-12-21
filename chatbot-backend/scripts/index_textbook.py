"""
Script to index all textbook markdown files into the vector store.
Parses markdown, chunks content, and creates embeddings using Gemini.
"""

import os
import sys
import re
from pathlib import Path
from typing import List, Dict
import markdown
from bs4 import BeautifulSoup

# Add parent to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
load_dotenv()

from app.vector_store import vector_store
from app.embeddings import embedding_service


def parse_markdown(file_path: str) -> Dict:
    """Parse a markdown file and extract content and metadata."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Extract frontmatter
    metadata = {}
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) >= 3:
            frontmatter = parts[1]
            content = parts[2]
            
            for line in frontmatter.strip().split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    metadata[key.strip()] = value.strip().strip('"\'')
    
    # Convert markdown to text
    html = markdown.markdown(content, extensions=['fenced_code', 'tables'])
    soup = BeautifulSoup(html, 'html.parser')
    
    # Get plain text
    text = soup.get_text(separator='\n')
    
    # Clean up
    text = re.sub(r'\n{3,}', '\n\n', text)
    text = text.strip()
    
    return {
        'content': text,
        'metadata': metadata,
        'raw_markdown': content
    }


def chunk_text(
    text: str,
    chunk_size: int = 500,
    overlap: int = 100
) -> List[str]:
    """Split text into overlapping chunks."""
    chunks = []
    
    # Split by paragraphs first
    paragraphs = text.split('\n\n')
    
    current_chunk = ""
    
    for para in paragraphs:
        para = para.strip()
        if not para:
            continue
            
        # If adding this paragraph exceeds chunk size, save current and start new
        if len(current_chunk) + len(para) > chunk_size and current_chunk:
            chunks.append(current_chunk.strip())
            # Keep overlap from end of previous chunk
            words = current_chunk.split()
            overlap_words = words[-overlap//5:] if len(words) > overlap//5 else []
            current_chunk = ' '.join(overlap_words) + ' ' + para
        else:
            current_chunk += '\n\n' + para if current_chunk else para
    
    # Don't forget the last chunk
    if current_chunk.strip():
        chunks.append(current_chunk.strip())
    
    return chunks


def index_textbook(docs_path: str):
    """Index all markdown files from the docs directory."""
    docs_dir = Path(docs_path)
    
    if not docs_dir.exists():
        print(f"Error: Directory not found: {docs_dir}")
        return
    
    # Find all markdown files
    md_files = list(docs_dir.rglob('*.md'))
    print(f"Found {len(md_files)} markdown files")
    
    all_documents = []
    
    for md_file in md_files:
        print(f"Processing: {md_file.relative_to(docs_dir)}")
        
        try:
            parsed = parse_markdown(str(md_file))
            
            # Get relative path for source
            relative_path = str(md_file.relative_to(docs_dir))
            title = parsed['metadata'].get('title', md_file.stem)
            
            # Chunk the content
            chunks = chunk_text(parsed['content'])
            
            for i, chunk in enumerate(chunks):
                if len(chunk) < 50:  # Skip very small chunks
                    continue
                    
                all_documents.append({
                    'content': chunk,
                    'metadata': {
                        'source': relative_path,
                        'title': title,
                        'chunk_index': i,
                        'total_chunks': len(chunks),
                        **{k: v for k, v in parsed['metadata'].items() 
                           if k not in ['source', 'title']}
                    }
                })
                
        except Exception as e:
            print(f"  Error processing {md_file}: {e}")
    
    print(f"\nTotal chunks to index: {len(all_documents)}")
    
    if all_documents:
        print("Indexing documents...")
        ids = vector_store.index_documents(all_documents)
        print(f"Successfully indexed {len(ids)} chunks")
        
        # Print stats
        stats = vector_store.get_collection_stats()
        print(f"\nVector store stats: {stats}")


def main():
    """Main entry point."""
    import asyncio
    
    # Default docs path
    docs_path = Path(__file__).parent.parent.parent / 'my-website' / 'docs'
    
    # Allow override via command line
    if len(sys.argv) > 1:
        docs_path = Path(sys.argv[1])
    
    print(f"Indexing textbook from: {docs_path}")
    print(f"Using Google Gemini embeddings")
    
    # Initialize collection
    async def init():
        await vector_store.init_collection()
    
    asyncio.run(init())
    
    # Index documents
    index_textbook(str(docs_path))
    
    print("\nIndexing complete!")


if __name__ == '__main__':
    main()
