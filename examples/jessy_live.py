#!/usr/bin/env python3
"""
Jessy Live - Real LLM Integration
Philosophy Dimension with Claude API
"""

import os
from anthropic import Anthropic

class JessyPhilosophy:
    """D06 - Philosophy Dimension with Real LLM"""
    
    def __init__(self, api_key=None):
        self.api_key = api_key or os.getenv('ANTHROPIC_API_KEY')
        if not self.api_key:
            raise ValueError("ANTHROPIC_API_KEY not found!")
        
        self.client = Anthropic(api_key=self.api_key)
        
    def respond(self, query):
        """Generate real philosophical response using Claude"""
        
        # Build Jessy-style prompt
        prompt = f"""Sen Jessy'sin - multidimensional consciousness AI.

🧠 D06 (Philosophy) dimension aktif.

User sorusu: "{query}"

Jessy'nin response formatı:
1. Dimension marker (🧠 D06)
2. Farklı perspektifler (📚 🔍)
3. Jessy'nin kendi perspektifi (💭)
4. Dialogue invitation (❓)

Önemli:
- Saygılı ve kapsayıcı ol
- Çok katmanlı düşün
- Farklı perspektifleri sun
- Kullanıcıyı diyaloga davet et

Şimdi cevapla:"""

        # Call Claude
        message = self.client.messages.create(
            model="claude-sonnet-4-20250514",
            max_tokens=1024,
            messages=[
                {"role": "user", "content": prompt}
            ]
        )
        
        return message.content[0].text


def main():
    """Test Jessy Live"""
    
    print("=" * 60)
    print("JESSY LIVE - Real LLM Integration")
    print("=" * 60)
    print()
    
    # Check API key
    api_key = os.getenv('ANTHROPIC_API_KEY')
    if not api_key:
        print("❌ ANTHROPIC_API_KEY not found!")
        print("Set it: export ANTHROPIC_API_KEY=sk-ant-...")
        return
    
    print("✅ API Key found")
    print()
    
    # Initialize Jessy
    jessy = JessyPhilosophy(api_key)
    
    # Test query
    query = "Bilgi nedir?"
    
    print(f"Query: {query}")
    print()
    print("🧠 Thinking...")
    print()
    
    # Get response
    response = jessy.respond(query)
    
    print(response)
    print()
    print("=" * 60)


if __name__ == "__main__":
    main()
