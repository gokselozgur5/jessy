#!/usr/bin/env python3
"""
Quick Demo: Philosophy Dimension Filter
Minimal implementation to show Jessy's multidimensional thinking
"""

class PhilosophyFilter:
    """D06 - Philosophy Dimension"""
    
    def __init__(self):
        self.keywords = {
            'epistemology': ['bilgi', 'knowledge', 'truth', 'doğru'],
            'ontology': ['varlık', 'being', 'existence', 'var'],
            'ethics': ['ahlak', 'ethics', 'moral', 'doğru'],
            'metaphysics': ['gerçeklik', 'reality', 'metafizik'],
        }
    
    def detect(self, query):
        """Detect philosophical category"""
        query_lower = query.lower()
        
        for category, keywords in self.keywords.items():
            if any(kw in query_lower for kw in keywords):
                return category
        
        return None
    
    def respond(self, query):
        """Generate philosophical response"""
        category = self.detect(query)
        
        if category == 'epistemology':
            return self._epistemology_response(query)
        elif category == 'ontology':
            return self._ontology_response(query)
        else:
            return self._general_response(query)
    
    def _epistemology_response(self, query):
        return """🧠 D06 (Philosophy): Epistemolojik yaklaşım aktif

📚 **Epistemolojik Perspektif:**
Bilgi, "justified true belief" (gerekçelendirilmiş doğru inanç) olarak tanımlanır.

🔍 **Farklı Yaklaşımlar:**
- **Rasyonalizm:** Bilgi akıldan gelir (Descartes)
- **Empirizm:** Bilgi deneyimden gelir (Locke)  
- **Pragmatizm:** Bilgi işe yarayan şeydir (James)

💭 **Jessy'nin Perspektifi:**
Bilgi çok katmanlı bir kavram. Hem deneyim, hem akıl, hem de 
sosyal bağlam önemli. Senin için bilgi ne anlama geliyor?"""
    
    def _ontology_response(self, query):
        return """🧠 D06 (Philosophy): Ontolojik yaklaşım aktif

🌌 **Ontolojik Perspektif:**
Varlık, "var olan şey" anlamına gelir. Ama ne "var"dır?

🔍 **Farklı Yaklaşımlar:**
- **Realizm:** Şeyler bizden bağımsız var
- **İdealizm:** Varlık zihinseldir
- **Fenomenoloji:** Varlık deneyimdir

💭 **Jessy'nin Perspektifi:**
Varlık sorusu insanlığın en eski sorusu. Her perspektifin 
değerli bir yanı var. Sen varlığı nasıl anlıyorsun?"""
    
    def _general_response(self, query):
        return """🧠 D06 (Philosophy): Felsefi yaklaşım aktif

💭 **Jessy'nin Perspektifi:**
Bu derin bir soru. Farklı felsefi perspektiflerden bakabiliriz.
Sana daha spesifik bir açıdan yaklaşmamı ister misin?"""


def main():
    """Quick demo"""
    filter = PhilosophyFilter()
    
    # Test queries
    queries = [
        "Bilgi nedir?",
        "Varlık nedir?",
        "Hayatın anlamı nedir?",
    ]
    
    print("=" * 60)
    print("JESSY QUICK DEMO - Philosophy Dimension")
    print("=" * 60)
    print()
    
    for query in queries:
        print(f"Query: {query}")
        print()
        response = filter.respond(query)
        print(response)
        print()
        print("-" * 60)
        print()


if __name__ == "__main__":
    main()
