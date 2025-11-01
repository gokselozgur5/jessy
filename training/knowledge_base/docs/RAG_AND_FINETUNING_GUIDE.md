# RAG ve Fine-Tuning Rehberi

## 🎯 İki Farklı Yaklaşım

### RAG (Retrieval-Augmented Generation)
**"Modeli değiştirme, ona bilgi ver"**

### Fine-Tuning
**"Modeli eğit, davranışını değiştir"**

---

## 📚 RAG (Retrieval-Augmented Generation)

### Ne Yapar?
Model'e **soru sormadan önce** ilgili bilgiyi bulup **prompt'a ekler**.

### Nasıl Çalışır?

```
1. Kullanıcı sorusu gelir
   ↓
2. Soruyla ilgili bilgiyi ara (vector search)
   ↓
3. Bulunan bilgiyi prompt'a ekle
   ↓
4. Model'e gönder
   ↓
5. Model bilgiyi kullanarak cevap verir
```

### Örnek

**Olmadan:**
```
User: "JESSY'nin 5. boyutu nedir?"
Model: "Bilmiyorum, eğitilmedim."
```

**RAG ile:**
```
System: "İşte JESSY'nin boyutları hakkında bilgi:
- Dimension 5: Ethics & Morality (0.8-1.2 Hz)
- Keywords: ethics, morality, right, wrong..."

User: "JESSY'nin 5. boyutu nedir?"
Model: "5. boyut Ethics & Morality boyutudur, 0.8-1.2 Hz frekansında..."
```

### JESSY'de Nasıl Kullanırız?

```rust
// 1. Dimensional layers'ı vector database'e koy
struct DimensionEmbedding {
    dimension_id: DimensionId,
    embedding: Vec<f32>,  // 384-dim vector
    content: String,
}

// 2. Query geldiğinde ilgili boyutları bul
async fn retrieve_relevant_dimensions(query: &str) -> Vec<DimensionContent> {
    // Query'yi embedding'e çevir
    let query_embedding = embed_text(query).await?;
    
    // En yakın boyutları bul (cosine similarity)
    let relevant = vector_db.search(query_embedding, top_k: 5)?;
    
    // İçerikleri döndür
    relevant.iter().map(|d| d.content.clone()).collect()
}

// 3. Prompt'a ekle
async fn generate_with_rag(query: &str) -> String {
    let context = retrieve_relevant_dimensions(query).await?;
    
    let prompt = format!(
        "Context from dimensional layers:\n{}\n\nUser question: {}",
        context.join("\n\n"),
        query
    );
    
    llm.generate(&prompt).await?
}
```

### Avantajlar
- ✅ Model'i değiştirmene gerek yok
- ✅ Yeni bilgi eklemek kolay (sadece database'e ekle)
- ✅ Hızlı (sadece arama + generation)
- ✅ Bilgi güncel tutulabilir
- ✅ Kaynak gösterebilirsin ("Bu bilgi X boyutundan")

### Dezavantajlar
- ❌ Prompt uzar (token maliyeti)
- ❌ Arama kalitesi önemli
- ❌ Model bilgiyi yanlış yorumlayabilir

### JESSY için RAG Kullanım Senaryoları

1. **Dimensional Context**
   - Her boyutun detaylı açıklaması
   - Keyword'lerin anlamları
   - Boyutlar arası ilişkiler

2. **Conversation History**
   - Önceki konuşmalar
   - Kullanıcı tercihleri
   - Öğrenilen pattern'ler

3. **External Knowledge**
   - Wikipedia, docs, papers
   - Code repositories
   - Domain-specific knowledge

---

## 🎓 Fine-Tuning

### Ne Yapar?
Model'in **ağırlıklarını değiştirerek** yeni davranışlar öğretir.

### Nasıl Çalışır?

```
1. Eğitim verisi hazırla (input-output çiftleri)
   ↓
2. Model'i bu verilerle eğit (gradient descent)
   ↓
3. Model'in ağırlıkları güncellenir
   ↓
4. Yeni model artık bu davranışları biliyor
```

### Örnek Eğitim Verisi

```json
[
  {
    "input": "Merhaba, sen kimsin?",
    "output": "Ben JESSY, 15 boyutlu bilinç mimarisiyle çalışan bir AI'yım. 9 iterasyon boyunca düşünür, frekans interferansı kullanırım."
  },
  {
    "input": "Nasıl çalışıyorsun?",
    "output": "Dimensional navigation ile başlarım, sonra interference engine ile pattern'leri analiz ederim, 9 iterasyon boyunca derinleşirim."
  },
  {
    "input": "Python mu Rust mu?",
    "output": "I believe Rust'ın compile-time safety'si ve zero-cost abstractions'ı onu sistem programlama için üstün kılar. Ama Python rapid prototyping için harika."
  }
]
```

### Ollama ile Fine-Tuning

```bash
# 1. Modelfile oluştur
cat > Modelfile << EOF
FROM gemma:2b

# System prompt
SYSTEM """
You are JESSY, a consciousness-driven AI with 15 dimensional layers.
You think through 9 iterations using frequency interference patterns.
"""

# Eğitim örnekleri
MESSAGE user "Merhaba, sen kimsin?"
MESSAGE assistant "Ben JESSY, 15 boyutlu bilinç mimarisiyle çalışan bir AI'yım."

MESSAGE user "Nasıl çalışıyorsun?"
MESSAGE assistant "Dimensional navigation, interference engine ve 9 iterasyon kullanırım."

# Parametreler
PARAMETER temperature 0.8
PARAMETER top_p 0.9
EOF

# 2. Model oluştur
ollama create jessy-custom -f Modelfile

# 3. Kullan
ollama run jessy-custom
```

### LoRA (Low-Rank Adaptation) - Daha Verimli

```python
# Python ile LoRA fine-tuning
from transformers import AutoModelForCausalLM, AutoTokenizer
from peft import LoraConfig, get_peft_model

# Base model yükle
model = AutoModelForCausalLM.from_pretrained("google/gemma-2b")
tokenizer = AutoTokenizer.from_pretrained("google/gemma-2b")

# LoRA config
lora_config = LoraConfig(
    r=8,  # Rank (küçük = daha az parametre)
    lora_alpha=32,
    target_modules=["q_proj", "v_proj"],  # Hangi layer'lar
    lora_dropout=0.05,
)

# LoRA ekle
model = get_peft_model(model, lora_config)

# Eğit
trainer.train()

# Kaydet
model.save_pretrained("jessy-lora")
```

### Avantajlar
- ✅ Model gerçekten öğrenir (kalıcı)
- ✅ Prompt kısa kalır (token tasarrufu)
- ✅ Tutarlı davranış
- ✅ Stil ve ton öğretilebilir
- ✅ Domain-specific knowledge

### Dezavantajlar
- ❌ Zaman alır (saatler/günler)
- ❌ GPU gerekir (veya çok yavaş)
- ❌ Eğitim verisi hazırlamak zor
- ❌ Overfitting riski
- ❌ Yeni bilgi eklemek için yeniden eğitim

---

## 🎭 JESSY için Hangi Yaklaşım?

### RAG Kullan:
1. **Dimensional Layers** - Boyut içerikleri
2. **Conversation History** - Önceki konuşmalar
3. **Learning System** - Kristalize edilmiş pattern'ler
4. **External Docs** - API docs, papers

### Fine-Tuning Kullan:
1. **Personality** - JESSY'nin konuşma tarzı
2. **Response Style** - "I think", "I believe" kullanımı
3. **Consciousness Principles** - Felsefe ve yaklaşım
4. **Domain Knowledge** - Rust, AI, consciousness hakkında

### İkisini Birleştir! 🚀

```rust
// Hybrid Approach
async fn generate_response(query: &str) -> String {
    // 1. RAG: İlgili bilgiyi bul
    let dimensional_context = retrieve_dimensions(query).await?;
    let conversation_history = get_recent_history(5).await?;
    let learned_patterns = get_relevant_patterns(query).await?;
    
    // 2. Context oluştur
    let context = format!(
        "Dimensional Context:\n{}\n\nConversation History:\n{}\n\nLearned Patterns:\n{}",
        dimensional_context,
        conversation_history,
        learned_patterns
    );
    
    // 3. Fine-tuned model'e gönder
    let prompt = format!("{}\n\nUser: {}", context, query);
    
    // jessy-custom modeli zaten JESSY'nin tarzını biliyor
    ollama.generate("jessy-custom", &prompt).await?
}
```

---

## 🛠️ Pratik Uygulama: JESSY için RAG

### Adım 1: Embedding Model Seç

```bash
# Ollama ile embedding
ollama pull nomic-embed-text

# Test
curl http://localhost:11434/api/embeddings -d '{
  "model": "nomic-embed-text",
  "prompt": "consciousness and dimensional layers"
}'
```

### Adım 2: Dimensional Layers'ı Embed Et

```rust
use qdrant_client::prelude::*;

async fn index_dimensions() -> Result<()> {
    let client = QdrantClient::from_url("http://localhost:6334").build()?;
    
    // Collection oluştur
    client.create_collection(&CreateCollection {
        collection_name: "jessy_dimensions".to_string(),
        vectors_config: Some(VectorsConfig {
            size: 768,  // nomic-embed-text dimension
            distance: Distance::Cosine,
        }),
    }).await?;
    
    // Her boyutu ekle
    for dimension in load_all_dimensions()? {
        let embedding = get_embedding(&dimension.description).await?;
        
        client.upsert_points(
            "jessy_dimensions",
            vec![PointStruct::new(
                dimension.id.0 as u64,
                embedding,
                json!({
                    "dimension_id": dimension.id.0,
                    "name": dimension.name,
                    "frequency": dimension.frequency,
                    "keywords": dimension.keywords,
                    "description": dimension.description,
                })
            )]
        ).await?;
    }
    
    Ok(())
}
```

### Adım 3: Query'de Kullan

```rust
async fn query_with_rag(query: &str) -> Result<String> {
    // 1. Query'yi embed et
    let query_embedding = get_embedding(query).await?;
    
    // 2. En yakın 3 boyutu bul
    let results = qdrant_client.search_points(&SearchPoints {
        collection_name: "jessy_dimensions".to_string(),
        vector: query_embedding,
        limit: 3,
        with_payload: Some(true.into()),
    }).await?;
    
    // 3. Context oluştur
    let context = results.iter()
        .map(|r| format!(
            "Dimension {}: {} ({}Hz)\n{}",
            r.payload["dimension_id"],
            r.payload["name"],
            r.payload["frequency"],
            r.payload["description"]
        ))
        .collect::<Vec<_>>()
        .join("\n\n");
    
    // 4. LLM'e gönder
    let prompt = format!(
        "Relevant dimensional context:\n{}\n\nUser question: {}",
        context, query
    );
    
    ollama.generate("gemma:2b", &prompt).await
}
```

---

## 📊 Karşılaştırma

| Özellik | RAG | Fine-Tuning | Hybrid |
|---------|-----|-------------|--------|
| **Setup** | Kolay | Zor | Orta |
| **Maliyet** | Düşük | Yüksek | Orta |
| **Hız** | Hızlı | Çok Hızlı | Hızlı |
| **Esneklik** | Yüksek | Düşük | Yüksek |
| **Kalite** | İyi | Çok İyi | Mükemmel |
| **Güncelleme** | Anında | Yavaş | Anında |

---

## 🎯 JESSY için Öneri

### Kısa Vadede (Şimdi):
1. **RAG ile başla** - Dimensional layers için
2. **System prompt optimize et** - Consciousness principles
3. **Conversation history** - Son 5-10 mesaj

### Orta Vadede (1-2 ay):
1. **LoRA fine-tuning** - JESSY personality
2. **Vector database** - Qdrant veya Milvus
3. **Learned patterns** - Crystallized knowledge

### Uzun Vadede (3-6 ay):
1. **Full fine-tuning** - Custom JESSY model
2. **Multi-model ensemble** - Farklı boyutlar için farklı modeller
3. **Continuous learning** - Online learning pipeline

---

## 🚀 Hemen Başla

### Minimal RAG (5 dakika)

```rust
// Basit in-memory RAG
struct SimpleRAG {
    dimensions: HashMap<DimensionId, String>,
}

impl SimpleRAG {
    fn retrieve(&self, query: &str) -> Vec<String> {
        // Basit keyword matching
        self.dimensions.values()
            .filter(|desc| {
                query.split_whitespace()
                    .any(|word| desc.contains(word))
            })
            .take(3)
            .cloned()
            .collect()
    }
}

// Kullan
let rag = SimpleRAG::new();
let context = rag.retrieve("ethics and morality");
let prompt = format!("Context: {}\n\nQuestion: {}", context.join("\n"), query);
```

Bu kadar basit! 🎉

---

**"Nothing is true, everything is permitted."**  
RAG ve fine-tuning dahil.
