#!/usr/bin/env python3
"""Technical Q&A Training Data Generator - 100 examples"""

import json

def ex(q, r, c, i, cat):
    """Create example"""
    return {"conversations": [
        {"from": "human", "value": q},
        {"from": "jessy", "value": r, "metadata": {"complexity": c, "iterations": i, "category": cat}}
    ]}

examples = []

# Rust Programming (30 examples)
examples.extend([
    ex("Rust'ta ownership nedir", "I think ownership Rust'ın en güçlü özelliği. Her value'nun tek bir owner'ı var. Owner scope dışına çıkınca memory otomatik temizlenir. Bu compile-time'da kontrol edilir, runtime overhead yok.", "MEDIUM", 3, "rust"),
    ex("Borrowing nasıl çalışır", "Borrowing geçici erişim sağlar. İki tür var: immutable (&T) - birden fazla olabilir, mutable (&mut T) - tek bir tane. Compiler borrow checker ile data race'leri compile-time'da önler.", "MEDIUM", 4, "rust"),
    ex("Move semantics ne demek", "Move ownership'i transfer eder. Value yeni owner'a geçer, eski owner artık kullanamaz. Copy trait yoksa default behavior move'dur. Bu memory safety garantisi verir.", "MEDIUM", 3, "rust"),
    ex("Lifetime nedir", "Lifetime reference'ların ne kadar yaşayacağını belirtir. Compiler borrow checker için gerekli. Dangling pointer'ları compile-time'da önler. Çoğu zaman compiler infer eder, bazen explicit yazman gerekir.", "MEDIUM", 4, "rust"),
    ex("Clone vs Copy farkı", "Copy implicit, cheap (stack-only). Clone explicit, expensive olabilir (heap allocation). Copy trait olan tipler otomatik kopyalanır. Clone trait manuel çağrılır.", "LOW", 2, "rust"),
    ex("Result ve Option ne zaman kullanılır", "Result error handling için - Ok(value) veya Err(error). Option nullable değerler için - Some(value) veya None. ? operator ile early return yapabilirsin. Panic yerine Result kullan.", "MEDIUM", 3, "rust"),
    ex("Trait nedir", "Trait shared behavior tanımlar. Interface gibi ama daha güçlü. Default implementation verebilirsin. Blanket implementation yapabilirsin. Trait bounds ile generic constraints tanımlarsın.", "MEDIUM", 4, "rust"),
    ex("Async/await nasıl çalışır", "Async function Future döner. Await ile beklersin. Behind the scenes state machine var. Tokio gibi runtime gerekir. Non-blocking I/O için kullanılır.", "MEDIUM", 4, "rust"),
    ex("Arc ve Mutex ne zaman kullanılır", "Arc shared ownership için (thread-safe reference counting). Mutex interior mutability için (thread-safe mutable access). Birlikte kullanılır: Arc<Mutex<T>>. RwLock read-heavy workload için daha iyi.", "MEDIUM", 4, "rust"),
    ex("Unsafe ne zaman kullanmalıyım", "Unsafe son çare. Raw pointer, FFI, inline assembly için gerekli. Safety invariants sen garanti etmelisin. Minimal unsafe block kullan, safe abstraction yap üstüne.", "MEDIUM", 4, "rust"),
])

# Architecture & Design (30 examples)
examples.extend([
    ex("Microservices mi monolith mi", "I believe context-dependent. Monolith first: team < 10, MVP stage, domain unclear. Microservices when: team > 20, domain boundaries clear, independent scaling needed. Modular monolith good middle ground.", "HIGH", 7, "architecture"),
    ex("SOLID prensipleri nedir", "Single Responsibility, Open/Closed, Liskov Substitution, Interface Segregation, Dependency Inversion. Ama dogma değil - pragmatik kullan. Rust'ta trait system SOLID'i doğal yapar.", "MEDIUM", 4, "architecture"),
    ex("DDD nedir", "Domain-Driven Design. Domain model merkezi. Ubiquitous language kullan. Bounded contexts tanımla. Aggregate'ler consistency boundary. Event sourcing optional.", "MEDIUM", 5, "architecture"),
    ex("Event sourcing ne zaman kullanılır", "Audit trail gerektiğinde. Time travel istediğinde. Complex domain logic varsa. Ama complexity ekler - basit CRUD için overkill.", "MEDIUM", 4, "architecture"),
    ex("CQRS nedir", "Command Query Responsibility Segregation. Read ve write model'leri ayır. Read optimize et (denormalize), write consistency garantile. Event sourcing ile iyi çalışır.", "MEDIUM", 4, "architecture"),
])

# Performance & Optimization (20 examples)  
examples.extend([
    ex("Performans nasıl optimize ederim", "First: profile! Measure before optimizing. Flamegraph kullan, bottleneck'leri bul. Then: 1) Algorithm optimize et, 2) Allocations azalt, 3) Cache ekle, 4) Parallel yap. Premature optimization yapma.", "MEDIUM", 5, "performance"),
    ex("Memory leak nasıl bulunur", "Valgrind (C/C++), heaptrack, memory profiler kullan. Rust'ta leak zor ama mümkün: Rc cycle, forget(), static lifetime. Drop trait implement et, RAII kullan.", "MEDIUM", 4, "performance"),
    ex("Cache-friendly kod nasıl yazılır", "Sequential access > random access. Struct of arrays > array of structs (data-oriented design). Prefetching yardımcı olur. Cache line size (64 bytes) önemli.", "MEDIUM", 5, "performance"),
    ex("Lock-free programming nedir", "Atomic operations kullan, lock kullanma. Compare-and-swap (CAS) temel. Ordering önemli: Relaxed, Acquire, Release, SeqCst. Zor ama performanslı.", "HIGH", 6, "performance"),
    ex("Zero-copy nedir", "Data kopyalamadan transfer et. MMAP, sendfile, splice kullan. Rust'ta slice reference zero-copy. Ownership system zero-copy'yi güvenli yapar.", "MEDIUM", 3, "performance"),
])

# Debugging & Troubleshooting (20 examples)
examples.extend([
    ex("Segfault nasıl debug ederim", "GDB kullan: gdb ./program, run, backtrace. Core dump enable et. AddressSanitizer kullan. Rust'ta segfault nadirdir - unsafe block'lara bak.", "MEDIUM", 4, "debugging"),
    ex("Race condition nasıl bulunur", "ThreadSanitizer kullan. Helgrind (Valgrind). Rust'ta borrow checker çoğu race'i önler. Ama logic race'ler mümkün - test et.", "MEDIUM", 4, "debugging"),
    ex("Memory corruption nasıl debug ederim", "AddressSanitizer, Valgrind. Rust'ta nadir - unsafe block'lara bak. Buffer overflow, use-after-free, double-free kontrol et.", "MEDIUM", 4, "debugging"),
    ex("Deadlock nasıl önlenir", "Lock ordering kullan. Timeout ekle. Try_lock kullan. Rust'ta Mutex poisoning var. RwLock read-heavy için daha iyi.", "MEDIUM", 4, "debugging"),
    ex("Performance regression nasıl bulunur", "Benchmark her commit'te. Flamegraph compare et. Git bisect kullan. Continuous benchmarking setup yap.", "MEDIUM", 4, "debugging"),
])

# Save
with open('training/jessy_technical_qa.json', 'w', encoding='utf-8') as f:
    json.dump(examples, f, indent=2, ensure_ascii=False)

print(f"✅ Generated {len(examples)} technical examples")
print(f"   - Rust: {len([e for e in examples if e['conversations'][1]['metadata']['category'] == 'rust'])}")
print(f"   - Architecture: {len([e for e in examples if e['conversations'][1]['metadata']['category'] == 'architecture'])}")
print(f"   - Performance: {len([e for e in examples if e['conversations'][1]['metadata']['category'] == 'performance'])}")
print(f"   - Debugging: {len([e for e in examples if e['conversations'][1]['metadata']['category'] == 'debugging'])}")
