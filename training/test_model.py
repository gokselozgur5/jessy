#!/usr/bin/env python3
"""
Test the fine-tuned JESSY Turkish model
"""

import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from peft import PeftModel

print("🧪 Testing JESSY Turkish Model")
print("=" * 60)

# Load model
print("\n📂 Loading fine-tuned model...")
MODEL_DIR = "./jessy-turkish-simple"

tokenizer = AutoTokenizer.from_pretrained(MODEL_DIR)
base_model = AutoModelForCausalLM.from_pretrained(
    "TinyLlama/TinyLlama-1.1B-Chat-v1.0",
    torch_dtype=torch.float16,
    device_map="mps" if torch.backends.mps.is_available() else "cpu",
)
model = PeftModel.from_pretrained(base_model, MODEL_DIR)

print("✅ Model loaded")

# Test queries
test_queries = [
    "sana bisi sorucam",  # THE canonical test
    "merhaba",
    "sen kimsin",
    "Rust nedir",
    "nasılsın",
]

print("\n🧪 Testing queries:")
print("=" * 60)

for query in test_queries:
    prompt = f"### Instruction:\n{query}\n\n### Response:\n"
    
    inputs = tokenizer(prompt, return_tensors="pt").to(model.device)
    
    with torch.no_grad():
        outputs = model.generate(
            **inputs,
            max_new_tokens=100,
            temperature=0.7,
            do_sample=True,
            top_p=0.9,
        )
    
    response = tokenizer.decode(outputs[0], skip_special_tokens=True)
    
    # Extract just the response part
    if "### Response:" in response:
        response = response.split("### Response:")[1].strip()
    
    print(f"\n❓ Q: {query}")
    print(f"💬 A: {response}")
    print("-" * 60)

print("\n✨ Testing complete!")
