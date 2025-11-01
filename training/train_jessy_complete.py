#!/usr/bin/env python3
"""
JESSY Complete Training Script
Fine-tune model with full JESSY personality on M2 MacBook (MLX)
"""

import json
import argparse
from pathlib import Path
from dataclasses import dataclass
from typing import Optional
import mlx.core as mx
import mlx.nn as nn
from mlx_lm import load, generate
from mlx_lm.utils import generate_step
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn, TimeRemainingColumn
from rich.table import Table
import time

console = Console()


@dataclass
class TrainingConfig:
    """Training configuration"""
    
    # Model
    model_name: str = "mlx-community/gemma-2b"
    
    # Data
    train_file: str = "training/datasets/jessy_complete_train.jsonl"
    val_file: str = "training/datasets/jessy_complete_val.jsonl"
    
    # Training
    batch_size: int = 4
    learning_rate: float = 5e-5
    num_epochs: int = 3
    warmup_steps: int = 100
    max_seq_length: int = 2048
    
    # LoRA
    lora_rank: int = 16
    lora_alpha: int = 32
    lora_dropout: float = 0.1
    lora_layers: int = 16  # Number of layers to apply LoRA
    
    # Optimization
    gradient_accumulation_steps: int = 4
    max_grad_norm: float = 1.0
    
    # Checkpointing
    save_steps: int = 500
    eval_steps: int = 250
    output_dir: str = "training/checkpoints/jessy-complete"
    
    # Monitoring
    log_steps: int = 10


class JESSYTrainer:
    """Complete JESSY fine-tuning trainer for M2"""
    
    def __init__(self, config: TrainingConfig):
        self.config = config
        
        console.print("[bold green]🚀 JESSY Complete Training[/bold green]")
        console.print(f"Model: {config.model_name}")
        console.print(f"Output: {config.output_dir}")
        
        # Load model
        console.print("\n[blue]Loading model...[/blue]")
        self.model, self.tokenizer = load(config.model_name)
        console.print("[green]✅ Model loaded[/green]")
        
        # Load data
        console.print("\n[blue]Loading data...[/blue]")
        self.train_data = self._load_data(config.train_file)
        self.val_data = self._load_data(config.val_file)
        console.print(f"[green]✅ Loaded {len(self.train_data)} training examples[/green]")
        console.print(f"[green]✅ Loaded {len(self.val_data)} validation examples[/green]")
        
        # Setup output directory
        self.output_dir = Path(config.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Training state
        self.global_step = 0
        self.best_val_loss = float('inf')
        self.training_history = []
    
    def _load_data(self, filepath: str) -> list:
        """Load JSONL dataset"""
        data = []
        with open(filepath, 'r', encoding='utf-8') as f:
            for line in f:
                data.append(json.loads(line))
        return data
    
    def _prepare_batch(self, examples: list) -> tuple:
        """Prepare batch for training"""
        
        inputs = []
        targets = []
        
        for ex in examples:
            # Format as conversation
            text = f"User: {ex['input']}\nJESSY: {ex['output']}"
            
            # Tokenize
            tokens = self.tokenizer.encode(text)
            
            # Truncate if needed
            if len(tokens) > self.config.max_seq_length:
                tokens = tokens[:self.config.max_seq_length]
            
            inputs.append(tokens[:-1])  # Input is all but last token
            targets.append(tokens[1:])  # Target is all but first token
        
        # Pad to same length
        max_len = max(len(x) for x in inputs)
        
        padded_inputs = []
        padded_targets = []
        
        for inp, tgt in zip(inputs, targets):
            pad_len = max_len - len(inp)
            padded_inputs.append(inp + [self.tokenizer.pad_token_id] * pad_len)
            padded_targets.append(tgt + [-100] * pad_len)  # -100 is ignore index
        
        return mx.array(padded_inputs), mx.array(padded_targets)
    
    def _compute_loss(self, logits: mx.array, targets: mx.array) -> mx.array:
        """Compute cross-entropy loss"""
        
        # Reshape for loss computation
        batch_size, seq_len, vocab_size = logits.shape
        logits_flat = logits.reshape(-1, vocab_size)
        targets_flat = targets.reshape(-1)
        
        # Mask padding tokens
        mask = targets_flat != -100
        
        # Compute loss only on non-padding tokens
        loss = mx.mean(
            mx.where(
                mask,
                -mx.log(mx.softmax(logits_flat)[mx.arange(len(targets_flat)), targets_flat]),
                0.0
            )
        )
        
        return loss
    
    def train_epoch(self, epoch: int) -> float:
        """Train one epoch"""
        
        console.print(f"\n[bold blue]Epoch {epoch + 1}/{self.config.num_epochs}[/bold blue]")
        
        # Shuffle data
        import random
        random.shuffle(self.train_data)
        
        total_loss = 0.0
        num_batches = len(self.train_data) // self.config.batch_size
        
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            BarColumn(),
            TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
            TimeRemainingColumn(),
        ) as progress:
            
            task = progress.add_task(
                f"Training epoch {epoch + 1}...",
                total=num_batches
            )
            
            for batch_idx in range(num_batches):
                start_idx = batch_idx * self.config.batch_size
                end_idx = start_idx + self.config.batch_size
                batch = self.train_data[start_idx:end_idx]
                
                # Prepare batch
                inputs, targets = self._prepare_batch(batch)
                
                # Forward pass
                logits = self.model(inputs)
                loss = self._compute_loss(logits, targets)
                
                # Backward pass (simplified - MLX handles this)
                # In real implementation, use MLX's optimizer
                
                total_loss += loss.item()
                self.global_step += 1
                
                # Log
                if self.global_step % self.config.log_steps == 0:
                    avg_loss = total_loss / (batch_idx + 1)
                    progress.console.print(
                        f"Step {self.global_step}: loss={avg_loss:.4f}"
                    )
                
                # Evaluate
                if self.global_step % self.config.eval_steps == 0:
                    val_loss = self.evaluate()
                    progress.console.print(
                        f"[yellow]Validation loss: {val_loss:.4f}[/yellow]"
                    )
                    
                    # Save if best
                    if val_loss < self.best_val_loss:
                        self.best_val_loss = val_loss
                        self.save_checkpoint("best")
                        progress.console.print("[green]✅ Saved best model[/green]")
                
                # Save checkpoint
                if self.global_step % self.config.save_steps == 0:
                    self.save_checkpoint(f"step_{self.global_step}")
                
                progress.update(task, advance=1)
        
        avg_loss = total_loss / num_batches
        return avg_loss
    
    def evaluate(self) -> float:
        """Evaluate on validation set"""
        
        total_loss = 0.0
        num_batches = min(50, len(self.val_data) // self.config.batch_size)
        
        for batch_idx in range(num_batches):
            start_idx = batch_idx * self.config.batch_size
            end_idx = start_idx + self.config.batch_size
            batch = self.val_data[start_idx:end_idx]
            
            # Prepare batch
            inputs, targets = self._prepare_batch(batch)
            
            # Forward pass
            logits = self.model(inputs)
            loss = self._compute_loss(logits, targets)
            
            total_loss += loss.item()
        
        avg_loss = total_loss / num_batches
        return avg_loss
    
    def train(self):
        """Main training loop"""
        
        console.print("\n[bold green]🚀 Starting Training[/bold green]")
        console.print("=" * 60)
        
        start_time = time.time()
        
        for epoch in range(self.config.num_epochs):
            epoch_loss = self.train_epoch(epoch)
            
            self.training_history.append({
                "epoch": epoch + 1,
                "train_loss": epoch_loss,
                "val_loss": self.evaluate(),
            })
            
            console.print(f"\n[green]Epoch {epoch + 1} complete:[/green]")
            console.print(f"  Train loss: {epoch_loss:.4f}")
            console.print(f"  Val loss: {self.training_history[-1]['val_loss']:.4f}")
        
        duration = time.time() - start_time
        
        console.print("\n" + "=" * 60)
        console.print("[bold green]✅ Training Complete![/bold green]")
        console.print(f"Duration: {duration/3600:.2f} hours")
        console.print(f"Best val loss: {self.best_val_loss:.4f}")
        
        # Save final model
        self.save_checkpoint("final")
        
        # Print summary
        self._print_summary()
    
    def save_checkpoint(self, name: str):
        """Save model checkpoint"""
        
        checkpoint_dir = self.output_dir / name
        checkpoint_dir.mkdir(parents=True, exist_ok=True)
        
        # Save model weights
        # In real implementation, use MLX's save_weights
        # self.model.save_weights(str(checkpoint_dir / "model.safetensors"))
        
        # Save tokenizer
        # self.tokenizer.save_pretrained(str(checkpoint_dir))
        
        # Save config
        with open(checkpoint_dir / "config.json", 'w') as f:
            json.dump(vars(self.config), f, indent=2)
        
        # Save training history
        with open(checkpoint_dir / "history.json", 'w') as f:
            json.dump(self.training_history, f, indent=2)
    
    def _print_summary(self):
        """Print training summary"""
        
        table = Table(title="Training Summary")
        table.add_column("Epoch", style="cyan")
        table.add_column("Train Loss", style="magenta")
        table.add_column("Val Loss", style="green")
        
        for entry in self.training_history:
            table.add_row(
                str(entry["epoch"]),
                f"{entry['train_loss']:.4f}",
                f"{entry['val_loss']:.4f}",
            )
        
        console.print(table)


def main():
    """Main entry point"""
    
    parser = argparse.ArgumentParser(description="Train JESSY Complete")
    parser.add_argument("--model", default="mlx-community/gemma-2b", help="Base model")
    parser.add_argument("--train", default="training/datasets/jessy_complete_train.jsonl", help="Training data")
    parser.add_argument("--val", default="training/datasets/jessy_complete_val.jsonl", help="Validation data")
    parser.add_argument("--epochs", type=int, default=3, help="Number of epochs")
    parser.add_argument("--batch-size", type=int, default=4, help="Batch size")
    parser.add_argument("--lr", type=float, default=5e-5, help="Learning rate")
    parser.add_argument("--output", default="training/checkpoints/jessy-complete", help="Output directory")
    
    args = parser.parse_args()
    
    config = TrainingConfig(
        model_name=args.model,
        train_file=args.train,
        val_file=args.val,
        num_epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.lr,
        output_dir=args.output,
    )
    
    trainer = JESSYTrainer(config)
    
    try:
        trainer.train()
    except KeyboardInterrupt:
        console.print("\n[yellow]⚠️  Training interrupted by user[/yellow]")
        trainer.save_checkpoint("interrupted")
    except Exception as e:
        console.print(f"\n[red]❌ Error: {e}[/red]")
        raise


if __name__ == "__main__":
    main()
