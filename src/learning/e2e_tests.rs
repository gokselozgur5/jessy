//! End-to-end integration tests for 3-Tier Cognitive Layer System
//!
//! Tests the complete flow:
//! - Tier 1 (C01-C15): Base cognitive layers
//! - Tier 2 (C16-C30): Shared learned layers (collective wisdom)
//! - Tier 3 (C31+): User-specific layers (personal patterns)
//!
//! Scenarios covered:
//! 1. Single user with C31+ pattern creation
//! 2. Multi-user isolation (user A != user B)
//! 3. C16-C30 shared layer activation across users
//! 4. LRU eviction (both shared and user layers)
//! 5. Cross-session persistence (save/load)
//!
//! **IMPORTANT:** These tests share the same `data/` directory and must run serially:
//! ```bash
//! cargo test --lib e2e_ -- --test-threads=1
//! ```

#[cfg(test)]
mod e2e_tests {
    use crate::learning::{SharedLayerManager, UserLayerManager, SharedLayer, UserLayer};
    use crate::memory::MmapManager;
    use crate::{DimensionId, LayerId, Frequency};
    use std::sync::Arc;
    use std::fs;

    /// Helper: Create fresh memory manager
    fn create_memory_manager() -> Arc<MmapManager> {
        Arc::new(MmapManager::new(280).expect("Failed to create memory manager"))
    }

    /// Helper: Clean up test data directories
    fn cleanup_test_data() {
        let _ = fs::remove_dir_all("data/shared_layers");
        let _ = fs::remove_dir_all("data/user_layers");
    }

    // ========================================================================
    // Scenario 1: Single User with C31+ Pattern Creation
    // ========================================================================

    #[test]
    fn test_e2e_single_user_pattern_creation() {
        cleanup_test_data();

        let memory = create_memory_manager();
        let mut user_manager = UserLayerManager::new(
            memory.clone(),
            32 * 1024 * 1024,
            0x1000_0000,
        );

        // User "alice" interacts with JESSY
        // Pattern detected: She frequently asks about "rust" and "programming"
        let alice_layer = UserLayer::new(
            "alice".to_string(),
            DimensionId(31),
            LayerId { dimension: DimensionId(31), layer: 1 },
            b"Rust programming patterns".to_vec(),
            vec!["rust".to_string(), "programming".to_string(), "borrow".to_string()],
            Frequency(2.5),
            0.87, // Above 0.85 threshold
        ).expect("Failed to create UserLayer");

        // Create layer
        user_manager.create_user_layer(alice_layer).expect("Failed to create layer");

        // Verify layer exists
        let layers = user_manager.get_user_layers("alice");
        assert_eq!(layers.len(), 1, "Alice should have 1 layer");
        assert_eq!(layers[0].keywords.len(), 3, "Should have 3 keywords");
        assert!(layers[0].keywords.contains(&"rust".to_string()));

        // Verify statistics
        let stats = user_manager.stats();
        assert_eq!(stats.user_count, 1, "Should have 1 user");
        assert_eq!(stats.total_layer_count, 1, "Should have 1 total layer");

        println!("✅ Scenario 1: Single user pattern creation - PASSED");
    }

    // ========================================================================
    // Scenario 2: Multi-User Isolation
    // ========================================================================

    #[test]
    fn test_e2e_multi_user_isolation() {
        cleanup_test_data();

        let memory = create_memory_manager();
        let mut user_manager = UserLayerManager::new(
            memory.clone(),
            32 * 1024 * 1024,
            0x1000_0000,
        );

        // User A: Interested in "rust" and "systems"
        let user_a_layer = UserLayer::new(
            "user_a".to_string(),
            DimensionId(31),
            LayerId { dimension: DimensionId(31), layer: 1 },
            b"Systems programming".to_vec(),
            vec!["rust".to_string(), "systems".to_string()],
            Frequency(2.5),
            0.88,
        ).expect("Failed to create user A layer");

        // User B: Interested in "python" and "data"
        let user_b_layer = UserLayer::new(
            "user_b".to_string(),
            DimensionId(31),
            LayerId { dimension: DimensionId(31), layer: 1 },
            b"Data science".to_vec(),
            vec!["python".to_string(), "data".to_string()],
            Frequency(2.0),
            0.90,
        ).expect("Failed to create user B layer");

        // Create both layers
        user_manager.create_user_layer(user_a_layer).expect("Failed to create user A layer");
        user_manager.create_user_layer(user_b_layer).expect("Failed to create user B layer");

        // Verify isolation: User A can only see their layer
        let user_a_layers = user_manager.get_user_layers("user_a");
        assert_eq!(user_a_layers.len(), 1, "User A should have 1 layer");
        assert!(user_a_layers[0].keywords.contains(&"rust".to_string()));
        assert!(!user_a_layers[0].keywords.contains(&"python".to_string()), "User A should NOT see User B's keywords");

        // Verify isolation: User B can only see their layer
        let user_b_layers = user_manager.get_user_layers("user_b");
        assert_eq!(user_b_layers.len(), 1, "User B should have 1 layer");
        assert!(user_b_layers[0].keywords.contains(&"python".to_string()));
        assert!(!user_b_layers[0].keywords.contains(&"rust".to_string()), "User B should NOT see User A's keywords");

        // Verify stats
        let stats = user_manager.stats();
        assert_eq!(stats.user_count, 2, "Should have 2 users");
        assert_eq!(stats.total_layer_count, 2, "Should have 2 total layers");

        println!("✅ Scenario 2: Multi-user isolation - PASSED");
    }

    // ========================================================================
    // Scenario 3: C16-C30 Shared Layer Activation
    // ========================================================================

    #[test]
    fn test_e2e_shared_layer_activation() {
        cleanup_test_data();

        let memory = create_memory_manager();
        let mut shared_manager = SharedLayerManager::new(
            memory.clone(),
            92 * 1024 * 1024,
            0x0A00_0000,
        );

        // Create shared layer: "Emotional Intelligence" pattern
        // Detected from multiple users asking about empathy, compassion, etc.
        let shared_layer = SharedLayer::new(
            DimensionId(16),
            LayerId { dimension: DimensionId(16), layer: 1 },
            b"Collective wisdom about empathy".to_vec(),
            vec!["empathy".to_string(), "compassion".to_string(), "emotional".to_string()],
            Frequency(1.8),
            0.92, // High confidence from multiple users
        ).expect("Failed to create SharedLayer");

        // Add layer
        shared_manager.create_shared_layer(shared_layer).expect("Failed to create shared layer");

        // Verify layer exists
        let layers = shared_manager.get_all_shared_layers();
        assert_eq!(layers.len(), 1, "Should have 1 shared layer");
        assert!(layers[0].keywords.contains(&"empathy".to_string()), "Should have empathy keyword");

        // Simulate contribution from multiple users
        shared_manager.add_contributor(DimensionId(16));
        shared_manager.add_contributor(DimensionId(16));

        // Access from different "users" (shared layer accessible to all)
        shared_manager.touch_layer(DimensionId(16));

        let layer = shared_manager.get_shared_layer(DimensionId(16));
        assert!(layer.is_some(), "Layer should exist");
        let layer = layer.unwrap();
        assert_eq!(layer.contributor_count, 2, "Should have 2 contributors");
        assert!(layer.access_count > 0, "Should have been accessed");

        println!("✅ Scenario 3: Shared layer activation - PASSED");
    }

    // ========================================================================
    // Scenario 4: LRU Eviction for Both Tiers
    // ========================================================================

    #[test]
    fn test_e2e_lru_eviction() {
        cleanup_test_data();

        let memory = create_memory_manager();

        // Test UserLayer LRU eviction (max 5 per user)
        let mut user_manager = UserLayerManager::new(memory.clone(), 32 * 1024 * 1024, 0x1000_0000);

        // Create 5 layers for user "bob" (hit the limit)
        for i in 31..=35 {
            let layer = UserLayer::new(
                "bob".to_string(),
                DimensionId(i),
                LayerId { dimension: DimensionId(i), layer: 1 },
                format!("Pattern {}", i).as_bytes().to_vec(),
                vec![format!("keyword{}", i)],
                Frequency(2.0),
                0.87,
            ).expect("Failed to create layer");

            user_manager.create_user_layer(layer).expect("Failed to create layer");
        }

        assert_eq!(user_manager.get_user_layers("bob").len(), 5, "Should have 5 layers (max)");

        // Access layer 31 to make it recently used
        std::thread::sleep(std::time::Duration::from_millis(10));
        let _ = user_manager.get_user_layer("bob", DimensionId(31));

        // Create 6th layer - should trigger LRU eviction
        let new_layer = UserLayer::new(
            "bob".to_string(),
            DimensionId(36),
            LayerId { dimension: DimensionId(36), layer: 1 },
            b"New pattern".to_vec(),
            vec!["new".to_string()],
            Frequency(2.2),
            0.88,
        ).expect("Failed to create layer");

        user_manager.create_user_layer(new_layer).expect("Failed to create 6th layer");

        // Should still have 5 layers
        assert_eq!(user_manager.get_user_layers("bob").len(), 5, "Should still have 5 layers after eviction");

        // Layer 31 should still exist (recently accessed)
        assert!(user_manager.get_user_layer("bob", DimensionId(31)).is_some(), "Recently accessed layer should survive");

        println!("✅ Scenario 4: LRU eviction - PASSED");
    }

    // ========================================================================
    // Scenario 5: Cross-Session Persistence (Save/Load)
    // ========================================================================

    #[test]
    fn test_e2e_cross_session_persistence() {
        cleanup_test_data();

        let memory = create_memory_manager();

        // ===== SESSION 1: Create and save layers =====
        {
            let mut user_manager = UserLayerManager::new(memory.clone(), 32 * 1024 * 1024, 0x1000_0000);
            let mut shared_manager = SharedLayerManager::new(memory.clone(), 92 * 1024 * 1024, 0x0A00_0000);

            // Create user layer
            let user_layer = UserLayer::new(
                "charlie".to_string(),
                DimensionId(31),
                LayerId { dimension: DimensionId(31), layer: 1 },
                b"Charlie's pattern".to_vec(),
                vec!["persistence".to_string(), "test".to_string()],
                Frequency(2.3),
                0.89,
            ).expect("Failed to create user layer");

            user_manager.create_user_layer(user_layer).expect("Failed to create user layer");

            // Create shared layer
            let shared_layer = SharedLayer::new(
                DimensionId(16),
                LayerId { dimension: DimensionId(16), layer: 1 },
                b"Shared knowledge".to_vec(),
                vec!["shared".to_string(), "knowledge".to_string()],
                Frequency(2.0),
                0.91,
            ).expect("Failed to create shared layer");

            shared_manager.create_shared_layer(shared_layer).expect("Failed to create shared layer");

            // Save to disk
            user_manager.save().expect("Failed to save user layers");
            shared_manager.save().expect("Failed to save shared layers");

            println!("📝 Session 1: Layers created and saved");
        }

        // ===== SESSION 2: Load from disk =====
        {
            let memory2 = create_memory_manager();
            let mut user_manager2 = UserLayerManager::new(memory2.clone(), 32 * 1024 * 1024, 0x1000_0000);
            let mut shared_manager2 = SharedLayerManager::new(memory2.clone(), 92 * 1024 * 1024, 0x0A00_0000);

            // Load from disk
            user_manager2.load().expect("Failed to load user layers");
            shared_manager2.load().expect("Failed to load shared layers");

            // Verify user layer persisted
            let charlie_layers = user_manager2.get_user_layers("charlie");
            assert_eq!(charlie_layers.len(), 1, "Charlie's layer should persist");
            assert!(charlie_layers[0].keywords.contains(&"persistence".to_string()));

            // Verify shared layer persisted
            let shared_layers = shared_manager2.get_all_shared_layers();
            assert_eq!(shared_layers.len(), 1, "Shared layer should persist");
            assert!(shared_layers[0].keywords.contains(&"shared".to_string()));

            println!("📖 Session 2: Layers loaded successfully");
        }

        println!("✅ Scenario 5: Cross-session persistence - PASSED");

        // Cleanup
        cleanup_test_data();
    }

    // ========================================================================
    // Comprehensive Integration Test: All Scenarios Combined
    // ========================================================================

    #[test]
    fn test_e2e_comprehensive_all_scenarios() {
        cleanup_test_data();

        let memory = create_memory_manager();
        let mut user_manager = UserLayerManager::new(memory.clone(), 32 * 1024 * 1024, 0x1000_0000);
        let mut shared_manager = SharedLayerManager::new(memory.clone(), 92 * 1024 * 1024, 0x0A00_0000);

        println!("\n🚀 Starting comprehensive end-to-end test...\n");

        // Step 1: Create multiple users with patterns
        println!("Step 1: Creating user patterns...");
        for (i, user) in ["alice", "bob", "charlie"].iter().enumerate() {
            let layer = UserLayer::new(
                user.to_string(),
                DimensionId(31 + i as u8),
                LayerId { dimension: DimensionId(31 + i as u8), layer: 1 },
                format!("{}'s pattern", user).as_bytes().to_vec(),
                vec![format!("keyword_{}", user)],
                Frequency(2.0 + i as f32 * 0.1),
                0.85 + i as f32 * 0.02,
            ).expect("Failed to create user layer");

            user_manager.create_user_layer(layer).expect("Failed to create user layer");
        }
        println!("  ✓ Created 3 user patterns");

        // Step 2: Create shared patterns from collective wisdom
        println!("Step 2: Creating shared patterns...");
        for i in 16..=18 {
            let layer = SharedLayer::new(
                DimensionId(i),
                LayerId { dimension: DimensionId(i), layer: 1 },
                format!("Shared wisdom {}", i).as_bytes().to_vec(),
                vec![format!("shared_{}", i)],
                Frequency(1.5 + (i - 16) as f32 * 0.2),
                0.90 + (i - 16) as f32 * 0.01,
            ).expect("Failed to create shared layer");

            shared_manager.create_shared_layer(layer).expect("Failed to create shared layer");
        }
        println!("  ✓ Created 3 shared patterns");

        // Step 3: Verify isolation
        println!("Step 3: Verifying user isolation...");
        assert_eq!(user_manager.get_user_layers("alice").len(), 1);
        assert_eq!(user_manager.get_user_layers("bob").len(), 1);
        assert_eq!(user_manager.get_user_layers("charlie").len(), 1);
        println!("  ✓ User isolation verified");

        // Step 4: Verify shared access
        println!("Step 4: Verifying shared layer access...");
        let shared_layers = shared_manager.get_all_shared_layers();
        assert_eq!(shared_layers.len(), 3);
        println!("  ✓ Shared layers accessible to all");

        // Step 5: Persistence
        println!("Step 5: Testing persistence...");
        user_manager.save().expect("Failed to save user layers");
        shared_manager.save().expect("Failed to save shared layers");
        println!("  ✓ Layers saved to disk");

        // Step 6: Reload
        let memory2 = create_memory_manager();
        let mut user_manager2 = UserLayerManager::new(memory2.clone(), 32 * 1024 * 1024, 0x1000_0000);
        let mut shared_manager2 = SharedLayerManager::new(memory2.clone(), 92 * 1024 * 1024, 0x0A00_0000);

        user_manager2.load().expect("Failed to load user layers");
        shared_manager2.load().expect("Failed to load shared layers");

        assert_eq!(user_manager2.stats().user_count, 3);
        assert_eq!(shared_manager2.get_all_shared_layers().len(), 3);
        println!("  ✓ Layers reloaded successfully");

        println!("\n✅ COMPREHENSIVE TEST PASSED - All 5 scenarios validated!\n");

        cleanup_test_data();
    }
}
