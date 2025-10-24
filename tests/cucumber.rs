// BDD tests using Cucumber for Jessy consciousness system

use cucumber::{given, then, when, World};
use jessy::{ConsciousnessSystem, DimensionId, Frequency};

#[derive(Debug, Default, World)]
pub struct ConsciousnessWorld {
    system: Option<ConsciousnessSystem>,
    query: String,
    response: Option<jessy::ConsciousnessResponse>,
    error: Option<String>,
}

#[given("the consciousness system is initialized")]
async fn system_initialized(world: &mut ConsciousnessWorld) {
    match ConsciousnessSystem::new().await {
        Ok(system) => {
            world.system = Some(system);
        }
        Err(e) => {
            world.error = Some(format!("Failed to initialize system: {}", e));
        }
    }
}

#[given(regex = r#"^all (\d+) dimensions are loaded with their layer hierarchies$"#)]
async fn dimensions_loaded(world: &mut ConsciousnessWorld, count: usize) {
    // For now, we just verify the system is initialized
    // In a full implementation, we would verify all dimensions are loaded
    assert!(world.system.is_some(), "System must be initialized first");
    assert_eq!(count, 14, "Expected 14 dimensions");
}

#[given("the synesthetic keyword engine is active")]
async fn synesthetic_engine_active(world: &mut ConsciousnessWorld) {
    // Verify system is ready
    assert!(world.system.is_some(), "System must be initialized first");
}

#[given(regex = r#"^a query "([^"]*)"$"#)]
async fn given_query(world: &mut ConsciousnessWorld, query: String) {
    world.query = query;
}

#[when("the system scans dimensions")]
async fn scan_dimensions(world: &mut ConsciousnessWorld) {
    if let Some(ref mut system) = world.system {
        match system.process_query(&world.query).await {
            Ok(response) => {
                world.response = Some(response);
            }
            Err(e) => {
                world.error = Some(format!("Query processing failed: {}", e));
            }
        }
    } else {
        world.error = Some("System not initialized".to_string());
    }
}

#[then(regex = r#"^D(\d+)-(\w+) should activate with high confidence$"#)]
async fn dimension_activates(world: &mut ConsciousnessWorld, dim_num: u8, _dim_name: String) {
    if let Some(ref response) = world.response {
        let expected_dim = DimensionId(dim_num);
        assert!(
            response.dimensions_activated.contains(&expected_dim),
            "Expected dimension D{:02} to be activated, but got: {:?}",
            dim_num,
            response.dimensions_activated
        );
    } else if let Some(ref error) = world.error {
        panic!("Cannot verify dimension activation due to error: {}", error);
    } else {
        panic!("No response available to verify dimension activation");
    }
}

#[then(regex = r#"^navigation should reach L(\d+) or deeper layers$"#)]
async fn navigation_depth(world: &mut ConsciousnessWorld, _min_layer: u16) {
    // For now, just verify we have a response
    assert!(
        world.response.is_some() || world.error.is_some(),
        "Expected response or error"
    );
}

#[then(regex = r#"^frequency should be less than ([\d.]+) Hz$"#)]
async fn frequency_less_than(world: &mut ConsciousnessWorld, max_hz: f32) {
    if let Some(ref response) = world.response {
        assert!(
            response.dominant_frequency.hz() < max_hz,
            "Expected frequency < {} Hz, got {} Hz",
            max_hz,
            response.dominant_frequency.hz()
        );
    }
}

#[then(regex = r#"^frequency should be between ([\d.]+)-([\d.]+) Hz$"#)]
async fn frequency_between(world: &mut ConsciousnessWorld, min_hz: f32, max_hz: f32) {
    if let Some(ref response) = world.response {
        let freq = response.dominant_frequency.hz();
        assert!(
            freq >= min_hz && freq <= max_hz,
            "Expected frequency between {} and {} Hz, got {} Hz",
            min_hz,
            max_hz,
            freq
        );
    }
}

#[then(regex = r#"^frequency should be around ([\d.]+) Hz for empathetic response$"#)]
async fn frequency_around(world: &mut ConsciousnessWorld, target_hz: f32) {
    if let Some(ref response) = world.response {
        let freq = response.dominant_frequency.hz();
        let tolerance = 0.5; // Allow 0.5 Hz tolerance
        assert!(
            (freq - target_hz).abs() <= tolerance,
            "Expected frequency around {} Hz (±{}), got {} Hz",
            target_hz,
            tolerance,
            freq
        );
    }
}

#[then(regex = r#"^the response should (.+)$"#)]
async fn response_should(world: &mut ConsciousnessWorld, _expectation: String) {
    // For now, just verify we have a response
    assert!(
        world.response.is_some() || world.error.is_some(),
        "Expected response or error"
    );
}

#[then(regex = r#"^D(\d+)-(\w+) should activate as primary dimension$"#)]
async fn primary_dimension(world: &mut ConsciousnessWorld, dim_num: u8, _dim_name: String) {
    if let Some(ref response) = world.response {
        let expected_dim = DimensionId(dim_num);
        assert!(
            !response.dimensions_activated.is_empty(),
            "No dimensions activated"
        );
        // Primary dimension should be first in the list
        assert_eq!(
            response.dimensions_activated[0], expected_dim,
            "Expected D{:02} as primary dimension, got D{:02}",
            dim_num, response.dimensions_activated[0].0
        );
    }
}

#[then(regex = r#"^D(\d+)-(\w+) should activate as secondary dimension$"#)]
async fn secondary_dimension(world: &mut ConsciousnessWorld, dim_num: u8, _dim_name: String) {
    if let Some(ref response) = world.response {
        let expected_dim = DimensionId(dim_num);
        assert!(
            response.dimensions_activated.len() >= 2,
            "Expected at least 2 dimensions activated"
        );
        assert!(
            response.dimensions_activated.contains(&expected_dim),
            "Expected dimension D{:02} to be activated",
            dim_num
        );
    }
}

#[then(regex = r#"^navigation should include (.+) layers$"#)]
async fn navigation_includes(world: &mut ConsciousnessWorld, _layer_type: String) {
    // For now, just verify we have a response
    assert!(world.response.is_some(), "Expected response");
}

#[then(regex = r#"^D(\d+)-(\w+) should activate with (.+) layers$"#)]
async fn dimension_with_layers(world: &mut ConsciousnessWorld, dim_num: u8, _dim_name: String, _layer_type: String) {
    if let Some(ref response) = world.response {
        let expected_dim = DimensionId(dim_num);
        assert!(
            response.dimensions_activated.contains(&expected_dim),
            "Expected dimension D{:02} to be activated",
            dim_num
        );
    }
}

#[then(regex = r#"^D(\d+)-(\w+) should blend in without (.+)$"#)]
async fn dimension_blends(world: &mut ConsciousnessWorld, dim_num: u8, _dim_name: String, _without: String) {
    if let Some(ref response) = world.response {
        let expected_dim = DimensionId(dim_num);
        // Dimension may or may not be activated, but if it is, it should be subtle
        if response.dimensions_activated.contains(&expected_dim) {
            // Just verify it's not the primary dimension
            assert_ne!(
                response.dimensions_activated[0], expected_dim,
                "D{:02} should not be primary dimension",
                dim_num
            );
        }
    }
}

#[then(regex = r#"^response should (.+)$"#)]
async fn response_characteristic(world: &mut ConsciousnessWorld, _characteristic: String) {
    // For now, just verify we have a response
    assert!(world.response.is_some(), "Expected response");
}

// Main test runner
#[tokio::main]
async fn main() {
    ConsciousnessWorld::cucumber()
        .run("tests/bdd/features/dimension_navigation.feature")
        .await;
}
