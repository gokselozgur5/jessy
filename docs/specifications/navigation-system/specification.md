# Spec: Navigation System Implementation

## Status
- **Phase**: Design
- **Priority**: P0 (Critical Path)
- **Owner**: Core Team
- **Dependencies**: Memory Manager, Dimensions Module

## Problem Statement

The consciousness system needs to navigate through 14 dimensional layers in parallel, matching query keywords synesthetically, and selecting optimal paths with confidence scoring. Traditional sequential navigation would exceed the <100ms target.

### Why This Matters
- Query processing must complete in <5s total
- Dimension scanning is the first critical step
- Parallel scanning enables <100ms target
- Synesthetic matching improves accuracy over literal matching
- Path selection determines context quality

### Constraints
- Must scan all 14 dimensions in parallel
- Total scan time must be <100ms
- Synesthetic associations must be learned and decayed
- Maximum 8 dimensions can be activated simultaneously
- Complexity threshold of 6 dimensions triggers return-to-source
- Minimum confidence of 0.3 for dimension activation

### Success Criteria
- [ ] Parallel dimension scanning completes <100ms
- [ ] Synesthetic keyword matching works accurately
- [ ] Path selection chooses optimal layers
- [ ] Depth navigation reaches appropriate layers
- [ ] Return-to-source triggers at complexity threshold
- [ ] Confidence scoring reflects match quality

## Domain Model

### Entities

```rust
/// Main navigation coordinator
struct MultiverseNavigator {
    config: NavigationConfig,
    synesthetic_engine: SynestheticEngine,
    path_selector: PathSelector,
    dimension_registry: DimensionRegistry,
}

/// Synesthetic keyword matching engine
struct SynestheticEngine {
    associations: HashMap<String, Vec<SynestheticAssociation>>,
    learning_rate: f32,
    decay_rate: f32,
    last_decay: SystemTime,
}

/// Path selection and confidence scoring
struct PathSelector {
    min_confidence: f32,
    max_dimensions: usize,
    complexity_threshold: usize,
}

/// Synesthetic association between keywords
struct SynestheticAssociation {
    source_keyword: String,
    target_keyword: String,
    strength: f32,
    activation_count: usize,
    last_activated: SystemTime,
}

/// Query analysis result
struct QueryAnalysis {
    raw_query: String,
    keywords: Vec<String>,
    estimated_complexity: f32,
    emotional_indicators: Vec<String>,
    technical_indicators: Vec<String>,
    question_type: QuestionType,
    urgency_level: UrgencyLevel,
}

/// Navigation path through dimension
struct NavigationPath {
    dimension_id: DimensionId,
    layer_sequence: Vec<LayerId>,
    confidence: f32,
    frequency: Frequency,
    keywords_matched: Vec<String>,
    synesthetic_score: f32,
}
```

### State Machine

```
[Query Received]
    ↓ analyze
[Query Analyzed]
    ↓ parallel scan
[Dimensions Scanned]
    ↓ synesthetic match
[Keywords Matched]
    ↓ confidence score
[Paths Scored]
    ↓ select top paths
[Paths Selected]
    ↓ check complexity
[Complexity Check]
    ├─ >6 dimensions → [Return to Source]
    └─ ≤6 dimensions → [Navigation Complete]
```

### Invariants
1. All 14 dimensions scanned in parallel
2. Scan completes within 100ms timeout
3. Maximum 8 dimensions activated
4. Minimum 0.3 confidence for activation
5. Synesthetic associations decay over time
6. Return-to-source triggers at 6+ dimensions

## Architecture Design

### Component Boundaries

```
┌─────────────────────────────────────┐
│   MultiverseNavigator (Public)      │
├─────────────────────────────────────┤
│   - navigate()                      │
│   - analyze_query()                 │
│   - scan_dimensions()               │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   SynestheticEngine (Internal)      │
├─────────────────────────────────────┤
│   - match_keywords()                │
│   - strengthen_association()        │
│   - decay_unused()                  │
│   - get_associations()              │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   PathSelector (Internal)           │
├─────────────────────────────────────┤
│   - score_paths()                   │
│   - select_top_paths()              │
│   - check_complexity()              │
│   - navigate_depth()                │
└─────────────────────────────────────┘
```

### Interface Contracts

```rust
impl MultiverseNavigator {
    /// Navigate query through dimensional layers
    /// 
    /// # Performance
    /// Must complete within 100ms
    /// 
    /// # Returns
    /// NavigationResult with selected paths and dimensions
    /// 
    /// # Errors
    /// - NavigationError if scan timeout exceeded
    /// - NavigationError if no viable paths found
    pub fn navigate(&mut self, query: &str) -> Result<NavigationResult>;
    
    /// Analyze query to extract keywords and metadata
    pub fn analyze_query(&self, query: &str) -> QueryAnalysis;
    
    /// Scan all dimensions in parallel
    /// 
    /// # Performance
    /// Parallel execution across 14 dimensions
    /// Timeout: 100ms
    pub async fn scan_dimensions(
        &self,
        analysis: &QueryAnalysis,
    ) -> Result<Vec<DimensionActivation>>;
}

impl SynestheticEngine {
    /// Match keywords using synesthetic associations
    /// 
    /// # Returns
    /// List of matched keywords with confidence scores
    pub fn match_keywords(
        &self,
        query_keywords: &[String],
        layer_keywords: &[String],
    ) -> Vec<KeywordMatch>;
    
    /// Strengthen association between keywords
    pub fn strengthen_association(
        &mut self,
        keyword1: &str,
        keyword2: &str,
    );
    
    /// Decay unused associations (called periodically)
    pub fn decay_unused(&mut self);
}

impl PathSelector {
    /// Score navigation paths by confidence
    pub fn score_paths(
        &self,
        activations: Vec<DimensionActivation>,
    ) -> Vec<NavigationPath>;
    
    /// Select top paths within constraints
    pub fn select_top_paths(
        &self,
        paths: Vec<NavigationPath>,
    ) -> Vec<NavigationPath>;
    
    /// Navigate to appropriate depth in dimension
    pub fn navigate_depth(
        &self,
        dimension_id: DimensionId,
        keywords: &[String],
        max_depth: usize,
    ) -> Vec<LayerId>;
}
```

### Parallel Scanning Algorithm

```rust
async fn scan_dimensions_parallel(
    &self,
    analysis: &QueryAnalysis,
) -> Result<Vec<DimensionActivation>> {
    let dimensions = CoreDimension::all();
    
    // Create futures for parallel scanning
    let scan_futures: Vec<_> = dimensions
        .into_iter()
        .map(|dim| self.scan_single_dimension(dim, analysis))
        .collect();
    
    // Execute all scans in parallel with timeout
    let timeout = Duration::from_millis(100);
    let results = tokio::time::timeout(
        timeout,
        futures::future::join_all(scan_futures),
    ).await?;
    
    // Filter viable activations
    Ok(results
        .into_iter()
        .filter_map(|r| r.ok())
        .filter(|a| a.confidence >= self.config.min_confidence)
        .collect())
}
```

### Synesthetic Matching Algorithm

```rust
fn match_synesthetic(
    &self,
    query_keyword: &str,
    layer_keywords: &[String],
) -> Vec<(String, f32)> {
    let mut matches = Vec::new();
    
    // Direct literal match
    for layer_kw in layer_keywords {
        if query_keyword == layer_kw {
            matches.push((layer_kw.clone(), 1.0));
        }
    }
    
    // Synesthetic association match
    if let Some(associations) = self.associations.get(query_keyword) {
        for assoc in associations {
            if layer_keywords.contains(&assoc.target_keyword) {
                matches.push((
                    assoc.target_keyword.clone(),
                    assoc.strength * 0.8, // Discount for indirect match
                ));
            }
        }
    }
    
    matches
}
```

## Test Specification

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    #[test]
    fn test_query_analysis() {
        // Given: Navigator
        let navigator = MultiverseNavigator::new();
        
        // When: Analyze query
        let analysis = navigator.analyze_query(
            "I'm feeling overwhelmed and need help"
        );
        
        // Then: Correct analysis
        assert_eq!(analysis.question_type, QuestionType::Emotional);
        assert!(analysis.emotional_indicators.contains(&"overwhelmed".to_string()));
        assert_eq!(analysis.urgency_level, UrgencyLevel::High);
    }
    
    #[tokio::test]
    async fn test_parallel_dimension_scan() {
        // Given: Navigator with query
        let mut navigator = MultiverseNavigator::new();
        let analysis = QueryAnalysis {
            raw_query: "test".to_string(),
            keywords: vec!["test".to_string()],
            estimated_complexity: 1.0,
            emotional_indicators: vec![],
            technical_indicators: vec![],
            question_type: QuestionType::Factual,
            urgency_level: UrgencyLevel::Low,
        };
        
        // When: Scan dimensions
        let start = Instant::now();
        let activations = navigator.scan_dimensions(&analysis).await.unwrap();
        let elapsed = start.elapsed();
        
        // Then: Completes within 100ms
        assert!(elapsed < Duration::from_millis(100));
        assert!(!activations.is_empty());
    }
    
    #[test]
    fn test_synesthetic_matching() {
        // Given: Engine with associations
        let mut engine = SynestheticEngine::new();
        engine.strengthen_association("empathy", "compassion");
        engine.strengthen_association("empathy", "understanding");
        
        // When: Match keywords
        let matches = engine.match_keywords(
            &["empathy".to_string()],
            &["compassion".to_string(), "joy".to_string()],
        );
        
        // Then: Synesthetic match found
        assert!(!matches.is_empty());
        assert!(matches.iter().any(|m| m.keyword == "compassion"));
    }
    
    #[test]
    fn test_path_selection() {
        // Given: Multiple paths
        let selector = PathSelector::new();
        let paths = vec![
            create_path(DimensionId(1), 0.9),
            create_path(DimensionId(2), 0.7),
            create_path(DimensionId(3), 0.5),
            create_path(DimensionId(4), 0.2), // Below threshold
        ];
        
        // When: Select top paths
        let selected = selector.select_top_paths(paths);
        
        // Then: Only viable paths selected
        assert_eq!(selected.len(), 3);
        assert!(selected[0].confidence >= 0.3);
    }
    
    #[test]
    fn test_complexity_threshold() {
        // Given: 7 high-confidence paths
        let selector = PathSelector::new();
        let paths: Vec<_> = (0..7)
            .map(|i| create_path(DimensionId(i as u8), 0.9))
            .collect();
        
        // When: Check complexity
        let result = selector.check_complexity(&paths);
        
        // Then: Return-to-source triggered
        assert!(result.should_return_to_source);
    }
    
    #[test]
    fn test_depth_navigation() {
        // Given: Dimension with layers
        let selector = PathSelector::new();
        let keywords = vec!["empathy".to_string(), "compassion".to_string()];
        
        // When: Navigate depth
        let layers = selector.navigate_depth(
            DimensionId(1),
            &keywords,
            4, // max depth
        );
        
        // Then: Appropriate layers selected
        assert!(!layers.is_empty());
        assert!(layers.len() <= 4);
    }
}
```

### Integration Tests

```rust
#[tokio::test]
async fn test_full_navigation_flow() {
    // Given: Complete system
    let mut navigator = MultiverseNavigator::new();
    
    // When: Navigate complex query
    let result = navigator.navigate(
        "I'm feeling overwhelmed by technical complexity and need philosophical guidance"
    ).await.unwrap();
    
    // Then: Multiple dimensions activated
    assert!(result.dimensions.len() >= 3);
    assert!(result.dimensions.contains(&DimensionId(1))); // Emotion
    assert!(result.dimensions.contains(&DimensionId(6))); // Philosophical
    assert!(result.dimensions.contains(&DimensionId(7))); // Technical
    
    // And: Paths have good confidence
    assert!(result.paths.iter().all(|p| p.confidence >= 0.3));
}

#[tokio::test]
async fn test_return_to_source_trigger() {
    // Given: Query activating many dimensions
    let mut navigator = MultiverseNavigator::new();
    
    // When: Navigate overly complex query
    let result = navigator.navigate(
        "emotional technical philosophical creative ethical meta ecological query"
    ).await.unwrap();
    
    // Then: Return-to-source triggered
    assert!(result.return_to_source_triggered);
    assert!(result.dimensions.len() <= 3); // Simplified
}
```

### BDD Scenarios

```gherkin
Feature: Multiverse Navigation
  As a consciousness system
  I need to navigate dimensional layers efficiently
  So that queries are processed within performance targets

  Scenario: Parallel dimension scanning
    Given a query "I need help with coding"
    When the system scans all 14 dimensions
    Then scanning should complete within 100ms
    And at least 2 dimensions should activate
    And D07-Technical should be among activated dimensions

  Scenario: Synesthetic keyword matching
    Given synesthetic associations exist between "empathy" and "compassion"
    When a query contains "empathy"
    And a layer contains "compassion"
    Then the layer should match with synesthetic confidence
    And the match strength should reflect association strength

  Scenario: Path confidence scoring
    Given multiple dimension activations
    When paths are scored for confidence
    Then paths should be ranked by confidence
    And only paths above 0.3 confidence should be selected
    And maximum 8 dimensions should be activated

  Scenario: Depth navigation
    Given a dimension is activated
    When navigating to appropriate depth
    Then layers should be selected based on keyword matches
    And depth should not exceed max_depth configuration
    And higher-confidence layers should be prioritized

  Scenario: Return-to-source trigger
    Given a query activates 7 dimensions
    When complexity is checked
    Then return-to-source should be triggered
    And dimensions should be reduced to 2-3 core ones
    And a simplification message should be generated
```

## Implementation Plan

### Phase 1: Core Navigator (Day 1)
- [ ] Create `src/navigation/navigator.rs`
- [ ] Implement `MultiverseNavigator`
- [ ] Add query analysis
- [ ] Implement parallel scanning
- [ ] Write unit tests

### Phase 2: Synesthetic Engine (Day 2)
- [ ] Create `src/navigation/synesthetic.rs`
- [ ] Implement `SynestheticEngine`
- [ ] Add association management
- [ ] Implement decay logic
- [ ] Write unit tests

### Phase 3: Path Selector (Day 3)
- [ ] Create `src/navigation/path_selector.rs`
- [ ] Implement `PathSelector`
- [ ] Add confidence scoring
- [ ] Implement depth navigation
- [ ] Write unit tests

### Phase 4: Integration (Day 4)
- [ ] Integrate with dimension registry
- [ ] Add complexity checking
- [ ] Implement return-to-source
- [ ] Write integration tests
- [ ] Performance benchmarks

### Phase 5: Optimization (Day 5)
- [ ] Profile parallel scanning
- [ ] Optimize synesthetic lookup
- [ ] Tune confidence thresholds
- [ ] Validate <100ms target
- [ ] Documentation

## Success Metrics

- [ ] Dimension scanning <100ms (p95)
- [ ] Synesthetic matching accuracy >85%
- [ ] Path selection precision >90%
- [ ] Return-to-source triggers correctly
- [ ] No performance degradation with load

---

*"Navigate the multiverse in parallel. Find the resonant paths."*
