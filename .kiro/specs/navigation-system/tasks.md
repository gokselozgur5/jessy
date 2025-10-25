# Navigation System Implementation Plan

## Overview

This implementation plan breaks down the Navigation System MVP into discrete, actionable coding tasks. Each task builds incrementally on previous work, with all code integrated and tested before moving forward. The plan focuses exclusively on Phase 1 (MVP) features, deferring advanced capabilities to future phases.

## Task Structure

- Top-level tasks represent major components or features
- Sub-tasks are specific coding activities
- Tasks marked with `*` are optional (testing, documentation)
- All tasks reference specific requirements from requirements.md

## ⚠️ IMPORTANT: Git Commit Protocol

**AFTER COMPLETING EACH SMALL CHANGE:**
1. Stage changes: `git add <files>`
2. Commit with descriptive message following format:
   ```
   <type>(navigation): <description> (task X.Y)
   
   - Bullet point details
   - What was implemented/tested
   
   Requirements: X.Y-X.Z
   Task: X.Y (RED/GREEN phase)
   ```
3. Push to remote: `git push origin main`

**Commit Types:**
- `test`: Writing tests (RED phase)
- `feat`: Implementing features (GREEN phase)
- `refactor`: Code cleanup
- `docs`: Documentation updates

**Example:**
```
test(navigation): add frequency alignment tests (task 5.4)

- Test in range: query within dimension frequency range returns 1.0
- Test near range: query within 0.5Hz of range returns 0.5
- Test far outside: query >0.5Hz from range returns 0.0

Requirements: 4.5-4.7, 15.1-15.8
Task: 5.4 (RED phase)
```

---

## Implementation Tasks

### Test-Driven Development (TDD) Approach

Starting from Task 3, this plan follows strict Test-Driven Development methodology:

**Red-Green-Refactor Cycle:**
1. **RED**: Write failing tests first that define expected behavior
2. **GREEN**: Write minimal code to make tests pass
3. **REFACTOR**: Clean up code while keeping tests green

**Benefits:**
- Tests define requirements before implementation
- Prevents over-engineering (only code what's needed)
- Built-in regression protection
- Better API design through test-first thinking
- Higher confidence in correctness

**Task Structure:**
- Each feature broken into test/implement pairs
- Tests marked with **(RED)** - write these first, they should fail
- Implementation marked with **(GREEN)** - make tests pass
- Refactoring happens within GREEN phase

---

- [x] 1. Set up project structure and core types
  - Create `src/navigation/` module directory
  - Define core type aliases: `DimensionId`, `LayerId`, `ProtoDimensionId`
  - Define enums: `QuestionType`, `UrgencyLevel`, `SystemState`
  - Define error types: `NavigationError` with all variants
  - Create `NavigationConfig` struct with default values
  - _Requirements: 11.2, 12.1-12.5_

- [x] 2. Implement dimension registry
  - [x] 2.1 Create dimension metadata structures
    - Define `DimensionMetadata` struct with id, name, frequency_range, size_bytes, root_layer
    - Define `LayerMetadata` struct with id, dimension_id, depth, parent, keywords, frequency, mmap_offset
    - Implement `DimensionRegistry` with HashMap storage for O(1) lookups
    - _Requirements: 11.1, 11.5_
  
  - [x] 2.2 Implement registry initialization and validation
    - Write `load_dimensions()` to load 14 core dimensions from configuration
    - Write `validate()` to check unique IDs (1-14), frequency ranges (0.1-4.5 Hz), layer hierarchy
    - Implement `get_dimension()` and `get_layer()` lookup methods
    - Implement `get_child_layers()` for depth navigation support
    - _Requirements: 11.1-11.4, 11.11-11.12_
  
  - [x] 2.3 Create dimension configuration data
    - Create `dimensions.json` with 14 core dimensions and their metadata
    - Define layer hierarchies for each dimension (L0→L1→L2→L3)
    - Assign keywords to each layer for matching
    - Assign frequency values to each layer
    - _Requirements: 11.6-11.10, 16.1_
  
  - [x] 2.4 Write unit tests for dimension registry
    - Test dimension lookup performance (<1μs)
    - Test layer lookup and child layer queries
    - Test validation logic for IDs, frequencies, hierarchy
    - Test error handling for invalid configurations
    - _Requirements: 11.5, 11.11-11.12_

- [x] 3. Implement query analyzer (TDD approach)
  - [x] 3.1 Create vocabulary files and loading infrastructure
    - Create `data/emotional.txt` with 100+ emotional indicator words
    - Create `data/technical.txt` with 100+ technical indicator words
    - Create `data/stopwords.txt` with common words to filter
    - Create basic `QueryAnalyzer` struct skeleton with vocabulary fields
    - _Requirements: 14.4-14.6_
  
  - [x] 3.2 Write tests for vocabulary loading (RED)
    - Test `load_vocabulary()` reads file and returns HashSet
    - Test vocabulary validation (100+ words for emotional/technical)
    - Test error handling for missing files
    - Test error handling for empty files
    - _Requirements: 14.4-14.6, 14.11-14.12_
  
  - [x] 3.3 Implement vocabulary loading to pass tests (GREEN)
    - Write `load_vocabulary()` function to read word lists from files
    - Implement validation logic for minimum word counts
    - Implement error handling with descriptive messages
    - Refactor for clarity and performance
    - _Requirements: 14.4-14.6, 14.11-14.12_
  
  - [x] 3.4 Write tests for keyword extraction (RED)
    - Test basic tokenization: "hello world" → ["hello", "world"]
    - Test punctuation handling: "hello, world!" → ["hello", "world"]
    - Test lowercase normalization: "Hello WORLD" → ["hello", "world"]
    - Test stopword filtering: "the cat" → ["cat"] (if "the" is stopword)
    - Test max 50 keywords limit with long queries
    - Test empty query returns empty list
    - Test performance: extraction completes in <5ms
    - _Requirements: 1.1-1.5_
  
  - [x] 3.5 Implement keyword extraction to pass tests (GREEN)
    - Write `extract_keywords()` with basic whitespace tokenization
    - Add punctuation stripping using regex or char filtering
    - Add lowercase normalization
    - Add stopword filtering using HashSet lookup
    - Add max 50 keywords enforcement
    - Refactor for performance and clarity
    - _Requirements: 1.1-1.5_
  
  - [x] 3.6 Write tests for indicator classification (RED)
    - Test emotional indicator detection: ["anxious", "happy"] → emotional_count=2
    - Test technical indicator detection: ["algorithm", "database"] → technical_count=2
    - Test mixed indicators: ["anxious", "algorithm"] → emotional=1, technical=1
    - Test no indicators: ["random", "words"] → emotional=0, technical=0
    - Test case insensitivity
    - _Requirements: 1.6-1.7_
  
  - [x] 3.7 Implement indicator classification to pass tests (GREEN)
    - Write `classify_indicators()` using HashSet lookups
    - Return counts or lists of emotional and technical indicators
    - Ensure case-insensitive matching
    - Refactor for clarity
    - _Requirements: 1.6-1.7_
  
  - [x] 3.8 Write tests for question type classification (RED)
    - Test Emotional: 60% emotional indicators → QuestionType::Emotional
    - Test Technical: 60% technical indicators → QuestionType::Technical
    - Test Philosophical: contains "meaning", "purpose" → QuestionType::Philosophical
    - Test Factual: contains "what", "when", "where" → QuestionType::Factual
    - Test Mixed: 50% emotional, 50% technical → QuestionType::Mixed
    - Test edge cases: no indicators, all stopwords
    - _Requirements: 1.8-1.12_
  
  - [x] 3.9 Implement question type classification to pass tests (GREEN)
    - Write `classify_question_type()` with percentage logic
    - Implement philosophical keyword detection
    - Implement interrogative word detection
    - Handle edge cases gracefully
    - Refactor for clarity
    - _Requirements: 1.8-1.12_
  
  - [x] 3.10 Write tests for urgency detection (RED)
    - Test High urgency: "urgent help needed" → UrgencyLevel::High
    - Test Low urgency: "casual question" → UrgencyLevel::Low
    - Test Medium urgency: "need help" → UrgencyLevel::Medium
    - Test multiple urgency keywords
    - Test case insensitivity
    - _Requirements: 1.13-1.15_
  
  - [x] 3.11 Implement urgency detection to pass tests (GREEN)
    - Write `assign_urgency()` with keyword matching
    - Define urgency keyword lists (high: "urgent", "emergency", "critical", "immediately")
    - Implement case-insensitive matching
    - Refactor for clarity
    - _Requirements: 1.13-1.15_
  
  - [x] 3.12 Write tests for frequency estimation (RED)
    - Test base frequencies: Low=0.5Hz, Medium=2.0Hz, High=3.5Hz
    - Test Philosophical adjustment: -0.5Hz
    - Test Technical adjustment: +0.5Hz
    - Test high-intensity emotional adjustment: +1.0Hz
    - Test clamping to 0.1-4.5Hz range
    - Test combined adjustments
    - _Requirements: 15.1-15.8_
  
  - [x] 3.13 Implement frequency estimation to pass tests (GREEN)
    - Write `estimate_frequency()` with base frequency mapping
    - Add question type adjustments
    - Add emotional intensity detection and adjustment
    - Add clamping logic
    - Refactor for clarity
    - _Requirements: 15.1-15.8_
  
  - [x] 3.14 Write tests for complexity scoring (RED)
    - Test low complexity: few keywords, single indicator type → score ~1.0
    - Test medium complexity: moderate keywords, mixed indicators → score ~3.0
    - Test high complexity: many keywords, diverse indicators → score ~5.0
    - Test score range: always between 0.0 and 5.0
    - _Requirements: 1.17_
  
  - [x] 3.15 Implement complexity scoring to pass tests (GREEN)
    - Write `estimate_complexity()` based on keyword count and indicator diversity
    - Implement scoring formula
    - Clamp to 0.0-5.0 range
    - Refactor for clarity
    - _Requirements: 1.17_
  
  - [x] 3.16 Write tests for full query analysis (RED)
    - Test complete analysis flow: query → QueryAnalysisResult
    - Test all fields populated correctly
    - Test integration of all sub-components
    - Test end-to-end performance: <5ms total
    - Test various query types: emotional, technical, philosophical, factual, mixed
    - _Requirements: 1.16_
  
  - [x] 3.17 Implement full query analyzer to pass tests (GREEN)
    - Define `QueryAnalysisResult` struct with all fields
    - Implement `analyze()` method orchestrating all steps
    - Wire up all sub-components
    - Ensure proper error handling
    - Refactor for clarity and performance
    - _Requirements: 1.16_

- [x] 4. Implement parallel dimension scanner (TDD approach)
  - [x] 4.1 Create dimension activation structures
    - Define `DimensionActivation` struct with dimension_id, confidence, matched_keywords, scan_duration_ms
    - Create basic `ParallelScanner` struct skeleton
    - _Requirements: 2.4-2.6_
  
  - [x] 4.2 Write tests for keyword matching (RED)
    - Test exact match: query=["cat", "dog"], layer=["cat", "bird"] → matched=["cat"]
    - Test no match: query=["cat"], layer=["dog"] → matched=[]
    - Test all match: query=["cat", "dog"], layer=["cat", "dog"] → matched=["cat", "dog"]
    - Test case sensitivity
    - Test empty inputs
    - _Requirements: 2.4_
  
  - [x] 4.3 Implement keyword matching to pass tests (GREEN)
    - Write `match_keywords()` using HashSet intersection or iteration
    - Return Vec of matched keywords
    - Handle edge cases
    - Refactor for performance
    - _Requirements: 2.4_
  
  - [x] 4.4 Write tests for single dimension scan (RED)
    - Test successful scan returns DimensionActivation
    - Test confidence calculation: matched/total ratio
    - Test matched_keywords list is correct
    - Test scan_duration_ms is populated
    - Test with mock registry and dimension
    - _Requirements: 2.4_
  
  - [x] 4.5 Implement single dimension scanner to pass tests (GREEN)
    - Write `scan_dimension()` async function
    - Load dimension metadata from registry
    - Match query keywords against root layer keywords
    - Calculate confidence score
    - Track scan duration
    - Return DimensionActivation
    - Refactor for clarity
    - _Requirements: 2.4_
  
  - [x] 4.6 Write tests for parallel scanning (RED)
    - Test all 14 dimensions scanned concurrently
    - Test results collected correctly
    - Test confidence threshold filtering (>= 0.3)
    - Test scan completes within reasonable time
    - Test with various query types
    - _Requirements: 2.1-2.3, 2.5_
  
  - [x] 4.7 Implement parallel scanning to pass tests (GREEN)
    - Write `scan_all()` async function using Tokio
    - Spawn 14 concurrent tasks with tokio::spawn
    - Collect results using join_all or similar
    - Filter by confidence threshold
    - Refactor for clarity and performance
    - _Requirements: 2.1-2.3, 2.5_
  
  - [x] 4.8 Write tests for timeout handling (RED)
    - Test timeout triggers after 100ms
    - Test partial results returned on timeout
    - Test completed scans included in results
    - Test timeout error logged appropriately
    - Mock slow dimension scans to trigger timeout
    - _Requirements: 2.2-2.3_
  
  - [x] 4.9 Implement timeout handling to pass tests (GREEN)
    - Add tokio::time::timeout wrapper to scan_all
    - Handle timeout by returning partial results
    - Log timeout events
    - Refactor for clarity
    - _Requirements: 2.2-2.3_
  
  - [x] 4.10 Write integration tests for parallel scanner (RED)
    - Test with real dimension registry (14 dimensions)
    - Test concurrent execution (verify parallelism)
    - Test no data races using race detection tools
    - Test performance under load
    - _Requirements: 2.1-2.6, 8.1-8.2_
  
  - [x] 4.11 Finalize ParallelScanner implementation (GREEN)
    - Wire up all methods in ParallelScanner struct
    - Ensure thread safety (Arc<DimensionRegistry>)
    - Add comprehensive error handling
    - Refactor for final clarity and performance
    - _Requirements: 2.1-2.6_

- [x] 5. Implement path selector and confidence scoring (TDD approach)
  - [x] 5.1 Create PathSelector struct skeleton
    - Define `PathSelector` struct with config field
    - Define `NavigationPath` struct (if not already in types.rs)
    - _Requirements: 4.1-4.10_
  
  - [x] 5.2 Write tests for keyword match score (RED)
    - Test perfect match: 5 matched / 5 total → 1.0
    - Test partial match: 3 matched / 10 total → 0.3
    - Test no match: 0 matched / 5 total → 0.0
    - Test edge case: 0 total keywords → 0.0
    - _Requirements: 4.4_
  
  - [x] 5.3 Implement keyword match score to pass tests (GREEN)
    - Write `calculate_keyword_match_score()` as ratio
    - Handle division by zero
    - Refactor for clarity
    - _Requirements: 4.4_
  
  - [x] 5.4 Write tests for frequency alignment (RED)
    - Test in range: query=2.0Hz, dim=(1.5-2.5Hz) → 1.0
    - Test near range: query=1.0Hz, dim=(1.3-2.0Hz) → 0.5 (within 0.5Hz)
    - Test far outside: query=1.0Hz, dim=(2.0-3.0Hz) → 0.0
    - Test boundary cases
    - _Requirements: 4.5-4.7, 15.1-15.8_
  
  - [x] 5.5 Implement frequency alignment to pass tests (GREEN)
    - Write `calculate_frequency_alignment()` with range checks
    - Return 1.0 if in range, 0.5 if within 0.5Hz, 0.0 otherwise
    - Refactor for clarity
    - _Requirements: 4.5-4.7, 15.1-15.8_
  
  - [x] 5.6 Write tests for overall confidence calculation (RED)
    - Test weighted average: 70% keyword + 30% frequency
    - Test clamping to [0.0, 1.0]
    - Test various combinations
    - Example: keyword=0.8, freq=1.0 → confidence=0.86
    - _Requirements: 4.8_
  
  - [x] 5.7 Implement confidence calculation to pass tests (GREEN)
    - Write `calculate_confidence()` with weighted formula
    - Apply clamping
    - Refactor for clarity
    - _Requirements: 4.8_
  
  - [x] 5.8 Write tests for path ranking (RED)
    - Test sort by confidence descending
    - Test tiebreaker: higher keyword match count
    - Test second tiebreaker: lower dimension ID
    - Test with multiple paths having same confidence
    - _Requirements: 4.1, 4.9-4.10_
  
  - [x] 5.9 Implement path ranking to pass tests (GREEN)
    - Write `rank_paths()` with custom comparator
    - Implement multi-level sorting
    - Refactor for clarity
    - _Requirements: 4.1, 4.9-4.10_
  
  - [x] 5.10 Write tests for path selection (RED)
    - Test confidence threshold filtering (>= 0.3)
    - Test max 8 dimensions limit
    - Test ranking applied before limiting
    - Test empty activations returns empty paths
    - Test all activations below threshold returns empty paths
    - _Requirements: 4.2-4.3_
  
  - [x] 5.11 Implement path selection to pass tests (GREEN)
    - Write `select_paths()` with filtering and limiting
    - Apply ranking before limiting
    - Create NavigationPath for each selected dimension
    - Refactor for clarity
    - _Requirements: 4.2-4.3_
  
  - [x] 5.12 Write integration tests for PathSelector (RED)
    - Test full selection flow with realistic activations
    - Test with various query frequencies
    - Test edge cases: no activations, all low confidence, many high confidence
    - _Requirements: 4.1-4.10_
  
  - [x] 5.13 Finalize PathSelector implementation (GREEN)
    - Wire up all methods in PathSelector struct
    - Ensure proper error handling
    - Refactor for final clarity
    - _Requirements: 4.1-4.10_

- [x] 6. Implement depth navigator (TDD approach)
  - [x] 6.1 Create DepthNavigator struct skeleton
    - Define `DepthNavigator` struct with registry reference
    - _Requirements: 5.1-5.7_
  
  - [x] 6.2 Write tests for layer match calculation (RED)
    - Test perfect match: all layer keywords in query → 1.0
    - Test partial match: some layer keywords in query → 0.5
    - Test no match: no layer keywords in query → 0.0
    - Test empty layer keywords → 0.0
    - _Requirements: 5.2_
  
  - [x] 6.3 Implement layer match calculation to pass tests (GREEN)
    - Write `calculate_layer_match()` as ratio of matched to total query keywords
    - Handle edge cases
    - Refactor for clarity
    - _Requirements: 5.2_
  
  - [x] 6.4 Write tests for depth navigation (RED)
    - Test root layer always included in path
    - Test navigation stops when no children above threshold (0.1)
    - Test navigation selects highest scoring child
    - Test lexicographic tiebreaker when scores equal
    - Test max depth limit of 4 layers
    - Test path accumulation: [L0, L1, L2, ...]
    - Test with mock dimension hierarchy
    - _Requirements: 5.1, 5.3-5.7_
  
  - [x] 6.5 Implement depth navigation to pass tests (GREEN)
    - Write `navigate_depth()` starting from root layer
    - Get child layers from registry
    - Calculate match scores for all children
    - Select best child above 0.1 threshold
    - Apply lexicographic tiebreaker
    - Enforce max depth of 4
    - Build and return layer sequence
    - Refactor for clarity
    - _Requirements: 5.1, 5.3-5.7_
  
  - [x] 6.6 Write integration tests for depth navigator (RED)
    - Test with real dimension registry
    - Test various query types navigate to appropriate depths
    - Test edge cases: no children, all children below threshold
    - Test performance: navigation completes quickly
    - _Requirements: 5.1-5.7_
  
  - [x] 6.7 Finalize DepthNavigator implementation (GREEN)
    - Wire up all methods in DepthNavigator struct
    - Ensure proper error handling
    - Refactor for final clarity
    - _Requirements: 5.1-5.7_

- [x] 7. Implement navigation orchestrator (TDD approach)
  - [x] 7.1 Create navigation result structures
    - Define `NavigationPath` struct with dimension_id, confidence, layer_sequence, matched_keywords (if not in types.rs)
    - Define `NavigationResult` struct with paths, return_to_source, simplification_message, total_duration_ms, scanned_dimensions
    - Create `NavigationSystem` struct skeleton
    - _Requirements: 7.1-7.7_
  
  - [x] 7.2 Write tests for complexity management (RED)
    - Test no return-to-source when <= 6 dimensions
    - Test return-to-source triggered when > 6 dimensions
    - Test reduction to top 3 highest confidence dimensions
    - Test return_to_source flag set correctly
    - Test simplification_message populated
    - _Requirements: 6.1-6.5_
  
  - [x] 7.3 Implement complexity management to pass tests (GREEN)
    - Write `check_complexity()` to count dimensions
    - Implement return-to-source logic
    - Reduce to top 3 by confidence
    - Set flags and messages
    - Refactor for clarity
    - _Requirements: 6.1-6.5_
  
  - [x] 7.4 Write tests for navigation orchestration (RED)
    - Test simple emotional query: analyze → scan → select → navigate → result
    - Test simple technical query: routes to technical dimension
    - Test query with no matches: returns appropriate error or empty result
    - Test duration tracking: total_duration_ms populated
    - Test scanned_dimensions count correct
    - _Requirements: 7.1-7.7_
  
  - [x] 7.5 Implement navigation orchestration to pass tests (GREEN)
    - Write `navigate()` async method
    - Step 1: Analyze query using QueryAnalyzer
    - Step 2: Scan dimensions using ParallelScanner
    - Step 3: Select paths using PathSelector
    - Step 4: Check complexity and apply return-to-source
    - Step 5: Navigate depth for each selected dimension
    - Step 6: Assemble NavigationResult
    - Track total duration
    - Refactor for clarity
    - _Requirements: 7.1-7.7_
  
  - [x] 7.6 Write integration tests for complex scenarios (RED)
    - Test mixed query activating multiple dimensions
    - Test return-to-source with 10+ dimension activations
    - Test philosophical query routing
    - Test factual query routing
    - Test high urgency query with frequency adjustment
    - Test end-to-end performance: <150ms p95
    - _Requirements: 6.1-6.5, 7.1-7.7_
  
  - [x] 7.7 Finalize NavigationSystem implementation (GREEN)
    - Wire up all components in NavigationSystem struct
    - Ensure proper error propagation
    - Add comprehensive logging
    - Refactor for final clarity and performance
    - _Requirements: 7.1-7.7_

- [x] 8. Implement error handling and validation (TDD approach)
  - [x] 8.1 Write tests for query validation (RED)
    - Test empty query returns EmptyQuery error
    - Test query exceeding 10,000 characters returns QueryTooLong error
    - Test query with null bytes returns InvalidQuery error
    - Test valid query passes validation
    - _Requirements: 9.6-9.7_
  
  - [x] 8.2 Implement query validation to pass tests (GREEN)
    - Write `validate_query()` with all checks
    - Return appropriate NavigationError variants
    - Refactor for clarity
    - _Requirements: 9.6-9.7_
  
  - [x] 8.3 Write tests for graceful error handling (RED)
    - Test scan timeout returns partial results (not error)
    - Test single dimension failure doesn't fail entire scan
    - Test zero activations returns InsufficientMatches error
    - Test error context includes query text
    - Test error context includes completed dimensions count
    - _Requirements: 9.1-9.5_
  
  - [x] 8.4 Implement graceful error handling to pass tests (GREEN)
    - Update scan_all to handle timeouts gracefully
    - Update scan_all to continue on single dimension failures
    - Add zero activation check in navigate()
    - Add error context to all error returns
    - Refactor for clarity
    - _Requirements: 9.1-9.5_
  
  - [x] 8.5 Write integration tests for error scenarios (RED)
    - Test end-to-end with empty query
    - Test end-to-end with oversized query
    - Test end-to-end with query matching no dimensions
    - Test error messages are user-friendly
    - _Requirements: 9.1-9.7_
  
  - [x] 8.6 Integrate error handling into navigation flow (GREEN)
    - Add validate_query() at start of navigate()
    - Ensure all error paths tested
    - Refactor for final clarity
    - _Requirements: 9.1-9.7_

- [x] 9. Implement initialization and lifecycle (TDD approach)
  - [x] 9.1 Add system state management structures
    - Define `SystemState` enum: Uninitialized, Initializing, Ready, ShuttingDown, Failed (if not in types.rs)
    - Add state field to NavigationSystem
    - _Requirements: 14.13_
  
  - [x] 9.2 Write tests for vocabulary loading (RED)
    - Test successful load returns HashSet with correct size
    - Test validation fails if < 100 words for emotional/technical
    - Test missing file returns error
    - Test empty file returns error
    - _Requirements: 14.4-14.6, 14.11-14.12_
  
  - [x] 9.3 Implement vocabulary loading to pass tests (GREEN)
    - Write `load_vocabulary()` function (if not done in Task 3)
    - Add validation logic
    - Return appropriate errors
    - Refactor for clarity
    - _Requirements: 14.4-14.6, 14.11-14.12_
  
  - [x] 9.4 Write tests for system initialization (RED)
    - Test successful initialization sets state to Ready
    - Test initialization loads all vocabularies
    - Test initialization loads dimension registry
    - Test initialization creates all components
    - Test initialization failure sets state to Failed
    - Test navigation refused when not in Ready state
    - _Requirements: 14.1-14.3, 14.11-14.13_
  
  - [x] 9.5 Implement system initialization to pass tests (GREEN)
    - Write `initialize()` async method
    - Load configuration
    - Load and validate dimension registry
    - Load vocabularies
    - Initialize all components (QueryAnalyzer, ParallelScanner, etc.)
    - Set state to Ready on success, Failed on error
    - Add state check in navigate() method
    - Refactor for clarity
    - _Requirements: 14.1-14.3, 14.11-14.13_
  
  - [x] 9.6 Write integration tests for initialization (RED)
    - Test full initialization with real files
    - Test initialization with missing vocabulary files
    - Test initialization with invalid dimension registry
    - Test state transitions through lifecycle
    - _Requirements: 14.1-14.13_
  
  - [x] 9.7 Finalize initialization implementation (GREEN)
    - Ensure all error paths handled
    - Add comprehensive logging
    - Refactor for final clarity
    - _Requirements: 14.1-14.13_

- [ ] 10. Implement observability and diagnostics
  - [ ] 10.1 Add duration tracking
    - Track query analysis duration
    - Track individual dimension scan durations
    - Track path selection duration
    - Track total navigation duration
    - Include durations in NavigationResult
    - _Requirements: 10.1-10.2_
  
  - [ ] 10.2 Set up logging infrastructure
    - Add `tracing` and `tracing-subscriber` dependencies to Cargo.toml
    - Configure structured logging with JSON formatter
    - Set up log levels: ERROR, WARN, INFO, DEBUG, TRACE
    - Configure log output to stdout and file (navigation.log)
    - Add environment variable support for log level (RUST_LOG)
    - Initialize logging in main() before system startup
    - _Requirements: 10.1-10.5_
  
  - [ ] 10.3 Add diagnostic logging throughout system
    - Log query analysis results at DEBUG level with structured fields
    - Log dimension activations at DEBUG level with dimension_id, confidence, matched_keywords
    - Log path selection at INFO level with selected dimensions and confidences
    - Log return-to-source triggers at WARN level with original/reduced counts
    - Log navigation completion at INFO level with duration, dimensions, return_to_source flag
    - Log errors at ERROR level with full context and stack traces
    - Use tracing macros (info!, debug!, warn!, error!) with structured fields
    - Example: `info!(query = %query, dimensions = ?selected, duration_ms = %duration, "Navigation completed")`
    - _Requirements: 10.1-10.5_
  
  - [ ] 10.4 Add metrics collection and monitoring
    - Create `NavigationMetrics` struct with counters and histograms
    - Track dimensions activated count (histogram)
    - Track confidence score distribution (histogram)
    - Track return-to-source rate (counter)
    - Track scan timeouts (counter)
    - Track insufficient matches (counter)
    - Track validation errors (counter)
    - Track queries per second (counter)
    - Track concurrent requests (gauge)
    - Expose metrics endpoint for Prometheus/monitoring tools
    - _Requirements: 10.3-10.5_
  
  - [ ] 10.5 Add diagnostic events and tracing
    - Create `DiagnosticEvent` enum for all navigation events
    - Emit QueryAnalyzed event with keywords, type, urgency, frequency
    - Emit DimensionScanned event for each dimension with confidence
    - Emit PathSelected event for each selected path
    - Emit ReturnToSourceTriggered event with dimension counts
    - Emit NavigationCompleted event with summary
    - Add tracing spans for performance profiling
    - Support structured event export (JSON, OpenTelemetry)
    - _Requirements: 10.1-10.5_
  
  - [ ] 10.6 Create monitoring dashboard configuration
    - Create Grafana dashboard JSON for navigation metrics
    - Add panels for: latency (p50, p95, p99), throughput, error rate
    - Add panels for: dimension activation distribution, confidence scores
    - Add panels for: return-to-source rate, timeout rate
    - Add alerts for: high latency (>150ms), high error rate (>5%), high timeout rate (>10%)
    - Document dashboard setup and usage
    - _Requirements: 10.1-10.5_
  
  - [ ] 10.7 Write observability tests
    - Test duration tracking accuracy
    - Test logging output format and structure
    - Test metrics collection and export
    - Test diagnostic event emission
    - Test tracing span creation
    - Test metrics endpoint response
    - _Requirements: 10.1-10.5_

- [ ] 11. Implement concurrency support
  - [ ] 11.1 Add thread-safe registry access
    - Wrap DimensionRegistry in Arc for shared ownership
    - Registry is immutable after initialization (no locks needed)
    - _Requirements: 8.1-8.2_
  
  - [ ] 11.2 Ensure request isolation
    - Verify each navigation request uses independent data structures
    - Verify no shared mutable state between requests
    - _Requirements: 8.5_
  
  - [ ] 11.3 Add resource cleanup
    - Ensure temporary allocations are released within 10ms after navigation
    - Drop intermediate results promptly
    - _Requirements: 8.3_
  
  - [ ] 11.4 Write concurrency tests
    - Test 100 concurrent navigation requests
    - Test no data races using race detection tools
    - Test independent results (no cross-contamination)
    - Test performance under load (p95 < 100ms)
    - Test resource cleanup timing
    - _Requirements: 8.1-8.5_

- [ ] 12. Create integration with memory manager
  - [ ] 12.1 Define integration interface
    - Document NavigationResult structure for memory manager consumption
    - Document expected layer loading behavior
    - _Requirements: 7.1-7.7_
  
  - [ ] 12.2 Add integration example
    - Create example code showing how memory manager loads dimensions from NavigationResult
    - Document data flow from navigation to memory loading
    - _Requirements: 7.1-7.7_

- [ ] 13. Write BDD scenarios
  - [ ] 13.1 Create feature file for query navigation
    - Write scenario: Navigate emotional query
    - Write scenario: Navigate technical query
    - Write scenario: Navigate philosophical query
    - Write scenario: Return to source on complexity
    - _Requirements: 1-7_
  
  - [ ] 13.2 Implement step definitions
    - Implement "Given a navigation system with loaded dimensions"
    - Implement "When I navigate query"
    - Implement "Then dimension should be activated"
    - Implement "And confidence should be above X"
    - Implement "And layer sequence should include"
    - Implement "And return-to-source flag should be"
    - _Requirements: 1-7_
  
  - [ ] 13.3 Run BDD tests
    - Execute all scenarios
    - Verify all scenarios pass
    - _Requirements: 1-7_

- [ ] 14. Create performance benchmarks
  - [ ] 14.1 Create benchmark suite
    - Benchmark query analysis (<5ms target)
    - Benchmark single dimension scan (<10ms target)
    - Benchmark parallel scan of all 14 dimensions (<100ms p95 target)
    - Benchmark full navigation (<150ms p95 target)
    - Benchmark dimension registry lookup (<1μs target)
    - _Requirements: 1.1, 2.2, 11.5_
  
  - [ ] 14.2 Run benchmarks and validate
    - Execute benchmarks with various query types
    - Verify all performance targets are met
    - Document results
    - _Requirements: 1.1, 2.2, 11.5_

- [ ] 15. Create documentation
  - [ ] 15.1 Write API documentation
    - Document all public structs and methods
    - Include usage examples for NavigationSystem
    - Document error types and handling
    - Document configuration options
    - _Requirements: All_
  
  - [ ] 15.2 Write integration guide
    - Document how to initialize the navigation system
    - Document how to process navigation results
    - Document how to integrate with memory manager
    - Provide complete code examples
    - _Requirements: 14.1-14.13_
  
  - [ ] 15.3 Create README
    - Overview of navigation system
    - Quick start guide
    - Architecture diagram
    - Performance characteristics
    - Future roadmap (Phase 2/3)
    - _Requirements: All_

---

## Task Execution Notes

### Execution Order
1. Start with task 1 (project structure) - COMPLETED ✅
2. Complete task 2 (dimension registry) - COMPLETED ✅
3. Proceed sequentially through tasks 3-15 using TDD approach
4. For each feature, complete all test sub-tasks (RED) before implementation sub-tasks (GREEN)
5. Complete all sub-tasks before moving to next top-level task

### TDD Execution Guidelines
**For each feature:**
1. **Write tests first (RED phase)**: Create comprehensive test suite that defines expected behavior
2. **Run tests**: Verify they fail (proving tests are actually testing something)
3. **Implement minimally (GREEN phase)**: Write simplest code to make tests pass
4. **Run tests**: Verify they pass
5. **Refactor**: Clean up code while keeping tests green
6. **Repeat**: Move to next feature

**Key Principles:**
- Never write implementation before tests (except basic structure/types)
- Tests should fail initially (if they pass without implementation, they're not testing correctly)
- Write minimal code to pass tests (avoid "maybe we'll need this later")
- Refactor only when tests are green
- Each test should test one specific behavior

**Example TDD Workflow (Task 3.4-3.5: Keyword Extraction):**

```rust
// Step 1: Write tests first (RED) - Task 3.4
#[test]
fn test_extract_keywords_basic() {
    let analyzer = QueryAnalyzer::new(test_vocab());
    let keywords = analyzer.extract_keywords("hello world");
    assert_eq!(keywords, vec!["hello", "world"]);
}

#[test]
fn test_extract_keywords_with_punctuation() {
    let analyzer = QueryAnalyzer::new(test_vocab());
    let keywords = analyzer.extract_keywords("hello, world!");
    assert_eq!(keywords, vec!["hello", "world"]);
}

// Step 2: Run tests - they FAIL (extract_keywords doesn't exist yet)
// $ cargo test
// error[E0599]: no method named `extract_keywords` found

// Step 3: Implement minimally (GREEN) - Task 3.5
impl QueryAnalyzer {
    pub fn extract_keywords(&self, query: &str) -> Vec<String> {
        query.split_whitespace()
            .map(|w| w.trim_matches(|c: char| !c.is_alphanumeric()))
            .map(|w| w.to_lowercase())
            .filter(|w| !w.is_empty())
            .collect()
    }
}

// Step 4: Run tests - they PASS
// $ cargo test
// test test_extract_keywords_basic ... ok
// test test_extract_keywords_with_punctuation ... ok

// Step 5: Refactor (still in GREEN phase)
// - Add stopword filtering
// - Optimize performance
// - Add documentation
// - Keep running tests to ensure they stay green
```

### Testing Strategy
- **Unit tests**: Validate individual functions and methods (written in RED phase)
- **Integration tests**: Validate component interactions (written in RED phase)
- **BDD tests**: Validate user-facing behaviors (Task 13)
- **Performance benchmarks**: Validate non-functional requirements (Task 14)
- **TDD ensures**: Every requirement has a test before implementation

### Definition of Done
A task is complete when:
- All RED sub-tasks completed: Tests written and initially failing
- All GREEN sub-tasks completed: Implementation makes tests pass
- All tests are green (passing)
- Code is refactored for clarity and performance
- Code follows Rust best practices and project style
- Integration with previous tasks is verified
- No regressions in existing functionality
- Documentation updated (if applicable)

### Phase 2 Preparation
The following components are designed for future extension:
- QueryAnalyzer: Ready for synesthetic engine integration
- ParallelScanner: Ready for learned association matching
- PathSelector: Ready for 30% synesthetic weight addition
- NavigationSystem: Ready for 9-iteration deep thinking

---

## Requirements Coverage

### MVP Tasks (Phase 1)
- **Requirement 1**: Tasks 3.2-3.8 (Query Analysis)
- **Requirement 2**: Tasks 4.2-4.5 (Parallel Scanning)
- **Requirement 4**: Tasks 5.1-5.4 (Path Selection)
- **Requirement 5**: Tasks 6.1-6.3 (Depth Navigation)
- **Requirement 6**: Task 7.2 (Complexity Management)
- **Requirement 7**: Tasks 7.1, 7.3-7.4 (Result Assembly)
- **Requirement 8**: Task 11 (Concurrency)
- **Requirement 9**: Tasks 8.1-8.3 (Error Handling)
- **Requirement 10**: Task 10 (Observability)
- **Requirement 11**: Tasks 2.1-2.3 (Dimension Registry)
- **Requirement 12**: Task 1 (Configuration)
- **Requirement 14**: Tasks 9.1-9.4 (Initialization)
- **Requirement 15**: Task 3.6 (Frequency Estimation)

### Deferred Requirements (Phase 2/3)
- **Requirement 3**: Synesthetic matching (Phase 2)
- **Requirement 13**: Association memory (Phase 2)
- **Requirement 16**: Multiverse dimensions (Phase 2)
- **Requirement 17**: Fractal layers (Phase 2)
- **Requirement 18**: Emotional frequency tracking (Phase 2)
- **Requirement 19**: Hybrid memory (Phase 3)
- **Requirement 20**: Dimension emergence (Phase 3)

---

*Implementation plan created: 2025-10-25*
*Based on: requirements.md v1.0, design.md v1.0*
*Target: Phase 1 MVP - Core Navigation System*
