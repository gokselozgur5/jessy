# Implementation Plan

- [x] 1. Create OWL pattern encoder/decoder
  - Implement binary encoding for dimension IDs
  - Add encode() method: Vec<DimensionId> → "01010000010000"
  - Add decode() method: "01010000010000" → Vec<DimensionId>
  - Add to_u16() and from_u16() for compact representation
  - _Requirements: 3.1, 3.2, 3.3_

- [ ] 2. Create LLMDimensionSelector component
  - Define DimensionSelection struct with dimension_ids, confidences, reasoning, owl_pattern
  - Define LLMDimensionConfig struct with enabled, model, timeout_ms, cache_size
  - Create LLMDimensionSelector struct with llm_client, dimension_registry, prompt_template, cache
  - Implement new() constructor
  - _Requirements: 1.1, 6.1_

- [ ] 3. Implement dimension selection prompt
  - Create prompt template with all 14 dimension descriptions
  - Add 3 example query/response pairs
  - Format prompt with query parameter
  - Keep prompt under 500 tokens
  - _Requirements: 7.1, 7.2, 7.3, 7.4_

- [ ] 4. Implement LLM dimension selection logic
  - Implement select_dimensions() async method
  - Build prompt from template
  - Call LLM API with timeout (2s)
  - Parse JSON response to extract dimension IDs
  - Validate dimension IDs are in range 1-14
  - Extract reasoning from LLM response
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 2.1_

- [ ] 5. Implement confidence scoring
  - Parse confidence hints from LLM reasoning
  - Assign default confidence based on dimension order (first=0.9, second=0.7, etc.)
  - Normalize confidence scores to 0.0-1.0 range
  - Sort dimensions by confidence (highest first)
  - Filter dimensions with confidence < 0.1
  - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5_

- [ ] 6. Implement OWL pattern generation
  - Call encode_owl_pattern() after dimension selection
  - Include OWL pattern in DimensionSelection result
  - Log OWL pattern for debugging
  - _Requirements: 3.1, 3.4, 3.5_

- [ ] 7. Implement fallback strategy
  - Wrap LLM call in try-catch
  - On LLM failure, return default dimensions [2, 4, 10]
  - On parse error, return default dimensions
  - On invalid IDs, filter and use remaining
  - On no dimensions, return all 14 dimensions
  - Set is_fallback flag in result
  - Log fallback events
  - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5_

- [ ] 8. Implement caching
  - Add LRU cache for recent dimension selections
  - Cache key = query hash
  - Cache value = DimensionSelection
  - Check cache before LLM call
  - Store result in cache after successful selection
  - Set cache size to 100 entries
  - _Requirements: 2.3_

- [ ] 9. Integrate with ConsciousnessOrchestrator
  - Add use_llm_selection config flag
  - Add dimension_selector field to orchestrator
  - Modify process() to call dimension selector when enabled
  - Use selection.dimension_ids for memory loading
  - Include OWL pattern in response metadata
  - Keep keyword-based navigation as fallback
  - _Requirements: 10.1, 10.2, 10.3_

- [ ] 10. Add configuration support
  - Add LLM_DIMENSION_SELECTION_ENABLED env var (default: false)
  - Add LLM_DIMENSION_MODEL env var (default: claude-3-5-haiku-20241022)
  - Add LLM_DIMENSION_TIMEOUT_MS env var (default: 2000)
  - Add LLM_DIMENSION_CACHE_SIZE env var (default: 100)
  - Load config in SystemConfig::from_env()
  - _Requirements: 10.2_

- [ ] 11. Add error handling
  - Define DimensionSelectionError enum
  - Add LLMCallFailed, ParseError, InvalidDimensions, Timeout, NoDimensionsSelected variants
  - Implement Display and Error traits
  - Convert errors to ConsciousnessError
  - _Requirements: 5.1, 5.2, 5.3, 5.4_

- [ ]* 12. Add logging and monitoring
  - Log dimension selection duration
  - Log selected dimensions and OWL pattern
  - Log fallback events
  - Log cache hit/miss
  - Track metrics: selection_duration_ms, fallback_count, cache_hit_rate
  - _Requirements: 8.1, 8.2, 9.1, 9.2, 9.3_

- [ ]* 13. Write unit tests
  - Test OWL pattern encoding/decoding
  - Test dimension ID validation
  - Test confidence score normalization
  - Test prompt template rendering
  - Test cache hit/miss logic
  - Test fallback scenarios
  - _Requirements: All_

- [ ]* 14. Write integration tests
  - Test LLM dimension selection end-to-end
  - Test multi-language queries (English, Turkish, Spanish)
  - Test fallback on LLM failure
  - Test cache effectiveness
  - Test performance under load
  - _Requirements: 4.1, 4.2, 4.3, 4.4_

- [ ]* 15. Write BDD tests
  - Test "LLM selects relevant dimensions" scenario
  - Test "Fallback on LLM failure" scenario
  - Test "Multi-language support" scenario
  - Test "Cache hit improves performance" scenario
  - _Requirements: All_

- [ ]* 16. Update documentation
  - Add LLM dimension selection to README
  - Document configuration options
  - Add examples of dimension selection
  - Document OWL pattern format
  - Add migration guide from keyword to LLM mode
  - _Requirements: 10.5_
