Feature: Memory Manager - Layer Access
  As a query processor
  I need zero-copy access to layer content
  So that I can process queries within performance targets

  Background:
    Given the memory manager is initialized with 280MB limit
    And dimension D01 is loaded with layers

  # Scenario 1: Zero-copy layer access
  Scenario: Access layer content with zero-copy
    Given layer L01-05 exists in dimension D01
    When I access layer L01-05
    Then the content should be returned as a reference
    And no data copying should occur
    And access should complete within 1ms
    And the content should be valid UTF-8

  # Scenario 2: Fast layer access performance
  Scenario: Layer access meets performance target
    Given layer L01-03 exists in dimension D01
    When I access layer L01-03 100 times
    Then 99% of accesses should complete within 1ms
    And the average access time should be under 500μs
    And no memory allocations should occur in hot path

  # Scenario 3: Access non-existent layer
  Scenario: Attempt to access non-existent layer
    Given layer L99-99 does not exist
    When I attempt to access layer L99-99
    Then a LayerNotFound error should be returned
    And the error should include dimension ID 99
    And the error should include layer number 99
    And the system should remain operational

  # Scenario 4: Access layer from unloaded dimension
  Scenario: Access layer before dimension is loaded
    Given dimension D50 is not loaded
    When I attempt to access layer L50-01
    Then a LayerNotFound error should be returned
    And the error should indicate dimension not loaded
    And no crash should occur

  # Scenario 5: Concurrent layer access
  Scenario: Multiple threads access same layer
    Given layer L01-05 exists in dimension D01
    When 100 threads access layer L01-05 simultaneously
    Then all threads should receive identical content
    And no data corruption should occur
    And no data races should be detected
    And performance degradation should be less than 10%

  # Scenario 6: Access layer with metadata
  Scenario: Retrieve layer with frequency and keywords
    Given layer L01-05 has frequency 1.5 Hz
    And layer L01-05 has keywords "emotion", "empathy"
    When I load layer context for L01-05
    Then the context should include content
    And the context should include frequency 1.5 Hz
    And the context should include keywords
    And the layer ID should match L01-05

  # Scenario 7: Batch layer access
  Scenario: Load multiple layers efficiently
    Given layers L01-01, L01-02, L01-03 exist
    When I load contexts for all three layers
    Then all three contexts should be returned
    And total access time should be under 3ms
    And contexts should be in correct order
    And each context should have valid content

  # Scenario 8: Access layer from MMAP region
  Scenario: Read content from memory-mapped file
    Given layer L01-05 is stored in MMAP region
    When I read the layer content
    Then the content should come from mapped memory
    And no file I/O should occur
    And the OS should handle paging automatically
    And access should be zero-copy

  # Scenario 9: Access layer from heap (proto-dimension)
  Scenario: Read content from heap-based proto-dimension
    Given a proto-dimension D99 exists in heap
    And layer L99-00 is stored in heap
    When I access layer L99-00
    Then the content should be returned from heap
    And access should still be fast
    And the content should be identical to original

  # Scenario 10: Access layer during crystallization
  Scenario: Read layer while it's being crystallized
    Given layer L99-00 is in heap
    And crystallization to MMAP is in progress
    When I access layer L99-00 during crystallization
    Then I should receive valid content
    And content should be from either heap or MMAP
    And no partial/corrupted data should be returned
    And atomic pointer swap should ensure consistency

  # Scenario 11: Bounds checking on layer access
  Scenario: Verify bounds checking prevents out-of-bounds access
    Given layer L01-05 has size 1024 bytes
    When I attempt to read beyond layer boundaries
    Then an OutOfBounds error should be returned
    And the error should include offset and size
    And no buffer overflow should occur
    And the system should remain stable

  # Scenario 12: Access layer with invalid UTF-8
  Scenario: Handle layer with invalid UTF-8 content
    Given layer L50-01 contains invalid UTF-8 bytes
    When I attempt to read layer as string
    Then an appropriate error should be returned
    And the error should indicate UTF-8 validation failure
    And raw bytes should still be accessible
    And the system should not crash

  # Scenario 13: Layer access after dimension unload
  Scenario: Attempt to access layer after dimension unloaded
    Given dimension D01 was loaded
    And dimension D01 has been unloaded
    When I attempt to access layer L01-05
    Then a LayerNotFound error should be returned
    And the error should indicate dimension not available
    And no dangling pointer access should occur

  # Scenario 14: Access layer with large content
  Scenario: Read large layer content efficiently
    Given layer L01-10 has 10MB of content
    When I access layer L01-10
    Then the content should be returned as reference
    And no copying of 10MB should occur
    And access should still complete within 1ms
    And memory usage should not spike

  # Scenario 15: Layer access statistics
  Scenario: Track layer access patterns
    Given layer L01-05 exists
    When I access layer L01-05 multiple times
    Then access count should be tracked
    And access patterns should be recorded
    And statistics should be available for monitoring
    And hot layers should be identifiable

  # Scenario 16: Content location tracking
  Scenario: Verify content location is tracked correctly
    Given layer L01-05 is in MMAP region 1
    When I query the layer location
    Then the location should indicate MMAP
    And the region ID should be 1
    And the offset should be recorded
    And the size should be recorded

  # Scenario 17: Hybrid content access
  Scenario: Access layer with hybrid storage (MMAP + heap overlay)
    Given layer L01-05 has base content in MMAP
    And layer L01-05 has overlay content in heap
    When I access layer L01-05
    Then both MMAP and heap content should be combined
    And the combined view should be seamless
    And access should remain fast
    And content integrity should be maintained

  # Scenario 18: Layer access with frequency filtering
  Scenario: Access layers within frequency range
    Given multiple layers exist with different frequencies
    When I request layers with frequency 1.0-2.0 Hz
    Then only matching layers should be returned
    And each layer should have frequency in range
    And access should be efficient

  # Scenario 19: Navigation path to context conversion
  Scenario: Convert navigation path to loaded contexts
    Given a navigation path with layers L01-01, L01-02, L01-03
    When I convert the path to contexts
    Then all three contexts should be loaded
    And contexts should maintain path order
    And frequency information should be preserved
    And keywords should be included

  # Scenario 20: Context collection formatting
  Scenario: Format contexts for LLM processing
    Given I have loaded contexts for L01-01, L01-02, L01-03
    When I format the collection for LLM
    Then the output should include dimension metadata
    And each layer should show frequency
    And keywords should be listed
    And content should be properly formatted
    And the format should be LLM-friendly

  # Scenario 21: Concurrent access during dimension load
  Scenario: Access layers while new dimension is loading
    Given dimension D01 is loaded with layers
    And dimension D02 is being loaded
    When I access layers from D01 during D02 load
    Then D01 layers should remain accessible
    And no blocking should occur
    And read operations should not wait for write lock
    And performance should not degrade

  # Scenario 22: Layer access error recovery
  Scenario: Recover from layer access errors
    Given layer L01-05 exists
    And a temporary error occurs during access
    When I retry the layer access
    Then the retry should succeed
    And the content should be correct
    And the system should have recovered
    And no permanent damage should occur

  # Scenario 23: Memory pressure during access
  Scenario: Access layers under memory pressure
    Given memory usage is at 85% capacity
    And layer L01-05 exists
    When I access layer L01-05
    Then the access should still succeed
    And performance should remain acceptable
    And no allocation failures should occur
    And warnings should be logged

  # Scenario 24: Layer access with caching
  Scenario: Benefit from OS page cache
    Given layer L01-05 has been accessed recently
    When I access layer L01-05 again
    Then the access should be faster due to caching
    And the OS should serve from page cache
    And no disk I/O should occur
    And performance should be optimal

  # Scenario 25: Access layer sequence for query
  Scenario: Load layer sequence for query processing
    Given a query requires layers L01-01, L01-03, L01-05
    When I load the layer sequence
    Then all three layers should be loaded
    And the sequence should be optimized
    And total time should be under 3ms
    And contexts should be ready for processing
    And memory usage should be tracked
