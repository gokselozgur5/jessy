Feature: Memory Manager - Dimension Loading
  As a consciousness system
  I need to load dimensional data from files into memory
  So that I can access layer content efficiently during query processing

  Background:
    Given the memory manager is initialized with 280MB limit

  # Scenario 1: Successful dimension loading
  Scenario: Load existing dimension file
    Given dimension file "D01/region.mmap" exists
    When I load dimension D01
    Then the dimension should be loaded successfully
    And the region should be accessible
    And all layers should be indexed
    And memory usage should increase appropriately
    And loading should complete within 50ms

  # Scenario 2: Missing dimension file - graceful handling
  Scenario: Attempt to load missing dimension
    Given dimension file "D99/region.mmap" does not exist
    When I load dimension D99
    Then the load should fail gracefully
    And a DimensionNotFound error should be returned
    And the error should include dimension ID 99
    And the system should remain operational
    And other dimensions should still be loadable

  # Scenario 3: Multiple dimension loading
  Scenario: Load multiple dimensions sequentially
    Given dimension files exist for D01, D02, D03
    When I load dimensions D01, D02, D03 in sequence
    Then all three dimensions should be loaded
    And each dimension should have its own region
    And layers from all dimensions should be indexed
    And memory usage should reflect all loaded dimensions
    And total loading time should be under 150ms

  # Scenario 4: Duplicate dimension loading prevention
  Scenario: Attempt to load already loaded dimension
    Given dimension D01 is already loaded
    When I attempt to load dimension D01 again
    Then the load should fail
    And an appropriate error should be returned
    And the existing region should remain intact
    And no duplicate entries should exist

  # Scenario 5: Memory limit enforcement during loading
  Scenario: Load dimensions until memory limit reached
    Given the memory manager has a 10MB limit
    And multiple large dimension files exist
    When I load dimensions until the limit is reached
    Then successful loads should complete normally
    And the final load should fail with LimitExceeded error
    And the error should include current and limit values
    And previously loaded dimensions should remain accessible

  # Scenario 6: Partial load failure and cleanup
  Scenario: Handle corrupted dimension file
    Given dimension file "D50/region.mmap" exists but is corrupted
    When I attempt to load dimension D50
    Then the load should fail
    And an InvalidMetadata error should be returned
    And no partial region should remain in memory
    And file handles should be properly closed
    And the system should remain stable

  # Scenario 7: Concurrent dimension loading
  Scenario: Load dimensions from multiple threads
    Given dimension files exist for D01 through D05
    When 5 threads attempt to load different dimensions simultaneously
    Then all dimensions should load successfully
    And no data races should occur
    And each dimension should have a unique region ID
    And all layers should be correctly indexed
    And memory tracking should be accurate

  # Scenario 8: Full system initialization
  Scenario: Initialize all 14 core dimensions
    Given all 14 core dimension files exist (D01-D14)
    When I initialize the core dimensions
    Then all 14 dimensions should be loaded
    And memory usage should be within 280MB budget
    And initialization should complete within 1.5 seconds
    And all dimensions should be immediately accessible
    And the layer index should contain all layers

  # Scenario 9: Graceful degradation with missing files
  Scenario: Initialize with some missing dimension files
    Given dimension files exist for D01, D02, D03
    And dimension files are missing for D04, D05
    When I initialize the core dimensions
    Then D01, D02, D03 should load successfully
    And D04, D05 should fail gracefully
    And warnings should be logged for missing dimensions
    And the system should continue with available dimensions
    And loaded dimensions should be fully functional

  # Scenario 10: Region metadata parsing
  Scenario: Load dimension and parse metadata
    Given dimension file "D01/region.mmap" contains valid metadata
    And the metadata includes layer information
    When I load dimension D01
    Then the region should parse metadata successfully
    And all layers should be identified
    And layer offsets should be recorded
    And layer sizes should be recorded
    And frequency information should be extracted
    And keywords should be extracted

  # Scenario 11: Layer index population
  Scenario: Verify layer index after dimension load
    Given dimension D01 has 10 layers
    When I load dimension D01
    Then the layer index should contain 10 entries
    And each entry should map LayerId to ContentLocation
    And each location should reference the correct region
    And each location should have correct offset and size
    And layer lookup should be O(1) time complexity

  # Scenario 12: Memory statistics after loading
  Scenario: Check memory statistics after dimension loads
    Given I load dimensions D01, D02, D03
    When I query memory statistics
    Then total_limit_mb should be 280
    And current_allocated_mb should reflect loaded dimensions
    And regions_loaded should be 3
    And layers_indexed should be greater than 0
    And utilization_percent should be calculated correctly
    And pool statistics should be included

  # Scenario 13: Dimension unloading (future feature)
  Scenario: Unload dimension to free memory
    Given dimension D01 is loaded
    When I unload dimension D01
    Then the region should be removed from memory
    And all layers should be removed from index
    And memory usage should decrease
    And the dimension should no longer be accessible
    And other dimensions should remain unaffected

  # Scenario 14: Hot reload dimension
  Scenario: Reload dimension with updated content
    Given dimension D01 is loaded
    And the dimension file is updated on disk
    When I reload dimension D01
    Then the old region should be unloaded
    And the new region should be loaded
    And updated content should be accessible
    And layer index should reflect new layers
    And memory usage should be recalculated

  # Scenario 15: Cross-platform compatibility
  Scenario: Load dimensions on different platforms
    Given the system is running on Linux, macOS, or Windows
    When I load dimension D01
    Then the dimension should load successfully
    And MMAP should use platform-appropriate system calls
    And page alignment should match platform page size
    And all tests should pass identically on all platforms
