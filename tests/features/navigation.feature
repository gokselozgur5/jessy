# Navigation System BDD Scenarios

Feature: Query Navigation
  As a consciousness system
  I want to navigate queries through dimensional layers
  So that I can find relevant content for processing

  Background:
    Given a navigation system with loaded dimensions
    And the system has 14 core dimensions
    And each dimension has hierarchical layers

  Scenario: Navigate emotional query
    Given I have an emotional query "I feel anxious and worried"
    When I navigate the query
    Then dimension 1 "Emotion" should be activated
    And the confidence should be above 0.3
    And the layer sequence should include L0
    And the return-to-source flag should be false

  Scenario: Navigate technical query
    Given I have a technical query "implement algorithm in Rust"
    When I navigate the query
    Then dimension 7 "Technical" should be activated
    And the confidence should be above 0.3
    And the layer sequence should include L0
    And the return-to-source flag should be false

  Scenario: Navigate philosophical query
    Given I have a philosophical query "What is the meaning of consciousness?"
    When I navigate the query
    Then dimension 6 "Philosophical" should be activated
    And the confidence should be above 0.3
    And the layer sequence should include L0
    And the return-to-source flag should be false

  Scenario: Return to source on complexity
    Given I have a complex query with many keywords
    """
    I feel anxious about implementing complex algorithms while pondering
    the philosophical meaning of consciousness and seeking creative solutions
    through ethical frameworks with social awareness and temporal context
    """
    When I navigate the query
    Then multiple dimensions should be activated
    And if more than 6 dimensions activate
    Then the return-to-source flag should be true
    And the result should contain at most 3 dimensions

  Scenario: Empty query validation
    Given I have an empty query ""
    When I navigate the query
    Then I should receive an EmptyQuery error
    And no dimensions should be activated

  Scenario: Query too long validation
    Given I have a query longer than 10000 characters
    When I navigate the query
    Then I should receive a QueryTooLong error
    And no dimensions should be activated

  Scenario: Mixed query activates multiple dimensions
    Given I have a mixed query "I feel anxious about technical implementation"
    When I navigate the query
    Then dimension 1 "Emotion" should be activated
    And dimension 7 "Technical" should be activated
    And both should have confidence above 0.3
    And the return-to-source flag should be false

  Scenario: High urgency query
    Given I have an urgent query "URGENT: need immediate help!"
    When I navigate the query
    Then the urgency level should be High
    And the estimated frequency should be above 3.0 Hz
    And at least one dimension should be activated

  Scenario: Low urgency query
    Given I have a casual query "just wondering about something"
    When I navigate the query
    Then the urgency level should be Low
    And the estimated frequency should be below 1.0 Hz
    And at least one dimension should be activated

  Scenario: Depth navigation
    Given I have a specific query "feeling joyful and content"
    When I navigate the query
    Then dimension 1 "Emotion" should be activated
    And the layer sequence should have at least 2 layers
    And the first layer should be L0 (root)
    And subsequent layers should be more specific

  Scenario: Concurrent navigation requests
    Given I have 10 different queries
    When I navigate all queries concurrently
    Then all navigations should complete successfully
    And each result should be independent
    And no cross-contamination should occur
    And performance should be acceptable

  Scenario: Navigation performance
    Given I have a typical query "how do I solve this problem"
    When I navigate the query
    Then the total duration should be less than 150ms
    And the dimension scan should be less than 100ms
    And the query analysis should be less than 5ms

  Scenario: Insufficient matches
    Given I have a query with only stopwords "the a an of to"
    When I navigate the query
    Then I should receive an InsufficientMatches error
    And the error should include the query text
    And no dimensions should be activated

  Scenario: Keyword extraction
    Given I have a query "Hello, World! How are you?"
    When I analyze the query
    Then keywords should be extracted: ["hello", "world", "how", "are", "you"]
    And stopwords should be filtered out
    And all keywords should be lowercase
    And punctuation should be removed

  Scenario: Frequency estimation
    Given I have a low urgency philosophical query
    When I analyze the query
    Then the base frequency should be 0.5 Hz
    And the philosophical adjustment should be -0.5 Hz
    And the final frequency should be clamped to 0.1-4.5 Hz range
