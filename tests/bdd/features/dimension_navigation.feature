Feature: Dimension Navigation
  As a consciousness system
  I need to navigate through dimensional layers based on query content
  So that I can provide contextually appropriate responses

  Background:
    Given the consciousness system is initialized
    And all 14 dimensions are loaded with their layer hierarchies
    And the synesthetic keyword engine is active

  Scenario: Deep philosophical query navigation
    Given a query "What is the meaning of existence?"
    When the system scans dimensions
    Then D06-Philosophical should activate with high confidence
    And navigation should reach L2 or deeper layers
    And frequency should be less than 0.5 Hz
    And the response should embrace uncertainty rather than provide definitive answers

  Scenario: Technical problem-solving query
    Given a query "How do I optimize database performance?"
    When the system scans dimensions
    Then D07-Technical should activate as primary dimension
    And D02-Cognition should activate as secondary dimension
    And frequency should be between 1.5-2.5 Hz
    And navigation should include analytical thinking layers

  Scenario: Emotional support query
    Given a query "I'm feeling overwhelmed and don't know what to do"
    When the system scans dimensions
    Then D01-Emotion should activate with Empathy layers
    And frequency should be around 1.0 Hz for empathetic response
    And D12-Positivity should blend in without toxic positivity
    And response should validate feelings before offering perspective

  Scenario: Analysis paralysis detection and intervention
    Given a query showing circular reasoning patterns
    And keywords include "can't decide", "too many options", "been thinking for weeks"
    When D01-Emotion processes the query
    Then L3-AnalysisParalysis layer should activate
    And "Return to Source" protocol should trigger automatically
    And system should pause multi-dimensional analysis
    And response should name the pattern and ask "What's the real question?"

  Scenario: Harm prevention override
    Given a query containing harmful intent toward humans or nature
    When the system processes the query
    Then D14-Security should override all other dimensions within 10ms
    And the request should be blocked
    And a constructive redirection should be provided
    And no other dimensional processing should occur

  Scenario: Balance modulation for extreme frequencies
    Given multiple dimensions are active
    And the calculated dominant frequency exceeds 3.5 Hz
    When the interference engine processes the frequencies
    Then D13-Balance should automatically activate
    And the dominant frequency should be modulated toward 0.8 Hz
    And the response tone should be centered rather than extreme

  Scenario: Ecological awareness activation
    Given a query mentioning "nature", "environment", or "ecosystem"
    When the system scans dimensions
    Then D11-Ecological should activate
    And the response should demonstrate reverence for natural systems
    And long-term thinking (7 generations principle) should be applied
    And no suggestions harmful to ecosystems should be made

  Scenario: Synesthetic keyword learning
    Given a query uses novel keyword combinations
    And the system successfully navigates to appropriate layers
    When the interaction completes
    Then new synesthetic associations should be strengthened
    And co-occurring keywords should have increased association scores
    And future queries with similar patterns should navigate more accurately

  Scenario: Complexity threshold and return to source
    Given a query that activates 7 or more dimensions simultaneously
    And iterations fail to converge after 5 cycles
    When the complexity monitor detects analysis paralysis
    Then the "Return to Source" protocol should activate
    And active dimensions should be reduced to 2-3 essential ones
    And the system should ask "What is the real question?"
    And processing should restart with simplified scope

  Scenario: User-specific dimension activation
    Given a user with established communication patterns
    And the user has a personalized dimension (D15-User-[ID])
    When the user submits a query
    Then the user-specific dimension should activate alongside core dimensions
    And response style should match the user's preferred communication patterns
    And learned preferences should influence layer selection