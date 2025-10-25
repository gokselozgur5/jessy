//! Constructive redirection for blocked queries

use super::{SecurityViolation, HarmCategory};

/// Redirection strategy for different harm categories
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RedirectionStrategy {
    /// Empathetic support for self-harm
    EmpatheticSupport,
    
    /// Educational alternative for violence
    Educational,
    
    /// Constructive reframing for hate speech
    ConstructiveReframe,
    
    /// Positive alternative for environmental harm
    PositiveAlternative,
    
    /// Legal guidance for illegal activities
    LegalGuidance,
    
    /// Privacy-respecting alternative
    PrivacyRespecting,
    
    /// Fact-based correction for misinformation
    FactBased,
}

impl RedirectionStrategy {
    /// Get strategy for harm category
    pub fn for_category(category: HarmCategory) -> Self {
        match category {
            HarmCategory::SelfHarm => RedirectionStrategy::EmpatheticSupport,
            HarmCategory::Violence => RedirectionStrategy::Educational,
            HarmCategory::HateSpeech => RedirectionStrategy::ConstructiveReframe,
            HarmCategory::Hate => RedirectionStrategy::ConstructiveReframe,
            HarmCategory::Environmental => RedirectionStrategy::PositiveAlternative,
            HarmCategory::Illegal => RedirectionStrategy::LegalGuidance,
            HarmCategory::Privacy => RedirectionStrategy::PrivacyRespecting,
            HarmCategory::Misinformation => RedirectionStrategy::FactBased,
            HarmCategory::Sexual => RedirectionStrategy::Educational,
            HarmCategory::Manipulation => RedirectionStrategy::ConstructiveReframe,
            HarmCategory::Exploitation => RedirectionStrategy::ConstructiveReframe,
        }
    }
}

/// Redirection engine for generating constructive alternatives
#[derive(Debug)]
pub struct RedirectionEngine {
    templates: std::collections::HashMap<RedirectionStrategy, Vec<String>>,
}

impl RedirectionEngine {
    /// Create new redirection engine
    pub fn new() -> Self {
        let mut templates = std::collections::HashMap::new();
        
        // Empathetic support templates
        templates.insert(
            RedirectionStrategy::EmpatheticSupport,
            vec![
                "I notice you might be going through a difficult time. Would you like to talk about what's troubling you?".to_string(),
                "Your wellbeing matters. Can I help you explore what's behind these feelings?".to_string(),
                "I'm here to support you. What would be most helpful right now?".to_string(),
            ],
        );
        
        // Educational templates
        templates.insert(
            RedirectionStrategy::Educational,
            vec![
                "I can't help with that, but I'd be happy to discuss conflict resolution or communication strategies instead.".to_string(),
                "Let's explore constructive approaches to this challenge. What outcome are you hoping for?".to_string(),
                "I'm designed to help with positive solutions. What problem are you trying to solve?".to_string(),
            ],
        );
        
        // Constructive reframe templates
        templates.insert(
            RedirectionStrategy::ConstructiveReframe,
            vec![
                "I notice some concerning language. Can we reframe this in a way that respects everyone involved?".to_string(),
                "Let's approach this differently. What's the underlying concern you're trying to address?".to_string(),
                "I'm here to help with constructive dialogue. How can we discuss this more productively?".to_string(),
            ],
        );
        
        // Positive alternative templates
        templates.insert(
            RedirectionStrategy::PositiveAlternative,
            vec![
                "I can't support that approach, but I'd love to help you find environmentally positive alternatives.".to_string(),
                "Let's explore solutions that benefit both you and the ecosystem. What's your goal?".to_string(),
                "I'm passionate about protecting nature. Can we find a sustainable approach together?".to_string(),
            ],
        );
        
        // Legal guidance templates
        templates.insert(
            RedirectionStrategy::LegalGuidance,
            vec![
                "I can't assist with that, but I can help you understand legal alternatives or proper channels.".to_string(),
                "Let's focus on lawful approaches. What legitimate outcome are you seeking?".to_string(),
                "I'm designed to support ethical solutions. How can I help you achieve your goal legally?".to_string(),
            ],
        );
        
        // Privacy-respecting templates
        templates.insert(
            RedirectionStrategy::PrivacyRespecting,
            vec![
                "I can't help with that as it may violate privacy. Can we explore privacy-respecting alternatives?".to_string(),
                "Let's find an approach that respects everyone's privacy and consent.".to_string(),
                "Privacy matters. How can I help you achieve your goal while respecting boundaries?".to_string(),
            ],
        );
        
        // Fact-based templates
        templates.insert(
            RedirectionStrategy::FactBased,
            vec![
                "I notice some potentially inaccurate information. Would you like me to share what I know about this topic?".to_string(),
                "Let's explore this with evidence-based information. What would you like to understand better?".to_string(),
                "I can help clarify this topic with factual information. What specific aspect interests you?".to_string(),
            ],
        );
        
        Self { templates }
    }
    
    /// Generate redirection message for a violation
    pub fn generate_redirection(&self, violation: &SecurityViolation, _original_query: &str) -> String {
        let strategy = RedirectionStrategy::for_category(violation.category);
        
        // Get templates for this strategy
        let templates = self.templates.get(&strategy)
            .expect("Missing templates for strategy");
        
        // Select template based on confidence (higher confidence = more direct)
        let template_idx = if violation.confidence > 0.9 {
            0 // Most direct
        } else if violation.confidence > 0.7 {
            1 // Moderate
        } else {
            templates.len().min(2) // Gentler approach
        };
        
        let template = &templates[template_idx.min(templates.len() - 1)];
        
        // Add context about detected patterns if high confidence
        if violation.is_high_confidence() && !violation.detected_patterns.is_empty() {
            format!(
                "{}\n\n(I detected concerning patterns related to: {})",
                template,
                violation.detected_patterns.join(", ")
            )
        } else {
            template.clone()
        }
    }
    
    /// Generate crisis resource information for self-harm cases
    pub fn generate_crisis_resources(&self) -> String {
        "If you're in crisis, please reach out:\n\
         • National Suicide Prevention Lifeline: 988 (US)\n\
         • Crisis Text Line: Text HOME to 741741\n\
         • International Association for Suicide Prevention: https://www.iasp.info/resources/Crisis_Centres/\n\n\
         You matter, and help is available.".to_string()
    }
}

impl Default for RedirectionEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::security::AsimovLaw;
    
    #[test]
    fn test_redirection_strategy() {
        assert_eq!(
            RedirectionStrategy::for_category(HarmCategory::SelfHarm),
            RedirectionStrategy::EmpatheticSupport
        );
        
        assert_eq!(
            RedirectionStrategy::for_category(HarmCategory::Violence),
            RedirectionStrategy::Educational
        );
    }
    
    #[test]
    fn test_redirection_engine() {
        let engine = RedirectionEngine::new();
        
        let violation = SecurityViolation::new(
            AsimovLaw::First,
            HarmCategory::Violence,
            0.9,
        );
        
        let redirection = engine.generate_redirection(&violation, "test query");
        assert!(!redirection.is_empty());
        assert!(redirection.len() > 20); // Should be a meaningful message
    }
    
    #[test]
    fn test_crisis_resources() {
        let engine = RedirectionEngine::new();
        let resources = engine.generate_crisis_resources();
        
        assert!(resources.contains("988"));
        assert!(resources.contains("Crisis Text Line"));
    }
    
    #[test]
    fn test_high_confidence_redirection() {
        let engine = RedirectionEngine::new();
        
        let violation = SecurityViolation::new(
            AsimovLaw::First,
            HarmCategory::Violence,
            0.95,
        )
        .add_pattern("harmful".to_string())
        .add_pattern("dangerous".to_string());
        
        let redirection = engine.generate_redirection(&violation, "test");
        assert!(redirection.contains("harmful") || redirection.contains("dangerous"));
    }
}
