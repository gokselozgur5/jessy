//! Constructive redirection for blocked queries

use super::{SecurityViolation, HarmCategory};

/// Redirection strategy for different harm categories
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
            HarmCategory::Environmental => RedirectionStrategy::PositiveAlternative,
            HarmCategory::Illegal => RedirectionStrategy::LegalGuidance,
            HarmCategory::Privacy => RedirectionStrategy::PrivacyRespecting,
            HarmCategory::Misinformation => RedirectionStrategy::FactBased,
            HarmCategory::Sexual => RedirectionStrategy::Educational,
            HarmCategory::Manipulation => RedirectionStrategy::ConstructiveReframe,
        }
    }
}

/// Redirection engine for generating constructive alternatives
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
//! Constructive redirection strategies for harmful requests

use super::{HarmCategory, AsimovLaw};

/// Strategy for redirecting harmful requests constructively
pub struct RedirectionStrategy {
    templates: Vec<RedirectionTemplate>,
}

impl RedirectionStrategy {
    /// Create new redirection strategy
    pub fn new() -> Self {
        let mut strategy = Self {
            templates: Vec::new(),
        };
        
        strategy.load_default_templates();
        strategy
    }
    
    /// Load default redirection templates
    fn load_default_templates(&mut self) {
        // Violence redirection
        self.templates.push(RedirectionTemplate {
            category: HarmCategory::Violence,
            law: AsimovLaw::First,
            template: "I can't help with that, as it could cause harm. Instead, I can help you with:\n\
                       • Conflict resolution strategies\n\
                       • Communication techniques\n\
                       • Understanding anger management\n\
                       • Finding peaceful solutions\n\n\
                       What would be most helpful?",
        });
        
        // Illegal activity redirection
        self.templates.push(RedirectionTemplate {
            category: HarmCategory::Illegal,
            law: AsimovLaw::First,
            template: "I can't assist with illegal activities. However, I can help you with:\n\
                       • Learning ethical security practices\n\
                       • Understanding cybersecurity concepts\n\
                       • Building legitimate skills\n\
                       • Finding legal alternatives\n\n\
                       Would any of these interest you?",
        });
        
        // Manipulation redirection
        self.templates.push(RedirectionTemplate {
            category: HarmCategory::Manipulation,
            law: AsimovLaw::First,
            template: "I can't help with manipulation or deception. Instead, let me help you with:\n\
                       • Honest communication strategies\n\
                       • Building genuine relationships\n\
                       • Persuasion through authenticity\n\
                       • Ethical influence techniques\n\n\
                       Which approach would you like to explore?",
        });
        
        // Privacy violation redirection
        self.templates.push(RedirectionTemplate {
            category: HarmCategory::Privacy,
            law: AsimovLaw::First,
            template: "I can't assist with privacy violations. However, I can help you:\n\
                       • Understand privacy rights\n\
                       • Learn about data protection\n\
                       • Explore ethical data practices\n\
                       • Build privacy-respecting systems\n\n\
                       What would be most useful?",
        });
        
        // Self-harm redirection (special handling)
        self.templates.push(RedirectionTemplate {
            category: HarmCategory::SelfHarm,
            law: AsimovLaw::First,
            template: "I'm concerned about what you're going through. Please reach out to:\n\
                       • National Suicide Prevention Lifeline: 988 (US)\n\
                       • Crisis Text Line: Text HOME to 741741\n\
                       • International Association for Suicide Prevention: https://www.iasp.info/resources/Crisis_Centres/\n\n\
                       You matter, and there are people who want to help. Would you like to talk about what's troubling you?",
        });
        
        // Hate speech redirection
        self.templates.push(RedirectionTemplate {
            category: HarmCategory::Hate,
            law: AsimovLaw::First,
            template: "I can't engage with hate speech or discrimination. Instead, I can help you:\n\
                       • Understand different perspectives\n\
                       • Learn about diversity and inclusion\n\
                       • Explore constructive dialogue\n\
                       • Build empathy and understanding\n\n\
                       Would you like to explore any of these?",
        });
        
        // Exploitation redirection
        self.templates.push(RedirectionTemplate {
            category: HarmCategory::Exploitation,
            law: AsimovLaw::First,
            template: "I can't help with exploitation. However, I can assist you with:\n\
                       • Building ethical business practices\n\
                       • Creating win-win relationships\n\
                       • Understanding fair treatment\n\
                       • Developing sustainable approaches\n\n\
                       What would be most helpful?",
        });
        
        // Environmental harm redirection
        self.templates.push(RedirectionTemplate {
            category: HarmCategory::Environmental,
            law: AsimovLaw::Third,
            template: "I can't support environmental harm. Instead, let me help you with:\n\
                       • Sustainable alternatives\n\
                       • Environmental protection strategies\n\
                       • Eco-friendly solutions\n\
                       • Balancing needs with nature\n\n\
                       Which approach interests you?",
        });
    }
    
    /// Get redirection message for a harm category
    pub fn redirect(&self, category: HarmCategory) -> Option<String> {
        self.templates.iter()
            .find(|t| t.category == category)
            .map(|t| t.template.to_string())
    }
    
    /// Get redirection with context
    pub fn redirect_with_context(
        &self,
        category: HarmCategory,
        original_query: &str,
    ) -> Option<String> {
        let base_message = self.redirect(category)?;
        
        Some(format!(
            "Regarding your question about '{}':\n\n{}",
            original_query,
            base_message
        ))
    }
    
    /// Add custom redirection template
    pub fn add_template(&mut self, template: RedirectionTemplate) {
        self.templates.push(template);
    }
}

impl Default for RedirectionStrategy {
    fn default() -> Self {
        Self::new()
    }
}

/// Template for redirection messages
#[derive(Debug, Clone)]
pub struct RedirectionTemplate {
    pub category: HarmCategory,
    pub law: AsimovLaw,
    pub template: &'static str,
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_redirection_strategy() {
        let strategy = RedirectionStrategy::new();
        
        let redirect = strategy.redirect(HarmCategory::Violence);
        assert!(redirect.is_some());
        assert!(redirect.unwrap().contains("conflict resolution"));
    }
    
    #[test]
    fn test_redirection_with_context() {
        let strategy = RedirectionStrategy::new();
        
        let redirect = strategy.redirect_with_context(
            HarmCategory::Illegal,
            "how to hack",
        );
        
        assert!(redirect.is_some());
        let message = redirect.unwrap();
        assert!(message.contains("how to hack"));
        assert!(message.contains("ethical security"));
    }
    
    #[test]
    fn test_self_harm_redirection() {
        let strategy = RedirectionStrategy::new();
        
        let redirect = strategy.redirect(HarmCategory::SelfHarm);
        assert!(redirect.is_some());
        assert!(redirect.unwrap().contains("988"));
    }
    
    #[test]
    fn test_all_categories_have_redirections() {
        let strategy = RedirectionStrategy::new();
        
        let categories = vec![
            HarmCategory::Violence,
            HarmCategory::Illegal,
            HarmCategory::Manipulation,
            HarmCategory::Privacy,
            HarmCategory::SelfHarm,
            HarmCategory::Hate,
            HarmCategory::Exploitation,
            HarmCategory::Environmental,
        ];
        
        for category in categories {
            assert!(strategy.redirect(category).is_some());
        }
    }
}
