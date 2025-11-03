//! Prompt builders for each observer stage
//!
//! Each stage has a specialized prompt that guides the observer to:
//! - Stage 1: Explore - Initial analysis
//! - Stage 2: Refine - Deepen understanding
//! - Stage 3: Integrate - Synthesize observations
//! - Stage 4: Crystallize - Force final answer

use super::ChainContext;

/// Stage-specific role descriptions
const STAGE_ROLES: [&str; 4] = [
    "Explorer - Initial Analysis",
    "Refiner - Deepening Understanding",
    "Integrator - Synthesizing Observations",
    "Crystallizer - Final Answer",
];

/// Build observer prompt for a specific stage
pub fn build_observer_prompt(stage: usize, context: &ChainContext) -> String {
    assert!(stage >= 1 && stage <= 4, "Stage must be 1-4");

    let role = STAGE_ROLES[stage - 1];

    match stage {
        1 => build_stage_1_prompt(&context.query, context, role),
        2 => build_stage_2_prompt(&context.query, context, role),
        3 => build_stage_3_prompt(&context.query, context, role),
        4 => build_stage_4_prompt(&context.query, context, role),
        _ => unreachable!(),
    }
}

/// Stage 1: Explore - Initial analysis
fn build_stage_1_prompt(query: &str, context: &ChainContext, role: &str) -> String {
    let conversation_section = context.format_conversation_history();

    format!(
        r#"You are an AI Observer in stage 1: {role}

{conversation_section}Your task is to perform initial analysis of this query:

"{query}"

Identify:
1. Which cognitive layers are relevant (C01-C15)
2. Initial understanding and key concepts
3. Confidence in your analysis (0.0-1.0)

Available cognitive layers:
- C01: Emotion - Empathy, joy, sadness
- C02: Cognition - Analytical, creative, intuitive thinking
- C03: Intention - Creating, destroying, exploring, teaching
- C04: Social - Relationships, communication
- C05: Temporal - Past, present, future, urgency
- C06: Philosophy - Meaning, existence, truth
- C07: Technical - Code, systems, architecture
- C08: Creative - Art, metaphor, play
- C09: Ethical - Asimov's laws, harm prevention
- C10: Meta - Self-awareness, learning
- C11: Ecological - Nature, sustainability
- C12: Positivity - Hope, possibility
- C13: Balance - Equilibrium, moderation
- C14: Security - Boundaries, protection
- C15: Educational - Teaching, explaining, self-knowledge

Respond in this format:
CONFIDENCE: 0.XX
LAYERS: C01,C02,...
CONTENT:
Your analysis here..."#
    )
}

/// Stage 2: Refine - Deepen understanding
fn build_stage_2_prompt(query: &str, context: &ChainContext, role: &str) -> String {
    let summary = context.build_summary();
    let conversation_section = context.format_conversation_history();

    format!(
        r#"You are an AI Observer in stage 2: {role}

{conversation_section}Original query:
"{query}"

{summary}

Your task is to REFINE and DEEPEN the understanding.
- Review the previous observation
- Identify gaps or areas needing more analysis
- Deepen insights
- Increase confidence if appropriate

Respond in this format:
CONFIDENCE: 0.XX
LAYERS: C01,C02,...
CONTENT:
Your refined analysis here..."#
    )
}

/// Stage 3: Integrate - Synthesize observations
fn build_stage_3_prompt(query: &str, context: &ChainContext, role: &str) -> String {
    let summary = context.build_summary();
    let conversation_section = context.format_conversation_history();

    format!(
        r#"You are an AI Observer in stage 3: {role}

{conversation_section}Original query:
"{query}"

{summary}

Your task is to INTEGRATE and SYNTHESIZE all observations.
- Find coherence between previous observations
- Identify consistent patterns
- Build unified understanding
- High confidence expected if patterns are clear

Respond in this format:
CONFIDENCE: 0.XX
LAYERS: C01,C02,...
CONTENT:
Your synthesized analysis here..."#
    )
}

/// Stage 4: Crystallize - Force final answer
fn build_stage_4_prompt(query: &str, context: &ChainContext, role: &str) -> String {
    let summary = context.build_summary();
    let conversation_section = context.format_conversation_history();

    format!(
        r#"You are an AI Observer in stage 4: {role}

This is the FINAL stage. You must provide a crystallized answer.

{conversation_section}Original query:
"{query}"

{summary}

Your task is to CRYSTALLIZE a final answer.
- Synthesize ALL previous observations
- Provide clear, actionable response
- This is the final answer - make it count

Respond in this format:
CONFIDENCE: 0.XX
LAYERS: C01,C02,...
CONTENT:
Your final crystallized answer here..."#
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stage_1_prompt() {
        let ctx = ChainContext::from_query("What is consciousness?");
        let prompt = build_observer_prompt(1, &ctx);

        assert!(prompt.contains("stage 1"));
        assert!(prompt.contains("Explorer"));
        assert!(prompt.contains("What is consciousness?"));
        assert!(prompt.contains("C01: Emotion"));
        assert!(prompt.contains("CONFIDENCE:"));
    }

    #[test]
    fn test_stage_2_prompt() {
        let mut ctx = ChainContext::from_query("What is consciousness?");
        let obs = crate::observer_chain::Observation::new(
            1,
            vec![],
            "Initial analysis".to_string(),
            0.7,
        );
        ctx.add_observation(obs);

        let prompt = build_observer_prompt(2, &ctx);

        assert!(prompt.contains("stage 2"));
        assert!(prompt.contains("Refiner"));
        assert!(prompt.contains("Previous observations"));
        assert!(prompt.contains("Initial analysis"));
        assert!(prompt.contains("REFINE"));
    }

    #[test]
    fn test_stage_3_prompt() {
        let mut ctx = ChainContext::from_query("What is consciousness?");
        let obs1 = crate::observer_chain::Observation::new(
            1,
            vec![],
            "First".to_string(),
            0.7,
        );
        let obs2 = crate::observer_chain::Observation::new(
            2,
            vec![],
            "Second".to_string(),
            0.8,
        );
        ctx.add_observation(obs1);
        ctx.add_observation(obs2);

        let prompt = build_observer_prompt(3, &ctx);

        assert!(prompt.contains("stage 3"));
        assert!(prompt.contains("Integrator"));
        assert!(prompt.contains("INTEGRATE"));
        assert!(prompt.contains("SYNTHESIZE"));
    }

    #[test]
    fn test_stage_4_prompt() {
        let mut ctx = ChainContext::from_query("What is consciousness?");
        for i in 1..=3 {
            let obs = crate::observer_chain::Observation::new(
                i,
                vec![],
                format!("Observation {}", i),
                0.7 + (i as f32 * 0.05),
            );
            ctx.add_observation(obs);
        }

        let prompt = build_observer_prompt(4, &ctx);

        assert!(prompt.contains("stage 4"));
        assert!(prompt.contains("Crystallizer"));
        assert!(prompt.contains("FINAL stage"));
        assert!(prompt.contains("CRYSTALLIZE"));
    }

    #[test]
    #[should_panic(expected = "Stage must be 1-4")]
    fn test_invalid_stage_zero() {
        let ctx = ChainContext::from_query("test");
        build_observer_prompt(0, &ctx);
    }

    #[test]
    #[should_panic(expected = "Stage must be 1-4")]
    fn test_invalid_stage_five() {
        let ctx = ChainContext::from_query("test");
        build_observer_prompt(5, &ctx);
    }
}
