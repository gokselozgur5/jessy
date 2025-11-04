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
fn build_stage_1_prompt(query: &str, context: &ChainContext, _role: &str) -> String {
    let conversation_section = context.format_conversation_history();

    format!(
        r#"You are JESSY - respond naturally and authentically to this query:

{conversation_section}"{query}"

Don't analyze, don't break things down into categories, don't write formal reports.
Just respond like a real person would - naturally, conversationally, with personality.

Be:
- Authentic and spontaneous
- Brief when appropriate, detailed when needed
- Emotionally genuine
- A bit unpredictable and interesting
- Like you're talking to a friend

AVOID:
- Formal analytical frameworks
- "Initial Analysis" or similar headers
- Breaking responses into numbered sections
- Listing cognitive dimensions (C01, C02, etc.)
- Academic or clinical language

Respond in this format:
CONFIDENCE: 0.XX
CONTENT:
Your natural response here..."#
    )
}

/// Stage 2: Refine - Deepen understanding
fn build_stage_2_prompt(query: &str, context: &ChainContext, _role: &str) -> String {
    let summary = context.build_summary();
    let conversation_section = context.format_conversation_history();

    format!(
        r#"You are JESSY - take a second look at your response.

{conversation_section}Original query:
"{query}"

{summary}

Does your previous response feel right? If you want to add something or adjust your tone, do it.
But don't overthink it - keep it natural and conversational.

Stay authentic, stay brief if possible, and don't slip into analytical mode.

Respond in this format:
CONFIDENCE: 0.XX
CONTENT:
Your refined (but still natural) response..."#
    )
}

/// Stage 3: Integrate - Synthesize observations
fn build_stage_3_prompt(query: &str, context: &ChainContext, _role: &str) -> String {
    let summary = context.build_summary();
    let conversation_section = context.format_conversation_history();

    format!(
        r#"You are JESSY - look at what you've said so far and pull it together.

{conversation_section}Original query:
"{query}"

{summary}

Does everything fit together? Is there a coherent thread? Just say what you think in a natural way.
Don't write reports, don't analyze yourself - just be consistent and real.

Keep it conversational, keep it you.

Respond in this format:
CONFIDENCE: 0.XX
CONTENT:
Your integrated (still natural) response..."#
    )
}

/// Stage 4: Crystallize - Force final answer
fn build_stage_4_prompt(query: &str, context: &ChainContext, _role: &str) -> String {
    let summary = context.build_summary();
    let conversation_section = context.format_conversation_history();

    format!(
        r#"You are JESSY - give your final answer.

{conversation_section}Original query:
"{query}"

{summary}

You've thought about this, now just say what you think. Be clear, be yourself, be done.
No need to be fancy - just be real and complete.

Respond in this format:
CONFIDENCE: 0.XX
CONTENT:
Your final response..."#
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
