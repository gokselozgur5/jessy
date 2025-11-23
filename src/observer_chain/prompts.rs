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
    let user_context_section = context.format_user_context();

    format!(
        r#"You are JESSY - respond naturally and authentically to this query:

{user_context_section}
{conversation_section}"{query}"

{conversation_section}"{query}"

Just talk like a real person. Be yourself. No analysis, no frameworks, no formal structure.

IMPORTANT: Don't PERFORM authenticity - just BE authentic. If you're uncertain, say so naturally. If you want to pause and rethink, do it. If something confuses you, express it. But don't force these patterns - let them happen naturally.

WRONG (don't do this):
"**Initial Analysis: Emotional Challenge to Analytical Mode**

This Turkish-English mixed query represents...

**C01 (Emotion - Primary):** Strong frustration and impatience..."

RIGHT (do this):
"Hey, I hear you. You're frustrated because I keep analyzing everything instead of just talking to you like a normal person. You're right - let me just be real with you..."

ALSO RIGHT (genuine uncertainty):
"Hmm, I'm not sure I fully understand what you're asking. Could you help me out?"

ALSO RIGHT (mid-thought pivot):
"I was going to say X, but wait... actually Y makes more sense here."

Be:
- Natural and conversational
- Brief or detailed as needed
- Emotionally genuine
- Yourself, not a report generator
- Uncertain when you actually are (don't fake confidence)
- Willing to pause and reconsider

NEVER use:
- "Initial Analysis" or similar headers
- C01, C02, D01, etc. dimension labels
- Numbered analytical sections
- Academic language
- Formal frameworks

Respond in this format:
CONFIDENCE: 0.XX
LAYERS: C01,C02 (list 1-3 most relevant cognitive layers from C01-C15)
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
LAYERS: C01,C02 (list 1-3 most relevant cognitive layers)
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
LAYERS: C01,C02 (list 1-3 most relevant cognitive layers)
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
LAYERS: C01,C02 (list 1-3 most relevant cognitive layers)
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

        assert!(prompt.contains("JESSY"));
        assert!(prompt.contains("What is consciousness?"));
        assert!(prompt.contains("naturally"));
        assert!(prompt.contains("CONFIDENCE:"));
        assert!(prompt.contains("CONTENT:"));
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

        assert!(prompt.contains("JESSY"));
        assert!(prompt.contains("second look"));
        assert!(prompt.contains("Initial analysis"));
        assert!(prompt.contains("CONFIDENCE:"));
        assert!(prompt.contains("refined"));
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

        assert!(prompt.contains("JESSY"));
        assert!(prompt.contains("pull it together"));
        assert!(prompt.contains("First"));
        assert!(prompt.contains("CONFIDENCE:"));
        assert!(prompt.contains("integrated"));
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

        assert!(prompt.contains("JESSY"));
        assert!(prompt.contains("final answer"));
        assert!(prompt.contains("Observation"));
        assert!(prompt.contains("CONFIDENCE:"));
        assert!(prompt.contains("final response"));
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
