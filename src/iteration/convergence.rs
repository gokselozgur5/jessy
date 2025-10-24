//! Convergence detection for iteration processing

use super::IterationStep;

/// Convergence detector for iteration chains
pub struct ConvergenceDetector {
    threshold: f32,
    window_size: usize,
}

impl ConvergenceDetector {
    /// Create new convergence detector
    pub fn new(threshold: f32) -> Self {
        Self {
            threshold,
            window_size: 2, // Compare last 2 iterations
        }
    }
    
    /// Check if iterations have converged
    pub fn check_convergence(&self, steps: &[IterationStep]) -> ConvergenceState {
        if steps.len() < self.window_size {
            return ConvergenceState::InProgress;
        }
        
        // Get last N steps
        let recent_steps = &steps[steps.len() - self.window_size..];
        
        // Check similarity between consecutive steps
        for window in recent_steps.windows(2) {
            if let Some(similarity) = window[1].similarity_to_previous {
                if similarity >= self.threshold {
                    return ConvergenceState::Converged {
                        at_iteration: window[1].iteration_number,
                        similarity,
                    };
                }
            }
        }
        
        ConvergenceState::InProgress
    }
    
    /// Detect if analysis paralysis is occurring
    pub fn detect_paralysis(&self, steps: &[IterationStep]) -> bool {
        if steps.len() < 5 {
            return false;
        }
        
        // Check if confidence is decreasing
        let recent_confidences: Vec<f32> = steps[steps.len() - 3..]
            .iter()
            .map(|s| s.confidence)
            .collect();
        
        // If confidence is consistently low or decreasing, might be paralysis
        let avg_confidence: f32 = recent_confidences.iter().sum::<f32>() / recent_confidences.len() as f32;
        
        avg_confidence < 0.5
    }
}

/// State of convergence detection
#[derive(Debug, Clone, PartialEq)]
pub enum ConvergenceState {
    /// Still iterating, no convergence yet
    InProgress,
    
    /// Converged at specific iteration
    Converged {
        at_iteration: usize,
        similarity: f32,
    },
    
    /// Analysis paralysis detected
    Paralysis,
}

impl ConvergenceState {
    /// Check if converged
    pub fn is_converged(&self) -> bool {
        matches!(self, ConvergenceState::Converged { .. })
    }
    
    /// Check if paralysis
    pub fn is_paralysis(&self) -> bool {
        matches!(self, ConvergenceState::Paralysis)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Frequency;
    
    #[test]
    fn test_convergence_detection() {
        let detector = ConvergenceDetector::new(0.95);
        
        let mut steps = vec![];
        
        // First iteration
        steps.push(IterationStep::new(
            1,
            "First thought".to_string(),
            Frequency::new(1.0),
            0.5,
        ));
        
        assert_eq!(detector.check_convergence(&steps), ConvergenceState::InProgress);
        
        // Second iteration with high similarity
        steps.push(
            IterationStep::new(
                2,
                "First thought again".to_string(),
                Frequency::new(1.0),
                0.6,
            ).with_similarity(0.96)
        );
        
        let state = detector.check_convergence(&steps);
        assert!(state.is_converged());
    }
    
    #[test]
    fn test_paralysis_detection() {
        let detector = ConvergenceDetector::new(0.95);
        
        let mut steps = vec![];
        
        // Add steps with decreasing confidence
        for i in 1..=6 {
            steps.push(IterationStep::new(
                i,
                format!("Thought {}", i),
                Frequency::new(1.0),
                0.4, // Low confidence
            ));
        }
        
        assert!(detector.detect_paralysis(&steps));
    }
}
