//! Pattern detection from observations

use super::{Observation, DetectedPattern, PatternId, LearningConfig};
use std::collections::HashMap;

/// Pattern detector that identifies recurring themes from observations
pub struct PatternDetector {
    config: LearningConfig,
    next_pattern_id: u64,
}

impl PatternDetector {
    /// Create new pattern detector
    pub fn new(config: LearningConfig) -> Self {
        Self {
            config,
            next_pattern_id: 1,
        }
    }
    
    /// Detect patterns from observations
    ///
    /// Returns patterns that meet minimum observation count and confidence threshold.
    pub fn detect_patterns(&mut self, observations: &[Observation]) -> Vec<DetectedPattern> {
        if observations.len() < self.config.min_observations {
            return vec![];
        }
        
        let min_observations = self.config.min_observations;
        let confidence_threshold = self.config.confidence_threshold;
        
        // Cluster observations by keyword similarity
        let clusters = self.cluster_by_keywords(observations);
        
        // Analyze each cluster for patterns
        let mut patterns = Vec::new();
        for cluster in clusters {
            if cluster.len() >= min_observations {
                let confidence = self.calculate_confidence(&cluster);
                
                if confidence >= confidence_threshold {
                    patterns.push(self.create_pattern(cluster, confidence));
                }
            }
        }
        
        patterns
    }
    
    /// Cluster observations by keyword similarity
    fn cluster_by_keywords(&self, observations: &[Observation]) -> Vec<Vec<&Observation>> {
        let mut clusters: Vec<Vec<&Observation>> = Vec::new();
        
        for obs in observations {
            let mut found_cluster = false;
            
            // Try to add to existing cluster
            for cluster in &mut clusters {
                if let Some(first) = cluster.first() {
                    let overlap = self.keyword_overlap(obs, first);
                    if overlap > 0.5 {
                        cluster.push(obs);
                        found_cluster = true;
                        break;
                    }
                }
            }
            
            // Create new cluster if no match
            if !found_cluster {
                clusters.push(vec![obs]);
            }
        }
        
        clusters
    }
    
    /// Calculate keyword overlap ratio between two observations
    fn keyword_overlap(&self, obs1: &Observation, obs2: &Observation) -> f32 {
        if obs1.keywords.is_empty() || obs2.keywords.is_empty() {
            return 0.0;
        }
        
        let set1: std::collections::HashSet<_> = obs1.keywords.iter().collect();
        let set2: std::collections::HashSet<_> = obs2.keywords.iter().collect();
        
        let intersection = set1.intersection(&set2).count();
        let union = set1.union(&set2).count();
        
        if union == 0 {
            0.0
        } else {
            intersection as f32 / union as f32
        }
    }
    
    /// Calculate confidence score for a cluster
    fn calculate_confidence(&self, cluster: &[&Observation]) -> f32 {
        let keyword_consistency = self.calculate_keyword_consistency(cluster);
        let frequency_consistency = self.calculate_frequency_consistency(cluster);
        let temporal_consistency = self.calculate_temporal_consistency(cluster);
        
        // Weighted average
        (keyword_consistency * 0.5) + 
        (frequency_consistency * 0.3) + 
        (temporal_consistency * 0.2)
    }
    
    /// Calculate keyword consistency (how similar keywords are)
    fn calculate_keyword_consistency(&self, cluster: &[&Observation]) -> f32 {
        if cluster.len() < 2 {
            return 1.0;
        }
        
        let mut total_overlap = 0.0;
        let mut comparisons = 0;
        
        for i in 0..cluster.len() {
            for j in (i + 1)..cluster.len() {
                total_overlap += self.keyword_overlap(cluster[i], cluster[j]);
                comparisons += 1;
            }
        }
        
        if comparisons == 0 {
            0.0
        } else {
            total_overlap / comparisons as f32
        }
    }
    
    /// Calculate frequency consistency (how similar frequencies are)
    fn calculate_frequency_consistency(&self, cluster: &[&Observation]) -> f32 {
        if cluster.is_empty() {
            return 0.0;
        }
        
        let frequencies: Vec<f32> = cluster.iter()
            .map(|obs| obs.frequency.hz())
            .collect();
        
        let mean = frequencies.iter().sum::<f32>() / frequencies.len() as f32;
        let variance = frequencies.iter()
            .map(|f| (f - mean).powi(2))
            .sum::<f32>() / frequencies.len() as f32;
        
        // Lower variance = higher consistency
        // Use inverse with clamping
        let std_dev = variance.sqrt();
        if std_dev < 0.1 {
            1.0
        } else {
            (1.0 / (1.0 + std_dev)).min(1.0)
        }
    }
    
    /// Calculate temporal consistency (how evenly distributed over time)
    fn calculate_temporal_consistency(&self, cluster: &[&Observation]) -> f32 {
        if cluster.len() < 2 {
            return 1.0;
        }
        
        // For now, simple heuristic: more observations = more consistent
        // Could be enhanced with actual temporal distribution analysis
        let ratio = cluster.len() as f32 / self.config.min_observations as f32;
        ratio.min(1.0)
    }
    
    /// Create pattern from cluster
    fn create_pattern(&mut self, cluster: Vec<&Observation>, confidence: f32) -> DetectedPattern {
        let pattern_id = PatternId(self.next_pattern_id);
        self.next_pattern_id += 1;
        
        // Extract common keywords
        let mut keyword_counts: HashMap<String, usize> = HashMap::new();
        for obs in &cluster {
            for keyword in &obs.keywords {
                *keyword_counts.entry(keyword.clone()).or_insert(0) += 1;
            }
        }
        
        // Keep keywords that appear in >50% of observations
        let threshold = cluster.len() / 2;
        let keywords: Vec<String> = keyword_counts
            .into_iter()
            .filter(|(_, count)| *count > threshold)
            .map(|(keyword, _)| keyword)
            .collect();
        
        // Calculate frequency range
        let frequencies: Vec<f32> = cluster.iter()
            .map(|obs| obs.frequency.hz())
            .collect();
        let min_freq = frequencies.iter().copied().fold(f32::INFINITY, f32::min);
        let max_freq = frequencies.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        
        DetectedPattern {
            pattern_id,
            keywords,
            frequency_range: (min_freq, max_freq),
            observation_count: cluster.len(),
            confidence,
            suggested_dimension: None, // Will be set later if needed
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{DimensionId, Frequency};
    
    fn create_test_observation(keywords: Vec<&str>, frequency: f32) -> Observation {
        Observation::new(
            "test query".to_string(),
            vec![DimensionId(1)],
            keywords.iter().map(|s| s.to_string()).collect(),
            Frequency::new(frequency),
        )
    }
    
    #[test]
    fn test_pattern_detector_creation() {
        let config = LearningConfig::default();
        let detector = PatternDetector::new(config);
        assert_eq!(detector.next_pattern_id, 1);
    }
    
    #[test]
    fn test_detect_patterns_insufficient_observations() {
        let config = LearningConfig {
            min_observations: 50,
            ..Default::default()
        };
        let mut detector = PatternDetector::new(config);
        
        let observations = vec![
            create_test_observation(vec!["test"], 1.0),
        ];
        
        let patterns = detector.detect_patterns(&observations);
        assert!(patterns.is_empty());
    }
    
    #[test]
    fn test_detect_patterns_low_confidence() {
        let config = LearningConfig {
            min_observations: 3,
            confidence_threshold: 0.85,
            ..Default::default()
        };
        let mut detector = PatternDetector::new(config);
        
        // Create observations with very different keywords (low confidence)
        let observations = vec![
            create_test_observation(vec!["apple"], 1.0),
            create_test_observation(vec!["banana"], 1.0),
            create_test_observation(vec!["cherry"], 1.0),
        ];
        
        let patterns = detector.detect_patterns(&observations);
        // Should not detect pattern due to low confidence
        assert!(patterns.is_empty() || patterns[0].confidence < 0.85);
    }
    
    #[test]
    fn test_detect_patterns_high_confidence() {
        let config = LearningConfig {
            min_observations: 3,
            confidence_threshold: 0.85,
            ..Default::default()
        };
        let mut detector = PatternDetector::new(config);
        
        // Create observations with similar keywords (high confidence)
        let observations = vec![
            create_test_observation(vec!["emotion", "feeling", "happy"], 1.0),
            create_test_observation(vec!["emotion", "feeling", "sad"], 1.0),
            create_test_observation(vec!["emotion", "feeling", "joy"], 1.0),
        ];
        
        let patterns = detector.detect_patterns(&observations);
        assert!(!patterns.is_empty());
        assert!(patterns[0].confidence >= 0.85);
        assert!(patterns[0].keywords.contains(&"emotion".to_string()));
        assert!(patterns[0].keywords.contains(&"feeling".to_string()));
    }
    
    #[test]
    fn test_keyword_overlap() {
        let config = LearningConfig::default();
        let detector = PatternDetector::new(config);
        
        let obs1 = create_test_observation(vec!["a", "b", "c"], 1.0);
        let obs2 = create_test_observation(vec!["b", "c", "d"], 1.0);
        
        let overlap = detector.keyword_overlap(&obs1, &obs2);
        // Intersection: {b, c} = 2, Union: {a, b, c, d} = 4
        assert!((overlap - 0.5).abs() < 0.01);
    }
    
    #[test]
    fn test_keyword_consistency() {
        let config = LearningConfig::default();
        let detector = PatternDetector::new(config);
        
        let obs1 = create_test_observation(vec!["a", "b"], 1.0);
        let obs2 = create_test_observation(vec!["a", "b"], 1.0);
        let obs3 = create_test_observation(vec!["a", "b"], 1.0);
        let observations = vec![&obs1, &obs2, &obs3];
        
        let consistency = detector.calculate_keyword_consistency(&observations);
        assert!(consistency > 0.9); // Very consistent
    }
    
    #[test]
    fn test_frequency_consistency() {
        let config = LearningConfig::default();
        let detector = PatternDetector::new(config);
        
        let obs1 = create_test_observation(vec!["test"], 1.0);
        let obs2 = create_test_observation(vec!["test"], 1.1);
        let obs3 = create_test_observation(vec!["test"], 0.9);
        let observations = vec![&obs1, &obs2, &obs3];
        
        let consistency = detector.calculate_frequency_consistency(&observations);
        assert!(consistency > 0.5); // Reasonably consistent
    }
    
    #[test]
    fn test_pattern_creation() {
        let config = LearningConfig::default();
        let mut detector = PatternDetector::new(config);
        
        let obs1 = create_test_observation(vec!["emotion", "happy"], 1.0);
        let obs2 = create_test_observation(vec!["emotion", "sad"], 1.5);
        let cluster = vec![&obs1, &obs2];
        
        let pattern = detector.create_pattern(cluster, 0.9);
        
        assert_eq!(pattern.pattern_id, PatternId(1));
        assert!(pattern.keywords.contains(&"emotion".to_string()));
        assert_eq!(pattern.observation_count, 2);
        assert_eq!(pattern.confidence, 0.9);
        assert_eq!(pattern.frequency_range, (1.0, 1.5));
    }
}
