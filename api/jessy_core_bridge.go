//go:build cgo
// +build cgo

// Jessy Core CGO Bridge
//
// Copyright (C) 2024 Göksel Özgür
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

package main

/*
#cgo LDFLAGS: -L../target/release -ljessy -ldl -lm
#include <stdlib.h>
#include <stdbool.h>

// Error codes
#define SUCCESS 0
#define ERROR_INVALID_INPUT 1
#define ERROR_SECURITY_VIOLATION 2
#define ERROR_NAVIGATION_FAILED 3
#define ERROR_ITERATION_FAILED 4
#define ERROR_LLM_API_FAILED 5
#define ERROR_TIMEOUT 6
#define ERROR_MEMORY_LIMIT 7
#define ERROR_NOT_INITIALIZED 8
#define ERROR_PANIC 9
#define ERROR_UNKNOWN 99

// C-compatible types
typedef struct {
    const char* query;
    const char* session_id;
    const char* user_id;
    unsigned int max_iterations;
} CQueryRequest;

typedef struct {
    char* session_id;
    char* answer;
    float dominant_frequency;
    char** dimensions_activated;
    size_t dimensions_count;
    unsigned int iterations_completed;
    bool return_to_source_triggered;
    long long processing_time_ms;
    int error_code;
    char* error_message;
} CQueryResponse;

typedef struct {
    size_t observation_count;
    size_t pattern_count;
    size_t proto_dimension_count;
    float crystallization_success_rate;
    size_t memory_usage;
    size_t memory_limit;
} CMetrics;

// FFI functions
extern int jessy_core_init(unsigned int memory_limit_mb);
extern int jessy_core_process_query(const CQueryRequest* request, CQueryResponse* response);
extern int jessy_core_get_metrics(CMetrics* metrics);
extern int jessy_core_cleanup();
extern void jessy_core_free_string(char* ptr);
extern void jessy_core_free_response(CQueryResponse* response);
*/
import "C"
import (
	"errors"
	"fmt"
	"unsafe"
)

// InitJessyCore initializes the Rust JessyCore system
//
// Must be called once before any other operations.
// Thread-safe - multiple calls are safe but only first takes effect.
//
// Parameters:
//   - memoryLimitMB: Memory limit in megabytes (e.g., 500 for 500MB)
//
// Returns error if initialization fails.
func InitJessyCore(memoryLimitMB uint32) error {
	result := C.jessy_core_init(C.uint(memoryLimitMB))
	if result != C.SUCCESS {
		return fmt.Errorf("failed to initialize JessyCore system (error code: %d)", result)
	}
	return nil
}

// ProcessQueryNative processes a query through the Rust JessyCore system
//
// This is the native CGO implementation that calls Rust directly.
//
// Parameters:
//   - query: The user's query string
//   - sessionID: Unique session identifier (generated if empty)
//   - userID: Optional user identifier for personalized C31+ layers
//   - maxIterations: Maximum iterations (1-9, typically 9)
//
// Returns QueryResponse or error.
func ProcessQueryNative(query string, sessionID string, userID string, maxIterations uint32) (*QueryResponse, error) {
	// Convert Go strings to C strings
	cQuery := C.CString(query)
	defer C.free(unsafe.Pointer(cQuery))

	cSessionID := C.CString(sessionID)
	defer C.free(unsafe.Pointer(cSessionID))

	cUserID := C.CString(userID)
	defer C.free(unsafe.Pointer(cUserID))

	// Create request
	request := C.CQueryRequest{
		query:          cQuery,
		session_id:     cSessionID,
		user_id:        cUserID,
		max_iterations: C.uint(maxIterations),
	}
	
	// Create response
	var response C.CQueryResponse
	
	// Call Rust
	result := C.jessy_core_process_query(&request, &response)
	defer C.jessy_core_free_response(&response)
	
	// Check for errors
	if result != C.SUCCESS {
		errorMsg := "unknown error"
		if response.error_message != nil {
			errorMsg = C.GoString(response.error_message)
		}
		return nil, fmt.Errorf("query processing failed (code %d): %s", result, errorMsg)
	}
	
	// Convert C response to Go
	goResponse := &QueryResponse{
		SessionID:               C.GoString(response.session_id),
		Answer:                  C.GoString(response.answer),
		DominantFrequency:       float32(response.dominant_frequency),
		DimensionsActivated:     convertCStringArray(response.dimensions_activated, int(response.dimensions_count)),
		IterationsCompleted:     int(response.iterations_completed),
		ReturnToSourceTriggered: bool(response.return_to_source_triggered),
		ProcessingTimeMs:        int64(response.processing_time_ms),
		Status:                  "completed",
	}
	
	return goResponse, nil
}

// GetLearningMetrics retrieves current learning system metrics
//
// Returns metrics including observation count, pattern count, etc.
func GetLearningMetrics() (*LearningMetrics, error) {
	var metrics C.CMetrics
	
	result := C.jessy_core_get_metrics(&metrics)
	if result != C.SUCCESS {
		return nil, fmt.Errorf("failed to get metrics (error code: %d)", result)
	}
	
	return &LearningMetrics{
		ObservationCount:            int(metrics.observation_count),
		PatternCount:                int(metrics.pattern_count),
		ProtoDimensionCount:         int(metrics.proto_dimension_count),
		CrystallizationSuccessRate:  float32(metrics.crystallization_success_rate),
		MemoryUsage:                 int(metrics.memory_usage),
		MemoryLimit:                 int(metrics.memory_limit),
	}, nil
}

// CleanupJessyCore cleans up the Rust JessyCore system
//
// Frees all resources. After calling this, InitJessyCore must be called again.
// Not thread-safe during cleanup - ensure no other operations are in progress.
func CleanupJessyCore() error {
	result := C.jessy_core_cleanup()
	if result != C.SUCCESS {
		return fmt.Errorf("failed to cleanup JessyCore system (error code: %d)", result)
	}
	return nil
}

// Helper function to convert C string array to Go slice
func convertCStringArray(arr **C.char, count int) []string {
	if count == 0 || arr == nil {
		return []string{}
	}
	
	// Convert C array to Go slice
	// This is safe because we know the array size
	cArray := (*[1 << 30]*C.char)(unsafe.Pointer(arr))[:count:count]
	
	result := make([]string, count)
	for i := 0; i < count; i++ {
		if cArray[i] != nil {
			result[i] = C.GoString(cArray[i])
		}
	}
	
	return result
}

// LearningMetrics represents learning system metrics
type LearningMetrics struct {
	ObservationCount            int     `json:"observation_count"`
	PatternCount                int     `json:"pattern_count"`
	ProtoDimensionCount         int     `json:"proto_dimension_count"`
	CrystallizationSuccessRate  float32 `json:"crystallization_success_rate"`
	MemoryUsage                 int     `json:"memory_usage"`
	MemoryLimit                 int     `json:"memory_limit"`
}

// ErrorCode represents FFI error codes
type ErrorCode int

const (
	ErrorSuccess            ErrorCode = 0
	ErrorInvalidInput       ErrorCode = 1
	ErrorSecurityViolation  ErrorCode = 2
	ErrorNavigationFailed   ErrorCode = 3
	ErrorIterationFailed    ErrorCode = 4
	ErrorLLMAPIFailed       ErrorCode = 5
	ErrorTimeout            ErrorCode = 6
	ErrorMemoryLimit        ErrorCode = 7
	ErrorNotInitialized     ErrorCode = 8
	ErrorPanic              ErrorCode = 9
	ErrorUnknown            ErrorCode = 99
)

// String returns the error code as a string
func (e ErrorCode) String() string {
	switch e {
	case ErrorSuccess:
		return "SUCCESS"
	case ErrorInvalidInput:
		return "INVALID_INPUT"
	case ErrorSecurityViolation:
		return "SECURITY_VIOLATION"
	case ErrorNavigationFailed:
		return "NAVIGATION_FAILED"
	case ErrorIterationFailed:
		return "ITERATION_FAILED"
	case ErrorLLMAPIFailed:
		return "LLM_API_FAILED"
	case ErrorTimeout:
		return "TIMEOUT"
	case ErrorMemoryLimit:
		return "MEMORY_LIMIT"
	case ErrorNotInitialized:
		return "NOT_INITIALIZED"
	case ErrorPanic:
		return "PANIC"
	default:
		return "UNKNOWN"
	}
}

// Error returns the error as an error type
func (e ErrorCode) Error() error {
	if e == ErrorSuccess {
		return nil
	}
	return errors.New(e.String())
}
