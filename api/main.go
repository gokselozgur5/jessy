// Jessy API Server
//
// Copyright (C) 2024 Göksel Özgür
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

package main

import (
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/gofiber/fiber/v2"
	"github.com/gofiber/fiber/v2/middleware/cors"
	"github.com/gofiber/fiber/v2/middleware/logger"
	"github.com/gofiber/fiber/v2/middleware/recover"
	"github.com/gofiber/websocket/v2"
	"github.com/rs/zerolog"
	zlog "github.com/rs/zerolog/log"
)

func main() {
	// Configure structured logging
	zerolog.TimeFieldFormat = zerolog.TimeFormatUnix
	
	// Use JSON logging in production, pretty console in development
	if os.Getenv("GO_ENV") == "production" {
		zlog.Logger = zerolog.New(os.Stdout).With().Timestamp().Caller().Logger()
	} else {
		zlog.Logger = zlog.Output(zerolog.ConsoleWriter{Out: os.Stderr, TimeFormat: "15:04:05"})
	}
	
	// Set log level from environment
	logLevel := os.Getenv("LOG_LEVEL")
	switch logLevel {
	case "debug":
		zerolog.SetGlobalLevel(zerolog.DebugLevel)
	case "info":
		zerolog.SetGlobalLevel(zerolog.InfoLevel)
	case "warn":
		zerolog.SetGlobalLevel(zerolog.WarnLevel)
	case "error":
		zerolog.SetGlobalLevel(zerolog.ErrorLevel)
	default:
		zerolog.SetGlobalLevel(zerolog.InfoLevel)
	}

	// Initialize Fiber app with optimized settings
	app := fiber.New(fiber.Config{
		Prefork:       false, // Disable for development
		CaseSensitive: true,
		StrictRouting: true,
		ServerHeader:  "Resonance API",
		AppName:       "Resonance v1.0.0",
		BodyLimit:     4 * 1024 * 1024, // 4MB max request size
	})

	// Middleware
	app.Use(recover.New())
	app.Use(logger.New(logger.Config{
		Format: "${time} ${status} - ${method} ${path} ${latency}\n",
	}))
	app.Use(cors.New(cors.Config{
		AllowOrigins: "*",
		AllowMethods: "GET,POST,PUT,DELETE,OPTIONS",
		AllowHeaders: "Origin,Content-Type,Accept,Authorization",
	}))

	// Initialize consciousness system
	consciousness, err := NewConsciousnessService()
	if err != nil {
		log.Fatal("Failed to initialize consciousness system:", err)
	}
	defer consciousness.Close()

	// Health check (at root /api/health for docker healthcheck)
	app.Get("/api/health", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{
			"status":  "healthy",
			"service": "jessy-api",
			"version": "1.0.0",
		})
	})

	// API Routes
	api := app.Group("/api/v1")
	
	// Health check (also available at v1)
	api.Get("/health", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{
			"status":  "healthy",
			"service": "jessy-api",
			"version": "1.0.0",
		})
	})

	// Process query endpoint
	api.Post("/query", consciousness.ProcessQuery)
	
	// Get system status
	api.Get("/status", consciousness.GetStatus)
	
	// WebSocket for real-time iteration streaming
	api.Get("/stream", websocket.New(consciousness.StreamIterations))

	// Static files for web interface
	app.Static("/", "./web/dist")

	// Start server
	port := os.Getenv("PORT")
	if port == "" {
		port = "3000"
	}

	// Start server in a goroutine
	go func() {
		zlog.Info().Str("port", port).Msg("Starting Jessy API server")
		if err := app.Listen(":" + port); err != nil {
			zlog.Fatal().Err(err).Msg("Server failed to start")
		}
	}()

	// Wait for interrupt signal for graceful shutdown
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, os.Interrupt, syscall.SIGTERM)
	sig := <-quit

	zlog.Info().
		Str("signal", sig.String()).
		Msg("Received shutdown signal, initiating graceful shutdown")

	// Gracefully shutdown with timeout
	zlog.Info().Msg("Shutting down HTTP server...")
	if err := app.ShutdownWithTimeout(30 * time.Second); err != nil {
		zlog.Error().Err(err).Msg("Server forced to shutdown")
	} else {
		zlog.Info().Msg("HTTP server stopped gracefully")
	}

	// Close consciousness service
	zlog.Info().Msg("Closing consciousness service...")
	if err := consciousness.Close(); err != nil {
		zlog.Error().Err(err).Msg("Error closing consciousness service")
	} else {
		zlog.Info().Msg("Consciousness service closed successfully")
	}

	zlog.Info().Msg("Graceful shutdown complete")
}