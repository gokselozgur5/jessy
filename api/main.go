package main

import (
	"log"
	"os"

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
	zlog.Logger = zlog.Output(zerolog.ConsoleWriter{Out: os.Stderr})

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

	// API Routes
	api := app.Group("/api/v1")
	
	// Health check
	api.Get("/health", func(c *fiber.Ctx) error {
		return c.JSON(fiber.Map{
			"status": "healthy",
			"service": "resonance-api",
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
		port = "8080"
	}

	zlog.Info().Str("port", port).Msg("Starting Resonance API server")
	log.Fatal(app.Listen(":" + port))
}