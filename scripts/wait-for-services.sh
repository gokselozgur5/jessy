#!/bin/bash
# Wait for Docker Compose services to be healthy
# Usage: ./wait-for-services.sh service1 service2 ...

set -e

TIMEOUT=${TIMEOUT:-60}
INTERVAL=2
elapsed=0

echo "⏳ Waiting for services to be healthy (timeout: ${TIMEOUT}s)..."

while [ $elapsed -lt $TIMEOUT ]; do
    all_healthy=true
    
    for service in "$@"; do
        status=$(docker-compose ps --format json | jq -r ".[] | select(.Service == \"$service\") | .Health" 2>/dev/null || echo "")
        
        if [ "$status" = "healthy" ]; then
            echo "✓ $service is healthy"
        elif [ "$status" = "unhealthy" ]; then
            echo "✗ $service is unhealthy"
            docker-compose logs --tail=20 "$service"
            exit 1
        else
            all_healthy=false
            echo "⏳ Waiting for $service... (status: ${status:-starting})"
        fi
    done
    
    if [ "$all_healthy" = true ]; then
        echo "✅ All services are healthy!"
        exit 0
    fi
    
    sleep $INTERVAL
    elapsed=$((elapsed + INTERVAL))
done

echo "❌ Timeout waiting for services to become healthy"
docker-compose ps
exit 1
