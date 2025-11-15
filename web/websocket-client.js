/**
 * JESSY WebSocket Client
 * 
 * Handles real-time streaming communication with Jessy backend
 * Features:
 * - Automatic reconnection with exponential backoff
 * - Token-by-token streaming with natural rhythm
 * - Thinking marker visualization
 * - Stage transition notifications
 */

class JessyWebSocket {
    constructor(url, options = {}) {
        this.url = url;
        this.ws = null;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = options.maxReconnectAttempts || 5;
        this.reconnectDelay = options.reconnectDelay || 1000;
        this.userId = options.userId || null;
        this.sessionId = options.sessionId || null;
        
        // Callbacks
        this.onToken = options.onToken || (() => {});
        this.onThinkingMarker = options.onThinkingMarker || (() => {});
        this.onStageTransition = options.onStageTransition || (() => {});
        this.onTyping = options.onTyping || (() => {});
        this.onComplete = options.onComplete || (() => {});
        this.onError = options.onError || (() => {});
        this.onConnectionChange = options.onConnectionChange || (() => {});
    }

    /**
     * Connect to WebSocket server
     */
    connect() {
        console.log('🔌 Connecting to WebSocket:', this.url);
        
        try {
            this.ws = new WebSocket(this.url);
            
            this.ws.onopen = () => {
                console.log('✅ WebSocket connected');
                this.reconnectAttempts = 0;
                this.onConnectionChange(true);
            };
            
            this.ws.onmessage = (event) => {
                this.handleMessage(event.data);
            };
            
            this.ws.onerror = (error) => {
                console.error('❌ WebSocket error:', error);
                this.onError(error);
            };
            
            this.ws.onclose = () => {
                console.log('🔌 WebSocket disconnected');
                this.onConnectionChange(false);
                this.attemptReconnect();
            };
        } catch (error) {
            console.error('Failed to create WebSocket:', error);
            this.onError(error);
            this.attemptReconnect();
        }
    }

    /**
     * Handle incoming WebSocket message
     */
    handleMessage(data) {
        try {
            const message = JSON.parse(data);
            
            switch (message.type) {
                case 'token':
                    this.onToken(message.content, message.token_type);
                    break;
                    
                case 'thinking_marker':
                    this.onThinkingMarker(message.marker_type, message.content);
                    break;
                    
                case 'stage_transition':
                    this.onStageTransition(message.from_stage, message.to_stage);
                    break;
                    
                case 'typing':
                    this.onTyping(message.is_typing);
                    break;
                    
                case 'complete':
                    this.sessionId = message.session_id;
                    this.onComplete(message.session_id);
                    break;
                    
                case 'error':
                    console.error('Server error:', message.message);
                    this.onError(new Error(message.message));
                    break;
                    
                case 'pong':
                    // Heartbeat response
                    break;
                    
                default:
                    console.warn('Unknown message type:', message.type);
            }
        } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
        }
    }

    /**
     * Send chat message to server
     */
    sendMessage(message) {
        if (!this.isConnected()) {
            console.error('Cannot send message: WebSocket not connected');
            return false;
        }
        
        const payload = {
            type: 'chat',
            message: message,
            user_id: this.userId,
            session_id: this.sessionId
        };
        
        this.ws.send(JSON.stringify(payload));
        return true;
    }

    /**
     * Send ping to server
     */
    ping() {
        if (this.isConnected()) {
            this.ws.send(JSON.stringify({ type: 'ping' }));
        }
    }

    /**
     * Check if WebSocket is connected
     */
    isConnected() {
        return this.ws && this.ws.readyState === WebSocket.OPEN;
    }

    /**
     * Attempt to reconnect with exponential backoff
     */
    attemptReconnect() {
        if (this.reconnectAttempts >= this.maxReconnectAttempts) {
            console.error('Max reconnection attempts reached');
            this.onError(new Error('Failed to reconnect after multiple attempts'));
            return;
        }
        
        this.reconnectAttempts++;
        const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1);
        
        console.log(`🔄 Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
        
        setTimeout(() => {
            this.connect();
        }, delay);
    }

    /**
     * Close WebSocket connection
     */
    close() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
    }
}

/**
 * Token renderer with type-specific styling
 */
class TokenRenderer {
    constructor(container) {
        this.container = container;
        this.currentMessage = null;
    }

    /**
     * Start a new message
     */
    startMessage() {
        this.currentMessage = document.createElement('div');
        this.currentMessage.className = 'message jessy-message';
        this.container.appendChild(this.currentMessage);
        this.scrollToBottom();
    }

    /**
     * Append token to current message
     */
    appendToken(content, tokenType = 'normal') {
        if (!this.currentMessage) {
            this.startMessage();
        }
        
        const span = document.createElement('span');
        span.className = `token token-${tokenType}`;
        span.textContent = content;
        
        this.currentMessage.appendChild(span);
        this.scrollToBottom();
    }

    /**
     * Show thinking marker
     */
    showThinkingMarker(markerType, content) {
        if (!this.currentMessage) {
            this.startMessage();
        }
        
        const marker = document.createElement('span');
        marker.className = `thinking-marker thinking-marker-${markerType}`;
        marker.textContent = content || this.getMarkerSymbol(markerType);
        
        this.currentMessage.appendChild(marker);
        this.scrollToBottom();
    }

    /**
     * Get symbol for thinking marker type
     */
    getMarkerSymbol(markerType) {
        const symbols = {
            pause: '...',
            pivot: '↻',
            correction: '✓',
            uncertainty: '?',
            insight: '💡'
        };
        return symbols[markerType] || '•';
    }

    /**
     * Show stage transition
     */
    showStageTransition(fromStage, toStage) {
        const transition = document.createElement('div');
        transition.className = 'stage-transition';
        transition.textContent = `${fromStage} → ${toStage}`;
        this.container.appendChild(transition);
        this.scrollToBottom();
    }

    /**
     * Complete current message
     */
    completeMessage() {
        this.currentMessage = null;
    }

    /**
     * Scroll to bottom of container
     */
    scrollToBottom() {
        this.container.scrollTop = this.container.scrollHeight;
    }
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { JessyWebSocket, TokenRenderer };
}
