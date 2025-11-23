// JESSY Web Chat - Frontend JavaScript
// Global shared session - everyone talks to the same JESSY
let sessionId = 'global-jessy-session';

// API Base URL - auto-detect environment
const API_BASE_URL = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1'
    ? '' // Localhost: same origin
    : 'https://jessy-backend.fly.dev'; // Production: Fly.io backend

// WebSocket URL
const WS_URL = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1'
    ? 'ws://localhost:8080/api/ws'
    : 'wss://jessy-backend.fly.dev/api/ws';

// DOM elements
const chatForm = document.getElementById('chatForm');
const messageInput = document.getElementById('messageInput');
const sendButton = document.getElementById('sendButton');
const chatMessages = document.getElementById('chatMessages');
const dimensionList = document.getElementById('dimensionList');

// WebSocket client
let jessyWs = null;
let tokenRenderer = null;
let useWebSocket = true; // Try WebSocket first, fallback to HTTP

// Cognitive Layer names mapping
const cognitiveLayerNames = {
    1: 'Emotion',
    2: 'Cognition',
    3: 'Intention',
    4: 'Social',
    5: 'Temporal',
    6: 'Philosophy',
    7: 'Technical',
    8: 'Creative',
    9: 'Ethical',
    10: 'Meta',
    11: 'Ecological',
    12: 'Positivity',
    13: 'Balance',
    14: 'Security'
};

// Initialize
document.addEventListener('DOMContentLoaded', () => {
    messageInput.focus();

    // Check health
    checkHealth();

    // Initialize WebSocket connection
    initializeWebSocket();

    // Add connection status indicator
    addConnectionStatusIndicator();

    // Log session info
    console.log('🌐 Connected to global JESSY session');
});

// Check API health
async function checkHealth() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/health`);
        const data = await response.json();
        console.log('✅ API Health:', data);
    } catch (error) {
        console.error('❌ API Health check failed:', error);
    }
}

// Handle form submission
chatForm.addEventListener('submit', async (e) => {
    e.preventDefault();

    const message = messageInput.value.trim();
    if (!message) return;

    // Clear input
    messageInput.value = '';

    // Add user message to chat
    addMessage('user', message);

    // Disable input while processing
    setInputEnabled(false);

    // Try WebSocket first, fallback to HTTP
    if (useWebSocket && jessyWs && jessyWs.isConnected()) {
        // Send via WebSocket for real-time streaming
        jessyWs.sendMessage(message);
    } else {
        // Fallback to HTTP POST
        await sendMessageHTTP(message);
    }
});

// Send message via HTTP (fallback)
async function sendMessageHTTP(message) {
    const loadingId = addLoadingIndicator();

    try {
        const response = await fetch(`${API_BASE_URL}/api/chat`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                message,
                session_id: sessionId
            })
        });

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }

        const data = await response.json();

        // Remove loading indicator
        removeLoadingIndicator(loadingId);

        // Add assistant response
        addMessage('assistant', data.response);

        // Update cognitive layer state
        updateDimensionalState(data.dimensions_activated, data.selection_duration_ms, data.contexts_loaded);

    } catch (error) {
        console.error('Error:', error);
        removeLoadingIndicator(loadingId);
        addMessage('error', `Error: ${error.message}`);
    } finally {
        setInputEnabled(true);
        messageInput.focus();
    }
}

// Add message to chat
function addMessage(role, content) {
    const messageDiv = document.createElement('div');
    messageDiv.className = `message-${role} mb-4`;

    if (role === 'user') {
        messageDiv.innerHTML = `
            <div class="flex justify-end">
                <div class="max-w-3xl">
                    <div class="px-4 py-3 bg-purple-600 bg-opacity-50 border border-purple-500/50 rounded-lg text-white">
                        ${escapeHtml(content)}
                    </div>
                </div>
            </div>
        `;
    } else if (role === 'assistant') {
        messageDiv.innerHTML = `
            <div class="flex justify-start">
                <div class="max-w-3xl">
                    <div class="px-4 py-3 bg-purple-900 bg-opacity-20 border border-purple-500/30 rounded-lg text-purple-100">
                        ${formatMarkdown(content)}
                    </div>
                </div>
            </div>
        `;
    } else if (role === 'error') {
        messageDiv.innerHTML = `
            <div class="flex justify-center">
                <div class="max-w-3xl">
                    <div class="px-4 py-3 bg-red-900 bg-opacity-30 border border-red-500/50 rounded-lg text-red-300">
                        ⚠️ ${escapeHtml(content)}
                    </div>
                </div>
            </div>
        `;
    }

    chatMessages.appendChild(messageDiv);
    scrollToBottom();
}

// Update cognitive layer state display
function updateDimensionalState(dimensions, durationMs, contextsLoaded) {
    dimensionList.innerHTML = '';

    dimensions.forEach(dimId => {
        const badge = document.createElement('span');
        badge.className = 'px-2 py-1 bg-purple-600 bg-opacity-30 border border-purple-500/50 rounded text-xs text-purple-300';
        badge.textContent = `L${String(dimId).padStart(2, '0')} ${cognitiveLayerNames[dimId] || 'Unknown'}`;
        dimensionList.appendChild(badge);
    });

    // Add stats
    const stats = document.createElement('span');
    stats.className = 'text-gray-500 text-xs ml-2';
    stats.textContent = `(${durationMs}ms, ${contextsLoaded} contexts)`;
    dimensionList.appendChild(stats);
}

// Enable/disable input
function setInputEnabled(enabled) {
    messageInput.disabled = !enabled;
    sendButton.disabled = !enabled;

    if (enabled) {
        sendButton.classList.remove('opacity-50', 'cursor-not-allowed');
    } else {
        sendButton.classList.add('opacity-50', 'cursor-not-allowed');
    }
}

// Add loading indicator
function addLoadingIndicator() {
    const loadingId = `loading-${Date.now()}`;
    const messageDiv = document.createElement('div');
    messageDiv.id = loadingId;
    messageDiv.className = 'message-loading mb-4';
    messageDiv.innerHTML = `
        <div class="flex justify-start">
            <div class="max-w-3xl">
                <div class="px-4 py-3 bg-purple-900 bg-opacity-20 border border-purple-500/50 rounded-lg">
                    <div class="flex items-center space-x-3">
                        <div class="flex space-x-1">
                            <div class="w-2 h-2 bg-purple-400 rounded-full animate-bounce" style="animation-delay: 0s"></div>
                            <div class="w-2 h-2 bg-purple-400 rounded-full animate-bounce" style="animation-delay: 0.15s"></div>
                            <div class="w-2 h-2 bg-purple-400 rounded-full animate-bounce" style="animation-delay: 0.3s"></div>
                        </div>
                        <span class="text-purple-300 text-sm">JESSY is analyzing through cognitive layers...</span>
                    </div>
                </div>
            </div>
        </div>
    `;
    chatMessages.appendChild(messageDiv);
    scrollToBottom();
    return loadingId;
}

// Remove loading indicator
function removeLoadingIndicator(loadingId) {
    const loadingDiv = document.getElementById(loadingId);
    if (loadingDiv) {
        loadingDiv.remove();
    }
}

// Scroll to bottom of chat
function scrollToBottom() {
    chatMessages.scrollTop = chatMessages.scrollHeight;
}

// Escape HTML to prevent XSS
function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

// Simple markdown formatting
function formatMarkdown(text) {
    let html = escapeHtml(text);

    // Bold: **text**
    html = html.replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>');

    // Italic: *text*
    html = html.replace(/\*(.+?)\*/g, '<em>$1</em>');

    // Code: `code`
    html = html.replace(/`(.+?)`/g, '<code class="px-1 py-0.5 bg-gray-900 rounded text-sm">$1</code>');

    // Line breaks
    html = html.replace(/\n/g, '<br>');

    return html;
}

// Initialize WebSocket connection
function initializeWebSocket() {
    console.log('🔌 Initializing WebSocket connection...');
    
    // Create token renderer
    tokenRenderer = new TokenRenderer(chatMessages);
    
    // Create WebSocket client
    jessyWs = new JessyWebSocket(WS_URL, {
        userId: 'goksel', // Göksel persona profile
        sessionId: sessionId,
        maxReconnectAttempts: 5,
        reconnectDelay: 1000,
        
        // Token streaming callback
        onToken: (content, tokenType) => {
            tokenRenderer.appendToken(content, tokenType);
        },
        
        // Thinking marker callback
        onThinkingMarker: (markerType, content) => {
            tokenRenderer.showThinkingMarker(markerType, content);
        },
        
        // Stage transition callback
        onStageTransition: (fromStage, toStage) => {
            tokenRenderer.showStageTransition(fromStage, toStage);
        },
        
        // Typing indicator callback
        onTyping: (isTyping) => {
            if (isTyping) {
                showTypingIndicator();
            } else {
                hideTypingIndicator();
            }
        },
        
        // Completion callback
        onComplete: (completedSessionId) => {
            sessionId = completedSessionId;
            tokenRenderer.completeMessage();
            setInputEnabled(true);
            messageInput.focus();
            console.log('✅ Response complete');
        },
        
        // Error callback
        onError: (error) => {
            console.error('❌ WebSocket error:', error);
            addMessage('error', `WebSocket error: ${error.message}`);
            // Fallback to HTTP
            useWebSocket = false;
            updateConnectionStatus('disconnected');
        },
        
        // Connection status callback
        onConnectionChange: (connected) => {
            updateConnectionStatus(connected ? 'connected' : 'disconnected');
            if (connected) {
                console.log('✅ WebSocket connected');
                useWebSocket = true;
            } else {
                console.log('❌ WebSocket disconnected');
            }
        }
    });
    
    // Connect
    jessyWs.connect();
}

// Add connection status indicator to page
function addConnectionStatusIndicator() {
    const indicator = document.createElement('div');
    indicator.id = 'connectionStatus';
    indicator.className = 'connection-status connecting';
    indicator.textContent = '⚡ Connecting...';
    document.body.appendChild(indicator);
}

// Update connection status
function updateConnectionStatus(status) {
    const indicator = document.getElementById('connectionStatus');
    if (!indicator) return;
    
    indicator.className = `connection-status ${status}`;
    
    switch (status) {
        case 'connected':
            indicator.textContent = '🟢 WebSocket Connected';
            break;
        case 'disconnected':
            indicator.textContent = '🔴 Disconnected (HTTP Fallback)';
            break;
        case 'connecting':
            indicator.textContent = '⚡ Connecting...';
            break;
    }
}

// Show typing indicator
let typingIndicatorElement = null;
function showTypingIndicator() {
    if (typingIndicatorElement) return;
    
    typingIndicatorElement = document.createElement('div');
    typingIndicatorElement.id = 'typingIndicator';
    typingIndicatorElement.className = 'ws-typing-indicator';
    typingIndicatorElement.innerHTML = `
        <span></span>
        <span></span>
        <span></span>
    `;
    
    chatMessages.appendChild(typingIndicatorElement);
    scrollToBottom();
}

// Hide typing indicator
function hideTypingIndicator() {
    if (typingIndicatorElement) {
        typingIndicatorElement.remove();
        typingIndicatorElement = null;
    }
}

// Example queries for demo
const exampleQueries = [
    "What is consciousness?",
    "How does multidimensional thinking work?",
    "Explain the concept of frequency in consciousness",
    "What are the ethical implications of AI?",
];

console.log('💡 Try these example queries:', exampleQueries);
