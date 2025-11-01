// JESSY Web Chat - Frontend JavaScript
// Global shared session - everyone talks to the same JESSY
let sessionId = 'global-jessy-session';

// API Base URL - auto-detect environment
const API_BASE_URL = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1'
    ? '' // Localhost: same origin
    : ''; // Production: same origin (jessys.online)

// DOM elements
const chatForm = document.getElementById('chatForm');
const messageInput = document.getElementById('messageInput');
const sendButton = document.getElementById('sendButton');
const chatMessages = document.getElementById('chatMessages');
const dimensionList = document.getElementById('dimensionList');

// Dimension names mapping
const dimensionNames = {
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

// Track displayed messages to avoid duplicates
let displayedMessageCount = 0;

// Initialize
document.addEventListener('DOMContentLoaded', () => {
    messageInput.focus();

    // Check health
    checkHealth();

    // Load existing conversation
    loadConversation();

    // Poll for new messages every 3 seconds
    setInterval(loadConversation, 3000);

    // Log session info
    console.log('🌐 Connected to global JESSY session');
});

// Load conversation history from server
async function loadConversation() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/conversation/${sessionId}`);
        const data = await response.json();

        // Only update if there are new messages
        if (data.message_count > displayedMessageCount) {
            // Clear chat and redraw all messages
            chatMessages.innerHTML = '';
            displayedMessageCount = 0;

            data.messages.forEach(msg => {
                addMessage(msg.role, msg.content, false); // false = don't scroll yet
                displayedMessageCount++;
            });

            // Scroll to bottom after all messages loaded
            scrollToBottom();
        }
    } catch (error) {
        console.error('Failed to load conversation:', error);
    }
}

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

    // Don't add message locally - polling will show it
    // addMessage('user', message);

    // Disable input while processing
    setInputEnabled(false);

    // Send message to API
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

        // Trigger immediate reload to show new messages
        await loadConversation();

        // Update dimensional state
        updateDimensionalState(data.dimensions_activated, data.selection_duration_ms, data.contexts_loaded);

    } catch (error) {
        console.error('Error:', error);
        addMessage('error', `Error: ${error.message}`);
    } finally {
        setInputEnabled(true);
        messageInput.focus();
    }
});

// Add message to chat
function addMessage(role, content, shouldScroll = true) {
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
                    <div class="px-4 py-3 bg-gray-800 bg-opacity-50 border border-gray-700 rounded-lg text-gray-100">
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
    if (shouldScroll) {
        scrollToBottom();
    }
}

// Update dimensional state display
function updateDimensionalState(dimensions, durationMs, contextsLoaded) {
    dimensionList.innerHTML = '';

    dimensions.forEach(dimId => {
        const badge = document.createElement('span');
        badge.className = 'px-2 py-1 bg-purple-600 bg-opacity-30 border border-purple-500/50 rounded text-xs text-purple-300';
        badge.textContent = `D${String(dimId).padStart(2, '0')} ${dimensionNames[dimId] || 'Unknown'}`;
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

// Scroll to bottom of chat
function scrollToBottom() {
    window.scrollTo({
        top: document.body.scrollHeight,
        behavior: 'smooth'
    });
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

// Example queries for demo
const exampleQueries = [
    "What is consciousness?",
    "How does multidimensional thinking work?",
    "Explain the concept of frequency in consciousness",
    "What are the ethical implications of AI?",
];

console.log('💡 Try these example queries:', exampleQueries);
