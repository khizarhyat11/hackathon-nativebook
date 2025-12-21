/**
 * Floating Chatbot Component for Docusaurus
 * A RAG-powered AI assistant for the Physical AI textbook.
 */

import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from './Chatbot.module.css';

// API Configuration
const API_BASE_URL = 'http://localhost:8001';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Array<{ title: string; source: string; score: number }>;
}

interface ChatState {
  messages: Message[];
  conversationId: string | null;
  isLoading: boolean;
  error: string | null;
}

const ChatIcon: React.FC<{ onClick: () => void }> = ({ onClick }) => (
  <button className={styles.chatIcon} onClick={onClick} aria-label="Open chat">
    <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
    </svg>
  </button>
);

const CloseIcon: React.FC = () => (
  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <line x1="18" y1="6" x2="6" y2="18" />
    <line x1="6" y1="6" x2="18" y2="18" />
  </svg>
);

const SendIcon: React.FC = () => (
  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <line x1="22" y1="2" x2="11" y2="13" />
    <polygon points="22 2 15 22 11 13 2 9 22 2" />
  </svg>
);

const MessageBubble: React.FC<{ message: Message }> = ({ message }) => {
  const isUser = message.role === 'user';
  
  return (
    <div className={`${styles.messageBubble} ${isUser ? styles.userMessage : styles.assistantMessage}`}>
      <div className={styles.messageContent}>
        {message.content.split('\n').map((line, i) => (
          <React.Fragment key={i}>
            {line}
            {i < message.content.split('\n').length - 1 && <br />}
          </React.Fragment>
        ))}
      </div>
      {message.sources && message.sources.length > 0 && (
        <div className={styles.sources}>
          <span className={styles.sourcesLabel}>Sources:</span>
          {message.sources.slice(0, 2).map((source, i) => (
            <span key={i} className={styles.sourceTag}>
              {source.title || source.source}
            </span>
          ))}
        </div>
      )}
    </div>
  );
};

const TypingIndicator: React.FC = () => (
  <div className={styles.typingIndicator}>
    <span></span>
    <span></span>
    <span></span>
  </div>
);

export const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [input, setInput] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [state, setState] = useState<ChatState>({
    messages: [],
    conversationId: null,
    isLoading: false,
    error: null,
  });
  
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);
  
  // Scroll to bottom on new messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [state.messages]);
  
  // Focus input when chat opens
  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);
  
  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim().length > 10) {
        setSelectedText(selection.toString().trim());
      }
    };
    
    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);
  
  const sendMessage = useCallback(async () => {
    if (!input.trim() || state.isLoading) return;
    
    const userMessage: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: input.trim(),
      timestamp: new Date(),
    };
    
    setState(prev => ({
      ...prev,
      messages: [...prev.messages, userMessage],
      isLoading: true,
      error: null,
    }));
    
    setInput('');
    
    try {
      const response = await fetch(`${API_BASE_URL}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message: userMessage.content,
          conversation_id: state.conversationId,
          selected_text: selectedText || null,
        }),
      });
      
      if (!response.ok) {
        throw new Error('Failed to get response');
      }
      
      const data = await response.json();
      
      const assistantMessage: Message = {
        id: `assistant-${Date.now()}`,
        role: 'assistant',
        content: data.response,
        timestamp: new Date(),
        sources: data.sources,
      };
      
      setState(prev => ({
        ...prev,
        messages: [...prev.messages, assistantMessage],
        conversationId: data.conversation_id,
        isLoading: false,
      }));
      
      // Clear selected text after use
      setSelectedText('');
      
    } catch (error) {
      setState(prev => ({
        ...prev,
        isLoading: false,
        error: 'Sorry, I had trouble connecting. Please try again.',
      }));
    }
  }, [input, state.conversationId, state.isLoading, selectedText]);
  
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };
  
  const clearChat = () => {
    setState({
      messages: [],
      conversationId: null,
      isLoading: false,
      error: null,
    });
    setSelectedText('');
  };
  
  if (!isOpen) {
    return <ChatIcon onClick={() => setIsOpen(true)} />;
  }
  
  return (
    <div className={styles.chatContainer}>
      {/* Header */}
      <div className={styles.chatHeader}>
        <div className={styles.headerInfo}>
          <div className={styles.headerIcon}>🤖</div>
          <div className={styles.headerTitle}>Physical AI Assistant</div>
        </div>
        <div className={styles.headerActions}>
          <button onClick={clearChat} className={styles.actionButton} title="Clear chat">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
              <polyline points="3 6 5 6 21 6"></polyline>
              <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path>
            </svg>
          </button>
          <button onClick={() => setIsOpen(false)} className={styles.actionButton}>
            <CloseIcon />
          </button>
        </div>
      </div>
      
      {/* Selected Text Banner */}
      {selectedText && (
        <div className={styles.selectedTextBanner}>
          <span>📝 Use selected text as context</span>
          <button onClick={() => setSelectedText('')}>✕</button>
        </div>
      )}
      
      {/* Messages */}
      <div className={styles.messagesContainer}>
        {state.messages.length === 0 && (
          <div className={styles.welcomeMessage}>
            <p className={styles.welcomeText}>
              👋 Hi! I'm your <strong>Physical AI & Humanoid Robotics</strong> assistant.
            </p>
            <p className={styles.welcomeSubtext}>I can help you with:</p>
            <ul className={styles.welcomeList}>
              <li>ROS 2 and robotics middleware</li>
              <li>Digital twin simulation</li>
              <li>NVIDIA Isaac and navigation</li>
              <li>Vision-Language-Action systems</li>
            </ul>
            <div className={styles.tipBox}>
              <span className={styles.tipIcon}>💡</span>
              <span><strong>Tip:</strong> Select any text on the page, then ask me about it!</span>
            </div>
          </div>
        )}
        
        {state.messages.map(message => (
          <MessageBubble key={message.id} message={message} />
        ))}
        
        {state.isLoading && (
          <div className={styles.assistantMessage}>
            <TypingIndicator />
          </div>
        )}
        
        {state.error && (
          <div className={styles.errorMessage}>
            {state.error}
          </div>
        )}
        
        <div ref={messagesEndRef} />
      </div>
      
      {/* Input */}
      <div className={styles.inputContainer}>
        <textarea
          ref={inputRef}
          value={input}
          onChange={e => setInput(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder={selectedText ? "Ask about selection..." : "Ask about robotics, ROS 2, or AI..."}
          className={styles.input}
          rows={1}
        />
        <button 
          onClick={sendMessage} 
          className={styles.sendButton}
          disabled={!input.trim() || state.isLoading}
        >
          <SendIcon />
        </button>
      </div>
    </div>
  );
};

export default Chatbot;
