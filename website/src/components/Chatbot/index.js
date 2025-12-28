import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  // Load chat history from localStorage on component mount
  useEffect(() => {
    try {
      const savedMessages = localStorage.getItem('chatHistory');
      if (savedMessages) {
        const parsedMessages = JSON.parse(savedMessages);
        if (Array.isArray(parsedMessages) && parsedMessages.length > 0) {
          setMessages(parsedMessages);
        }
      } else {
        setMessages([
          {
            text: "Hello! I'm a helpful assistant. Ask me anything about the content in this book.",
            isUser: false,
          }
        ]);
      }
    } catch (error) {
      console.error("Failed to load chat history from localStorage", error);
        setMessages([
            {
                text: "Hello! I'm a helpful assistant. Ask me anything about the content in this book.",
                isUser: false,
            }
        ]);
    }
  }, []);

  // Save chat history to localStorage whenever it changes
  useEffect(() => {
    try {
      if (messages.length > 0) {
        localStorage.setItem('chatHistory', JSON.stringify(messages));
      }
    } catch (error) {
      console.error("Failed to save chat history to localStorage", error);
    }
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (e) => {
    setInputValue(e.target.value);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;

    const userMessage = { text: inputValue, isUser: true };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:8000/ask', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question: inputValue }),
      });

      if (!response.ok) {
        throw new Error('API Error: ' + response.statusText);
      }

      const data = await response.json();
      const botMessage = { text: data.answer, isUser: false };
      setMessages((prevMessages) => [...prevMessages, botMessage]);
    } catch (error) {
      console.error('Failed to fetch from chatbot API:', error);
      const errorMessage = { text: 'Sorry, something went wrong. Please try again later.', isUser: false, isError: true };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      <button className={styles.chatbotToggleButton} onClick={toggleChat}>
        {isOpen ? 'X' : '🤖'}
      </button>
      {isOpen && (
        <div className={styles.chatbotWindow}>
          <div className={styles.chatbotHeader}>
            <h2>AI Assistant</h2>
          </div>
          <div className={styles.chatbotMessages}>
            {messages.map((message, index) => (
              <div
                key={index}
                className={`${styles.chatMessage} ${
                  message.isUser ? styles.userMessage : styles.botMessage
                } ${message.isError ? styles.errorMessage : ''}`}
              >
                {message.text}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.chatMessage} ${styles.botMessage} ${styles.loading}`}>
                <span>.</span><span>.</span><span>.</span>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form className={styles.chatbotInputForm} onSubmit={handleSubmit}>
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              placeholder="Ask a question..."
              aria-label="Ask a question"
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading}>Send</button>
          </form>
        </div>
      )}
    </div>
  );
};

export default Chatbot;
