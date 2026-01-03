# Quick Start: Vision-Language-Action Module

## Prerequisites

- ROS 2 Humble installed
- OpenAI Whisper installed (pip install openai-whisper)
- LLM API access (OpenAI or local model)
- Microphone for voice input

## Getting Started

### Test Whisper Transcription

```bash
# Run Whisper on audio file
python3 -c "import whisper; model = whisper.load_model('base'); result = model.transcribe('test.wav'); print(result['text'])"
```

### Run Voice Pipeline Node

```bash
# Start voice pipeline
ros2 run vla_package whisper_node

# In another terminal, view transcriptions
ros2 topic echo /voice/transcription
```

### Test LLM Task Parsing

```bash
# Run task parser with test input
ros2 run vla_package task_parser --ros-args -p command:="go to the kitchen"
```

### Run Complete VLA System

```bash
# Launch all VLA nodes
ros2 launch vla_package vla_system.launch.py

# Give voice command
# Say: "Robot, go to the table and pick up the cup"
```

## Key Concepts

1. Whisper converts speech to text in real-time
2. LLM parses natural language into action sequences
3. ROS 2 actions execute robot movements
4. Safety mechanisms prevent dangerous actions
