# Speech Recognizer with WhisperSpeechRecognizer

`WhisperSpeechRecognizer` is a concrete implementation of the `ISpeechRecognizer` interface that uses [`whisper.cpp`](https://github.com/ggerganov/whisper.cpp) for real-time or near-real-time speech recognition from microphone input.

It supports:

- Performing offline speech recognition using OpenAI Whisper (C++ implementation)
- Capturing audio directly from microphone using PortAudio
- Transcribing microphone input to text in Turkish or other supported languages
- Modular integration into ROS 2 nodes or standalone C++ applications
- Swappable backend by following the `ISpeechRecognizer` interface

## Author

**Halil İbrahim ŞENAYDIN**  
E-mail: halilsenaydin@gmail.com  
GitHub: [github.com/halilsenaydin](https://github.com/halilsenaydin)

## Integration with ROS 2

If you are using this library inside a ROS 2 package (e.g., mcqueen_hw):

- The package is set up with `ament_cmake`
- You can include and use `WhisperSpeechRecognizer` inside your ROS 2 nodes
- Just add `mcqueen_hw_lib` as a dependency and link it in your `CMakeLists.txt`
- Create and manage `WhisperSpeechRecognizer` objects in your ROS nodes similar to other communication devices

### Install Dependencies

The following commands install essential dependencies for building and running the project with real-time audio input and Whisper speech recognition:

```bash
# Install PortAudio development headers (required for real-time audio I/O)
sudo apt-get install portaudio19-dev

# Clone whisper.cpp as a Git submodule under third_party/
cd ~/workspace/src/mcqueen_hw
mkdir third_party
git submodule add -f https://github.com/ggerganov/whisper.cpp third_party/whisper.cpp
git submodule update --init --recursive
```

## Building

### ROS 2 Build

Inside your ROS 2 workspace:

```bash
colcon build --packages-select mcqueen_hw
```
