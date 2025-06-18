#ifndef WHISPER_SPEECH_RECOGNIZER_H
#define WHISPER_SPEECH_RECOGNIZER_H

#include "mcqueen_hw/abstract/ISpeechRecognizer.h"

#include <string>

class WhisperSpeechRecognizer : public ISpeechRecognizer {
public:
    explicit WhisperSpeechRecognizer(const std::string& model_path);
    ~WhisperSpeechRecognizer() override;

    std::string transcribe(const std::string& wav_path) override;

private:
    struct whisper_context* ctx;
};

#endif // WHISPER_SPEECH_RECOGNIZER_H
