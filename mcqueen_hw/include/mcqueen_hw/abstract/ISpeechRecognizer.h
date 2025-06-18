#ifndef ISPEECH_RECOGNIZER_H
#define ISPEECH_RECOGNIZER_H

#include <string>

class ISpeechRecognizer {
public:
    virtual ~ISpeechRecognizer();
    
    virtual std::string transcribe(const std::string& wav_path) = 0;
};

#endif // ISPEECH_RECOGNIZER_H
