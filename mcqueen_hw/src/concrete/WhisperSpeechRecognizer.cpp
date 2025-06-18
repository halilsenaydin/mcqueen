#include "mcqueen_hw/concrete/WhisperSpeechRecognizer.h"

#include "whisper.h"
#include <iostream>

WhisperSpeechRecognizer::WhisperSpeechRecognizer(const std::string& model_path) {
    ctx = whisper_init_from_file(model_path.c_str());
    if (!ctx) {
        std::cerr << "[WhisperSpeechRecognizer] Failed to load model: " << model_path << std::endl;
    } else {
        std::cout << "[WhisperSpeechRecognizer] Model loaded successfully." << std::endl;
    }
}

WhisperSpeechRecognizer::~WhisperSpeechRecognizer() {
    if (ctx) {
        whisper_free(ctx);
    }
}

std::string WhisperSpeechRecognizer::transcribe(const std::string& wav_path) {
    if (!ctx) {
        return "[WhisperSpeechRecognizer] No model context initialized.";
    }

    whisper_full_params params = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
    params.print_progress = false;
    params.print_special = false;
    params.print_realtime = false;
    params.print_timestamps = false;

    if (whisper_full(ctx, params, wav_path.c_str()) != 0) {
        return "[WhisperSpeechRecognizer] Transcription failed.";
    }

    std::string result;
    int n_segments = whisper_full_n_segments(ctx);
    
    for (int i = 0; i < n_segments; ++i) {
        result += whisper_full_get_segment_text(ctx, i);
    }

    return result;
}
