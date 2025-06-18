from mcqueen_teleop.voice_assistant.command_handler import CommandHandler


class McQueenCommandHandler(CommandHandler):
    SUBJECT_KEYS = ["mcqueen", "araba", "arac"]
    ACTION_KEYS = {
        "ileri": ["ileri", "ilerle", "git"],
        "geri": ["geri", "dön", "geri git"],
        "dur": ["dur", "bekle", "duraklat"],
        "yuvarlak": ["yuvarlak", "dön", "çiz"],
        "sağa": ["sağa", "sağ"],
        "sola": ["sola", "sol"],
    }

    def match(self, tokens):
        return any(key in tokens for key in self.SUBJECT_KEYS)

    def handle(self, tokens):
        for action, synonyms in self.ACTION_KEYS.items():
            if any(key in tokens for key in synonyms):
                return f"mcqueen_{action}"

        return "mcqueen_anlaşılamadı"
