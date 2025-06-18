from mcqueen_teleop.voice_assistant.command_handler import CommandHandler


class LightCommandHandler(CommandHandler):
    SUBJECT_KEYS = ["ışık", "lamba", "aydınlatma"]
    ACTION_KEYS = {"aç": ["aç", "yak", "açık"], "kapat": ["kapat", "söndür", "kapan"]}

    def match(self, tokens):
        return any(key in tokens for key in self.SUBJECT_KEYS)

    def handle(self, tokens):
        for action, synonyms in self.ACTION_KEYS.items():
            if any(key in tokens for key in synonyms):
                return f"ışık_{action}"

        return "ışık_anlaşılamadı"
