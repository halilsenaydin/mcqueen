from mcqueen_teleop.voice_assistant.command_handler import CommandHandler
from datetime import datetime


class TimeCommandHandler(CommandHandler):
    SUBJECT_KEYS = ["saat", "zaman", "kaç", "gün", "tarih", "bugün", "takvim"]

    def match(self, tokens):
        return any(key in tokens for key in self.SUBJECT_KEYS)

    def handle(self, tokens):
        if any(key in tokens for key in ["saat", "zaman", "kaç"]):
            return datetime.now().strftime("Şu an saat %H:%M:%S")

        if any(key in tokens for key in ["gün", "tarih", "bugün", "takvim"]):
            return datetime.now().strftime("Bugün günlerden %A, tarih %d.%m.%Y")

        return "zaman_anlaşılamadı"
