import spacy
from mcqueen_teleop.voice_assistant.light_command_handler import LightCommandHandler
from mcqueen_teleop.voice_assistant.mcqueen_command_handler import McQueenCommandHandler
from mcqueen_teleop.voice_assistant.time_command_handler import TimeCommandHandler


class CommandProcessor:
    def __init__(self):
        self.nlp = spacy.load("xx_ent_wiki_sm")
        self.handlers = [
            LightCommandHandler(),
            McQueenCommandHandler(),
            TimeCommandHandler(),
        ]

    def turkish_lower(self, text):
        replacements = {
            "I": "ı",
            "İ": "i",
        }
        for k, v in replacements.items():
            text = text.replace(k, v)
        return text.lower()

    def parse_command(self, sentence):
        normalized_sentence = self.turkish_lower(sentence)
        doc = self.nlp(normalized_sentence)
        tokens = [token.text for token in doc]

        for handler in self.handlers:
            matched = handler.match(tokens)
            if matched:
                result = handler.handle(tokens)
                return result

        return "anlaşılamadı"
