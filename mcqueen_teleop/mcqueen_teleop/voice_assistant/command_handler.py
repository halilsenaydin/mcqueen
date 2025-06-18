from abc import ABC, abstractmethod


class CommandHandler(ABC):
    @abstractmethod
    def match(self, tokens):
        """Return True if this handler matches the given tokens."""
        pass

    @abstractmethod
    def handle(self, tokens):
        """Process the tokens and return the response string."""
        pass
