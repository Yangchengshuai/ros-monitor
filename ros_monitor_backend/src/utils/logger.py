import logging

_formatter = logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")
_handler = logging.StreamHandler()
_handler.setFormatter(_formatter)

logging.basicConfig(level=logging.INFO, handlers=[_handler])


def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)
