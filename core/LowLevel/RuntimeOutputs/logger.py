# core/LowLevel/RuntimeOutputs/logger.py
import os, logging
from colorlog import ColoredFormatter

def setup_logger(
    name="logger-empty",
    dir_name='',
    ext='log',
    fmt=None,
    datefmt=None,
    level=logging.DEBUG,
):
    # Create a custom logger
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Handlers
    os.makedirs(dir_name, exist_ok=True)
    filename = os.path.join(dir_name, f"{name}.{ext}")
    file_handler = logging.FileHandler(filename, mode="w", delay=False)
    stream_handler = logging.StreamHandler()

    # Formatters
    formatter = ColoredFormatter(
        fmt=fmt,
        datefmt=datefmt,
        log_colors={
            'DEBUG': 'cyan',
            'INFO': 'green',
            'WARNING': 'yellow',
            'ERROR': 'red',
            'CRITICAL': 'bold_red',
        }
    )
    file_handler.setFormatter(formatter)
    stream_handler.setFormatter(formatter)

    # Add handlers (avoid duplicates if setup is called twice)
    if not logger.handlers:
        logger.addHandler(file_handler)
        logger.addHandler(stream_handler)

    return logger
