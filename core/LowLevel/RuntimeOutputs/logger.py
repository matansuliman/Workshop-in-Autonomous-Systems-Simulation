import logging


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
    filename = f"{dir_name}\\{name}.{ext}"
    file_handler = logging.FileHandler(filename, mode="w", delay=False)
    stream_handler = logging.StreamHandler()

    # Formatters
    formatter = logging.Formatter(fmt=fmt, datefmt=datefmt)
    file_handler.setFormatter(formatter)
    stream_handler.setFormatter(formatter)

    # Add handlers (avoid duplicates if setup is called twice)
    if not logger.handlers:
        logger.addHandler(file_handler)
        logger.addHandler(stream_handler)

    return logger
