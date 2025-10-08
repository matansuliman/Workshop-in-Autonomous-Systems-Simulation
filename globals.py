from environment import ENV
from logger import setup_logger
from config import load_config

CONFIG = load_config()
LOGGER = setup_logger(CONFIG["logger"])
ENVIRONMENT = ENV(CONFIG["environment"])
