from .environment import ENV
from .logger import setup_logger
from .config import CONFIG

LOGGER = setup_logger(
    name=CONFIG["logger"]["name"],
    log_file=CONFIG["logger"]["path"],
    fmt=CONFIG["logger"]["fmt"],
    datefmt=CONFIG["logger"]["datefmt"]
)
ENVIRONMENT = ENV(CONFIG["path_to_xml"])
