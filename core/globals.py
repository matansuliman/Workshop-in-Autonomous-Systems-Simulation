from .environment import ENV
from .logger import setup_logger
from .config import CONFIG

LOGGER = setup_logger(
    name=CONFIG["logger"]["name"],
    dir_name = CONFIG["logger"]["dir_name"],
    ext = CONFIG["logger"]["ext"],
    fmt=CONFIG["logger"]["fmt"],
    datefmt=CONFIG["logger"]["datefmt"]
)
ENVIRONMENT = ENV(CONFIG["path_to_xml"])
