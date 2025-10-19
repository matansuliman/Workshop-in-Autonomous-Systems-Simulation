# core/LowLevel/Utilities/globals.py
import datetime

from ...LowLevel.SimulationDecoupling.environment import ENV
from ...LowLevel.RuntimeOutputs.logger import setup_logger
from ...LowLevel.Utilities.config import CONFIG

LOGGER = setup_logger(
    name=CONFIG["logger"]["name"],
    dir_name = CONFIG["logger"]["dir_name"],
    ext = CONFIG["logger"]["ext"],
    fmt=CONFIG["logger"]["fmt"],
    datefmt=CONFIG["logger"]["datefmt"]
)
ENVIRONMENT = ENV(CONFIG["path_to_xml"])

RUN_TIMESTAMP = datetime.datetime.now().strftime("%H_%M_%S__%Y_%m_%d")
