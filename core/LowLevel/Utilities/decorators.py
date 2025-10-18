# core/LowLevel/Utilities/decorators.py
import traceback
from functools import wraps

from ...LowLevel.Utilities.globals import LOGGER, CONFIG  # or import your global logger

def safe_call(default=None, log_traceback=CONFIG["decorators"]["default_log_traceback"], suppress=CONFIG["decorators"]["default_suppress"],
              max_tb_lines=int(CONFIG["decorators"]["max_traceback_lines"])):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                # try to get class name if method
                class_name = args[0].__class__.__name__ if args else ""
                LOGGER.error(f" {class_name}.{func.__name__} (class.func) Failed with {type(e).__name__}: {e}")
                if log_traceback:
                    tb_str = "".join(traceback.format_exception(type(e), e, e.__traceback__))
                    last_lines = tb_str.strip().splitlines()[-max_tb_lines:]
                    [LOGGER.error(line) for line in last_lines]
                if not suppress:
                    raise
                return default
        return wrapper
    return decorator
