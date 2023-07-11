import time
import logging

LOG_LEVEL = logging.INFO
# LOG_LEVEL = logging.DEBUG

LOG_FORMAT_STR = "[%(asctime)s] %(levelname)-18s --- %(message)s (%(name)s)"

class _Formatter(logging.Formatter):
    """Custom formatter that adds color to log messages based on the log level."""

    COLORS = {
        logging.DEBUG:    "\033[90m",   # Grey
        logging.INFO:     "\033[92m",   # Green
        logging.WARNING:  "\033[93m",   # Yellow
        logging.ERROR:    "\033[91m",   # Red
        logging.CRITICAL: "\033[91m",   # Red
    }
    RESET_COLOR = "\033[0m"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.start_time = time.time()  # Store the process start time in milliseconds

    def formatTime(self, record, datefmt=None):
        elapsed_time = time.time() - self.start_time  # Calculate elapsed time in milliseconds
        return f"{elapsed_time:.4f}"  # Format and return elapsed time

    def format(self, record):
        log_level = record.levelno
        log_level_color = self.COLORS.get(log_level, "")
        record.levelname = f"{log_level_color}{record.levelname}\033[0m"  # Apply color to log level

        self._style._fmt = LOG_FORMAT_STR  # Override the format string

        return super().format(record)


def init_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(LOG_LEVEL)

    formatter = _Formatter()

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)

    logger.addHandler(console_handler)

    return logger

