import time
import logging
import multiprocessing as mp

from abstract_service import AbstractService

LOG_LEVEL = logging.INFO
# LOG_LEVEL = logging.DEBUG

LOG_FORMAT_STR = "[%(asctime)s] %(levelname)s --- %(message)s (%(name)s)"

g_log_queue: mp.Queue = mp.Queue(-1)

class LogQueueingService(AbstractService):
    def main(self, in_conn, out_conn):
        while True:
            try:
                print( g_log_queue.get(block=True) )

            except Exception as e:
                print(f"REALLY BAD ERROR: LOG QUEUEING SERVICE FAILED TO HANDLE LOG RECORD: {e}")


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

class _QueuedLogsHandler(logging.Handler):
    def __init__(self):
        super().__init__()

    def emit(self, record):
        try:
            log_entry = self.format(record)
            g_log_queue.put(log_entry, block=False)

        except Exception:
            self.handleError(record)


def init_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(LOG_LEVEL)

    formatter = _Formatter()

    queued_logs_handler = _QueuedLogsHandler()
    queued_logs_handler.setFormatter(formatter)

    logger.addHandler(queued_logs_handler)

    return logger

