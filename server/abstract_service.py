import threading
import queue
from typing import Any
from abc import ABC, abstractmethod

class AbstractService(ABC):
    def __init__(self, in_queue: queue.Queue, out_queue: queue.Queue):
        self._thread   = threading.Thread(target=self.mainloop, daemon=True)

        self.in_queue  = in_queue
        self.out_queue = out_queue

    def start(self):
        self._thread.start()

    @abstractmethod
    def mainloop(self):
        pass
