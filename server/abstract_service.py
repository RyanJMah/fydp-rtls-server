import multiprocessing as mp
from typing import Any
from abc import ABC, abstractmethod

class AbstractService(ABC):
    def __init__(self, in_conn: Any, out_conn: Any, daemon: bool = False):
        self.in_conn = in_conn
        self.out_conn = out_conn

        self._process = mp.Process( target=self._supervisor,
                                    args=(self.in_conn, self.out_conn,),
                                    daemon=daemon )

    def _supervisor(self, in_conn: Any, out_conn: Any):
        while (1):
            try:
                self.main(in_conn, out_conn)

            except Exception as e:
                print(f"REALLY BAD ERROR: SERVICE MAINLOOP CRASHED: {e}")

    def start(self):
        self._process.start()

    def join(self):
        self._process.join()

    def kill(self):
        self._process.kill()

    @abstractmethod
    def main(self, in_conn, out_conn):
        pass
