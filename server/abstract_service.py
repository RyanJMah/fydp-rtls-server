import multiprocessing as mp
from typing import TypeAlias, Optional
from abc import ABC, abstractmethod

class AbstractService(ABC):
    def __init__(self, in_conn, out_conn):
        self.in_conn = in_conn
        self.out_conn = out_conn

        self._process = mp.Process( target=self.main,
                                    args=(self.in_conn, self.out_conn,),
                                    daemon=True )

    def start(self):
        self._process.start()

    def join(self):
        self._process.join()

    def kill(self):
        self._process.kill()

    @abstractmethod
    def main(self, in_conn, out_conn):
        pass
