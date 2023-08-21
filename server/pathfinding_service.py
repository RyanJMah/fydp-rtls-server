from typing import Optional

from abstract_service import AbstractService
from localization_service import LocalizationService_OutData

class PathfindingService(AbstractService):
    def main(self, in_conn, out_conn):
        while (1):
            position_data: LocalizationService_OutData = in_conn.recv()     # type: ignore
