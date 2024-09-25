"""Tools for interfacing with InfluxDB."""

from datetime import datetime, timezone
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS


def _current_time() -> str:
    """This is a specific timestamp influx uses

    Returns:
        str: timestamp
    """
    return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%fZ')


class Influx:
    """InfluxDB client wrapper."""
    
    def __init__(self, tags: dict, db_name: str = 'tabletop'):
        """Initialize the Influx class.
        
        Args:
            tags (Dict): Tags to add to every element, e.g. {'subject': 'nick'}.
            db_name (str): Name of the database to write to.
        """
        self._tags = tags
        self._client = InfluxDBClient(
            url='http://localhost:8086', token='', org='')
        self._write_api = self._client.write_api(write_options=SYNCHRONOUS)
        # self._client.create_database(db_name)
        # self._client.switch_database(db_name)
        # self._client.create_retention_policy(
        #     'hot_data', '1d', 1, db_name, True, '3h')
        
    def write(self, measurements: dict[str, float]) -> list:
        """Writes a dictionary of elements to influx points.

        Args:
            measurements (Dict[str, float]): Measurements to write, e.g.
                {'Trial count': 17, 'Succes': True, 'Reaction time': 0.376}.

        Returns:
            points: List of influx points.
        """
        time = _current_time()
        points = []
        for key in measurements.keys():
            points.append({
                'measurement': key,
                'tags': self._tags,
                'time': time,
                'fields': {'value': measurements[key]}
            })
        self._write_api.write(points)
