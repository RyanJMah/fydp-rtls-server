import re
import logs
import paho.mqtt.client as mqtt
from typing import Callable, Union, Type, List, Dict
from threading import Event

logger = logs.init_logger(__name__)

# So that we can import it from this file
MqttMsg = Type[mqtt.MQTTMessage]

class _TopicHandle():
    def __init__(self, topic: str, callback: Callable):
        self.callback  = callback

        self.topic_str = topic
        self.topic_with_wildcard = _TopicHandle._topic_args_to_wildcard(topic)

        self.arg_positions: List[int] = []

        for i, level in enumerate( topic.split("/") ):
            if _TopicHandle._is_arg(level):
                self.arg_positions.append(i)

    @staticmethod
    def _topic_args_to_wildcard(path: str) -> str:
        return re.sub("<(.*?)>", "+", path)

    @staticmethod
    def _is_arg(topic: str):
        return re.match("<(.*?)>", topic)

    def matches(self, received_topic: str):
        return mqtt.topic_matches_sub(self.topic_with_wildcard, received_topic)

    def run(self, client: "MqttClient", received_msg: MqttMsg):
        # get the arguments from the received_topic
        topic_levels = received_msg.topic.split("/")    # type: ignore

        args = []
        for pos in self.arg_positions:
            args.append( int(topic_levels[pos]) )

        # run the callback...
        self.callback(client, received_msg, *args)


class MqttClient():
    def __init__(self):
        self._mqtt = mqtt.Client()

        # self._mqtt.user_data_set(self)

        self._mqtt.on_message    = self._client_msg_callback
        self._mqtt.on_connect    = self._client_connect_callback
        self._mqtt.on_disconnect = self._client_disconnect_callback
        self._mqtt.on_subscribe  = self._client_subscribe_callback

        self._user_connect_callback    = lambda *args: None     # lambda function that does nothing lol
        self._user_disconnect_callback = lambda *args: None
        self._user_subscribe_callback  = lambda *args: None

        self._sub_topic_handles: Dict[str, "_TopicHandle"] = {}     # type: ignore
        self._sub_mids = {}

        self._is_connected = Event()
        self._is_connected.clear()


    def _client_msg_callback(self, client, userdata, msg):
        for topic, handle in self._sub_topic_handles.items():
            if handle.matches(msg.topic):
                handle.run(self, msg)


    def _client_connect_callback(self, client, userdata, flags, rc):
        logger.info(f"Connected to broker with result code {rc}")

        self._is_connected.set()

        self._user_connect_callback(self)

        for topic in self._sub_topic_handles.keys():
            self._do_subscribe(topic)


    def _client_disconnect_callback(self, client, userdata, rc):
        logger.warning("Disconnected from server...")

        self._is_connected.clear()
        self._user_disconnect_callback(self)


    def _client_subscribe_callback(self, client, userdata, mid, granted_qos):
        logger.info(f"Successfully subscribed to {self._sub_mids[mid]}")

        del self._sub_mids[mid]

        self._user_subscribe_callback(self)


    def _do_subscribe(self, topic: str, qos=0):
        logger.debug(f"Subscribing to {topic}...")

        err_code, mid = self._mqtt.subscribe(topic, qos)

        if err_code != mqtt.MQTT_ERR_SUCCESS:
            logger.error(f"Failed to subscribe to {topic}, err_code={err_code}")

        self._sub_mids[mid] = topic


    def register_connect_callback(self, callback: Callable):
        """
        Example callback shown beloew.

        ```python
        def example_callback(client: MqttClient):
            pass
        ```
        """

        self._user_connect_callback = callback


    def register_disconnect_callback(self, callback: Callable):
        """
        Example callback shown below.

        ```python
        def example_callback(client: MqttClient):
            pass
        ```
        """
        self._user_disconnect_callback = callback


    def register_subscribe_callback(self, callback: Callable):
        """
        Example callback shown below.

        ```python
        def example_callback(client: MqttClient):
            pass
        ```
        """
        self._user_subscribe_callback = callback


    def connect(self, host: str, port: int, keepalive=30):
        logger.debug(f"Connecting to broker at {host}...")

        self._mqtt.connect(host, port=port, keepalive=keepalive)


    def subscribe(self, topic: str, callback: Callable, qos=0):
        """
        Example Callback shown for /gl/anchor/<id>/heartbeat

        ```python
        def example_callback(client: MqttClient, msg: MqttMessage, id: int):
            pass
        ```
        """

        topic_handle = _TopicHandle(topic, callback)

        self._sub_topic_handles[topic_handle.topic_with_wildcard] = topic_handle

        if self._is_connected.is_set():
            self._do_subscribe(topic, qos)


    def publish(self, topic: str, payload=None, qos=0, retain=False):
        logger.debug(f"Publishing to {topic}")

        self._mqtt.publish(topic, payload, qos, retain)


    def run_mainloop(self):
        self._mqtt.loop_forever()


    def start_mainloop(self):
        self._mqtt.loop_start()


    def stop_mainloop(self):
        self._mqtt.loop_stop()


    def disconnect(self):
        self._mqtt.disconnect()
