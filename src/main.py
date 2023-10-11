#!/usr/bin/env python3
import asyncio
import argparse
from pathlib import Path

import rospy

from farm_ng.core.event_client import EventClient
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.core.event_service_pb2 import (
    EventServiceConfigList,
    SubscribeRequest,
)


clients = {}
publishers = {}

# TODO: implement me
def _farmng_to_ros_type(farmng_type):
    """Map the farmng type to the ros type"""
    pass


def _farmng_to_ros(farmng_msg):
    """Convert the farmng message to the ros message"""
    pass


async def _subscribe(client, subscribe_request, ros_publisher):
    async for event, message in client.subscribe(subscribe_request, decode=True):
        print(f"Got reply: {message}")
        ros_msg = _farmng_to_ros(message)
        ros_publisher.publish(ros_msg)


async def subscribe(service_name):
    client = clients[service_name]

    uris = await client.list_uris()

    for uri in uris:
        if not uri.path in publishers:
            # TODO: create better the ros pulblishers
            ros_msg_type = _farmng_to_ros_type(uri.path)
            publishers[uri.path] = rospy.Publisher(uri.path, ros_msg_type, queue_size=10)

        subscribe_request = SubscribeRequest(uri=uri)
        subscribe_task = asyncio.create_task(_subscribe(client, subscribe_request, publishers[uri.path]))
        


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='AMiGA ROS Bridge')
    parser.add_argument('--service-config', type=Path, required=True, help='Path to config file')
    args = parser.parse_args()

    # config with all the configs
    config_list: EventServiceConfigList = proto_from_json_file(
        args.service_config, EventServiceConfigList()
    )

    # populate the clients
    for config in config_list.configs:
        if config.port != 0:
            clients[config.name] = EventClient(config)
    
    # start the amiga subscriptions to publish to ros 
    for service_name in clients.keys():
        asyncio.create_task(subscribe(service_name))
    
    # TODO: start the ros subscriptions to publish to amiga
    
    # start the ros node

    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")