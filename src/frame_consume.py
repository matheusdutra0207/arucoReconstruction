import re
import sys
import socket
from is_msgs.common_pb2 import ConsumerList
from is_wire.core import Message, Subscription, Logger, Channel
from is_wire.rpc import ServiceProvider, LogInterceptor
from is_msgs.image_pb2 import ObjectAnnotations, ObjectAnnotation, Vertex

from is_msgs.common_pb2 import ConsumerList

channel = Channel("amqp://10.10.3.188:30000")
subscription = Subscription(channel)
subscription.subscribe("BrokerEvents.Consumers")


message = channel.consume()
consumers = message.unpack(ConsumerList)

print(consumers)