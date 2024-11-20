#!/usr/bin/env python

import rospy
import rosgraph
from rospy import Publisher, Subscriber
from rosgraph.masterapi import Master
from importlib import import_module

class DynamicSubPub:
    def __init__(self):
        rospy.init_node("dynamic_sub_pub", anonymous=True)

        
        self.publishers = {}
        self.subscribers = {}

       
        master = Master(rospy.get_name())
        self.topic_list = master.getPublishedTopics("")
        rospy.loginfo("Found topics: {}".format(self.topic_list))

       
        for topic, msg_type in self.topic_list:
            self.setup_subscriber_publisher(topic, msg_type)

    def setup_subscriber_publisher(self, topic, msg_type):
       
        try:
           
            module_name, msg_name = msg_type.split("/")
            msg_module = __import__(module_name + ".msg", fromlist=[msg_name])
            msg_class = getattr(msg_module, msg_name)

           
            pub_topic = "{}_copy".format(topic)
            self.publishers[topic] = Publisher(pub_topic, msg_class, queue_size=10)
            rospy.loginfo("Publisher created for topic: {}".format(pub_topic))

            
            self.subscribers[topic] = Subscriber(
                topic,
                msg_class,
                callback=self.create_callback(topic, pub_topic)
            )
            rospy.loginfo("Subscriber created for topic: {}".format(topic))
        except Exception as e:
            rospy.logwarn("Failed to process topic {} with type {}: {}".format(topic, msg_type, e))

    def create_callback(self, topic, pub_topic):
      
        def callback(msg):
            rospy.loginfo("Forwarding message from {} to {}".format(topic, pub_topic))
            self.publishers[topic].publish(msg)

        return callback

    def spin(self):
        rospy.loginfo("Dynamic subscription and publishing setup complete.")
        rospy.spin()


if __name__ == "__main__":
    try:
        node = DynamicSubPub()
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down dynamic_sub_pub node.")
