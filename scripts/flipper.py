#!/usr/bin/python
import rospy, rocon_gateway, os, rostopic
from std_msgs.msg import String
from sys import argv

import gateway_msgs.srv as gateway_srvs
import gateway_msgs.msg as gateway_msgs

node_name = 'flipper'
rospy.init_node(node_name)
os.system('export HOSTNAME')

inputs = argv[1:] 
to_send = zip(inputs[1::2], inputs[::2])

hub_gateway_name = "Hub Gateway"
print "This is the name of the gateway: '%s'"%hub_gateway_name
flip_service = rospy.ServiceProxy('/gateway/flip', gateway_srvs.Remote)
flip_request = gateway_srvs.RemoteRequest(cancel = False, remotes = [])

#build up the flip_request in this loop
for old_name, ttype in to_send:
    #build up each message
    topic_type = gateway_msgs.ConnectionType.PUBLISHER if ttype == '-p' else gateway_msgs.ConnectionType.SUBSCRIBER
    #set up the relay

    new_name = '/'+os.getenv('HOSTNAME')+old_name
    if ttype == '-p':
        cmd = old_name + ' ' + new_name #if it's a publisher, relay the old name to the new name
    else:
        topic_class, _, _ = rostopic.get_topic_class(old_name)
        try:
            rospy.Publisher(new_name, topic_class, queue_size = 10)#the first part of the relay needs to know what type it is
        except ValueError as e:
            rospy.logerr(e)
        cmd = new_name + ' ' + old_name #relay new to old if a subscriber

    os.system("rosrun topic_tools relay " + cmd + "&")#start the relay

    rule = gateway_msgs.Rule(name = new_name, type = topic_type, node = "")
    remote_rule = gateway_msgs.RemoteRule(hub_gateway_name, rule) 
    flip_request.remotes.append(remote_rule)#add the rule to the request
    rospy.loginfo("Trying to flip connection ['name: %s', type: '%s'] to %s"%(rule.name, rule.type, hub_gateway_name))


#send request to service
resp = flip_service(flip_request)
rospy.loginfo(resp)
if resp.result != 0:
    rospy.logerr("Flip: %s"%resp.error_message)

rospy.loginfo("Finished flipping %d connections."%len(to_send))
rospy.spin()
    


