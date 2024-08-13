import boto3
import json
mote_name = 'Arduino_CM32'
iot_client = boto3.client('iot-data',
  aws_access_key_id = 'key_id',
  aws_secret_access_key = 'XXXXXXX',
  region_name='us-east-2')
def read_mote_state(mote):
  response = iot_client.get_thing_shadow(thingName = mote)
  streamingBody = response["payload"]
  curr_shadow = json.loads(streamingBody.read())
  desired_state = str(curr_shadow["state"]["desired"]["sleeping"])
  reported_state = str(curr_shadow["state"]["reported"]["sleeping"])
  return desired_state, reported_state
def change_mote_state(mote, state):
  updated_state = {"state" : {"desired" : {"sleeping" : state}}}
  updated_state = json.dumps(updated_state)
  response = iot_client.update_thing_shadow(thingName = mote, payload = updated_state)
  if state==1: 
    # Send update shadow message to prompt sleep
    iot_client.publish(topic='$aws/things/' + mote + '/shadow/get', payload='')