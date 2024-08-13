import boto3
rule_client = boto3.client('iot',
  aws_access_key_id = 'key_id',
  aws_secret_access_key = 'XXXXXXX',
  region_name='us-east-2')
response = rule_client.get_topic_rule(ruleName='CM32_rule')
rule_state = str(int(response["rule"]["ruleDisabled"]))
print(rule_state)