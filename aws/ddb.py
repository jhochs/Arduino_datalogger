from __future__ import print_function # Python 2/3 compatibility
from builtins import input # Python 2/3 compatibility

import boto3
import json
import sys
import time

ddb_client = boto3.client('dynamodb',
		aws_access_key_id = 'key_id',
		aws_secret_access_key = 'XXXXXXX',
		region_name='us-east-2')

def create_table(table_name):
    response = ddb_client.create_table(
        AttributeDefinitions=[
            {
                'AttributeName': 't',
                'AttributeType': 'N'
            }
        ],
        TableName=table_name,
        KeySchema=[
            {
                'AttributeName': 't',
                'KeyType': 'HASH'
            }
        ],
        BillingMode='PAY_PER_REQUEST'
    )
    print(table_name + ' created')

def delete_table(table_name):
    table_info = ddb_client.describe_table(TableName=table_name)
    count = table_info['Table']['ItemCount']
    choice = input('Are you sure you want to delete table `' + table_name + '` with ' + str(count) + ' items? [y/n]: ')
    if choice=='y':
        ddb_client.delete_table(TableName=table_name)
        print(table_name + ' deleted')
    else:
        print('Aborted')

if str(sys.argv[1])=='-c':
    for table in sys.argv[2:]:
        print('Creating table ' + table + '...')
        create_table(table)
elif str(sys.argv[1])=='-d':
    for table in sys.argv[2:]:
        print('Deleting table ' + table + '...')
        delete_table(table)
elif str(sys.argv[1])=='-r':
    for table in sys.argv[2:]:
        print('Resetting table ' + table + '...')
        delete_table(table)
        time.sleep(2)
        create_table(table)
else:
    raise ValueError('Invalid argument supplied. Use:\n\t-c create\n\t-d delete\nE.g.: `python ddb.py -c cm32_data`')
