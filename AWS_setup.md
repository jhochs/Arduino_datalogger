# AWS setup
## Setting up an Arduino board to interact with AWS
Follow [this tutorial](https://docs.arduino.cc/tutorials/mkr-wifi-1010/securely-connecting-an-arduino-mkr-wifi-1010-to-aws-iot-core/).

The standard shadow format used for motes is:
```json
{
  "state": {
    "desired": {
      "sleeping": 0
    },
    "delta": {
      "sleeping": 0
    }
  }
}
```

## Creating a DynamoDB table and IoT rule
By following the tutorial above, you will be able to stream measurements from the Arduino to a MQTT topic. To save the contents of these messages to a table, follow these steps.
1. In the AWS console, go to **DynamoDB**. Choose "Create Table".
2. Give the table whatever name you want and enter "time" as the Partition key, and set it as a number. You do not need to add a sort key.
3. Uncheck "Use default settings" and then change the Read/write capacity mode from "Provisioned" to "On-demand" (under provisioned, only 5 reads/writes are permitted each second, meaning you won't be able to record faster than 5 Hz, and reading the table will be very slow).
4. Select “Create”.
5. Next, in the AWS console, go to **IoT Core**. On the left sidebar, select Act > Rules.
6. Select "Create". Give the rule a name and enter as the rule query statement: `SELECT * FROM 'cm2/outgoing'` where the text in quotes is the MQTT topic to which measurements are published. Note that this topic needs to match `TOPIC_OUTGOING` in the Arduino code.
7. Under "Set one or more actions", choose "Add action". Choose "Split message into multiple columns of a DynamoDB table (DynamoDBv2)".
8. Choose the table you created and create a new role. Trying to use the same role for multiple rules & tables causes issues so just create a new one for each.
9. Add action and then Create rule.
10. You can activate or deactivate your rule depending on when you want data to be logged to the table.

## Code for interacting with tables and rules
Since the above method is cumbersome to do multiple times, the functions in [aws/](aws/) can be used instead of navigating the UI. They are all python code and require the `boto3` library to be installed.

### DynamoDB - create/delete/clear a table
[ddb.py](aws/ddb.py) can be used as follows:
```python
python3 ddb.py -c 'cm32_data' # to create a new table
python3 ddb.py -r 'cm32_data' # to clear the table (deletes and recreates)
python3 ddb.py -d 'cm32_data' # to delete the table
```

### IoT - see thing shadow and update
The code snippet in [shadow.py](aws/shadow.py) can be integrated into a routine to read a mote's state and/or update it. 

### IoT - see rule state and activate/deactivate
The code snippet in [rule.py](aws/rule.py) can be integrated into a routine to read a rule state and/or activate/deactivate it.
