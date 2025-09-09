from drivers.umqtt import MQTTClient
import utime
import uasyncio as asyncio
import shared_data

import pheripherals

cfg = None
device_name = None
device_address = None
device_prefix = None
mqtt_client = None
mqtt_client_lock = None
publish_topics = []

def MQTT_callback(topic, payload):
    topic = topic.decode() if isinstance(topic, bytes) else topic
    payload = payload.decode().strip().lower() if isinstance(payload, bytes) else payload.strip().lower()

    parsed_topic = topic.split("/")
    if len(parsed_topic) < 5:
        return
    
    do = parsed_topic[4]
    try:
        pin_index = int(do[2:])
    except ValueError:
        return
    
    do_type = parsed_topic[5]
    
    if do_type == 'State':
        if payload == 'true':
            asyncio.create_task(pheripherals.do[pin_index].on())
        elif payload == 'false':
            asyncio.create_task(pheripherals.do[pin_index].off())
        else:
            try:
                timeout = float(payload)
            except:
                timeout = 0.5    # when no payload
            asyncio.create_task(pheripherals.do[pin_index].on_timeout(timeout))
            
            
    elif do_type == 'Duty':
        if payload == 'true':
            asyncio.create_task(pheripherals.do[pin_index].duty(100))
        elif payload == 'false':
            asyncio.create_task(pheripherals.do[pin_index].duty(0))
        else:
            try:
                asyncio.create_task(pheripherals.do[pin_index].duty(int(payload)))
            except:
                pass    

def MQTT_Subscription_Set(prefix, config_data):
    subscribed_topics = []
    for i in range(1, 17):
        if config_data['DO'][f'DO{i}']['mqtt']['subscribe']:
            if pheripherals.do[i].type == 'state':
                subscribed_topics.append(f"{prefix}/Set/DO{i}/State")
            elif pheripherals.do[i].type == 'pwm':
                subscribed_topics.append(f"{prefix}/Set/DO{i}/Duty")
    
    for topic in subscribed_topics:
        mqtt_client.subscribe(topic=topic, qos=1)

def build_publish_list(prefix, config_data, dht_config_data):
    global publish_topics

    if config_data["PCBTemp"]["mqtt"]["publish"]:
        topic = f"{prefix}/PCBTemp/Temp"
        interval = config_data["PCBTemp"]["mqtt"]["interval"]
        publish_topics.append([topic, interval, lambda: shared_data.pcb_temp.get()])

    for i in range(1, 17):
        do_name = f"DO{i}"
        if config_data["DO"][do_name]["mqtt"]["publish"]:
            if pheripherals.do[i].get_type() == "pwm":
                topic = f"{prefix}/{do_name}/Duty"
            else:
                topic = f"{prefix}/{do_name}/State"
            interval = config_data["DO"][do_name]["mqtt"]["interval"]
            publish_topics.append([topic, interval, lambda i=i: shared_data.dout[i].get()])

    for i in range(1, 17):
        di_name = f"DI{i}"
        if config_data["DI"][di_name]["mqtt"]["publish"]:
            if pheripherals.di[i].get_type() == "counter":
                topic = f"{prefix}/{di_name}/Value"
            else:
                topic = f"{prefix}/{di_name}/State"
            interval = config_data["DI"][di_name]["mqtt"]["interval"]
            publish_topics.append([topic, interval, lambda i=i: shared_data.din[i].get()])

    for i in range(1, 5):
        ai_name = f"AI{i}"
        if config_data["AIInt"][ai_name]["mqtt"]["publish"]:
            topic = f"{prefix}/AIInt{i}/Value"
            interval = config_data["AIInt"][ai_name]["mqtt"]["interval"]
            publish_topics.append([topic, interval, lambda i=i: shared_data.adci[i].get()])

    for i in range(1, 5):
        ai_name = f"AI{i}"
        if config_data["AIExt"][ai_name]["mqtt"]["publish"]:
            topic = f"{prefix}/AIExt{i}/Value"
            interval = config_data["AIExt"][ai_name]["mqtt"]["interval"]
            publish_topics.append([topic, interval, lambda i=i: shared_data.adce[i].get()])

    for i in range(1, 16):
        dht_name = f"DHT{i}"
        if dht_config_data[dht_name]['temperature']["mqtt"]["publish"]:
            topic = f"{prefix}/DHT{i}/Temp"
            interval = dht_config_data[dht_name]['temperature']["mqtt"]["interval"]
            publish_topics.append([topic, interval, lambda i=i: shared_data.dht_temp[i].get()])
    
    for i in range(1, 16):
        dht_name = f"DHT{i}"
        if dht_config_data[dht_name]['humidity']["mqtt"]["publish"]:
            topic = f"{prefix}/DHT{i}/Hum"
            interval = dht_config_data[dht_name]['humidity']["mqtt"]["interval"]
            publish_topics.append([topic, interval, lambda i=i: shared_data.dht_hum[i].get()])

def MQTT_Init(config_data, dht_config_data):
    global cfg
    global device_name
    global device_address
    global device_prefix
    global mqtt_client
    global mqtt_client_lock
    try:
        cfg = config_data
        mqtt_client_lock = asyncio.Lock()

        device_name = config_data['device']['dev_name']
        device_address = config_data['device']['address']
        device_prefix = f'/{device_name}/{device_address}'
                
        if config_data['MQTT']['enable'] and pheripherals.eth is not None and pheripherals.eth.isconnected():
            mqtt_client_id = device_name + '_' + device_address
            mqtt_server = config_data['MQTT']['server']
            mqtt_port = config_data['MQTT']['port']
            mqtt_ssl = config_data['MQTT']['ssl_enable']
            mqtt_username = config_data['MQTT']['username']
            mqtt_password = config_data['MQTT']['password']
            
            mqtt_client = MQTTClient(client_id=mqtt_client_id,
                                     server=mqtt_server,
                                     port=mqtt_port,
                                     user=mqtt_username,
                                     password=mqtt_password,
                                     ssl=mqtt_ssl)
            attempts = 0
            while attempts < 3:
                try:
                    mqtt_client.connect()
                    mqtt_client.set_callback(MQTT_callback)
                    MQTT_Subscription_Set(device_prefix, config_data)
                    build_publish_list(device_prefix, config_data, dht_config_data)
                    print('[OK] MQTT Init')
                    pheripherals.boot_print('[OK] MQTT Init')
                    break
                except:
                    attempts += 1
                    utime.sleep(1)
            else:
                print("[ERR] MQTT Init")
                pheripherals.boot_print("[ERR] MQTT Init")
    
    except:
        print("[ERR] MQTT Init")
        pheripherals.boot_print("[ERR] MQTT Init")
        return False
    
async def check_mqtt_subscribe_task(delay: float = 0.1):
    global mqtt_client
    while True:
        try:
            async with mqtt_client_lock:
                mqtt_client.check_msg()
        except:
            await pheripherals.disp.popup('[WARR] MQTT Lost', 0.5)
            try:
                mqtt_client.connect()
                mqtt_client.set_callback(MQTT_callback)
                MQTT_Subscription_Set(device_prefix, cfg)                
            except:
                pass
        await asyncio.sleep(delay)
            

async def publish_mqtt_task(topic, interval, func):
    global mqtt_client
    while True:
        try:
            value = await func()
            async with mqtt_client_lock:
                mqtt_client.publish(topic, str(value), True)
        except:
            try:
                mqtt_client.connect()
                mqtt_client.set_callback(MQTT_callback)
                MQTT_Subscription_Set(device_prefix, cfg)
            except:
                pass
        await asyncio.sleep(interval)



