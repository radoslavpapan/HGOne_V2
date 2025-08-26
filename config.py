import ujson

config_data = None
app_config_data = None
    
#Load json files
def Config_Init():
    try:
        global config_data
        
        with open ('/sd/config.json', 'r') as f:
            config_data = ujson.load(f)
        print("[OK] Config Init")
    except Exception as e:
        print("[ERR] Config Init:", e)
        return False

def Config_Update(keys, value):
    '''
    example:
    config.Config_Update(["device", "address"], '0xA1')
    '''
    try:
        global config_data
        if config_data is None:
            raise ValueError("Config not loaded")

        d = config_data
        for k in keys[:-1]:
            if k not in d:
                raise KeyError(f"Key: '{k}' not exist config.json")
            d = d[k]

        d[keys[-1]] = value

        with open('/sd/config.json', 'w') as f:
            ujson.dump(config_data, f)

        print(f"[OK] Config Update: {'.'.join(keys)} = {value}")
        return True

    except Exception as e:
        print("[ERR] Config Update:", e)
        return False