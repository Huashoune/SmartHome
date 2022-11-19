export interface SubTopicType {
  
}


export interface DeviceType {
  Fan: boolean
  Curtain: boolean
  Humidifier: boolean
  Lamp: boolean
  Access: boolean
  Buzzer: boolean
}

export interface DeviceMatchType {
  name?: string
  status?: boolean
  icon?: string
  viewName?: string
}

export enum MQTT {
  MQTT_PUB_TEMP = 'ESP32/DHT22/TEMP',
  MQTT_PUB_HUM = 'ESP32/DHT22/HUMI',
  MQTT_PUB_LUX = 'ESP32/BH1750',
  MQTT_PUB_MQ4 = 'ESP32/MQ4',
  MQTT_PUB_HCSR501 = 'ESP32/HCSR501',
}

export const LinkageMap = {
  温度: MQTT.MQTT_PUB_TEMP,
  湿度: MQTT.MQTT_PUB_HUM,
  光照: MQTT.MQTT_PUB_LUX,
  燃气: MQTT.MQTT_PUB_MQ4,
  人体红外: MQTT.MQTT_PUB_HCSR501
}

export const DEVICE = {
  Fan: '风扇',
  Curtain: '窗帘',
  Humidifier: '加湿器',
  Lamp: '射灯',
  Access: '门禁',
  Buzzer: '蜂鸣器',
}

export const DEVICE_AGAINST = {
  风扇: 'fan',
  窗帘: 'curtain',
  加湿器: 'humidifier',
  射灯: 'lamp',
  门禁: 'access',
  蜂鸣器: 'buzzer',
}
export const esp32_JDQ = 'ESP32/Switch'

export interface SensorType {
  MQTT_PUB_TEMP: SensorItemType
  MQTT_PUB_HUM: SensorItemType
  MQTT_PUB_MQ4: SensorItemType
  MQTT_PUB_Bh1750: SensorItemType
  MQTT_PUB_HCSR501: SensorItemType
}

export interface SensorItemType {
  name: string
  value: string
  img: string
  unit: string
}

export interface LinkageType {
  sensor: string
  condition: string
  value: {
    inputValue: string
    gasValue: string
    humanValue: string
  }
  state: string
  device: string
  launch: boolean
}
