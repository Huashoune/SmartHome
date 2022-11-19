import { makeAutoObservable } from 'mobx'
import type { MqttClient } from 'mqtt'
import mqtt from 'mqtt'
import message from 'react-message-popup'
import { v4 as uuidv4 } from 'uuid'

import { isClientSide } from '~/utils/env'

import type {
  DeviceMatchType,
  DeviceType,
  LinkageType,
  SensorType,
} from './type'
import { DEVICE, DEVICE_AGAINST, LinkageMap, MQTT, esp32_JDQ } from './type'

export default class DeviceStore {
  device: Partial<DeviceType> = {
    Fan: false,
    Curtain: false,
    Humidifier: false,
    Lamp: false,
    Access: false,
    Buzzer: false,
  }

  sensorMap = new Map()
  sensor: SensorType = {
    MQTT_PUB_TEMP: {
      name: '温度',
      value: '0',
      img: '/img/temperature.png',
      unit: ' ℃',
    },
    MQTT_PUB_HUM: {
      name: '湿度',
      value: '0',
      img: '/img/humidity.png',
      unit: ' %',
    },
    MQTT_PUB_Bh1750: {
      name: '光照',
      value: '0',
      img: '/img/light.png',
      unit: ' Lux',
    },
    MQTT_PUB_MQ4: {
      name: '燃气',
      value: '安全',
      img: '/img/gas.png',
      unit: '',
    },
    MQTT_PUB_HCSR501: {
      name: '人体红外',
      value: '无人',
      img: '/img/human.png',
      unit: '',
    },
  }

  client: MqttClient | null = null
  setInsterval
  mode = 'home'

  linkage: LinkageType[] = [
    {
      sensor: '温度',
      condition: '>',
      value: {
        inputValue: '',
        gasValue: '安全',
        humanValue: '无人',
      },
      state: '开',
      device: '风扇',
      launch: false,
    },
  ]

  constructor() {
    makeAutoObservable(this)
    this.setMap()
    if (isClientSide()) {
      this.mqttConnect()
      this.subscribe()
    }
  }

  setMap() {
    this.sensorMap.set('ESP32/DHT22/TEMP', this.sensor.MQTT_PUB_TEMP)
    this.sensorMap.set('ESP32/DHT22/HUMI', this.sensor.MQTT_PUB_HUM)
    this.sensorMap.set('ESP32/BH1750', this.sensor.MQTT_PUB_Bh1750)
    this.sensorMap.set('ESP32/MQ4', this.sensor.MQTT_PUB_MQ4)
    this.sensorMap.set('ESP32/HCSR501', this.sensor.MQTT_PUB_HCSR501)
  }

  mqttConnect() {
    this.client = mqtt.connect('ws://43.143.48.34', {
      port: 8083,
      clientId: uuidv4(),
      path: '/mqtt',
    })
    this.client?.publish(esp32_JDQ, 'Ui_State', { qos: 0 })  //qos: 1
  }

  subscribe() {
    if (this.client) {
      this.client.subscribe(MQTT.MQTT_PUB_TEMP, { qos: 0 })
      this.client.subscribe(MQTT.MQTT_PUB_HUM, { qos: 0 })
      this.client.subscribe(MQTT.MQTT_PUB_LUX, { qos: 0 })
      this.client.subscribe(MQTT.MQTT_PUB_MQ4, { qos: 0 })
      this.client.subscribe(MQTT.MQTT_PUB_HCSR501, { qos: 0 })

      this.client.subscribe('MQTT.MQTT_PUB_HCSR501', { qos: 0 })
      this.client.subscribe('ESP32/Ui/Fan', { qos: 0 })
      this.client.subscribe('ESP32/Ui/Curtain', { qos: 0 })
      this.client.subscribe('ESP32/Ui/Humidifier', { qos: 0 })
      this.client.subscribe('ESP32/Ui/Lamp', { qos: 0 })
      this.client.subscribe('ESP32/Ui/Access', { qos: 0 })
      this.client.subscribe('ESP32/Ui/Buzzer', { qos: 0 })
      this.client.on('message', (topic: keyof typeof MQTT, message) => {
        const payload = { topic, message: message.toString() }
        if (this.sensorMap.get(payload.topic)) {
          this.sensorMap.get(payload.topic).value = payload.message
        } else {
          this.device[
            payload.topic.split('/')[payload.topic.split('/').length - 1]
          ] = payload.message.includes('Open') ? true : false
          if (payload.message.includes('Open') ) {
            this.mode = 'home'
          }
        }
      })

      this.client.on('error', () => {
        console.log('error')
      })

      this.client.on('reconnect', () => {
        console.log('Reconnecting')
      })
    }
  }

  getDeviceName() {
    const deviceMatch: Partial<DeviceMatchType[]> = []
    Object.keys(this.device).map((item) =>
      deviceMatch.push({
        name: item,
        status: this.device[item],
        icon: `/img/${item.replace(item[0], item[0].toLowerCase())}.png`,
        viewName: DEVICE[item],
      }),
    )
    return deviceMatch
  }

  changeDeviceStatus(name: string) {
    this.client?.publish(
      esp32_JDQ,
      !this.device[name] ? `${name}_Open` : `${name}_Close`,
      { qos: 0 },  //qos: 1
    )
    if (this.mode === 'leave') {
      this.changeMode('home')
    }
  }

  changeMode(name: string) {
    this.mode = name
    if (name === 'leave') {
      this.leaveHomeMode()
    }
  }

  leaveHomeMode() {
    this.client?.publish(esp32_JDQ, 'All_Close', { qos: 0}) //qos: 1
    clearInterval(this.setInsterval)
    this.linkage.forEach((item) => (item.launch = false))
  }

  addLinkage() {
    this.linkage.push({
      sensor: '温度',
      condition: '>',
      value: {
        inputValue: '',
        gasValue: '安全',
        humanValue: '无人',
      },
      state: '开',
      device: '风扇',
      launch: false,
    })
  }

  reduceLinkage(index: number) {
    if (this.linkage.length > 1) {
      this.linkage.splice(index, 1)
      clearInterval(this.setInsterval)
      this.linkage.forEach((item) => (item.launch = false))
    } else {
      message.error('至少保留一个联动')
    }
  }

  startLinkage() {
    clearInterval(this.setInsterval)
    this.setInsterval = setInterval(() => {
      this.linkage.map((item) => {
        const {
          sensor,
          condition,
          value: { inputValue, gasValue, humanValue },
          state,
          device,
          launch,
        } = item
        if (launch) {
          const _sensor = this.sensorMap.get(LinkageMap[sensor]).value
          const _device = DEVICE_AGAINST[device]
          if (sensor === '人体红外' && _sensor === humanValue) {
            this.controlDevice(_device, state)
          } else if (sensor === '燃气' && _sensor === gasValue) {
            this.controlDevice(_device, state)
          } else if (
            condition === '>' &&
            Number(_sensor) > Number(inputValue)
          ) {
            this.controlDevice(_device, state)
          } else if (
            condition === '<' &&
            Number(_sensor) < Number(inputValue)
          ) {
            this.controlDevice(_device, state)
          }
        }
      })
    }, 1000)
  }

  controlDevice(device: string, state: string) {
    const upperCaseDevice = device
      .toLowerCase()
      .replace(/( |^)[a-z]/g, (L) => L.toUpperCase())

    this.mode = 'home'
    if (this.device[upperCaseDevice] != (state === '开' ? true : false)) {
      this.client?.publish(
        esp32_JDQ,
        state === '开' ? `${upperCaseDevice}_Open` : `${upperCaseDevice}_Close`,
        { qos: 0 },   //qos: 1
      )
    }
  }
}
