import AppUIStore from './app'
import DeviceStore from './device'

export interface RootStore {
  appUIStore: AppUIStore
  deviceStore: DeviceStore
}
export class RootStore {
  constructor() {
    this.appUIStore = new AppUIStore()
    this.deviceStore = new DeviceStore()
  }

  get appStore() {
    return this.appUIStore
  }
}
