import { observer } from 'mobx-react-lite'
import type { FC } from 'react'
import type { DeviceMatchType } from '~/store/device/type'

import { useStore } from '../../../store/index'

const DeviceControl = () => {
  const { deviceStore } = useStore()
  return (
    <div >
      <h1 className="font-ui text-4xl font-medium select-none phone:text-2xl">设备控制</h1>
      <div className="grid mt-3 grid-cols-6 w900:grid-cols-3 phone:gap-3">
        {deviceStore.getDeviceName().map((item) => (
          <DeviceItem
            key={item?.name}
            name={item?.name}
            status={item?.status}
            icon={item?.icon}
            viewName={item?.viewName}
          />
        ))}
      </div>
    </div>
  )
}

const DeviceItem: FC<DeviceMatchType> = ({ name, status, icon,viewName }) => {
  const { deviceStore } = useStore()
  return (
    <div className="flex flex-col items-center">
      <div
        className={`w-30 h-30 phone:w-20 phone:h-20 rounded-full bg-white ${
          !status && 'opacity-40'
        } flex justify-center items-center select-none transition-color duration-300 shadow-xl cursor-pointer`}
        onClick={() => {
          deviceStore.changeDeviceStatus(name as string)
        }}
      >
        <img src={icon} alt="" className="w-1/2 h-1/2" />
      </div>
      <p className="text-center select-none text-xl mt-1 ">{viewName}</p>
    </div>
  )
}
export default observer(DeviceControl)
