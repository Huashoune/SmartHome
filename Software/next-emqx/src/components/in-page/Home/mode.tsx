
import { useStore } from '../../../store/index'
import { observer } from 'mobx-react-lite';

const Mode = () => {
  return (
    <div className=''>
      <h1 className="font-ui text-4xl font-medium select-none phone:text-2xl">模式</h1>
      <div className="mt-3">
        <ModeItem />
      </div>
    </div>
  )
}

const ModeItem = observer(() => {
  const { deviceStore } = useStore()

  return (
    <div className="flex justify-between phone:gap-4">
      <div
        className={`bg-white rounded-2xl h-22 w-70 phone:w-full p-2 flex items-center gap-3 shadow-xl cursor-pointer transition-opacity duration-300 ${
          deviceStore.mode === 'leave' && 'opacity-40'
        }`}
        onClick={() => deviceStore.changeMode('home')}
      >
        <img src="/img/away_home.png" className="h-1/2" />
        <p className="text-black font-bold text-xl">普通模式</p>
      </div>

      <div
        className={`bg-white rounded-2xl h-22 w-70 phone:w-full p-2 flex items-center gap-3 shadow-xl cursor-pointer transition-opacity duration-300
      ${deviceStore.mode === 'home' && 'opacity-40'}
      `}
        onClick={() => deviceStore.changeMode('leave')}
      >
        <img src="/img/away_home.png" className="h-1/2" />
        <p className="text-black font-bold text-xl">离家模式</p>
      </div>
    </div>
  )
})

export default Mode
