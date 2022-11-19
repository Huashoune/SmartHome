import type { Variants } from 'framer-motion'
import { m } from 'framer-motion'
import { observer } from 'mobx-react-lite'
import { useRouter } from 'next/router'
import { memo, useState } from 'react'
import {
  IoAddOutline,
  IoRemoveOutline,
  IoSettingsOutline,
} from 'react-icons/io5'
import { message } from 'react-message-popup'

import {
  Button,
  Checkbox,
  Dropdown,
  Input,
  Modal,
  Text,
} from '@nextui-org/react'

import type { LinkageType } from '~/store/device/type';
import { DEVICE } from '~/store/device/type'

import { useStore } from '../../../store/index'

const buttonAnimation: Variants = {
  whileHover: {
    scale: 1.1,
    transition: {
      type: 'spring',
      stiffness: 300,
      damping: 17,
    },
  },
  whileTap: {
    scale: 0.8,
  },
}

export const Setting = memo(() => {
  const [visible, setVisible] = useState(false)
  const router = useRouter()
  const { deviceStore } = useStore()
  const closeHandler = () => {
    if (visible) {
      setVisible(false)
      deviceStore.startLinkage()
    }
  }

  const handleChange = (e) => {
    if (e === 'linkage') {
      setVisible(true)
    } else if (e === 'logout') {
      localStorage.removeItem('smarthome-token')
      router.push('/login')
      message.success('成功退出登录')
    }
  }
  return (
    <div>
      <Modal
        closeButton
        aria-labelledby="modal-title"
        open={visible}
        preventClose={true}
        onClose={closeHandler}
        width={'550px'}
      >
        <Modal.Header>
          <Text h3>联动设置</Text>
        </Modal.Header>
        <Modal.Body>
          <LinkageBody />
        </Modal.Body>

        <Modal.Footer>
          <Button auto onClick={closeHandler}>
            确定
          </Button>
        </Modal.Footer>
      </Modal>

      <div className="absolute right-5 top-5 phone:right-1 phone:top-1 cursor-pointer flex justify-center items-center">
        <Dropdown placement="bottom-left">
          <Dropdown.Trigger>
            <m.div
              whileHover="whileHover"
              whileTap="whileTap"
              variants={buttonAnimation}
            >
              <IoSettingsOutline size={30} />
            </m.div>
          </Dropdown.Trigger>
          <Dropdown.Menu
            color="secondary"
            aria-label="Avatar Actions"
            onAction={(key) => handleChange(key)}
          >
            <Dropdown.Item key="profile" css={{ height: '$18' }}>
              <Text b color="inherit" css={{ d: 'flex' }}>
                当前账号
              </Text>
              <Text b color="inherit" css={{ d: 'flex' }}>
                admin
              </Text>
            </Dropdown.Item>
            <Dropdown.Item key="linkage" withDivider color="primary">
              联动模式
            </Dropdown.Item>
            <Dropdown.Item key="logout" color="error" withDivider>
              退出登录
            </Dropdown.Item>
          </Dropdown.Menu>
        </Dropdown>
      </div>
    </div>
  )
})

const LinkageBodyArray = ['温度', '湿度', '光照', '燃气', '人体红外']

const LinkageBody = observer(() => {
  const { deviceStore } = useStore()
  return (
    <div className='flex flex-col gap-4'>
      {deviceStore.linkage.map((item,index) => (
        <LinkageItem key={index} linkage={item} index={index}/>
      ))}
    </div>
  )
})

const LinkageItem = observer(({linkage,index}:{linkage:LinkageType,index:number}) => {
  const {deviceStore} = useStore()
  const currentLinkValue = () => {
    const current =
      linkage.sensor === '燃气'
        ? ['安全', '危险']
        : ['无人', '有人']
    return current
  }
  return (
    <div className="flex gap-2 items-center">
     <Checkbox.Group
        value={linkage.launch ? ['open'] : []}
        className="flex items-center justify-center"
      >
        <Checkbox
          value="open"
          size="lg"
          onChange={(e: boolean) => (linkage.launch = e)}
        />
      </Checkbox.Group>

      <Dropdown>
        <Dropdown.Button flat css={{ tt: 'capitalize' }}>
          {linkage.sensor}
        </Dropdown.Button>
        <Dropdown.Menu
          aria-label="Single selection actions"
          disallowEmptySelection
          selectionMode="single"
          selectedKeys={linkage.sensor}
          onSelectionChange={(key: any) => {
            linkage.sensor = key.anchorKey
          }}
        >
          {LinkageBodyArray.map((item) => (
            <Dropdown.Item key={item}>{item}</Dropdown.Item>
          ))}
        </Dropdown.Menu>
      </Dropdown>

      {linkage.sensor === '燃气' ||
     linkage.sensor === '人体红外' ? (
        <Dropdown>
          <Dropdown.Button flat css={{ tt: 'capitalize' }}>
            {linkage.sensor === '燃气'
              ? linkage.value.gasValue
              : linkage.value.humanValue}
          </Dropdown.Button>
          <Dropdown.Menu
            aria-label="Single selection actions"
            disallowEmptySelection
            selectionMode="single"
            selectedKeys={
              linkage.sensor === '燃气'
                ? linkage.value.gasValue
                : linkage.value.humanValue
            }
            onSelectionChange={(key: any) =>
              linkage.sensor === '燃气'
                ? (linkage.value.gasValue = key.anchorKey)
                : (linkage.value.humanValue = key.anchorKey)
            }
          >
            {currentLinkValue().map((item) => (
              <Dropdown.Item key={item}>{item}</Dropdown.Item>
            ))}
          </Dropdown.Menu>
        </Dropdown>
      ) : (
        <>
          <Dropdown>
            <Dropdown.Button flat css={{ tt: 'capitalize' }}>
              {linkage.condition}
            </Dropdown.Button>
            <Dropdown.Menu
              aria-label="Single selection actions"
              disallowEmptySelection
              selectionMode="single"
              selectedKeys={linkage.condition}
              onSelectionChange={(key: any) =>
                (linkage.condition = key.anchorKey)
              }
            >
              <Dropdown.Item key={'>'}>大于</Dropdown.Item>
              <Dropdown.Item key={'<'}>小于</Dropdown.Item>
            </Dropdown.Menu>
          </Dropdown>

          <Input
            width="52px"
            placeholder="多少"
            value={linkage.value.inputValue}
            onChange={(e) =>
              (linkage.value.inputValue = e.target.value)
            }
          />
        </>
      )}

      <Dropdown>
        <Dropdown.Button flat css={{ tt: 'capitalize' }}>
          {linkage.state}
        </Dropdown.Button>
        <Dropdown.Menu
          aria-label="Single selection actions"
          disallowEmptySelection
          selectionMode="single"
          selectedKeys={linkage.state}
          onSelectionChange={(key: any) =>
            (linkage.state = key.anchorKey)
          }
        >
          <Dropdown.Item key={'开'}>开</Dropdown.Item>
          <Dropdown.Item key={'关'}>关</Dropdown.Item>
        </Dropdown.Menu>
      </Dropdown>

      <Dropdown>
        <Dropdown.Button flat css={{ tt: 'capitalize' }}>
          {linkage.device}
        </Dropdown.Button>
        <Dropdown.Menu
          aria-label="Single selection actions"
          disallowEmptySelection
          selectionMode="single"
          selectedKeys={linkage.device}
          onSelectionChange={(key: any) =>
            (linkage.device = key.anchorKey)
          }
        >
          {Object.keys(DEVICE).map((key,index) => (
            <Dropdown.Item key={DEVICE[key]}>{DEVICE[key]}</Dropdown.Item>
          ))}
          
        </Dropdown.Menu>
      </Dropdown>
      <Button.Group size='sm' rounded >
         <Button css={{padding:'10px',height:'30px'}} onClick={()=>deviceStore.reduceLinkage(index)}>
          <IoRemoveOutline />
        </Button>
        <Button css={{padding:'10px',height:'30px'}} onClick={()=>deviceStore.addLinkage()}>
          <IoAddOutline />
        </Button>
      </Button.Group>
    </div>
  )
})
