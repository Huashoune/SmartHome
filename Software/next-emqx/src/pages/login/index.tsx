import type { NextPage } from 'next'
import { useRouter } from 'next/router'
import { useEffect, useRef } from 'react'
import { IoPersonOutline } from 'react-icons/io5'
import { message } from 'react-message-popup'

import { Button, Input } from '@nextui-org/react'

import LoginLayout from '~/components/layouts/LoginLayout'
import { Avatar } from '~/components/universal/Avatar'

const LoginView: NextPage = () => {
  const router = useRouter()
  const loginForm = useRef({
    username: '',
    password: '',
  })

  const handleLogin = () => {
    if (
      loginForm.current.username === 'huashou' &&
      loginForm.current.password === '123456'
    ) {
      message.success('登录成功')
      localStorage.setItem('smarthome-token', 'success')
      router.push('/')
    } else {
      message.error('用户名或密码错误')
    }
  }
  useEffect(() => {
    if (localStorage.getItem('smarthome-token') === 'success') {
      router.push('/')
    }
  }, [])

  return (
    <LoginLayout>
      <div className="flex justify-center">
        <Avatar
          imageUrl="/img/imagessmarthome.png"
          shadow={false}
          lazy={false}
        />
      </div>
      <h1 className="text-4xl font-medium tracking-wide text-center">
        智能家居控制中心
      </h1>
      <div className="flex flex-col gap-8 w-full">
        <Input
          placeholder="用户名"
          onChange={(e) => (loginForm.current.username = e.target.value)}
          size="xl"
          contentRight={<IoPersonOutline />}
        />
        <Input.Password
          placeholder="密码"
          onChange={(e) => (loginForm.current.password = e.target.value)}
          size="xl"
          onKeyDown={(e) => e.keyCode === 13 && handleLogin()}
        />
      </div>
      <Button size="lg" onClick={() => handleLogin()}>
        登录
      </Button>
    </LoginLayout>
  )
}

export default LoginView
