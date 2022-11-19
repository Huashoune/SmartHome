import type { NextPage } from 'next'
import { useRouter } from 'next/router'
import React, { useEffect } from 'react'

import DeviceControl from '~/components/in-page/Home/device-control'
import Mode from '~/components/in-page/Home/mode'
import Sensor from '~/components/in-page/Home/sensor'
import HomeLayout from '~/components/layouts/HomeLayout'


const Home: NextPage = () => {
  const router = useRouter()
  useEffect(() => {
    if (localStorage.getItem('smarthome-token') !== 'success') {
      router.push('/login')
    }
  }, [])
  
  return (
    <HomeLayout>
      <DeviceControl />
      <Sensor />
      <Mode />
    </HomeLayout>
  )
}

export default Home
