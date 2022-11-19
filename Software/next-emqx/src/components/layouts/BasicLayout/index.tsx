import { domMax, LazyMotion } from 'framer-motion'
import type { FC, PropsWithChildren } from 'react'
import { useEffect } from 'react'

import { NextUIProvider } from '@nextui-org/react'

import { store } from '~/store'

const BasicLayout: FC<PropsWithChildren> = ({ children }) => {
  useEffect(() => {
    store.appUIStore.updateViewport()
    window.onresize = () => {
      store.appUIStore.updateViewport()
    }
  }, [])
  return (
    <NextUIProvider>
      <LazyMotion strict features={domMax}>
        <main className="min-h-screen">{children}</main>
      </LazyMotion>
    </NextUIProvider>
  )
}

export default BasicLayout
