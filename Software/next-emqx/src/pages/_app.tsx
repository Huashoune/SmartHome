import 'windi.css'
import 'assets/styles/main.css'

import type { NextPage } from 'next'
import type { FC, PropsWithChildren } from 'react'
import { memo } from 'react'

import BasicLayout from '~/components/layouts/BasicLayout'
import { RootStoreProvider } from '~/context/root-store'

const App: NextPage<{ Component: any; pageProps: any; err: any }> = (props) => {
  const { Component, pageProps } = props
  return (
    <RootStoreProvider>
      <Wrapper>
        <Component {...pageProps} />
      </Wrapper>
    </RootStoreProvider>
  )
}

const Wrapper: FC<PropsWithChildren> = memo((props) => {
  return <BasicLayout>{props.children}</BasicLayout>
})

export default App
