import type { FC, PropsWithChildren } from 'react'

import { Slideshow } from './slideshow'

const LoginLayout: FC<PropsWithChildren> = ({ children }) => {
  return (
    <section className="flex h-screen">
      <Slideshow />
      <div className="flex-1 flex justify-center items-center flex-col">
        <div className='min-w-[25rem] max-w-[50%] flex flex-col gap-10'>{children}</div>
      </div>
    </section>
  )
}

export default LoginLayout
