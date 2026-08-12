declare module 'react-katex' {
  import type { ComponentType } from 'react'

  interface KatexProps {
    children?: string
    errorColor?: string
    math?: string
    renderError?: (error: Error) => React.ReactNode
  }

  export const BlockMath: ComponentType<KatexProps>
  export const InlineMath: ComponentType<KatexProps>
}
