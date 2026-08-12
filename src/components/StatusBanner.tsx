import type { ReactNode } from 'react'

export type StatusTone = 'info' | 'success' | 'warning' | 'error'

export interface StatusBannerProps {
  tone?: StatusTone
  title: string
  children?: ReactNode
}

export function StatusBanner({ tone = 'info', title, children }: StatusBannerProps) {
  const assertive = tone === 'error'
  return (
    <section
      aria-live={assertive ? 'assertive' : 'polite'}
      className={`status-banner status-banner--${tone}`}
      role={assertive ? 'alert' : 'status'}
    >
      <strong>{title}</strong>
      {children !== undefined && <div>{children}</div>}
    </section>
  )
}
