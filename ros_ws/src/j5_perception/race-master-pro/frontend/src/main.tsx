import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import { Toaster } from 'sonner'
import App from './App'
import './index.css'

createRoot(document.getElementById('root')!).render(
    <StrictMode>
        <Toaster
            position="top-right"
            richColors
            theme="dark"
            toastOptions={{
                style: {
                    background: 'var(--color-bg-card)',
                    border: '1px solid rgba(255,255,255,0.08)',
                    color: 'var(--color-text-primary)',
                },
            }}
        />
        <App />
    </StrictMode>,
)
