import { Component, useState, useCallback, type ReactNode } from 'react'
import type { ViewTab, LiveRaceState } from '@/types'
import TopBar from '@/components/layout/TopBar'
import VerticalTabs from '@/components/layout/VerticalTabs'
import BottomBar from '@/components/layout/BottomBar'
import Dashboard from '@/components/dashboard/Dashboard'
import Analytics from '@/components/analytics/Analytics'
import ChampionshipView from '@/components/championship/ChampionshipView'
import RacerManager from '@/components/racers/RacerManager'
import VisionPanel from '@/components/vision/VisionPanel'
import SettingsPanel from '@/components/settings/SettingsPanel'
import IntegrationCheckPanel from '@/components/vision/IntegrationCheckPanel'
import { RaceProvider, useRaceContext } from '@/stores/raceStore'

class AppErrorBoundary extends Component<{ children: ReactNode }, { hasError: boolean; message: string }> {
    constructor(props: { children: ReactNode }) {
        super(props)
        this.state = { hasError: false, message: '' }
    }

    static getDerivedStateFromError(error: unknown) {
        const message = error instanceof Error ? error.message : 'Unknown render error'
        return { hasError: true, message }
    }

    override render() {
        if (this.state.hasError) {
            return (
                <div className="race-card m-4 p-4 text-sm text-rose-300">
                    <p className="font-semibold">A panel crashed while rendering.</p>
                    <p className="mt-1 text-xs text-rose-200">{this.state.message}</p>
                    <p className="mt-2 text-xs text-slate-300">Try refreshing the page. If this repeats, capture console logs and report this panel state.</p>
                </div>
            )
        }
        return this.props.children
    }
}

function AppContent() {
    const [activeTab, setActiveTab] = useState<ViewTab>('dashboard')
    const { liveRace, connectionStatus } = useRaceContext()

    const renderContent = useCallback(() => {
        switch (activeTab) {
            case 'dashboard':
                return <Dashboard />
            case 'analytics':
                return <Analytics />
            case 'championship':
                return <ChampionshipView />
            case 'racers':
                return <RacerManager />
            case 'vision':
                return <VisionPanel />
            case 'integration':
                return <IntegrationCheckPanel />
            case 'settings':
                return <SettingsPanel />
            default:
                return <Dashboard />
        }
    }, [activeTab])

    return (
        <div className="app-shell">
            <TopBar
                connectionStatus={connectionStatus}
                activeRace={liveRace}
            />
            <VerticalTabs
                activeTab={activeTab}
                onTabChange={setActiveTab}
            />
            <main className="app-main">
                <AppErrorBoundary>
                    {renderContent()}
                </AppErrorBoundary>
            </main>
            <BottomBar
                liveRace={liveRace}
                connectionStatus={connectionStatus}
            />
        </div>
    )
}

export default function App() {
    return (
        <RaceProvider>
            <AppContent />
        </RaceProvider>
    )
}
