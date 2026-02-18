import { useState, useCallback } from 'react'
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
import { RaceProvider, useRaceContext } from '@/stores/raceStore'

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
                {renderContent()}
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
