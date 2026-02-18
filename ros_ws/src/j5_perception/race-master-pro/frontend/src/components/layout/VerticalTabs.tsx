import { LayoutDashboard, BarChart3, Trophy, Users, Camera, Settings } from 'lucide-react'
import type { ViewTab } from '@/types'

interface VerticalTabsProps {
    activeTab: ViewTab
    onTabChange: (tab: ViewTab) => void
}

const tabs: { id: ViewTab; icon: typeof LayoutDashboard; label: string }[] = [
    { id: 'dashboard', icon: LayoutDashboard, label: 'Dashboard' },
    { id: 'analytics', icon: BarChart3, label: 'Analytics' },
    { id: 'championship', icon: Trophy, label: 'Championship' },
    { id: 'racers', icon: Users, label: 'Racers' },
    { id: 'vision', icon: Camera, label: 'Vision' },
    { id: 'settings', icon: Settings, label: 'Settings' },
]

export default function VerticalTabs({ activeTab, onTabChange }: VerticalTabsProps) {
    return (
        <nav className="app-sidebar">
            {tabs.map(({ id, icon: Icon, label }) => (
                <button
                    key={id}
                    className={`sidebar-tab ${activeTab === id ? 'active' : ''}`}
                    onClick={() => onTabChange(id)}
                    title={label}
                    aria-label={label}
                    aria-current={activeTab === id ? 'page' : undefined}
                >
                    <Icon size={18} />
                </button>
            ))}
        </nav>
    )
}
