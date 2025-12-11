import Head from 'next/head';
import styles from '../styles/Home.module.css';

const apiBase = process.env.NEXT_PUBLIC_API_BASE ?? 'http://localhost:4000';
const wsUrl = process.env.NEXT_PUBLIC_WS_URL ?? 'ws://localhost:4000/ws';

export default function Home() {
  return (
    <div className={styles.container}>
      <Head>
        <title>Race Manager UI</title>
        <meta
          name="description"
          content="Simple placeholder UI for Race Manager API connectivity"
        />
      </Head>

      <main className={styles.main}>
        <h1 className={styles.title}>Race Manager UI</h1>
        <p className={styles.description}>
          Set <code>NEXT_PUBLIC_API_BASE</code> and <code>NEXT_PUBLIC_WS_URL</code> to
          point at your FastAPI service. Defaults are
          <span className={styles.inlineValue}>{apiBase}</span> and
          <span className={styles.inlineValue}>{wsUrl}</span>.
        </p>
        <p className={styles.description}>
          Subscribe to websocket events (<code>lap</code>, <code>leaderboard</code>,
          <code>race_status</code>, <code>championship</code>) and render your own
          leaderboard experience here.
        </p>
      </main>
    </div>
  );
}
