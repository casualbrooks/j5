# Race Master Pro

A comprehensive real-time race tracking system with AI-powered computer vision, designed as a ROS2 perception module. Uses cameras as data sources and AI models (YOLO + DeepSORT) to track racing vehicles, compute lap times, and visualize race data.

## Quick Start (3 Steps)

### 1. Start the Backend
```bash
cd backend
pip install -r requirements.txt
python -m app.main
```
Backend starts at `http://localhost:8080` (API docs at `/docs`).

### 2. Start the Frontend
```bash
cd frontend
npm install
npm run dev
```
Frontend starts at `http://localhost:5173`.

### 3. Run the Perception System
```bash
# Standalone mode (no ROS2 needed, uses mock cameras)
python -m perception.standalone.standalone_runner

# Or with ROS2
ros2 run racetracker_perception perception_node
```

## Features

| Feature | Status |
|---|---|
| Real-time race dashboard | ✅ |
| Leaderboard with position/timing | ✅ |
| Track visualization (canvas) | ✅ |
| Season/championship management | ✅ |
| Racer profiles & behavioral metrics | ✅ |
| SQLite database (offline-first) | ✅ |
| WebSocket real-time updates | ✅ |
| REST API (30+ endpoints) | ✅ |
| Computer vision integration | ✅ |
| AI model selector (YOLO family) | ✅ |
| ROS2 perception node | ✅ |
| Standalone dev mode | ✅ |
| Docker deployment | ✅ |

## AI Model Setup

```bash
# List available models
python -m perception.ai_models.model_manager list

# Download a model
python -m perception.ai_models.model_manager download yolov8n
```

## Project Structure

```
├── frontend/          React 19 + TypeScript + Tailwind CSS
├── backend/           FastAPI + SQLite (async)
├── perception/        AI pipeline + ROS2 node + standalone mode
├── config/            YAML configuration files
├── docs/              Detailed documentation
└── scripts/           Setup and utility scripts
```

## Configuration

Edit `config/default.yaml` for:
- Server ports
- Database path
- AI model selection
- Camera sources
- Perception mode (standalone vs ROS2)

## Documentation

- [Quick Start](docs/QUICKSTART.md)
- [Camera Setup](docs/CAMERA_SETUP.md)
- [AI Models](docs/AI_MODELS.md)
- [Cloud Deployment](docs/CLOUD_DEPLOY.md)
- [ROS2 Integration](docs/ROS2_INTEGRATION.md)

## License

MIT
