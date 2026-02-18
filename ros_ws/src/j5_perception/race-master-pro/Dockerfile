FROM python:3.12-slim AS backend

WORKDIR /app/backend
COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY backend/ .

# Create data directory for SQLite
RUN mkdir -p data

EXPOSE 8080
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8080"]

# ── Frontend Build ──────────────────────────────────────────
FROM node:20-slim AS frontend-build

WORKDIR /app/frontend
COPY frontend/package.json frontend/package-lock.json* ./
RUN npm install
COPY frontend/ .
RUN npm run build

# ── Production ──────────────────────────────────────────────
FROM python:3.12-slim AS production

WORKDIR /app

# Install backend deps
COPY backend/requirements.txt backend/
RUN pip install --no-cache-dir -r backend/requirements.txt

# Copy backend code
COPY backend/ backend/
RUN mkdir -p backend/data

# Copy frontend build
COPY --from=frontend-build /app/frontend/dist /app/frontend/dist

# Copy config
COPY config/ config/

EXPOSE 8080
CMD ["uvicorn", "backend.app.main:app", "--host", "0.0.0.0", "--port", "8080"]
