"""Entrypoint to run the race manager FastAPI service with uvicorn."""

from __future__ import annotations

import uvicorn

from .config import get_settings
from .main import app


def main() -> None:
    settings = get_settings()
    uvicorn.run(app, host=settings.http_host, port=settings.http_port)


if __name__ == "__main__":
    main()
