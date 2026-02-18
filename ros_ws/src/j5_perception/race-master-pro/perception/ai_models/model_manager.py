"""
AI Model Manager — Download, load, and swap detection/tracking models.
Supports YOLO v8 variants and custom models via a pluggable interface.
"""

import os
import subprocess
import sys
from pathlib import Path
from typing import Optional


MODELS_DIR = Path(__file__).parent.parent / "models"

# Known pre-trained models with download URLs
KNOWN_MODELS = {
    "yolov8n": {
        "url": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt",
        "filename": "yolov8n.pt",
        "description": "YOLOv8 Nano — fastest, lowest accuracy (recommended for development)",
    },
    "yolov8s": {
        "url": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s.pt",
        "filename": "yolov8s.pt",
        "description": "YOLOv8 Small — good balance of speed and accuracy",
    },
    "yolov8m": {
        "url": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8m.pt",
        "filename": "yolov8m.pt",
        "description": "YOLOv8 Medium — higher accuracy, slower",
    },
    "yolov8l": {
        "url": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8l.pt",
        "filename": "yolov8l.pt",
        "description": "YOLOv8 Large — highest accuracy, requires GPU",
    },
}


class ModelManager:
    def __init__(self, models_dir: Optional[Path] = None):
        self.models_dir = models_dir or MODELS_DIR
        self.models_dir.mkdir(parents=True, exist_ok=True)
        self._loaded_detector = None
        self._loaded_tracker = None

    def list_available(self) -> list[dict]:
        """List all known models and whether they're downloaded."""
        result = []
        for name, info in KNOWN_MODELS.items():
            model_path = self.models_dir / info["filename"]
            result.append(
                {
                    "name": name,
                    "description": info["description"],
                    "downloaded": model_path.exists(),
                    "path": str(model_path),
                    "size_mb": round(model_path.stat().st_size / 1024 / 1024, 1)
                    if model_path.exists()
                    else None,
                }
            )
        # Also list any custom models in the directory
        for f in self.models_dir.glob("*.pt"):
            if f.name not in [m["filename"] for m in KNOWN_MODELS.values()]:
                result.append(
                    {
                        "name": f.stem,
                        "description": "Custom model",
                        "downloaded": True,
                        "path": str(f),
                        "size_mb": round(f.stat().st_size / 1024 / 1024, 1),
                    }
                )
        return result

    def download_model(self, model_name: str) -> str:
        """Download a pre-trained model. Returns the local path."""
        if model_name not in KNOWN_MODELS:
            raise ValueError(
                f"Unknown model: {model_name}. Available: {list(KNOWN_MODELS.keys())}"
            )

        info = KNOWN_MODELS[model_name]
        target_path = self.models_dir / info["filename"]

        if target_path.exists():
            print(f"Model {model_name} already downloaded at {target_path}")
            return str(target_path)

        print(f"Downloading {model_name} from {info['url']}...")

        try:
            import urllib.request

            urllib.request.urlretrieve(info["url"], str(target_path))
            print(f"Downloaded {model_name} to {target_path}")
        except Exception as e:
            # Fallback: try curl
            try:
                subprocess.run(
                    ["curl", "-L", "-o", str(target_path), info["url"]],
                    check=True,
                )
                print(f"Downloaded {model_name} to {target_path}")
            except Exception:
                raise RuntimeError(f"Failed to download {model_name}: {e}")

        return str(target_path)

    def get_model_path(self, model_name: str) -> Optional[str]:
        """Get path to a downloaded model, or None if not downloaded."""
        if model_name in KNOWN_MODELS:
            path = self.models_dir / KNOWN_MODELS[model_name]["filename"]
        else:
            path = self.models_dir / f"{model_name}.pt"
        return str(path) if path.exists() else None

    def ensure_dependencies(self):
        """Ensure required Python packages are installed for AI inference."""
        try:
            import ultralytics  # noqa: F401

            print("ultralytics package found")
        except ImportError:
            print("Installing ultralytics (YOLO)...")
            subprocess.check_call(
                [sys.executable, "-m", "pip", "install", "ultralytics"]
            )

        try:
            import cv2  # noqa: F401

            print("OpenCV found")
        except ImportError:
            print("Installing opencv-python...")
            subprocess.check_call(
                [sys.executable, "-m", "pip", "install", "opencv-python-headless"]
            )


# Convenience function for CLI use
def main():
    """CLI entry point: python -m perception.ai_models.model_manager [list|download <name>]"""
    import sys

    mgr = ModelManager()

    if len(sys.argv) < 2:
        print(
            "Usage: python -m perception.ai_models.model_manager [list|download <name>]"
        )
        sys.exit(1)

    cmd = sys.argv[1]
    if cmd == "list":
        print("\nAvailable Models:")
        print("-" * 60)
        for m in mgr.list_available():
            status = "✅ Downloaded" if m["downloaded"] else "⬇️  Not downloaded"
            size = f" ({m['size_mb']} MB)" if m["size_mb"] else ""
            print(f"  {m['name']:12s} {status}{size}")
            print(f"               {m['description']}")
        print()

    elif cmd == "download":
        if len(sys.argv) < 3:
            print(
                "Usage: python -m perception.ai_models.model_manager download <model_name>"
            )
            print(f"Available: {', '.join(KNOWN_MODELS.keys())}")
            sys.exit(1)
        model_name = sys.argv[2]
        path = mgr.download_model(model_name)
        print(f"\nModel ready at: {path}")

    else:
        print(f"Unknown command: {cmd}")
        sys.exit(1)


if __name__ == "__main__":
    main()
