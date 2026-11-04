"""Launch target for the fetch HTTP server."""
from engine.fetch_server import app

__all__ = ["app"]


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8080)
