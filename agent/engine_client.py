import httpx
import os

class EngineClient:
    def __init__(self, base_url=None):
        self.base_url = base_url or os.getenv('ENGINE_URL', 'http://localhost:8080')
        self.client = httpx.Client(base_url=self.base_url, timeout=30.0)

    def run_start(self, seed, run_id, command):
        r = self.client.post('/run/start', json={'seed': seed, 'run_id': run_id, 'command': command})
        r.raise_for_status()
        return r.json()

    def state(self):
        r = self.client.get('/state')
        r.raise_for_status()
        return r.json()

    def look(self):
        r = self.client.get('/look')
        r.raise_for_status()
        return r.content, r.headers

    def depth(self):
        r = self.client.get('/depth')
        r.raise_for_status()
        return r.json()

    def move(self, distance):
        r = self.client.post('/move', json={'distance': distance})
        r.raise_for_status()
        return r.json()

    def turn(self, degrees):
        r = self.client.post('/turn', json={'degrees': degrees})
        r.raise_for_status()
        return r.json()

    def pickup(self):
        r = self.client.post('/pickup', json={})
        r.raise_for_status()
        return r.json()

    def drop(self):
        r = self.client.post('/drop', json={})
        r.raise_for_status()
        return r.json()

    def run_end(self):
        r = self.client.post('/run/end')
        r.raise_for_status()
        return r.json()
