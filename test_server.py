#!/usr/bin/env python3
"""Test the premium environment via server API."""
import requests
import json
import time
import sys

BASE_URL = "http://127.0.0.1:8080"

def test_api():
    print("=" * 60)
    print("PREMIUM SCI-FI FACTORY - API TEST")
    print("=" * 60)
    
    # Start test
    print("\n1. Starting new run...")
    resp = requests.post(f"{BASE_URL}/run/start", json={
        "seed": 42,
        "run_id": "test_001",
        "command": "coral sphere"
    })
    print(f"   Status: {resp.status_code}")
    print(f"   Response: {resp.json()}")
    
    # Get state
    print("\n2. Getting robot state...")
    resp = requests.get(f"{BASE_URL}/state")
    state = resp.json()
    print(f"   Position: ({state['x']:.2f}, {state['y']:.2f})")
    print(f"   Heading: {state['heading_deg']:.1f}°")
    print(f"   Holding: {state['holding']}")
    print(f"   Step: {state['step']}")
    
    # Get camera image
    print("\n3. Getting camera image...")
    resp = requests.get(f"{BASE_URL}/look")
    print(f"   Status: {resp.status_code}")
    print(f"   Content-Type: {resp.headers.get('content-type')}")
    print(f"   Image size: {len(resp.content)} bytes")
    
    # Test move
    print("\n4. Testing robot movement...")
    resp = requests.post(f"{BASE_URL}/move", json={"distance": 0.5})
    result = resp.json()
    print(f"   Moved: {result['moved_m']:.3f}m")
    print(f"   Collided: {result['collided']}")
    print(f"   State: {result['state']}")
    
    # Test turn
    print("\n5. Testing robot turn...")
    resp = requests.post(f"{BASE_URL}/turn", json={"degrees": 90})
    result = resp.json()
    print(f"   Turned: {result['turned_deg']:.1f}°")
    print(f"   New heading: {result['heading_deg']:.1f}°")
    
    # Get new state
    print("\n6. Getting updated state...")
    resp = requests.get(f"{BASE_URL}/state")
    state = resp.json()
    print(f"   Position: ({state['x']:.2f}, {state['y']:.2f})")
    print(f"   Heading: {state['heading_deg']:.1f}°")
    
    # Test pickup
    print("\n7. Testing pickup...")
    resp = requests.post(f"{BASE_URL}/pickup")
    result = resp.json()
    print(f"   Success: {result['success']}")
    print(f"   Holding: {result['holding']}")
    
    # Get camera after pickup
    print("\n8. Camera view after pickup...")
    resp = requests.get(f"{BASE_URL}/look")
    print(f"   Image size: {len(resp.content)} bytes")
    
    # Move to drop zone
    print("\n9. Moving to drop zone...")
    for i in range(5):
        resp = requests.post(f"{BASE_URL}/move", json={"distance": 0.5})
        result = resp.json()
        if result.get("collided"):
            print(f"   Step {i+1}: collided!")
        else:
            print(f"   Step {i+1}: moved {result['moved_m']:.2f}m")
    
    # Get final state
    print("\n10. Final state...")
    resp = requests.get(f"{BASE_URL}/state")
    state = resp.json()
    print(f"   Position: ({state['x']:.2f}, {state['y']:.2f})")
    print(f"   State: {state['state']}")
    print(f"   Step: {state['step']}")
    
    # Test drop
    print("\n11. Testing drop...")
    resp = requests.post(f"{BASE_URL}/drop")
    result = resp.json()
    print(f"   Released: {result['released']}")
    print(f"   In zone: {result['in_zone']}")
    print(f"   State: {result['state']}")
    
    # End run
    print("\n12. Ending run...")
    resp = requests.post(f"{BASE_URL}/run/end")
    result = resp.json()
    print(f"   Outcome: {result['outcome']}")
    print(f"   Steps: {result['steps']}")
    
    print("\n" + "=" * 60)
    print("API TEST COMPLETE!")
    print("=" * 60)

if __name__ == "__main__":
    try:
        test_api()
    except requests.exceptions.ConnectionError:
        print("ERROR: Server not running!")
        print("Start server with: python -m engine.fetch_server")
        sys.exit(1)