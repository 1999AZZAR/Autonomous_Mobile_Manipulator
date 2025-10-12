#!/usr/bin/env python3

import requests
import time
import json

def test_automation_system():
    """Test the automation system"""
    print("🤖 Testing Robot Automation System")
    print("=" * 50)
    
    # Test REST API
    print("\n📡 Testing REST API...")
    try:
        response = requests.get('http://localhost:5678/health', timeout=5)
        if response.status_code == 200:
            print("✅ REST API is responding")
            print(f"   Response: {response.json()}")
        else:
            print(f"❌ REST API returned status {response.status_code}")
    except requests.exceptions.ConnectionError:
        print("❌ REST API connection failed - service may not be running")
    except requests.exceptions.Timeout:
        print("❌ REST API timeout - service may be slow to start")
    except Exception as e:
        print(f"❌ REST API error: {e}")
    
    # Test robot status
    print("\n📊 Testing Robot Status...")
    try:
        response = requests.get('http://localhost:5678/api/robot/status', timeout=5)
        if response.status_code == 200:
            print("✅ Robot status API is working")
            data = response.json()
            print(f"   Robot mode: {data.get('data', {}).get('mode', 'Unknown')}")
            print(f"   Robot state: {data.get('data', {}).get('state', 'Unknown')}")
        else:
            print(f"❌ Robot status API returned status {response.status_code}")
    except Exception as e:
        print(f"❌ Robot status error: {e}")
    
    # Test emergency stop
    print("\n🛑 Testing Emergency Stop...")
    try:
        response = requests.post('http://localhost:5678/api/robot/emergency-stop', 
                               json={"activate": False, "reason": "Test"}, timeout=5)
        if response.status_code == 200:
            print("✅ Emergency stop API is working")
            data = response.json()
            print(f"   Emergency stop active: {data.get('emergency_stop_active', 'Unknown')}")
        else:
            print(f"❌ Emergency stop API returned status {response.status_code}")
    except Exception as e:
        print(f"❌ Emergency stop error: {e}")
    
    # Test n8n webhook compatibility
    print("\n🔗 Testing n8n Webhook Compatibility...")
    try:
        response = requests.post('http://localhost:5678/webhook/robot-control', 
                               json={"command": "stop", "speed": 0.0}, timeout=5)
        if response.status_code == 200:
            print("✅ n8n webhook is working")
            data = response.json()
            print(f"   Command result: {data.get('message', 'Unknown')}")
        else:
            print(f"❌ n8n webhook returned status {response.status_code}")
    except Exception as e:
        print(f"❌ n8n webhook error: {e}")
    
    print("\n" + "=" * 50)
    print("🎯 Automation System Test Complete!")

if __name__ == "__main__":
    test_automation_system()
