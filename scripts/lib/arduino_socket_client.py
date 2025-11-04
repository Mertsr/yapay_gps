import websocket
import json
import time
from threading import Thread, Lock
from typing import Dict, Optional

class MultiArduinoSocketClient:
    """Multi Arduino WebSocket client for ROS2 integration"""
    
    def __init__(self):
        self.devices = {
            'MEGA_BLDC': {'port': 8888, 'ws': None, 'connected': False},
            'MEGA_STEP': {'port': 8889, 'ws': None, 'connected': False},
            'UNO_SENSORS': {'port': 8890, 'ws': None, 'connected': False}
        }
        self.host = '127.0.0.1'
        self.response_lock = Lock()
        self.responses = {}
        
    def connect_all(self):
        """Connect to all Arduino devices"""
        for device_name, device_info in self.devices.items():
            try:
                self._connect_device(device_name)
            except Exception as e:
                print(f"Failed to connect to {device_name}: {e}")
    
    def _connect_device(self, device_name):
        """Connect to a specific Arduino device"""
        device_info = self.devices[device_name]
        url = f"ws://{self.host}:{device_info['port']}/"
        
        try:
            ws = websocket.WebSocketApp(
                url,
                on_message=lambda ws, msg: self._on_message(device_name, msg),
                on_error=lambda ws, error: self._on_error(device_name, error),
                on_close=lambda ws, close_status_code, close_msg: self._on_close(device_name),
                on_open=lambda ws: self._on_open(device_name)
            )
            
            # Start the WebSocket in a separate thread
            thread = Thread(target=ws.run_forever, daemon=True)
            thread.start()
            
            device_info['ws'] = ws
            time.sleep(0.5)  # Give time to connect
            
        except Exception as e:
            print(f"Error connecting to {device_name}: {e}")
    
    def _on_open(self, device_name):
        """Handle WebSocket connection opened"""
        print(f"Connected to {device_name}")
        self.devices[device_name]['connected'] = True
    
    def _on_message(self, device_name, message):
        """Handle received message"""
        try:
            data = json.loads(message)
            with self.response_lock:
                if device_name not in self.responses:
                    self.responses[device_name] = []
                self.responses[device_name].append(data)
        except json.JSONDecodeError:
            print(f"Invalid JSON from {device_name}: {message}")
    
    def _on_error(self, device_name, error):
        """Handle WebSocket error"""
        print(f"WebSocket error for {device_name}: {error}")
        self.devices[device_name]['connected'] = False
    
    def _on_close(self, device_name):
        """Handle WebSocket connection closed"""
        print(f"Connection closed for {device_name}")
        self.devices[device_name]['connected'] = False
    
    def send_command(self, command: str):
        """Send command to appropriate Arduino device based on command format"""
        if not command:
            return False
            
        # Parse command format: TYPE,COMMAND,VALUE
        try:
            parts = command.split(',')
            if len(parts) < 2:
                print(f"Invalid command format: {command}")
                return False
                
            command_type = parts[0].upper()
            
            # Route command to appropriate device
            target_device = self._get_target_device(command_type)
            if not target_device:
                print(f"Unknown command type: {command_type}")
                return False
                
            return self._send_to_device(target_device, command)
            
        except Exception as e:
            print(f"Error parsing command '{command}': {e}")
            return False
    
    def _get_target_device(self, command_type: str) -> Optional[str]:
        """Determine target device based on command type"""
        bldc_commands = ['M', 'MOTOR', 'BLDC']
        stepper_commands = ['S', 'STEPPER', 'STEP', 'ANGLE']
        sensor_commands = ['SENSOR', 'READ', 'STATUS']
        
        if command_type in bldc_commands:
            return 'MEGA_BLDC'
        elif command_type in stepper_commands:
            return 'MEGA_STEP'
        elif command_type in sensor_commands:
            return 'UNO_SENSORS'
        else:
            # Default to BLDC for unknown commands
            return 'MEGA_BLDC'
    
    def _send_to_device(self, device_name: str, command: str) -> bool:
        """Send command to specific device"""
        device_info = self.devices[device_name]
        
        if not device_info['connected'] or not device_info['ws']:
            print(f"Device {device_name} not connected")
            return False
            
        try:
            message = {
                "command": command,
                "timestamp": time.time()
            }
            device_info['ws'].send(json.dumps(message))
            print(f"Sent to {device_name}: {command}")
            return True
            
        except Exception as e:
            print(f"Error sending command to {device_name}: {e}")
            return False
    
    def get_device_status(self) -> Dict[str, bool]:
        """Get connection status of all devices"""
        return {name: info['connected'] for name, info in self.devices.items()}
    
    def close(self):
        """Close all WebSocket connections"""
        for device_name, device_info in self.devices.items():
            if device_info['ws']:
                try:
                    device_info['ws'].close()
                except:
                    pass
        print("All Arduino connections closed")

# Backwards compatibility for single Arduino usage
class ArduinoSocketClient:
    """Single Arduino socket client for backwards compatibility"""
    
    def __init__(self, port=8888):
        self.multi_client = MultiArduinoSocketClient()
        self.multi_client.connect_all()
        self.port = port
    
    def send_command(self, command: str):
        """Send command using multi client"""
        return self.multi_client.send_command(command)
    
    def close(self):
        """Close connections"""
        self.multi_client.close()

