#!/usr/bin/env python3
"""
Simple diagnostic tool to check if camera data is being received
Run this separately to isolate networking issues
"""

import socket
import struct
import time
import cv2
import numpy as np

def test_camera_feed(port, name, client_ip="192.168.2.1"):
    """Test if camera data is being received on the specified port"""
    print(f"\n🔍 Testing {name} camera feed on {client_ip}:{port}")
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((client_ip, port))
        s.settimeout(5.0)  # 5 second timeout
        
        print(f"✅ Successfully bound to {client_ip}:{port}")
        print("🔌 Waiting for data...")
        
        packet_count = 0
        start_time = time.time()
        
        while time.time() - start_time < 30:  # Test for 30 seconds
            try:
                data, addr = s.recvfrom(65536)
                packet_count += 1
                
                print(f"📦 Packet {packet_count}: {len(data)} bytes from {addr}")
                
                if len(data) >= 6:
                    try:
                        fid, cs, cid = struct.unpack("!HHH", data[:6])
                        print(f"   Frame ID: {fid}, Chunks: {cs}, Chunk ID: {cid}")
                    except struct.error as e:
                        print(f"   ⚠️ Header parse error: {e}")
                else:
                    print(f"   ⚠️ Packet too small for header")
                
                # Try to detect if this looks like JPEG data
                jpeg_payload = data[6:] if len(data) > 6 else data
                if jpeg_payload.startswith(b'\xff\xd8'):
                    print("   🖼️ Looks like JPEG start")
                elif b'\xff\xd9' in jpeg_payload:
                    print("   🖼️ Contains JPEG end marker")
                    
            except socket.timeout:
                elapsed = time.time() - start_time
                print(f"⏱️ No data received in 5s (total elapsed: {elapsed:.1f}s)")
                continue
                
        print(f"\n📊 Results for {name} (port {port}):")
        print(f"   Total packets received: {packet_count}")
        print(f"   Test duration: 30 seconds")
        
        if packet_count == 0:
            print(f"❌ No data received on port {port}")
            print("   Possible issues:")
            print("   - Camera not sending data")
            print("   - Wrong port number")
            print("   - Firewall blocking")
            print("   - Network routing issues")
        else:
            print(f"✅ Receiving data on port {port}")
            
    except OSError as e:
        print(f"❌ Failed to bind to port {port}: {e}")
        print("   Possible issues:")
        print("   - Port already in use")
        print("   - Insufficient permissions")
        print("   - Invalid port number")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        try:
            s.close()
        except:
            pass

def main():
    print("🚀 Camera Feed Network Diagnostic Tool")
    print("This will test if camera data is being received on the expected ports")
    
    client_ip = "192.168.2.1"  # Your client machine IP
    
    # Test real camera (port 10001) - this should receive from submarine at 192.168.2.11
    test_camera_feed(10001, "REAL", client_ip)
    
    # Test sim camera (port 60000) - this might be localhost/127.0.0.1
    print(f"\n🔍 Testing SIM camera feed on localhost:60000 (typical for local simulator)")
    test_camera_feed(60000, "SIM", "127.0.0.1")
    
    print("\n🏁 Diagnostic complete!")
    print("\nNext steps:")
    print("1. If no data received on port 10001, check your real camera sender")
    print("2. If data received but main app still shows no feed, it's a processing issue")
    print("3. Compare with port 60000 results to see differences")

if __name__ == "__main__":
    main()
