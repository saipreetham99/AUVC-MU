## IMU

Press Reset Position after a 30 secs on the client
```bash
# lan server
python mahony_filter_server.py
```
```bash
# lan client
python mahony_filter_client.py
```

```bash
# wifi server
python mahony_filter_server.py --host <server_ip>
```
```bash
# wifi client
python mahony_filter_client.py --host <server_ip>
```

## Thrusters

```bash
python thruster_client.py
```
```bash
python thruster_server.py
```

## Camera
```bash
python udp_video_client.py --server_ip <server_ip>
python udp_video_client_artag.py --server<server_ip>
```
```bash
# on the sub
python udp_video_server.py --client <client_ip> --quality <quality>
```
