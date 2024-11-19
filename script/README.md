### 1. Add executable to the script

```bash
chmod +x activate_vxcan.bash
```

### 2. Create a Systemd Service File

```bash
sudo vi /etc/systemd/system/activate_vxcan.service
```

### 3. Create a Systemd Service File

```bash
[Unit]
Description=Activate vxcan

[Service]
Type=oneshot
ExecStartPre=/bin/sleep 5
ExecStart=/home/hkclr/packaging_system/script/activate_vxcan.bash
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

### 4. Enable the Service

```bash
sudo systemctl enable activate_vxcan.service
```

### 5. Start the Service

```bash
sudo systemctl start activate_vxcan.service
```

### 5. Reboot

```bash
sudo reboot now
```

### Optional

To view the logs for this service, use the following command:

```bash
journalctl -u activate_vxcan.service -n 50
```