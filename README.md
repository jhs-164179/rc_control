# 1st: KENTECH_Public connection
```bash
sudo nmcli --ask connection up "KENTECH_Public"
```

# 2nd: Prepare venv
```bash
sudo apt-get update && sudo apt-get install python3-is-python python3-venv python3.11-venv -y
python -m venv .rc_control
# activate
source .rc_control/bin/activate
```