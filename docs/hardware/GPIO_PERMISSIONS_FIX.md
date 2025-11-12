# GPIO Permissions Fix for Raspberry Pi 5

## Problem
GPIO access fails with error: `can not open gpiochip`

## Temporary Fix
```bash
sudo chmod 666 /dev/gpiochip0
```

## Permanent Fix

### Option 1: Add User to gpio Group
```bash
sudo usermod -a -G gpio $USER
# Log out and log back in for changes to take effect
```

### Option 2: Create udev Rule
```bash
sudo nano /etc/udev/rules.d/99-gpio.rules
```

Add this line:
```
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0664"
```

Then reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Option 3: Use sudo (Not Recommended)
Run scripts with sudo, but this is not recommended for security reasons.

## Verify Fix
```bash
# Check if user is in gpio group
groups

# Test GPIO access
python3 -c "import lgpio; h=lgpio.gpiochip_open(0); print('GPIO access OK')"
```

## Note
The temporary fix (`sudo chmod 666 /dev/gpiochip0`) needs to be run after each reboot. Use one of the permanent fixes above to avoid this.

