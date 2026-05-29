## Compatibility
### Conexant SmartSCM-USB modems
- I-O Data P2GATE DFML-560/P2
- ASCII ASC-1605M56
- Aiwa PV-PS200
- Melco IGM-UB56PS2C

### PCTel Solsis modems
- Suntac OnlineStation MS56KPS2

## Usage
```bash
make
sudo modprobe usbserial
sudo rmmod smartscm_usb.ko;  sudo insmod smartscm_usb.ko
sudo rmmod onlinestation.ko; sudo insmod onlinestation.ko
```
