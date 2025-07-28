# Drivers

Drivers have sources for `ak09915` `bmp280` `icm20602`
`mmc5983` and `pca9685`

## Setup rpi-4b first!!
```bash
sudo cp ./config.txt /boot/firmware/config.txt
```

## Create a venv if not created
```bash
cd ~

# create venv
python -m venv test-env

# if you want to activate env on startup 
echo "source $HOME/test-env/bin/activate" >> ~/.bashrc
```

To install drivers run: 

```bash
mkdir -p ~/drivers
mv <path to this directory-drivers> ~/drivers
sudo chmod +x ./setup_drivers.sh 
sudo ./setup_drivers.sh 
```

To cat your i2c config into a file:
```bash
chmod +x ./i2c_config.sh
./i2c_config.sh
```
