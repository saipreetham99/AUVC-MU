for bus in /dev/i2c-*; do
    bus_num=$(basename "$bus" | cut -d'-' -f2)
    echo "=== i2cdetect for /dev/i2c-$bus_num ===" >> i2c_scan_results.txt
    i2cdetect -y "$bus_num" >> i2c_scan_results.txt
    echo -e "\n" >> i2c_scan_results.txt
done
