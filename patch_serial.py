Import("env")
import os

def patch_hardware_serial():
    """Patch the HardwareSerial.cpp file to fix the 'Serial not declared' error for M5Atom Echo"""
    # Find the Arduino framework path
    framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
    
    # Define the path to the problematic file
    hardware_serial_file = os.path.join(framework_dir, "cores", "esp32", "HardwareSerial.cpp")
    
    if not os.path.exists(hardware_serial_file):
        print("WARNING: HardwareSerial.cpp not found. Skipping patch.")
        return
    
    # Read the original file
    with open(hardware_serial_file, "r") as f:
        content = f.read()
    
    # Check if we need to patch the file
    if "if(serialEvent && Serial.available()) serialEvent();" in content:
        print("Patching HardwareSerial.cpp to fix Serial undefined issue for M5Atom Echo")
        
        # Replace the problematic line
        patched_content = content.replace(
            "if(serialEvent && Serial.available()) serialEvent();",
            "#if !defined(DISABLE_SERIAL) && !defined(DISABLE_SERIALEVENT)\n    if(serialEvent && Serial.available()) serialEvent();\n#endif"
        )
        
        # Backup the original file
        backup_file = hardware_serial_file + ".backup"
        if not os.path.exists(backup_file):
            with open(backup_file, "w") as f:
                f.write(content)
                print(f"Original file backed up to {backup_file}")
        
        # Write the patched content
        with open(hardware_serial_file, "w") as f:
            f.write(patched_content)
            print("HardwareSerial.cpp successfully patched")
    else:
        print("File already patched or different structure found. Skipping.")

# Run the patch function
patch_hardware_serial() 