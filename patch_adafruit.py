Import("env")
import os

def patch_adafruit_busio():
    """Patch the Adafruit_BusIO_Register.h file to work without Serial dependency"""
    
    # Find the Adafruit BusIO path - it's a library dependency, not a platform package
    project_dir = env.subst("$PROJECT_DIR")
    pioenv = env.subst("$PIOENV")
    
    # Most likely path for library dependency
    busio_dir = os.path.join(project_dir, ".pio", "libdeps", pioenv, "Adafruit BusIO")
    
    # Alternative paths to check
    if not os.path.exists(busio_dir):
        # Try with different casing or formats
        alternatives = [
            os.path.join(project_dir, ".pio", "libdeps", pioenv, "Adafruit_BusIO"),
            os.path.join(project_dir, ".pio", "libdeps", pioenv, "adafruit_busio"),
            os.path.join(project_dir, ".pio", "libdeps", pioenv, "adafruit busio")
        ]
        
        for alt_path in alternatives:
            if os.path.exists(alt_path):
                busio_dir = alt_path
                break
    
    if not os.path.exists(busio_dir):
        print("WARNING: Could not find Adafruit BusIO library directory. Skipping patch.")
        print(f"Library should be in: {os.path.join(project_dir, '.pio', 'libdeps', pioenv)}")
        # List the contents of the libdeps directory for debugging
        libdeps_dir = os.path.join(project_dir, ".pio", "libdeps", pioenv)
        if os.path.exists(libdeps_dir):
            print("Available libraries:")
            for lib in os.listdir(libdeps_dir):
                print(f"  - {lib}")
        return
    
    print(f"Found Adafruit BusIO at: {busio_dir}")
    
    # Define the path to the problematic file
    busio_file = os.path.join(busio_dir, "Adafruit_BusIO_Register.h")
    
    if not os.path.exists(busio_file):
        # Try to find the file if it's not in the expected location
        for root, dirs, files in os.walk(busio_dir):
            if "Adafruit_BusIO_Register.h" in files:
                busio_file = os.path.join(root, "Adafruit_BusIO_Register.h")
                break
    
    if not os.path.exists(busio_file):
        print(f"WARNING: Could not find Adafruit_BusIO_Register.h. Skipping patch.")
        return
    
    # Read the original file
    with open(busio_file, "r") as f:
        content = f.read()
    
    # Check if we need to patch the file
    if "&Serial" in content:
        print("Patching Adafruit_BusIO_Register.h to work without Serial dependency")
        
        # Replace "&Serial" with "nullptr" - will use provided stream instead of default
        patched_content = content.replace("Stream *s = &Serial", "Stream *s = nullptr")
        
        # Backup the original file
        backup_file = busio_file + ".backup"
        if not os.path.exists(backup_file):
            with open(backup_file, "w") as f:
                f.write(content)
                print(f"Original file backed up to {backup_file}")
        
        # Write the patched content
        with open(busio_file, "w") as f:
            f.write(patched_content)
            print("Adafruit_BusIO_Register.h successfully patched")
    else:
        print("File already patched or different structure found. Skipping.")

# Run the patch function
patch_adafruit_busio() 