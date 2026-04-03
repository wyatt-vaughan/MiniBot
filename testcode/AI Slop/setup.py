"""
Chess Robot Coordinator - Setup and Installation
"""

import subprocess
import sys

def install_dependencies():
    """Install required packages"""
    print("Installing dependencies...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])
    print("Dependencies installed successfully!")

if __name__ == "__main__":
    install_dependencies()
