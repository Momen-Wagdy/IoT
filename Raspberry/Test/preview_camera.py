import subprocess

def preview_camera():
    try:
        subprocess.run(['libcamera-hello'], check=True)
    except subprocess.CalledProcessError as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    preview_camera()
