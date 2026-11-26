# Connecting the RealSense camera to the ROS container (USB/IP)

Docker Desktop runs containers inside a small Linux virtual machine (VM). That VM can't see USB devices plugged into your laptop, so the camera won't show up in the ROS container by default.

To connect the camera, we need the **USB/IP server** that shares the camera over a network connection. The Docker VM attaches it as if it were plugged in.

> The steps below has been tested on Windows with Docker (WSL2 backend)

### One Time Setup
1. **Install usbipd-win.** Open **PowerShell as Administrator** (right click → *Run as administrator*) and run:
   ```powershell
   winget install --interactive --exact dorssel.usbipd-win
   ```
   If you don't have `winget`, use the `.msi` from the [usbipd-win releases page](https://github.com/dorssel/usbipd-win/releases). Reopen PowerShell afterwards.
2. **Find the camera's BUSID:**
   ```powershell
   usbipd list
   ```
   The output should be something like this.
   ```
   BUSID  VID:PID    DEVICE                                                        STATE
   2-3    8086:0b3a  Intel(R) RealSense(TM) Depth Camera 435i Depth, Intel(R) ...  Not shared
   ```
   Write down the BUSID (`2-3` here). Intel devices have VID `8086`. The BUSID changes if you use a different USB port.
3. **Share the camera**. In the example below, the BUSID is 2-3. In PowerShell as Administrator:
   ```powershell
   usbipd bind --busid 2-3
   ```
   `usbipd list` should now say `Shared`. This stays after a reboot.

### Every Session
1. **Start Docker Desktop** and wait for it to finish starting.
2. **Attach the camera**. In another terminal (regular PowerShell is fine) run:
   ```powershell
   usbipd attach --wsl --busid 2-3 --auto-attach
   ```
   **Leave this window open.** `--auto-attach` re-attaches the camera whenever it disconnects, which RealSense cameras do when the driver starts. `usbipd list` should now say `Attached`.
3. Start the container, or restart it if it was already running:
   ```powershell
   docker compose up -d
   ```
   or
   ```
   docker compose restart ros
   ```
4. Go to [Check and run the camera](#check-and-run-the-camera).

When you're done working with the camera, press `Ctrl+C` in the attach window and run `usbipd detach --busid 2-3`.

---

## Check and run the camera

1. Open the ros terminal:

    ```
    docker compose exec ros bash
    ```
    The subsequent steps are all done within this terminal.

2. Inside the ros terminal, check that the RealSense SDK can see the camera:

    ```bash
    rs-enumerate-devices --compact
    ```

    You should see the camera name and serial number. If the command isn't found, run `ls /dev/video*` instead. You should see several `/dev/videoX` entries.

3. Build the workspace and load it into this terminal:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select marker_tracking
    source install/setup.bash
    ```

    You only need to rebuild the first time, and again after you change anything in `marker_tracking`, including `config/realsense_params.yaml` and the launch file. ROS runs the installed copy in `install/`, not the files in `src/`, so an edit without a rebuild has no effect. You do need `source install/setup.bash` in every new terminal.

4. Launch the realsense camera launch file:

    ```bash
    ros2 launch marker_tracking realsense_camera.launch.py
    ```

5. In a second container terminal, check that images are arriving:

    ```bash
    ros2 topic list            # should include /camera/camera/color/image_raw
    ros2 topic hz /camera/camera/color/image_raw
    ```

6. To look at the frames/image, run rviz and open the display at http://localhost:8080/vnc.html
    ```
    rviz2
    ```

7. Set the **fixed frame** to **camera_color_frame**
8. **Add Display**: click Add -> By topic tab -> /camera -> /camera -> /color -> image_raw -> choose Image and click OK. You should see a new display in the bottom left corner.
---

## Common issues

- **The camera doesn't show up in `usbipd list` / `usbip list`.** Try another USB 3 port or cable, and close any app using the camera.

- **`usbipd bind` says the device is in use (Windows).** Close camera apps and try again. As a last resort, run `usbipd bind --busid <BUSID> --force` (Administrator), then unplug and replug.

- **`usbipd attach --wsl` fails with a WSL or distribution error (WSL 2 engine).** Make sure Docker Desktop is fully started, then update WSL by running `wsl --update` in PowerShell and restarting Docker Desktop. If it still fails, install Ubuntu with `wsl --install -d Ubuntu`, open it once to finish setup, then try again.

- **`usbip list -r host.docker.internal` hangs or can't connect (B, C).** Check that the server is running: on Windows, `usbipd list` shows `Shared`; on Mac, the `host` window is still open. On Windows, allow inbound TCP port 3240 in Windows Defender Firewall. On Mac, if you changed the address to `127.0.0.1` and it can't connect, change it back to `0, 0, 0, 0`, rebuild, and only run the server on a trusted network.

- **The server crashes on startup (C).** Run it with `sudo`. If it still crashes, unplug other USB devices so it has fewer devices to open.

- **`Cannot open '/dev/video0' Last Error: Permission denied`.** The `ros` user isn't in the `video` group. Make sure your `docker-compose.yaml` has `group_add: [video]` under the `ros` service (pull the latest version of the repo), then recreate the container with `docker compose up -d`. A plain `restart` doesn't apply compose changes. Inside the container, `id` should list `44(video)`.

- **`rs-enumerate-devices` finds nothing, but the attach worked.** Restart the ROS container with `docker compose restart ros`. The camera has to be attached before the container starts.

- **"Frames didn't arrive within 5 seconds" or "Incomplete video frame detected" or dropped frames.** USB/IP is slower than a direct USB connection. Keep the resolution and frame rate low (640x480 at 30 fps, like in `config/realsense_params.yaml`), keep the point cloud off, and make sure the camera is on a USB 3 port. `rs-enumerate-devices` should show the USB type as `3.2`. If it says `2.1`, try another port or cable. If it still doesn't work, try to lower the resolution by half.

- **Sharing on public Wi-Fi.** While a device is shared, other computers on the network may be able to attach it. On Windows, run `usbipd unbind --busid <BUSID>` when you're finished. On Mac, stop the server with `Ctrl+C`.
