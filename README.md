# Drone Show Studio

Takes a drone light show from a Blender animation to verified flight files, one per drone.

| Stage | Folder | What it does |
| --- | --- | --- |
| 1. Design | `stage1_designer/` | Blender add-on: turns animated 3D shapes into one point per drone, checks the motion, exports the show as JSON. |
| 2. Path planning | `stage2_core_engine/` | C++ engine (`drone_core`): assigns drones to points and plans smooth, collision-free paths. A separate check at 100 Hz rejects any show where two drones come too close. |
| 3. Digital twin | `stage3_simulation_packer/` | Simulates the fleet in wind, gusts and positioning noise (OpenCL, on the GPU or the CPU) and packs each drone's path into a checked binary flight file. |
| Desktop app | `apps/` | Drone Show Studio: runs stages 2 and 3 on a show file and shows the results — progress, why a show was rejected, 3D replay, stress test, flight files, run comparison. |

Windows 10/11 x64 is the supported platform.

## Screenshots

<table>
  <tr>
    <td width="33%"><a href="https://r2.brakenull.dev/Screenshots/drone-show-studio/01.png"><img src="https://r2.brakenull.dev/Screenshots/drone-show-studio/01.png" alt="Screenshot 1" width="100%"></a></td>
    <td width="33%"><a href="https://r2.brakenull.dev/Screenshots/drone-show-studio/02.png"><img src="https://r2.brakenull.dev/Screenshots/drone-show-studio/02.png" alt="Screenshot 2" width="100%"></a></td>
    <td width="33%"><a href="https://r2.brakenull.dev/Screenshots/drone-show-studio/03.png"><img src="https://r2.brakenull.dev/Screenshots/drone-show-studio/03.png" alt="Screenshot 3" width="100%"></a></td>
  </tr>
  <tr>
    <td width="33%"><a href="https://r2.brakenull.dev/Screenshots/drone-show-studio/04.png"><img src="https://r2.brakenull.dev/Screenshots/drone-show-studio/04.png" alt="Screenshot 4" width="100%"></a></td>
    <td width="33%"><a href="https://r2.brakenull.dev/Screenshots/drone-show-studio/05.png"><img src="https://r2.brakenull.dev/Screenshots/drone-show-studio/05.png" alt="Screenshot 5" width="100%"></a></td>
    <td width="33%"><a href="https://r2.brakenull.dev/Screenshots/drone-show-studio/06.png"><img src="https://r2.brakenull.dev/Screenshots/drone-show-studio/06.png" alt="Screenshot 6" width="100%"></a></td>
  </tr>
</table>

<sub>Click an image to open it full size.</sub>

---

## Setup

### 1. Install the prerequisites

| Tool | Notes |
| --- | --- |
| [Visual Studio 2022 or later](https://visualstudio.microsoft.com/downloads/) (Community or Build Tools) | Workload **Desktop development with C++**. It includes the compiler, CMake, Ninja and vcpkg; no separate vcpkg install is needed. |
| [Python 3.14](https://www.python.org/downloads/), 64-bit | Used to create the project's `.venv`. The Stage 2 engine is built for this exact Python version. |
| [Node.js](https://nodejs.org/) 20 or later (LTS) | For the desktop app. |
| [Rust](https://rustup.rs/) | For the desktop app (Tauri). |
| [Git](https://git-scm.com/download/win) | vcpkg downloads its package definitions with it. |
| [Blender](https://www.blender.org/download/) 4.0 or later | Only to design shows (stage 1). |

Stage 3 runs its simulation through **OpenCL**, which needs a driver:

* **With a GPU** (Intel, NVIDIA or AMD, including integrated graphics such as Intel Iris Xe): the normal graphics driver already includes OpenCL. Nothing else to install.
* **Without a usable GPU:** install Intel's free [CPU Runtime for OpenCL Applications](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-cpu-runtime-for-opencl-applications-with-sycl-support.html) (Windows installer; administrator rights needed). It supports Intel Core and Xeon processors.

To see which devices were found, after setup: `.venv\Scripts\python.exe tools\scripts\benchmark_stage3.py --list`.

### 2. Run the setup script

In PowerShell, from the repository folder:

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1
```

It checks the prerequisites (and lists anything missing, with links; it never installs them), then:

1. creates `.venv` and installs the Python packages (`requirements-dev.txt`);
2. builds and tests the stage 2 and stage 3 engines; the first run also builds the C++ libraries from `vcpkg.json`, which takes a few minutes;
3. checks that the pipeline is complete and runs a 4-drone test show through stage 2;
4. installs the desktop app's npm packages.

It is safe to run again, for example after pulling changes.

| Option | Effect |
| --- | --- |
| `-Python <path>` | Python 3.14 to create `.venv` with (default: `py -3.14`) |
| `-SkipApp` | Skip the desktop app; Node.js and Rust are then not needed |
| `-SkipTests` | Skip the unit tests and the test show |
| `-FullTests` | Also run the full Python test suite (about 5 minutes) |
| `-Clean` | Reconfigure both engines from scratch |

### 3. Start the app

```powershell
cd apps
npm run tauri dev
```

The app finds the repository, `.venv` and the built engines by itself when it runs from this folder. Runs are saved in `runs\`.

#### Or install it

```powershell
cd apps
npm run tauri build
```

This builds two installers. Either one works:

- `apps\src-tauri\target\release\bundle\msi\Drone Show Studio_<version>_x64_en-US.msi`
- `apps\src-tauri\target\release\bundle\nsis\Drone Show Studio_<version>_x64-setup.exe` (per-user, no admin rights needed)

The installed app contains only the window and its pages. All computation still runs from this repository, so keep the repository folder and its `.venv` on the machine. The installed app is outside the repository, so it cannot find it by itself. On first launch, open **Settings** and set:

| Setting | Value |
|---|---|
| Repository folder | this repository, e.g. `C:\...\drone-show-studio` |
| Python | `<repository>\.venv\Scripts\python.exe` |
| Run folders | `<repository>\runs` |

The app saves these settings in `%APPDATA%\com.brake.studio-desktop\settings.json` and reuses them on every launch. If you move the repository, set the paths again.

After a `git pull`, Python changes take effect right away and engine changes only need `setup.ps1`. Changes under `apps\` need a new `npm run tauri build` and a reinstall.

### 4. Install the Blender add-on (optional)

```powershell
.venv\Scripts\python.exe tools\scripts\build_blender_addon.py
```

In Blender: *Edit → Preferences → Add-ons*, then *Install* (Blender 4.0–4.1) or *Install from Disk* in the menu at the top right (4.2 and later), and choose `dist\stage1_designer.zip`.

Blender ships NumPy but not SciPy, which the add-on needs for sampling; msgpack is needed only for MessagePack export. Install them into Blender's own Python (adjust the path to your Blender version):

```powershell
& "C:\Program Files\Blender Foundation\Blender 4.2\4.2\python\bin\python.exe" -m pip install scipy msgpack
```

---

## Try the demo show

`export_200_cone.json` is a ready-made Blender export: 300 drones, about 108 seconds. Download it from the latest release, then:

1. Start the app and click **New run** in the sidebar.
2. Drop `export_200_cone.json` onto the page (or click to choose it). Studio checks the file; click **Create run**.
3. On the **Stage 2** tab, click **Run Stage 2** to plan collision-free paths. When it finishes, open the **Replay** tab to watch the show in 3D.
4. On the **Stage 3** tab, click **Start stress test** to fly the show in simulated weather.

From the command line instead:

```powershell
.venv\Scripts\python.exe tools\scripts\export_stage2_trajectories.py export_300_cube.json out\
```

---

## Building by hand

From a **Developer PowerShell for VS** (Start menu), in `stage2_core_engine\` or `stage3_simulation_packer\`:

```powershell
cmake --preset release
cmake --build --preset release
ctest --preset release
```

* The presets install the C++ libraries listed in `vcpkg.json` into `vcpkg_installed\` (shared by both engines). They use the vcpkg in `VCPKG_ROOT`, which the developer shell sets to Visual Studio's own copy; set it yourself to use another vcpkg.
* Stage 2 takes Python and pybind11 from `.venv`, so create it first (`python -m venv .venv`, then `.venv\Scripts\python.exe -m pip install -r requirements-dev.txt`). Pass `-DPython_EXECUTABLE=<python.exe>` to build for a different Python.
* Close the app before rebuilding stage 2: Windows can't replace `drone_core*.pyd` while it is loaded.

## Tests

```powershell
.venv\Scripts\python.exe -m pytest                                  # Python tests for all stages and the app's bridge
.venv\Scripts\python.exe tools\scripts\smoke_test_stage2.py         # 4-drone show through stage 2, checked independently
ctest --preset release                                              # C++ tests (developer shell, in an engine folder)
```

## Command-line use without the app

```powershell
# Stage 2: plan a show exported from Blender (writes trajectory_splines.json and .arrow)
.venv\Scripts\python.exe tools\scripts\export_stage2_trajectories.py <show.json> out\

# Stage 3: stress test in simulated weather, then pack and verify flight files
.venv\Scripts\python.exe -m stage3_simulation_packer.twin_sim.monte_carlo_runner out\trajectory_splines.json --runs 100
stage3_simulation_packer\build\pack_to_binary.exe out\trajectory_splines.json out\bin
stage3_simulation_packer\build\pack_to_binary.exe --verify out\bin
```

---

## Troubleshooting

| Problem | Fix |
| --- | --- |
| `running scripts is disabled on this system` | Start the script with `powershell -ExecutionPolicy Bypass -File .\setup.ps1`. |
| Setup reports Visual Studio missing although it is installed | Open the Visual Studio Installer, *Modify*, and add the **Desktop development with C++** workload. |
| `No module named 'drone_core'` | Stage 2 isn't built, or was built for another Python. Run `setup.ps1` again (with `-Clean` if needed). |
| `.venv` has the wrong Python version | Delete the `.venv` folder and run `setup.ps1` again. |
| vcpkg fails to download packages | vcpkg needs internet access and Git on the first build. |
| Stage 3 is slow | Choose the GPU in "Simulate on" (or `--device gpu`). On a CPU, 1 000 drones run at about real time; on an integrated GPU about ten times faster. |
| Stage 3 says "no OpenCL device found" | Update the graphics driver, or install Intel's CPU Runtime for OpenCL Applications (see Setup), then click *Check again* on the Settings page. |
