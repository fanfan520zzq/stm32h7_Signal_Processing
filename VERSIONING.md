# Versioning Notes

This workspace currently contains several independent change sets. Do not use
`git add .` until the vendor-line-ending noise has been isolated.

## Safety Branch

- `backup/dirty-before-recon-cleanup` points at the pre-cleanup HEAD.
- No reset or cleanup has been performed.

## Commit Groups

Use interactive staging for mixed files such as `main.c`, `CMakeLists.txt`,
`config.h`, and `DDS.c`.

1. Project hygiene
   - `.gitattributes`
   - `.gitignore`
   - `VERSIONING.md`

2. Sweep and classification logic
   - `Core/Src/sweep_grid.c`
   - `Core/Inc/classify.h`
   - `Core/Src/classify.c`
   - `BodePlot_Tools/plot_bode.py`
   - `BodePlot_Tools/serial_data.txt` if the captured data should be kept

3. IIR experiment
   - `Core/Inc/iir_runtime.h`
   - `Core/Src/iir_runtime.c`
   - IIR-related hunks in `Core/Task/ADCTask.c`, `Core/Task/DDS.c`, `Core/Src/main.c`, and `CMakeLists.txt`
   - `BodePlot_Tools/iir_reconstruct.py`

4. Reconstruction PLL pipeline
   - `Core/Inc/recon_*.h`
   - `Core/Src/recon_*.c`
   - reconstruction hunks in `Core/Src/main.c`
   - reconstruction hunks in `Core/Task/DDS.c`
   - reconstruction source entries in `CMakeLists.txt`

5. CubeMX / hardware configuration
   - `.mxproject`
   - `IIT6_Oscilliscope.ioc`
   - `Core/Inc/tim.h`
   - `Core/Src/tim.c`
   - `Core/Src/dma.c`
   - `Core/Src/stm32h7xx_it.c`
   - `Core/Inc/stm32h7xx_it.h`
   - `Core/Inc/stm32h7xx_hal_conf.h`
   - `cmake/stm32cubemx/CMakeLists.txt`

## Do Not Mix

- Do not mix `Drivers/` changes into feature commits. The current `Drivers/`
  changes appear to be broad line-ending/vendor noise and need separate review.
- Do not commit local plot images unless they are explicitly needed as results.
- Do not commit `cmake-build-*` outputs.

## Suggested Commands

Inspect source-only changes:

```powershell
git status --short -- Core CMakeLists.txt BodePlot_Tools .gitignore .gitattributes VERSIONING.md
```

Stage a clean hygiene commit:

```powershell
git add .gitattributes .gitignore VERSIONING.md
```

Stage feature changes carefully:

```powershell
git add -p Core/Src/main.c Core/Task/DDS.c CMakeLists.txt Core/Inc/config.h
git add Core/Inc/recon_*.h Core/Src/recon_*.c
```

