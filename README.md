# 🌨️ SkyRenderer

A cross-platform C++20 software renderer supporting flat, gouraud and per-pixel shading.
Supports optional SIMD acceleration via SIMDe [SIMDe](https://github.com/simd-everywhere/simde).
Although [Raylib](https://github.com/raysan5/raylib) is only used for cross-platform display, it is mainly used for input and display. All rendering is essentially done on an ARGB pixel vector and displayed to the screen by copying to a Raylib RenderTexture.

|  |  |
|----------|----------|
| ![Screenshot 2025-06-16 082634](https://github.com/user-attachments/assets/a31777b3-e8a3-4574-af61-d02de81562c0) | ![Screenshot 2025-06-16 082701](https://github.com/user-attachments/assets/488167b8-ac1c-435d-b5e5-0aee632d5af9)  |
| ![Screenshot 2025-06-16 082802](https://github.com/user-attachments/assets/d5600c83-a846-4675-ab94-c6f0751c176c) | ![Screenshot 2025-06-16 082718](https://github.com/user-attachments/assets/44ec3f04-e51c-4dcb-ad7e-66ca6b394c3c)  |

### Shading Approach Comparison
- **Flat** shading simply computes color on a per-face basis. This is the fastest draw mode.
- **Smooth** (or "phong" shading in the image) computes color on a per-pixel basis. By far the most intensive shading mode, but also the most-accurate.
- **Gouraud** shading is a compromise between the two models, computing color on a per-vertex basis and interpolating between them via barycentric coordinates. This can cause a large loss of fidelity on models with large faces (high space between vertices).

![Diagram comparing flat, Gouraud, and Phong shading](https://img.tfd.com/cde/_SHADING.GIF)

## Building

#### CMake + Ninja (recommended)
```
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cd build
```
Now, you can run with `.\SkyRenderer.exe` on windows or `./SkyRenderer` on linux.

#### Clion
- Simply open the root folder containing CMakeLists.txt in the IDE. When the "Open Project Wizard" opens, you can simply press OK if you wish to change no options. After a minute or two, the dependencies should be downloaded and set up to run.

#### Visual Studio
- Simply open the root folder containing CMakeLists.txt in the IDE. Open Solution explorer, right click on CMakeLists and click "Switch to CMake Targets View". After a minute or two of automatically setting up dependencies, youll be able to right click SkyRenderer (executable) and run.

#### Web (via [Emscripten](https://emscripten.org/))
This is one of the reasons I used Raylib. It makes building to web completely painless.
```
emcmake cmake -B build-web -DCMAKE_BUILD_TYPE=Release -DPLATFORM=Web
cmake --build build-web
cd build-web
python -m http.server 8080
```
Then, visit `http://localhost:8080/SkyRenderer.html`

In the future, I am planning to ...
- Implement uv texture sampling
- Add skybox support (panoramic, cubemap, etc)
- Further improve performance, fully taking advantage of memory locality and parallelism. This may require large rewrites.
- Perhaps switch to fixed-point maths (?)
