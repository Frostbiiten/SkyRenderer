// STD
#include <chrono>

// RAYLIB
#include <raylib.h>
#include <raylib-cpp.hpp>

// SKY
#include <sky/AppMan.h>
#include <sky/Model.h>
#include <sky/PixelBuff.h>
#include <sky/Camera.h>

namespace sky
{
	namespace time
	{
        std::chrono::time_point<std::chrono::high_resolution_clock> prevFrameTime;
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
        std::chrono::duration<double> deltaTime;

        void init()
        {
            startTime = std::chrono::high_resolution_clock::now();
        }

		void tick()
		{
            auto t = std::chrono::high_resolution_clock::now();
			deltaTime = (t - prevFrameTime);
            prevFrameTime = t;
		}
	}

    // Camera
    namespace scene
    {
        sky::Camera cam;
        constexpr float MOVE_SPEED = 3.0f;
        constexpr float LOOK_SPEED = 0.15f;
        float pitch = 0;
        float yaw = 0;

        int id = 0;
        std::array<std::pair<std::string, int>, 7> names
        {{

            {"rim-lighting", 4},
            {"low-poly flat", 1},
            {"high-poly flat", 1},
            {"gouraud shading", 1},
            {"smooth shading (expensive)", 1},
            {"many cubes", 4},
            {"clipping test", 1}
        }};

        int shading = 0;
        std::array<std::pair<sky::ShadeFunc, std::string>, 5> shading_models
        {{
             {sky::shade_lambert, "lambert"},
             {sky::shade_half_lambert, "half-lambert"},
             {sky::shade_toon, "toon*"},
             {sky::shade_phong, "phong"},
             {sky::shade_rim, "rim"},
        }};

        bool blit_depth = false;
    }

    namespace input
    {
        void handle(float dt)
        {
            Vector3 localMove = { 0, 0, 0 };

            if (IsKeyPressed(KEY_I))
            {
                scene::blit_depth = !scene::blit_depth;
            }

            if (IsKeyPressed(KEY_O))
            {
                scene::id = (scene::id + 1) % scene::names.size();
                scene::shading = scene::names[scene::id].second;
            }

            if (IsKeyPressed(KEY_P))
            {
                scene::shading = (scene::shading + 1) % scene::shading_models.size();
            }

            if (IsKeyDown(KEY_W)) localMove.z -= 1.0f;
            if (IsKeyDown(KEY_S)) localMove.z += 1.0f;
            if (IsKeyDown(KEY_A)) localMove.x -= 1.0f;
            if (IsKeyDown(KEY_D)) localMove.x += 1.0f;
            if (IsKeyDown(KEY_SPACE))  localMove.y += 1.0f;
            if (IsKeyDown(KEY_LEFT_SHIFT)) localMove.y -= 1.0f;

            if (Vector3LengthSqr(localMove) != 0.0f)
            {
                Vector3 worldMove = Vector3RotateByQuaternion(Vector3Normalize(localMove), scene::cam.rotation);
                scene::cam.position = Vector3Add(scene::cam.position, Vector3Scale(worldMove, scene::MOVE_SPEED * dt));
            }

            if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
            {
                Vector2 md = GetMouseDelta();
                scene::yaw   += -md.x * scene::LOOK_SPEED;
                scene::pitch += -md.y * scene::LOOK_SPEED;

                // prevent flipping upside down
                scene::pitch = Clamp(scene::pitch, -89.9f, 89.9f);

                Quaternion qPitch = QuaternionFromEuler(scene::pitch * DEG2RAD, 0, 0);
                Quaternion qYaw   = QuaternionFromEuler(0, scene::yaw * DEG2RAD, 0);

                scene::cam.rotation = QuaternionNormalize(QuaternionMultiply(qYaw, qPitch));
            }
        }
    }

	namespace render
	{
		std::unique_ptr<raylib::Window> windowPtr;
        std::unique_ptr<raylib::RenderTexture2D> bufferPtr;
        sky::PixelBuff softwareRenderBuffer(render::pixelWidth, render::pixelHeight);
        Texture2D screenTexture;
        Font main_font;

        void init()
		{
			windowPtr = std::make_unique<raylib::Window>(pixelWidth * scaleFactor, pixelHeight * scaleFactor, "Sky", FLAG_WINDOW_ALWAYS_RUN);
            bufferPtr = std::make_unique<raylib::RenderTexture2D>(pixelWidth, pixelHeight);
            scene::shading = scene::names[scene::id].second;

            screenTexture = LoadTextureFromImage(
                    Image{
                            .data = softwareRenderBuffer.get_frame().data(),
                            .width = render::pixelWidth,
                            .height = render::pixelHeight,
                            .mipmaps = 1,
                            .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
                    }
            );

            main_font = LoadFontEx("assets/SpaceMono.ttf", 25, 0, 0);
		}


        float time = 0;
        sky::Model cube("assets/cube.obj", MatrixTranslate(0.f, 0.f, -2.f));
        sky::Model suzanne_01("assets/suzanne_01.obj", MatrixTranslate(0.f, 0.f, -2.f));
        sky::Model suzanne_02("assets/suzanne_02.obj", MatrixTranslate(0.f, 0.f, -2.f));
        sky::Model suzanne_03("assets/suzanne_03.obj", MatrixTranslate(0.f, 0.f, -2.f));

        void draw()
		{
            // Begin buffer draw
            BeginTextureMode(*bufferPtr);
            ClearBackground(raylib::Color::Black());

            {
                softwareRenderBuffer.clear();
                std::chrono::duration<double> passed = (std::chrono::high_resolution_clock::now() - sky::time::startTime);

                switch (scene::id)
                {
                    default:
                    case 0:

                        for (int z = 0; z < 3; ++z)
                        {
                            for (int y = 0; y < 3; ++y)
                            {
                                for (int x = 0; x < 3; ++x)
                                {
                                    auto v = Vector3Scale(Vector3(x - 1, y - 1, z - 1),sin(passed.count() + x + y + z - 3));

                                    suzanne_01.set_transform(
                                            MatrixMultiply(
                                                MatrixMultiply(
                                                MatrixRotateXYZ(
                                                        Vector3Scale(
                                                                Vector3{1.f, 1.f, 1.f},
                                                                sin(passed.count() + x + y + z))), MatrixTranslate(v.x, v.y, v.z)),

                                            MatrixTranslate((x - 1) * 4, (y - 1) * 4, (z + 2) * -4)));

                                    softwareRenderBuffer.draw_model_flat(scene::cam, suzanne_01, scene::shading_models[scene::shading].first);
                                }
                            }
                        }

                        break;

                    case 1:
                        suzanne_01.set_transform(MatrixMultiply(MatrixRotateXYZ({0.f, sinf(passed.count() * 1.f), 0.f}), MatrixTranslate(0.f, 0.f, -2.f)));
                        softwareRenderBuffer.draw_model_flat(scene::cam, suzanne_01, scene::shading_models[scene::shading].first);
                        break;

                    case 2:
                        suzanne_03.set_transform(MatrixMultiply(MatrixRotateXYZ({0.f, sinf(passed.count() * 1.f), 0.f}), MatrixTranslate(0.f, 0.f, -2.f)));
                        softwareRenderBuffer.draw_model_flat(scene::cam, suzanne_03, scene::shading_models[scene::shading].first);
                        break;

                    case 3:
                        suzanne_02.set_transform(MatrixMultiply(MatrixRotateXYZ({0.f, sinf(passed.count() * 1.f), 0.f}), MatrixTranslate(0.f, 0.f, -2.f)));
                        softwareRenderBuffer.draw_model_gouraud(scene::cam, suzanne_02, scene::shading_models[scene::shading].first);
                        break;

                    case 4:
                        suzanne_02.set_transform(MatrixMultiply(MatrixRotateXYZ({0.f, sinf(passed.count() * 1.f), 0.f}), MatrixTranslate(0.f, 0.f, -2.f)));
                        softwareRenderBuffer.draw_model(scene::cam, suzanne_02, scene::shading_models[scene::shading].first);
                        break;

                    case 5:

                        for (int z = 0; z < 10; ++z)
                        {
                            for (int y = 0; y < 10; ++y)
                            {
                                for (int x = 0; x < 10; ++x)
                                {
                                    auto v = Vector3Scale(Vector3(x, y, z),sin(passed.count() + x + y + z));
                                    cube.set_transform(MatrixMultiply(
                                            MatrixTranslate(v.x, v.y, v.z),
                                            MatrixTranslate((x - 4.5f) * 7, (y - 1) * 7, (z + 2) * -7)));

                                    softwareRenderBuffer.draw_model_flat(scene::cam, cube, scene::shading_models[scene::shading].first);
                                }
                            }
                        }
                        break;

                    case 6:

                        for (int z = 0; z < 2; ++z)
                        {
                            for (int y = 0; y < 2; ++y)
                            {
                                for (int x = 0; x < 2; ++x)
                                {
                                    auto v = Vector3Scale(Vector3(x - 0.5f, y - 0.5f, z - 0.5f),sin(passed.count() + x + y + z - 2));

                                    suzanne_01.set_transform(
                                            MatrixMultiply(
                                                    MatrixMultiply(
                                                            MatrixRotateXYZ(
                                                                    Vector3Scale(
                                                                            Vector3{1.f, 1.f, 1.f},
                                                                            sin(passed.count() + x + y + z))), MatrixTranslate(v.x, v.y, v.z)),

                                                    MatrixTranslate((x - 0.5f) * 1, (y - 0.5f) * 1, (z + 2) * -1)));

                                    softwareRenderBuffer.draw_model_flat(scene::cam, suzanne_01, scene::shading_models[scene::shading].first);
                                }
                            }
                        }

                        break;
                }

                if (scene::blit_depth) softwareRenderBuffer.blit_depthbuffer();

                // Update and draw tex
                UpdateTexture(screenTexture, softwareRenderBuffer.get_frame().data());
                DrawTextureEx(screenTexture, {0, 0}, 0.0f, 1, WHITE);
            }
            EndTextureMode();

            // Begin screen blit
            BeginDrawing();

            // ClearBackground(BLACK); we don't actually need this ... because the buffer covers it anyway

            DrawTexturePro(
                    bufferPtr->texture,
                    { 0, 0, (float)pixelWidth, -(float)pixelHeight }, // flip Y
                    { 0, 0, (float)pixelWidth * scaleFactor, (float)pixelHeight * scaleFactor },
                    { 0, 0 },
                    0.0f,
                    WHITE
            );

            DrawTextEx(main_font, scene::names[scene::id].first.c_str(), (Vector2){ 20, 15 }, (float)main_font.baseSize, 0, WHITE);
            DrawTextEx(main_font, (std::to_string(int(1 / time::deltaTime.count())) + " fps").c_str(), (Vector2){ 20, 40 }, (float)main_font.baseSize, 0, WHITE);

            std::string txt = ("shading: " + scene::shading_models[scene::shading].second);
            txt += scene::blit_depth ? " (z-buffer)" : "";
            DrawTextEx(main_font, txt.c_str(), (Vector2){ 20, render::pixelHeight * render::scaleFactor - 40 }, (float)main_font.baseSize, 0, WHITE);

            EndDrawing();
		}
	}

	void init()
	{
		render::init();
        time::init();

        while (!render::windowPtr->ShouldClose())
        {
            time::tick();
            input::handle((float)sky::time::deltaTime.count());
            render::draw();
        }
	}
}