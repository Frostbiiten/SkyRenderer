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
    // leftovers from more complex timer... will keep for the time-being
	namespace time
	{
        namespace
        {
            std::chrono::time_point<std::chrono::high_resolution_clock> prevFrameTime;
            std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
            std::chrono::duration<double> deltaTime;
        }

		int frameSamples = 5;
		int samplesRemaining = frameSamples;
		float sampleTime;

		constexpr int fpsHistoryLen = 120;
		std::array<float, fpsHistoryLen> fpsHistory;
		int currentSampleIndex = 0;
		float sampleFps;

		int truncInterval = 20;
		int truncCount = truncInterval;
		float lerpFps, lerpFpsTrunc;

        void init()
        {
            startTime = std::chrono::high_resolution_clock::now();
        }

		void tick()
		{
			deltaTime = (std::chrono::high_resolution_clock::now() - prevFrameTime);
            prevFrameTime = std::chrono::high_resolution_clock::now();

			--samplesRemaining;
			if (samplesRemaining < 0)
			{
				samplesRemaining = frameSamples;
				sampleFps = 1000.f * (float)frameSamples / sampleTime;
				fpsHistory[currentSampleIndex] = sampleFps;
				if (++currentSampleIndex >= fpsHistoryLen) currentSampleIndex = 0;
				sampleTime = 0.f;

				if (--truncCount < 0)
				{
					truncCount = truncInterval;
					lerpFpsTrunc = lerpFps;
				}
			}

			sampleTime += (float)std::chrono::duration_cast<std::chrono::milliseconds>(deltaTime).count();
			lerpFps = std::lerp(lerpFps, sampleFps, 0.3f);
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
        std::array<std::string, 6> names
                {
                        "rim-lighting",
                        "high-poly",
                        "gouraud shading",
                        "smooth shading (expensive)",
                        "flat shading",
                };
    }

    namespace input
    {
        void handle(float dt)
        {
            Vector3 localMove = { 0, 0, 0 };

            if (IsKeyPressed(KEY_ENTER))
            {
                scene::id = (scene::id + 1) % scene::names.size();
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

            screenTexture = LoadTextureFromImage(
                    Image{
                            .data = softwareRenderBuffer.framebuffer.data(),
                            .width = render::pixelWidth,
                            .height = render::pixelHeight,
                            .mipmaps = 1,
                            .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
                    }
            );

            main_font = LoadFontEx("assets/SpaceMono.ttf", 25, 0, 0);
		}


        float time = 0;

        //sky::Model model("test_3.obj", MatrixTranslate(0.f, 0.f, -2.f));
        //sky::Model model("assets/test.obj", MatrixTranslate(0.f, 0.f, -2.f));
        //sky::Model ocean_model("assets/ocean.obj", MatrixTranslate(0.f, 0.f, 0.f));
        //sky::Model cube("cube.obj", MatrixTranslate(0.f, 0.f, 0.f));

        sky::Model smooth_suzanne("assets/test_smooth.obj", MatrixTranslate(0.f, 0.f, -2.f));
        sky::Model highpoly_suzanne("assets/test_3.obj", MatrixTranslate(0.f, 0.f, -2.f));

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
                                    smooth_suzanne.set_transform(MatrixMultiply(
                                            MatrixRotateXYZ(
                                                    Vector3Scale(
                                                            Vector3{1.f, 1.f, 1.f},
                                                            sin(passed.count() + x + y + z))),

                                            MatrixTranslate((x - 1) * 4, (y - 1) * 4, (z + 2) * -4)));

                                    softwareRenderBuffer.draw_model_flat(scene::cam, smooth_suzanne, sky::shade_rim);
                                }
                            }
                        }

                        break;

                    case 1:
                        highpoly_suzanne.set_transform(MatrixTranslate(0.f, 0.f, -2.f));
                        softwareRenderBuffer.draw_model_flat(scene::cam, smooth_suzanne, sky::shade_half_lambert);
                        break;

                    case 2:
                        highpoly_suzanne.set_transform(MatrixMultiply(MatrixRotateXYZ({0.f, sinf(passed.count() * 1.f), 0.f}), MatrixTranslate(0.f, 0.f, -2.f)));
                        softwareRenderBuffer.draw_model_gouraud(scene::cam, highpoly_suzanne, sky::shade_half_lambert);
                        break;
                }
                if (false)
                {
                    //softwareRenderBuffer.draw_model(cam, ocean_model);

                    //model.set_transform(MatrixTranslate(0.f, sin(passed.count() * 1.f), -2.f));
                    //softwareRenderBuffer.draw_model_flat(scene::cam, model);

                    //smooth_model.set_transform(MatrixTranslate(2.f, sin(passed.count() * 0.9f), -2.f));
                    //softwareRenderBuffer.draw_model(scene::cam, smooth_model);

                    //smooth_model.set_transform(MatrixTranslate(-2.f, sin(passed.count() * 0.9f), -2.f));
                    //softwareRenderBuffer.draw_model_gouraud(scene::cam, smooth_model);
                }

                ///*
                //*/

                //softwareRenderBuffer.blit_depthbuffer();
                //softwareRenderBuffer.apply_depth_blur(2.f, 1);
                softwareRenderBuffer.apply_glow();


                // Update and draw tex
                UpdateTexture(screenTexture, softwareRenderBuffer.framebuffer.data());
                DrawTextureEx(screenTexture, {0, 0}, 0.0f, 1, WHITE);
            }
            EndTextureMode();

            // Begin screen blit
            BeginDrawing();

            // ClearBackground(BLACK); we don't actually need this ...

            DrawTexturePro(
                    bufferPtr->texture,
                    { 0, 0, (float)pixelWidth, -(float)pixelHeight },// (flip Y)
                    { 0, 0, (float)pixelWidth * scaleFactor, (float)pixelHeight * scaleFactor },
                    { 0, 0 },
                    0.0f,
                    WHITE
            );

            DrawTextEx(main_font, scene::names[scene::id].c_str(), (Vector2){ 20, 15 }, (float)main_font.baseSize, 0, WHITE);
            DrawTextEx(main_font, (std::to_string(int(1 / GetFrameTime())) + " fps").c_str(), (Vector2){ 20, 40 }, (float)main_font.baseSize, 0, WHITE);
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