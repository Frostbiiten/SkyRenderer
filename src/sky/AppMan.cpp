// std
#include <stack>
#include <chrono>

// SFML
// #include <SFML/Graphics.hpp>
#include <raylib.h>
#include <raylib-cpp.hpp>

// SKY
#include <sky/AppMan.h>
#include <sky/Debug.h>
// #include <sky/Input.h>
// #include <sky/Tilemap.h>
// #include <sky/Camera.h>
#include <sky/Model.h>
#include <sky/PixelBuff.h>

// imgui
// #include <imgui.h>
// #include <imgui-SFML.h>

// FLECS
// #include <flecs.h>

namespace sky
{
	namespace time
	{
        namespace
        {
            std::chrono::time_point<std::chrono::high_resolution_clock> prevFrameTime;
            std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
            std::chrono::duration<double> deltaTime;
        }

		int frameSamples = 30;
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

	namespace render
	{
		std::unique_ptr<raylib::Window> windowPtr;
        std::unique_ptr<raylib::RenderTexture2D> bufferPtr;
        sky::PixelBuff softwareRenderBuffer(render::pixelWidth, render::pixelHeight);
        Texture2D screenTexture;

        //sky::Model model("test_3.obj", MatrixTranslate(0.f, 0.f, -2.f));
        sky::Model model("test_smooth.obj", MatrixTranslate(0.f, 0.f, -2.f));
        sky::Model ocean_model("ocean.obj", MatrixTranslate(0.f, 0.f, 0.f));
        //sky::Model cube("cube.obj", MatrixTranslate(0.f, 0.f, 0.f));
        sky::Camera cam;

        void init()
		{
			windowPtr = std::make_unique<raylib::Window>(pixelWidth * scaleFactor, pixelHeight * scaleFactor, "Sky", FLAG_WINDOW_ALWAYS_RUN);
            bufferPtr = std::make_unique<raylib::RenderTexture2D>(pixelWidth, pixelHeight);
			//windowPtr->setView(windowView);

            screenTexture = LoadTextureFromImage(
                    Image{
                            .data = softwareRenderBuffer.framebuffer.data(),
                            .width = render::pixelWidth,
                            .height = render::pixelHeight,
                            .mipmaps = 1,
                            .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
                    }
            );

			// Init imgui
            /*
			ImGui::SFML::Init(*windowPtr);
			ImGuiIO& ImGuiIO = ImGui::GetIO();
			ImGuiIO.FontDefault = ImGuiIO.Fonts->AddFontFromFileTTF("common/font/IBMPlexMono-Regular.ttf", 20.f);
			ImGui::SFML::UpdateFontTexture();
			misc::skinImGui();
            */

			// RenderTexture Buffer + Sprite
            /*
			sf::ContextSettings settings;
			settings.antialiasingLevel = render::aaLevel;
			bufferPtr = std::make_unique<sf::RenderTexture>();
			bufferPtr->create(pixelWidth, pixelHeight, settings);
            */

			//bufferSprite = sf::Sprite(bufferPtr->getTexture()); // <- stable ref to renderTexture result buffer
			//bufferSprite.setOrigin(pixelWidth / 2.f, pixelHeight / 2.f);

		}
        constexpr float MOVE_SPEED = 3.0f;
        constexpr float LOOK_SPEED = 0.15f;
        float pitch = 0;
        float yaw = 0;

        void handle_input(float dt)
        {
            Vector3 localMove = { 0, 0, 0 };

            if (IsKeyDown(KEY_W)) localMove.z -= 1.0f;
            if (IsKeyDown(KEY_S)) localMove.z += 1.0f;
            if (IsKeyDown(KEY_A)) localMove.x -= 1.0f;
            if (IsKeyDown(KEY_D)) localMove.x += 1.0f;
            if (IsKeyDown(KEY_SPACE))  localMove.y += 1.0f;
            if (IsKeyDown(KEY_LEFT_SHIFT)) localMove.y -= 1.0f;

            if (Vector3LengthSqr(localMove) != 0.0f)
            {
                Vector3 worldMove = Vector3RotateByQuaternion(Vector3Normalize(localMove), cam.rotation);
                cam.position = Vector3Add(cam.position, Vector3Scale(worldMove, MOVE_SPEED * dt));
            }

            if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
            {
                Vector2 md = GetMouseDelta();
                yaw   += -md.x * LOOK_SPEED;
                pitch += -md.y * LOOK_SPEED;

                // clamp pitch so you can't flip upside down
                pitch = Clamp(pitch, -89.9f, 89.9f);

                Quaternion qPitch = QuaternionFromEuler(pitch * DEG2RAD, 0, 0);
                Quaternion qYaw   = QuaternionFromEuler(0, yaw * DEG2RAD, 0);

                cam.rotation = QuaternionNormalize(QuaternionMultiply(qYaw, qPitch));
            }
        }


        float time = 0;

        void draw()
		{
            // Begin buffer draw
            BeginTextureMode(*bufferPtr);
            ClearBackground(raylib::Color::Black());

            {
                softwareRenderBuffer.clear();
                std::chrono::duration<double> passed = (std::chrono::high_resolution_clock::now() - sky::time::startTime);
                model.set_transform(MatrixTranslate(0.f, sin(passed.count() * 1.f), 2.f));

                for (int z = 0; z < 1; ++z)
                {
                    for (int y = 0; y < 1; ++y)
                    {
                        for (int x = 0; x < 1; ++x)
                        {
                            model.set_transform(MatrixMultiply(
                                                               MatrixRotateXYZ(
                                                                Vector3Scale(
                                                                        Vector3{1.f, 1.f, 1.f},
                                                                        sin(passed.count() + x + y + z))), MatrixTranslate((x - 0) * 4, y * 4, z * -4)));

                            //softwareRenderBuffer.draw_model(cam, model, sky::shade_rim);
                            softwareRenderBuffer.draw_model(cam, model, sky::shade_half_lambert);
                        }
                    }
                }

                {
                    //softwareRenderBuffer.draw_model(cam, ocean_model);
                    // softwareRenderBuffer.draw_model_multithreaded(cam, model);
                }


                // Update and draw tex
                UpdateTexture(screenTexture, softwareRenderBuffer.framebuffer.data());
                DrawTextureEx(screenTexture, {0, 0}, 0.0f, 1, WHITE);
            }
            EndTextureMode();

            // Begin screen blit
            BeginDrawing();
            ClearBackground(BLACK); // or whatever background
            DrawTexturePro(
                    bufferPtr->texture,
                    { 0, 0, (float)pixelWidth, -(float)pixelHeight },// (flip Y)
                    { 0, 0, (float)pixelWidth * scaleFactor, (float)pixelHeight * scaleFactor },
                    { 0, 0 },
                    0.0f,
                    WHITE
            );
            DrawText(std::to_string(sky::time::lerpFps).c_str(), 20, 20, 20, WHITE);
            EndDrawing();
		}
	}

	void init()
	{
		// Init debug
		dbg::init();
		// in::init(); TODO: fix input
		render::init();
        time::init();


        while (!render::windowPtr->ShouldClose())
        {
            time::tick();
            render::handle_input((float)sky::time::deltaTime.count());
            // scene::step(time::deltaTime);
            render::draw();
        }
	}
}