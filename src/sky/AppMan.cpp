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
        std::chrono::time_point<std::chrono::high_resolution_clock> prev_frame_time;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
        std::chrono::duration<double> delta_time;

        void init()
        {
            start_time = std::chrono::high_resolution_clock::now();
        }

		void tick()
		{
            auto t = std::chrono::high_resolution_clock::now();
            delta_time = (t - prev_frame_time);
            prev_frame_time = t;
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

        int id = 0; // current scene index loaded
        std::array<std::pair<std::string, int>, 7> scene_info
        {{

            {"rim-lighting", 4},
            {"low-poly flat", 1},
            {"high-poly flat", 1},
            {"gouraud shading", 1},
            {"smooth shading (expensive)", 1},
            {"many cubes", 4},
            {"clipping test", 1}
        }};

        // shading modes
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
            // Toggle z-buff view
            if (IsKeyPressed(KEY_I))
            {
                scene::blit_depth = !scene::blit_depth;
            }

            // Increment scene
            if (IsKeyPressed(KEY_O))
            {
                scene::id = (scene::id + 1) % scene::scene_info.size();
                scene::shading = scene::scene_info[scene::id].second;
            }

            // Increment shading model
            if (IsKeyPressed(KEY_P))
            {
                scene::shading = (scene::shading + 1) % scene::shading_models.size();
            }

            // local-movement
            Vector3 local_move {0, 0, 0 };
            if (IsKeyDown(KEY_W)) local_move.z -= 1.0f;
            if (IsKeyDown(KEY_S)) local_move.z += 1.0f;
            if (IsKeyDown(KEY_A)) local_move.x -= 1.0f;
            if (IsKeyDown(KEY_D)) local_move.x += 1.0f;
            if (IsKeyDown(KEY_SPACE)) local_move.y += 1.0f;
            if (IsKeyDown(KEY_LEFT_SHIFT)) local_move.y -= 1.0f;

            // check if there's actually any movement
            if (Vector3LengthSqr(local_move) != 0.0f)
            {
                Vector3 worldMove = Vector3RotateByQuaternion(Vector3NormalizeFast(local_move), scene::cam.rotation);
                scene::cam.position = Vector3Add(scene::cam.position, Vector3Scale(worldMove, scene::MOVE_SPEED * dt));
            }

            // lookaround
            if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
            {
                Vector2 md = GetMouseDelta();
                scene::yaw   += -md.x * scene::LOOK_SPEED;
                scene::pitch += -md.y * scene::LOOK_SPEED;

                // prevent camera going crazy at poles
                scene::pitch = Clamp(scene::pitch, -89.9f, 89.9f);

                Quaternion qPitch = QuaternionFromEuler(scene::pitch * DEG2RAD, 0, 0);
                Quaternion qYaw   = QuaternionFromEuler(0, scene::yaw * DEG2RAD, 0);

                scene::cam.rotation = QuaternionNormalize(QuaternionMultiply(qYaw, qPitch));
            }
        }
    }

	namespace render
	{
		std::unique_ptr<raylib::Window> window_ptr;
        std::unique_ptr<raylib::RenderTexture2D> buffer_ptr;
        sky::PixelBuff software_buff(render::PIXEL_WIDTH, render::PIXEL_HEIGHT);
        Texture2D screen_tex;
        Font main_font;

        void init()
		{
            window_ptr = std::make_unique<raylib::Window>(PIXEL_WIDTH * SCALE_FACTOR, PIXEL_HEIGHT * SCALE_FACTOR, "Sky", FLAG_WINDOW_ALWAYS_RUN);
            buffer_ptr = std::make_unique<raylib::RenderTexture2D>(PIXEL_WIDTH, PIXEL_HEIGHT);
            scene::shading = scene::scene_info[scene::id].second;

            screen_tex = LoadTextureFromImage(
                    Image{
                            .data = software_buff.get_frame().data(),
                            .width = render::PIXEL_WIDTH,
                            .height = render::PIXEL_HEIGHT,
                            .mipmaps = 1,
                            .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
                    }
            );

            main_font = LoadFontEx("assets/SpaceMono.ttf", 25, 0, 0);
		}


        // models and such
        sky::Model cube("assets/cube.obj", MatrixTranslate(0.f, 0.f, -2.f));
        sky::Model suzanne_01("assets/suzanne_01.obj", MatrixTranslate(0.f, 0.f, -2.f));
        sky::Model suzanne_02("assets/suzanne_02.obj", MatrixTranslate(0.f, 0.f, -2.f));
        sky::Model suzanne_03("assets/suzanne_03.obj", MatrixTranslate(0.f, 0.f, -2.f));

        void draw()
		{
            // begin buffer draw
            BeginTextureMode(*buffer_ptr);
            {
                software_buff.clear();
                std::chrono::duration<double> passed = (std::chrono::high_resolution_clock::now() - sky::time::start_time);

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

                                    software_buff.draw_model_flat(scene::cam, suzanne_01, scene::shading_models[scene::shading].first);
                                }
                            }
                        }

                        break;

                    case 1:
                        suzanne_01.set_transform(MatrixMultiply(MatrixRotateXYZ({0.f, sinf(passed.count() * 1.f), 0.f}), MatrixTranslate(0.f, 0.f, -2.f)));
                        software_buff.draw_model_flat(scene::cam, suzanne_01, scene::shading_models[scene::shading].first);
                        break;

                    case 2:
                        suzanne_03.set_transform(MatrixMultiply(MatrixRotateXYZ({0.f, sinf(passed.count() * 1.f), 0.f}), MatrixTranslate(0.f, 0.f, -2.f)));
                        software_buff.draw_model_flat(scene::cam, suzanne_03, scene::shading_models[scene::shading].first);
                        break;

                    case 3:
                        suzanne_02.set_transform(MatrixMultiply(MatrixRotateXYZ({0.f, sinf(passed.count() * 1.f), 0.f}), MatrixTranslate(0.f, 0.f, -2.f)));
                        software_buff.draw_model_gouraud(scene::cam, suzanne_02, scene::shading_models[scene::shading].first);
                        break;

                    case 4:
                        suzanne_02.set_transform(MatrixMultiply(MatrixRotateXYZ({0.f, sinf(passed.count() * 1.f), 0.f}), MatrixTranslate(0.f, 0.f, -2.f)));
                        software_buff.draw_model(scene::cam, suzanne_02, scene::shading_models[scene::shading].first);
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

                                    software_buff.draw_model_flat(scene::cam, cube, scene::shading_models[scene::shading].first);
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

                                    software_buff.draw_model_flat(scene::cam, suzanne_01, scene::shading_models[scene::shading].first);
                                }
                            }
                        }

                        break;
                }

                if (scene::blit_depth) software_buff.blit_depthbuffer();

                // Update and draw tex
                UpdateTexture(screen_tex, software_buff.get_frame().data());
                DrawTextureEx(screen_tex, {0, 0}, 0.0f, 1, WHITE);
            }
            EndTextureMode();

            BeginDrawing();
            DrawTexturePro( // software buffer blit
                buffer_ptr->texture,
                {0, 0, (float)PIXEL_WIDTH, -(float)PIXEL_HEIGHT }, // flip Y
                {0, 0, (float)PIXEL_WIDTH * SCALE_FACTOR, (float)PIXEL_HEIGHT * SCALE_FACTOR },
                { 0, 0 },
                0.0f,
                WHITE
            );

            // Draw scene text
            DrawTextEx(main_font, scene::scene_info[scene::id].first.c_str(), Vector2 { 20, 15 }, (float)main_font.baseSize, 0, WHITE);
            DrawTextEx(main_font, (std::to_string(int(1 / time::delta_time.count())) + " fps").c_str(), Vector2 { 20, 40 }, (float)main_font.baseSize, 0, WHITE);

            std::string txt = ("shading: " + scene::shading_models[scene::shading].second);
            txt += scene::blit_depth ? " (z-buffer)" : "";
            DrawTextEx(main_font, txt.c_str(), Vector2 { 20, render::PIXEL_HEIGHT * render::SCALE_FACTOR - 40 }, (float)main_font.baseSize, 0, WHITE);

            EndDrawing();
		}
	}

	void init()
	{
		render::init();
        time::init();

        while (!render::window_ptr->ShouldClose())
        {
            time::tick();
            input::handle((float)sky::time::delta_time.count());
            render::draw();
        }
	}
}