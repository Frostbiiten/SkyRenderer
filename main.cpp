#include <iostream>
#include <sky/AppMan.h>
#include <sky/AssetMan.h>

int main()
{
    sky::init();

    /*

    // Initialization
    const int screenWidth = 1280;
    const int screenHeight = 768;

    raylib::Window window(screenWidth, screenHeight, "raylib-cpp - basic window");
    raylib::Texture logo("raylib_logo.png");

    SetTargetFPS(60);

    while (!window.ShouldClose())
    {
        BeginDrawing();

        window.ClearBackground(RAYWHITE);

        DrawText("Congrats! You created your first window!", 190, 200, 20, LIGHTGRAY);

        // Object methods.
        logo.Draw(
                screenWidth / 2 - logo.GetWidth() / 2,
                screenHeight / 2 - logo.GetHeight() / 2);

        EndDrawing();
    }
    */

    // UnloadTexture() and CloseWindow() are called automatically.

    return 0;
}
