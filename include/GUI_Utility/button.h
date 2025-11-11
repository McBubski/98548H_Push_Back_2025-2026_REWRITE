#include <functional>
#include "vex.h"

using ButtonCallback = std::function<void()>;

#pragma once

class Button 
{
public:
    // Size and Position
    int x, y;
    int width, height;

    // Text to display
    const char* text;
    const char* text_color;
    fontType font;

    // Visual
    const char* backgroud_color;
    const char* stroke_color;
    int stroke_thickness;

    // Callback
    ButtonCallback callback;

    // Constructor
    Button(int x_, int y_, int width_, int height_, const char* label_, const char* color_, ButtonCallback callback_);

    // Functions
    void display(void);
    void checkPress(int touchX, int touchY);
};