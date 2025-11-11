#include "GUI_Utility/button.h"

Button::Button(int x_, int y_, int width_, int height_, const char* text_, const char* background_color_, ButtonCallback callback_) { 
    // Constructor Arguments

    x = x_;
    y = y_;
    width = width_;
    height = height_;
    text = text_;
    backgroud_color = background_color_;

    callback = callback_;

    // Other Defaults

    text_color = "#ffffffff";
    font = fontType::mono20;

    stroke_color = "#000000";
    stroke_thickness = 3;
}

void Button::display(void) {
    // Draw Rectangle

    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(backgroud_color);
    Brain.Screen.setPenWidth(stroke_thickness);
    Brain.Screen.drawRectangle(x, y, width, height);
    Brain.Screen.setFont(mono20);

    // Print Text

    int stringWidth = Brain.Screen.getStringWidth(text);
    int stringHeight = Brain.Screen.getStringHeight(text);

    int text_x = x + (width / 2) - (stringWidth / 2);
    int text_y = y + (height / 2) + (stringHeight / 4); // 4
    
    Brain.Screen.printAt(text_x, text_y, text);
}

void Button::checkPress(int touchX, int touchY) {
    if (touchX >= x && touchX <= x + width) {
        if (touchY >= y && touchY <= y + height) {
            if (callback) {
                callback();
            }
        }
    }
}