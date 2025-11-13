#pragma once

// THE COLOR TO KEEP

enum color_sort_mode {
    RED,
    BLUE,
    NONE
};

extern color_sort_mode colorSortMode;// = NONE;

int ColorSortTask(void);