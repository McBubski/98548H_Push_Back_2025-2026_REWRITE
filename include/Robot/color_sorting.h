#pragma once

// THE COLOR TO KEEP

enum color_sort_mode {
    RED,
    BLUE
};

extern color_sort_mode colorSortMode;// = NONE;
extern bool colorSorting;
extern bool colorSortingIndexerOverride;

int ColorSortTask(void);