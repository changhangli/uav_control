/*************************************************************************************************************************
 * Copyright 2024 Grifcc&Kylin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************************************************************/
#pragma once

// 根据给定的 id 值来填充二维平面上的不同位置
const float pose_table[95][3] = {
    {-0.01, -0.01, -0.01}, // 0 false
    {-28, -28, 0},         // 1
    {-23, -28, 0},         // 2
    {-18, -28, 0},         // 3
    {-10, -28, 0},         // 4
    {-5, -28, 0},          // 5
    {0, -28, 0},           // 6
    {5, -28, 0},           // 7
    {10, -28, 0},          // 8
    {18, -28, 0},          // 9
    {23, -28, 0},          // 10
    {28, -28, 0},          // 11
    {28, -23, 0},          // 12
    {28, -18, 0},          // 13
    {28, -10, 0},          // 14
    {28, -5, 0},           // 15
    {28, 0, 0},            // 16
    {28, 5, 0},            // 17
    {28, 10, 0},           // 18
    {28, 18, 0},           // 19
    {28, 23, 0},           // 20
    {28, 28, 0},           // 21
    {23, 28, 0},           // 22
    {18, 28, 0},           // 23
    {10, 28, 0},           // 24
    {5, 28, 0},            // 25
    {0, 28, 0},            // 26
    {-5, 28, 0},           // 27
    {-10, 28, 0},          // 28
    {-18, 28, 0},          // 29
    {-23, 28, 0},          // 30
    {-28, 28, 0},          // 31
    {-28, 23, 0},          // 32
    {-28, 18, 0},          // 33
    {-28, 10, 0},          // 34
    {-28, 5, 0},           // 35
    {-28, 0, 0},           // 36
    {-28, -5, 0},          // 37
    {-28, -10, 0},         // 38
    {-28, -18, 0},         // 39
    {-28, -23, 0},         // 40
    {0, 0, 0},             // 41
    {0, -5, 0},            // 42
    {5, 0, 0},             // 43
    {0, 5, 0},             // 44
    {-5, 0, 0},            // 45
    {0, -10, 0},           // 46
    {10, 0, 0},            // 47
    {0, 10, 0},            // 48
    {-10, 0, 0},           // 49
    {0, -23, 0},           // 50
    {23, 0, 0},            // 51
    {0, 23, 0},            // 52
    {-23, 0, 0},           // 53
    {0, -18, 0},           // 54
    {18, 0, 0},            // 55
    {0, 18, 0},            // 56
    {-18, 0, 0},           // 57
    {-0.01, -0.01, -0.01}, // 58 false
    {-0.01, -0.01, -0.01}, // 59 false
    {-0.01, -0.01, -0.01}, // 60 false
    {-0.01, -0.01, -0.01}, // 61 false
    {-0.01, -0.01, -0.01}, // 62 false
    {-0.01, -0.01, -0.01}, // 63 false
    {-0.01, -0.01, -0.01}, // 64 false
    {-0.01, -0.01, -0.01}, // 65 false
    {-0.01, -0.01, -0.01}, // 66 false
    {-0.01, -0.01, -0.01}, // 67 false
    {-0.01, -0.01, -0.01}, // 68 false
    {-0.01, -0.01, -0.01}, // 69 false
    {-0.01, -0.01, -0.01}, // 70 false
    {-0.01, -0.01, -0.01}, // 71 false
    {-0.01, -0.01, -0.01}, // 72 false
    {-0.01, -0.01, -0.01}, // 73 false
    {-0.01, -0.01, -0.01}, // 74 false
    {-0.01, -0.01, -0.01}, // 75 false
    {-0.01, -0.01, -0.01}, // 76 false
    {-0.01, -0.01, -0.01}, // 77 false
    {-0.01, -0.01, -0.01}, // 78 false
    {-0.01, -0.01, -0.01}, // 79 false
    {-0.01, -0.01, -0.01}, // 80 false
    {-0.01, -0.01, -0.01}, // 81 false
    {-0.01, -0.01, -0.01}, // 82 false
    {-0.01, -0.01, -0.01}, // 83 false
    {-0.01, -0.01, -0.01}, // 84 false
    {-0.01, -0.01, -0.01}, // 85 false
    {-0.01, -0.01, -0.01}, // 86 false
    {-0.01, -0.01, -0.01}, // 87 false
    {-0.01, -0.01, -0.01}, // 88 false
    {-0.01, -0.01, -0.01}, // 89 false
    {-0.01, -0.01, -0.01}, // 90 false
    {-14, -14, 0},         // 91
    {14, -14, 0},          // 92
    {-14, 14, 0},          // 93
    {14, 14, 0},           // 94
};

inline bool fill_value_from_id(float id2c_t[3], int id, float unit_len, double x_b, double y_b)
{
    auto id2c = pose_table[id];
    if (id2c[0] == -0.01 && id2c[1] == -0.01 && id2c[2] == -0.01)
    {
        return false;
    }
    id2c_t[0] = -id2c[0] * unit_len + x_b;
    id2c_t[1] = id2c[1] * unit_len - y_b;
    id2c_t[2] = id2c[2];
    return true;
}
