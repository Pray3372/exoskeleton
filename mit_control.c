

static int float_to_uint(float x, float x_min, float x_max, unsigned int bits) { 
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (int)((x - x_min) * ((1 << bits) - 1) / (x_max - x_min));
