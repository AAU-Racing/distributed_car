#ifndef FLOAT_PRINT_GUARD
#define FLOAT_PRINT_GUARD

void print_double(double v, uint8_t total_width, uint8_t decimals) {
    int width_pre_dot = total_width - (decimals + 1);
    int int_part = (int) v;
    double cpy = v - (double) int_part;
    for (int i = 0; i < decimals; i++) {
        cpy *= 10;
    }
    int fract_part = cpy;

    if (fract_part < 0) {
        fract_part *= -1;
    }

    printf("%*d.%0*d", width_pre_dot, int_part, decimals, fract_part);
}

#endif
