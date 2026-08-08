#ifndef CAMERA_COMMON_H
#define CAMERA_COMMON_H

typedef enum { SIZE_VGA = 0x00, SIZE_QVGA = 0x01, SIZE_QQVGA = 0x02 } image_size_t;

typedef enum { SUBSAMPLING_X1 = 0x20, SUBSAMPLING_X2 = 0x40, SUBSAMPLING_X4 = 0x80 } subsampling_t;

typedef enum { FORMAT_GREYSCALE = 0x00, FORMAT_COLOR = 0x01 } format_t;

#endif