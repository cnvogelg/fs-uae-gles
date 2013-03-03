#include <string.h>
#include <fs/emu.h>

void fs_emu_render_scanlines(uint8_t* out, fs_emu_video_buffer *buffer,
        int cx, int cy, int cw, int ch, int scanline_dark,
        int scanline_light) {
    if (buffer->bpp != 4) {
        // not written code for non-32-bit frames yet
        return;
    }

    int stride = buffer->width * buffer->bpp;
    unsigned char *src_line = (unsigned char *) buffer->data;
    src_line += cy * stride + cx * buffer->bpp;

    unsigned char *dst_line = (unsigned char *) out;
    dst_line += cy * stride + cx * buffer->bpp;

    unsigned char *src, *dst;

    int light_ia = 255 - scanline_light;
    int dark_ia = 255 - scanline_dark;

    // dividing by 256 in loop for performance -correct div. would be 255.
    // using integer math only for performance.

    int alt = 0;

    for (int y = 0; y < ch; y++) {
        src = src_line;
        src_line += stride;
        dst = dst_line;
        dst_line += stride;
        if ((++alt % 2) == 0) {
            if (scanline_light == 0) {
                memcpy(dst, src, stride);
                continue;
            }
            for (int x = 0; x < cw; x++) {
#ifdef __BIG_ENDIAN__
                src ++;
                dst ++;
#endif
                *dst++ = (*src++ * light_ia) / 256 + scanline_light;
                *dst++ = (*src++ * light_ia) / 256 + scanline_light;
                *dst++ = (*src++ * light_ia) / 256 + scanline_light;
#ifndef __BIG_ENDIAN__
                src ++;
                dst ++;
#endif
            }
        }
        else {
            if (scanline_dark == 0) {
                memcpy(dst, src, stride);
                continue;
            }
            for (int x = 0; x < cw; x++) {
#ifdef __BIG_ENDIAN__
                src ++;
                dst ++;
#endif
                *dst++ = (*src++ * dark_ia) / 256;
                *dst++ = (*src++ * dark_ia) / 256;
                *dst++ = (*src++ * dark_ia) / 256;
#ifndef __BIG_ENDIAN__
                src ++;
                dst ++;
#endif
            }
        }
    }
}
