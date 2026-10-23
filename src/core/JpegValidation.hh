#pragma once

#include <cstdio>
#include <jpeglib.h>
#include <csetjmp>
#include <memory>
#include <vector>

namespace dynamic_terrain::detail
{
struct JpegValidationError { jpeg_error_mgr manager; std::jmp_buf jump; };
struct JpegValidationState
{
    jpeg_decompress_struct decoder{};
    JpegValidationError error{};
    std::vector<unsigned char> row;
    ~JpegValidationState() { if (decoder.mem) jpeg_destroy_decompress(&decoder); }
};

inline void JpegValidationFailed(j_common_ptr decoder)
{
    std::longjmp(reinterpret_cast<JpegValidationError *>(decoder->err)->jump, 1);
}

inline void JpegValidationMessage(j_common_ptr decoder, int level)
{
    // Default libjpeg recovery synthesizes missing data and EOI. Such a result
    // must not become a successful cache entry, even when imdecode returns pixels.
    if (level < 0) JpegValidationFailed(decoder);
}

inline bool ValidateJpeg(const std::vector<unsigned char> &bytes)
{
    // Heap state survives longjmp, including modified decoder fields and row
    // storage. The owning pointer is constructed before setjmp and unchanged.
    auto state = std::make_unique<JpegValidationState>();
    state->decoder.err = jpeg_std_error(&state->error.manager);
    state->error.manager.error_exit = JpegValidationFailed;
    state->error.manager.emit_message = JpegValidationMessage;
    if (setjmp(state->error.jump)) return false;
    jpeg_create_decompress(&state->decoder);
    jpeg_mem_src(&state->decoder, bytes.data(), bytes.size());
    if (jpeg_read_header(&state->decoder, TRUE) != JPEG_HEADER_OK ||
        !jpeg_start_decompress(&state->decoder)) return false;
    state->row.resize(static_cast<std::size_t>(state->decoder.output_width) * state->decoder.output_components);
    while (state->decoder.output_scanline < state->decoder.output_height)
    {
        auto *row = state->row.data();
        if (jpeg_read_scanlines(&state->decoder, &row, 1) != 1) return false;
    }
    return jpeg_finish_decompress(&state->decoder);
}
}
