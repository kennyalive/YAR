#include "std.h"
#include "common.h"
#include "io.h"

#include "half/half.h"

struct Text_File_Parser {
private:
    size_t pos = 0;
    int line = -1;
    Text_File_Lines* text_file = nullptr;

public:
    Text_File_Parser(Text_File_Lines* text_file) : text_file(text_file) {}

    std::string_view next_token() {
        bool new_line = false;
        bool start_found = false;
        size_t start;

        for (; pos < text_file->text.size(); pos++) {
            // Check for new line.
            if (pos == text_file->line_start_positions[line + 1]) {
                line++;
                new_line = true;
            }
            // Process next character.
            if (text_file->text[pos] <= 32) { // whitespace is found
                if (start_found) {
                    start_found = false;
                    size_t length = pos - start;
                    pos++;
                    return std::string_view(text_file->text.c_str() + start, length);
                }
            } else { // not-whitespace character
                if (!start_found) {
                    if (new_line && text_file->text[pos] == '#') {
                        pos = text_file->line_start_positions[line + 1] - 1 /*take into account increment in for loop*/;
                        line++;
                    } else {
                        start = pos;
                        start_found = true;
                    }
                }
            }
        }
        // the last token in the file
        if (start_found) {
            return std::string_view(text_file->text.c_str() + start, text_file->text.length() - start);
        } else
            return std::string_view(); // signal end of stream
    }
};

YAR_File load_yar_file(const std::string& file_name) {
    Text_File_Lines text_file = read_text_file_by_lines(file_name);
    Text_File_Parser parser(&text_file) ;

    std::string_view token;
    while (!(token = parser.next_token()).empty()) {
        printf("%s\n", std::string(token).c_str());
    }

    YAR_File result;
    return result;
}

Scene_Data load_scene(Scene_Type scene_type, const std::string& scene_path) {
    ASSERT(false);
    Scene_Data scene_data;
    return scene_data;
}

// from third_party/miniexr.cpp
unsigned char* miniexr_write(unsigned width, unsigned height, unsigned channels, const void* rgba16f, size_t* out_size);

void write_exr_image(const char* file_name, const ColorRGB* pixels, int w, int h) {
    FILE* file;
    if (fopen_s(&file, file_name, "wb") != 0)
        return;

    std::vector<unsigned short> rgb16f(w * h * 3);

    unsigned short* p = rgb16f.data();
    const ColorRGB* pixel = pixels;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            *p++ = float_to_half(pixel->r);
            *p++ = float_to_half(pixel->g);
            *p++ = float_to_half(pixel->b);
            pixel++;
        }
    }

    size_t exr_size;
    unsigned char* exr_data = miniexr_write(w, h, 3, rgb16f.data(), &exr_size);

    size_t bytes_written = fwrite(exr_data, 1, exr_size, file);
    ASSERT(bytes_written == exr_size);

    free(exr_data);
    fclose(file);
}
