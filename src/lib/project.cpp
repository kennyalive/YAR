#include "std.h"
#include "common.h"
#include "project.h"

#include <charconv> // from_chars

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

    bool parse_integers(int* values, int count) {
        for (int i = 0; i < count; i++) {
            std::string_view token = next_token();
            if (!parse_int(token, &values[i]))
                return false;
        }
        return true;
    }

    bool parse_floats(float* values, int count) {
        for (int i = 0; i < count; i++) {
            std::string_view token = next_token();
            if (!parse_float(token, &values[i]))
                return false;
        }
        return true;
    }

private:
    bool parse_int(const std::string_view& token, int* value) {
        std::from_chars_result result = std::from_chars(token.data(), token.data() + token.length(), *value);
        return result.ptr == token.data() + token.length();
    }

    bool parse_float(const std::string_view& token, float* value) {
        std::from_chars_result result = std::from_chars(token.data(), token.data() + token.length(), *value);
        return result.ptr == token.data() + token.length();
    }
};

static YAR_Project parse_yar_project(const std::string& yar_file_name) {
    Text_File_Lines text_file = read_text_file_by_lines(yar_file_name);
    Text_File_Parser parser(&text_file);

    YAR_Project project{};
    std::string_view token;
    while (!(token = parser.next_token()).empty()) {
        if (token == "scene_type") {
            token = parser.next_token();
            if (token == "pbrt") {
                project.scene_type = Scene_Type::pbrt;
            }
            else if (token == "test") {
                project.scene_type = Scene_Type::test;
            } 
            else {
                error("uknown scene_type: %s", std::string(token).c_str());
            }
        }
        else if (token == "scene_path") {
            project.scene_path = parser.next_token();
        }
        else if (token == "image_resolution") {
            parser.parse_integers(&project.image_resolution.x, 2);
        }
        else if (token == "render_region") {
            parser.parse_integers(&project.render_region.p0.x, 4);
        }
        else if (token == "camera_to_world") {
            parser.parse_floats(&project.camera_to_world.a[0][0], 12);
        }
        else {
            error("unknown token: %s\n", std::string(token).c_str());
        }
    }
    return project;
}

YAR_Project initialize_project(const std::string& file_name) {
    fs::path path(file_name);
    if (!path.has_extension())
        error("Unknown file type: %s", file_name.c_str());

    std::string ext = to_lower(path.extension().string());
    if (ext == ".yar") {
        return parse_yar_project(file_name);
    }
    else if (ext == ".pbrt") {
        YAR_Project project;
        project.scene_type = Scene_Type::pbrt;
        project.scene_path = file_name;
        return project;
    }
    else {
        error("Unsupported file extension: %s", ext.c_str());
        return YAR_Project{};
    }
}

bool save_yar_file(const std::string& yar_file_name, const YAR_Project& project) {
    std::string abs_path = get_resource_path(yar_file_name);
    std::ofstream file(abs_path);
    if (!file)
        return false;

    if (project.scene_type == Scene_Type::pbrt)
        file << "scene_type pbrt\n";
    else if (project.scene_type == Scene_Type::test)
        file << "scene_type test\n";
    else
        error("save_yar_project: unknown scene type");

    file << "scene_path " << project.scene_path << "\n";
    file << "image_resolution " << project.image_resolution.x << " " << project.image_resolution.y << "\n";

    file << "camera_to_world\n";
    for (int i = 0; i < 3; i++) {
        const float* row = project.camera_to_world.a[i];
        file << row[0] << " " << row[1] << " " << row[2] << " " << row[3] << "\n";
    }
    return true;
}
