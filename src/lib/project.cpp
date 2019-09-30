#include "std.h"
#include "common.h"
#include "project.h"
#include "spectrum.h"

#define JSMN_STATIC 
#include "jsmn/jsmn.h"

#include <charconv> // from_chars

static std::string unescape_json_string(const std::string_view& escaped_json_string) {
    std::string str;
    str.reserve(escaped_json_string.size());
    for (int i = 0; i < (int)escaped_json_string.size(); i++) {
        if (escaped_json_string[i] != '\\') {
            str.push_back(escaped_json_string[i]);
            continue;
        }
        // Handle escaped character.
        if (++i < (int)escaped_json_string.size()) {
            if (escaped_json_string[i] == '\\')
                str.push_back('\\');
            else if (escaped_json_string[i] == '/')
                str.push_back('/');
            else if (escaped_json_string[i] == 't')
                str.push_back('\t');
            else if (escaped_json_string[i] == 'n')
                str.push_back('\n');
            else if (escaped_json_string[i] == '"')
                str.push_back('"');
        }
    }
    return str;
}

#define CHECK(expression) check(expression, "%s", #expression)

namespace {
struct Parser {
    const std::string& content;
    YAR_Project& project;
    std::vector<jsmntok_t> tokens;
    int next_token_index = 0;
    jsmntok_t token;

    Parser(const std::string& content, YAR_Project& project)
        : content(content), project(project)
    {}

    struct Error {
        std::string description;
    };

    void check(bool condition, const char* format, ...) const {
        if (!condition) {
            char buffer[512];
            va_list args;
            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
            throw Error{buffer};
        }
    }

    void next_token() {
        CHECK(next_token_index < tokens.size());
        token = tokens[next_token_index++];
    }

    std::string_view get_current_token_string() const {
        return std::string_view(content.data() + token.start, token.end - token.start);
    }

    bool match_string(const char* str) {
        CHECK(token.type == JSMN_STRING);
        if (strncmp(content.data() + token.start, str, token.end - token.start) != 0)
            return false;
        next_token();
        return true;
    }

    std::string get_string() {
        ASSERT(token.type == JSMN_STRING);
        std::string_view escaped_string = get_current_token_string();
        next_token();
        return unescape_json_string(escaped_string);
    }

    template <typename T>
    T get_numeric() {
        CHECK(token.type == JSMN_PRIMITIVE);
        CHECK(content[token.start] == '-' || (content[token.start] >= '0' && content[token.start] <= '9'));
        T value;
        std::from_chars_result result = std::from_chars(content.data() + token.start, content.data() + token.end, value);
        CHECK(result.ptr == content.data() + token.end);
        next_token();
        return value;
    }

    template <typename T>
    void get_fixed_numeric_array(int array_size, T* values) {
        CHECK(token.type == JSMN_ARRAY);
        CHECK(token.size == array_size);
        next_token();
        for (int i = 0; i < array_size; i++)
            values[i] = get_numeric<T>();
    }

    void parse_array_of_objects(std::function<void()> parse_object) {
        CHECK(token.type == JSMN_ARRAY);
        const int array_size = token.size;
        next_token();
        for (int i = 0; i < array_size; i++)
            parse_object();
    }

    // Main parsing routine.
    void parse() {
        int token_count;
        {
            jsmn_parser parser;
            jsmn_init(&parser);
            token_count = jsmn_parse(&parser, content.c_str(), content.size(), nullptr, 0);
            check(token_count >= 0, "JSMN parser failed to tokenize the document");
        }
        if (token_count == 0)
            return;

        tokens.resize(token_count + 1);
        {
            jsmn_parser parser;
            jsmn_init(&parser);
            int result = jsmn_parse(&parser, content.c_str(), content.size(), tokens.data(), token_count);
            CHECK(result == token_count);
            tokens[token_count] = jsmntok_t{ JSMN_UNDEFINED }; // terminator token
        }

        next_token();
        CHECK(token.type == JSMN_OBJECT); // root object
        next_token();

        while (token.type != JSMN_UNDEFINED)
            parse_top_level_property();
    }

    void parse_top_level_property() {
        if (match_string("comment")) {
            CHECK(token.type == JSMN_STRING);
            next_token();
        }
        else if (match_string("scene_type")) {
            if (match_string("pbrt"))
                project.scene_type = Scene_Type::pbrt;
            else if (match_string("obj"))
                project.scene_type = Scene_Type::obj;
            else
                check(false, "unknown scene_type: %.*s", (int)get_current_token_string().size(), get_current_token_string().data());
        }
        else if (match_string("scene_path")) {
            project.scene_path =  get_string();
        }
        else if (match_string("image_resolution")) {
            get_fixed_numeric_array(2, &project.image_resolution.x);
            project.has_image_resolution = true;
        }
        else if (match_string("render_region")) {
            get_fixed_numeric_array(4, &project.render_region.p0.x);
            project.has_render_region = true;
        }
        else if (match_string("camera_to_world")) {
            get_fixed_numeric_array(12, &project.camera_to_world.a[0][0]);
            project.has_camera_to_world = true;
        }
        else if (match_string("world_scale")) {
            project.world_scale = get_numeric<float>();
            CHECK(project.world_scale > 0.f);
        }
        else if (match_string("lights")) {
            parse_array_of_objects([this]() {parse_light_object();});
        }
        else {
            check(false, "Unknown token [%.*s]", (int)get_current_token_string().size(), get_current_token_string().data());
        }
    }

    void parse_light_object() {
        CHECK(token.type == JSMN_OBJECT);
        const int num_fields = token.size;
        next_token();
        if (!match_string("type"))
            check(false, "light definition should start with \'type\' attribute");

        if (match_string("point"))
            parse_point_light(num_fields - 1);
        else
            check(false, "unknown light type");
    }

    void parse_point_light(int num_fields) {
        Point_Light light{};
        std::string spectrum_shape = "constant";
        float luminous_flux = 0.f;
        for (int i = 0; i < num_fields; i++) {
            if (match_string("position")) {
                get_fixed_numeric_array(3, &light.position.x);
            }
            else if (match_string("spectrum_shape")) {
                spectrum_shape = get_string();
            }
            else if (match_string("luminous_flux")) {
                luminous_flux = get_numeric<float>();
            }
            else
                check(false, "unknown point light attribute [%.*s]", (int)get_current_token_string().size(), get_current_token_string().data());
        }

        if (spectrum_shape.empty() || spectrum_shape == "constant")
            light.intensity = convert_flux_to_constant_spectrum_to_rgb_intensity(luminous_flux);
        else
            check(false, "unknown spectrum_shape [%s]", spectrum_shape.c_str());

        project.lights.point_lights.emplace_back(std::move(light));
    }
};
} // namespace

#undef CHECK

static YAR_Project parse_yar_project(const std::string& yar_file_name) {
    std::string abs_path = get_resource_path(yar_file_name);
    std::string content = read_text_file(abs_path);

    YAR_Project project;
    Parser parser(content, project);
    try {
        parser.parse();
    } catch (const Parser::Error& parser_error) {
        error("Failed to parse yar project file [%s]: %s", yar_file_name.c_str(), parser_error.description.c_str());
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
    else if (project.scene_type == Scene_Type::obj)
        file << "scene_type obj\n";
    else
        error("save_yar_project: unknown scene type");

    file << "scene_path " << project.scene_path << "\n";

    if (project.has_image_resolution)
        file << "image_resolution " << project.image_resolution.x << " " << project.image_resolution.y << "\n";

    if (project.has_camera_to_world) {
        file << "camera_to_world\n";
        for (int i = 0; i < 3; i++) {
            const float* row = project.camera_to_world.a[i];
            file << row[0] << " " << row[1] << " " << row[2] << " " << row[3] << "\n";
        }
    }
    return true;
}

