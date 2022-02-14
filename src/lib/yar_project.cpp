#include "std.h"
#include "common.h"
#include "yar_project.h"

#include "colorimetry.h"
#include "spectrum.h"

#define JSMN_STATIC 
#include "jsmn/jsmn.h"

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

    bool get_bool() {
        CHECK(token.type == JSMN_PRIMITIVE);
        CHECK(content[token.start] == 't' || content[token.start] == 'f');
        bool result = content[token.start] == 't';
        next_token();
        return result;
    }

    template <typename T>
    void get_fixed_numeric_array(int array_size, T* values) {
        CHECK(token.type == JSMN_ARRAY);
        CHECK(token.size == array_size);
        next_token();
        for (int i = 0; i < array_size; i++)
            values[i] = get_numeric<T>();
    }

    std::vector<std::string> get_array_of_strings() {
        CHECK(token.type == JSMN_ARRAY);
        const int array_size = token.size;
        next_token();
        std::vector<std::string> strs;
        strs.reserve(array_size);
        for (int i = 0; i < array_size; i++)
            strs.push_back(get_string());
        return strs;
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
        else if (match_string("obj_info")) {
            parse_obj_info();
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
            project.scene_path = get_string();
        }
        else if (match_string("film_resolution")) {
            get_fixed_numeric_array(2, &project.film_resolution.x);
        }
        else if (match_string("render_region")) {
            get_fixed_numeric_array(4, &project.render_region.p0.x);
        }
        else if (match_string("camera_to_world")) {
            get_fixed_numeric_array(12, &project.camera_to_world.a[0][0]);
        }
        else if (match_string("world_scale")) {
            project.world_scale = get_numeric<float>();
            CHECK(project.world_scale > 0.f);
        }
        else if (match_string("camera_fov_y")) {
            project.camera_fov_y = get_numeric<float>();
            CHECK(project.camera_fov_y > 0.f);
        }
        else if (match_string("mesh_disable_backface_culling")) {
            project.mesh_disable_backfacing_culling = get_bool();
        }
        else if (match_string("mesh_invert_winding_order")) {
            project.mesh_invert_winding_order = get_bool(); 
        }
        else if (match_string("mesh_crease_angle")) {
            project.mesh_crease_angle = radians(get_numeric<float>());
            CHECK(project.mesh_crease_angle >= 0.f);
            project.mesh_use_crease_angle = true;
        }
        else if (match_string("lights")) {
            parse_array_of_objects([this]() {parse_light_object();});
        }
        else if (match_string("instances")) {
            parse_array_of_objects([this]() { parse_instance_object();});
        }
        else if (match_string("ignore_geometry_names")) {
            project.ignore_geometry_names = get_array_of_strings();
        }
        else {
            check(false, "Unknown token [%.*s]", (int)get_current_token_string().size(), get_current_token_string().data());
        }
    }

    void parse_obj_info() {
        CHECK(token.type == JSMN_OBJECT);
        const int num_fields = token.size;
        next_token();
        for (int i = 0; i < num_fields; i++) {
            if (match_string("z_is_up")) {
                project.obj_info.z_is_up = get_bool();
                project.obj_info.z_is_up_specified = true;
            }
            else if (match_string("left_handed")) {
                project.obj_info.left_handed = get_bool();
                project.obj_info.left_handed_specified = true;
            }
            else {
                check(false, "unknown coordinate system property");
            }
        }
    }

    void parse_light_object() {
        CHECK(token.type == JSMN_OBJECT);
        const int num_fields = token.size;
        next_token();
        if (!match_string("type"))
            check(false, "light definition should start with \'type\' attribute");

        if (match_string("point")) {
            parse_point_light(num_fields - 1);
        }
        else if (match_string("directional")) {
            parse_directional_light(num_fields - 1);
        }
        else if (match_string("diffuse_rectangular")) {
            parse_diffuse_rectangular_light(num_fields - 1);
        }
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

        project.point_lights.emplace_back(std::move(light));
    }

    void parse_directional_light(int num_fields) {
        Directional_Light light;

        for (int i = 0; i < num_fields; i++) {
            if (match_string("direction")) {
                get_fixed_numeric_array(3, &light.direction.x);
            }
            else if (match_string("rgb_irradiance")) {
                get_fixed_numeric_array(3, &light.irradiance.r);

            }
            else
                check(false, "unknown directional light attribute [%.*s]", (int)get_current_token_string().size(), get_current_token_string().data());
        }
        project.directional_lights.emplace_back(std::move(light));
    }

    void parse_diffuse_rectangular_light(int num_fields) {
        Diffuse_Rectangular_Light light{};
        float luminous_flux = 0.f;
        for (int i = 0; i < num_fields; i++) {
            if (match_string("size")) {
                get_fixed_numeric_array(2, &light.size.x);
            }
            else if (match_string("light_to_world_transform")) {
                get_fixed_numeric_array(12, &light.light_to_world_transform.a[0][0]);
            }
            else if (match_string("luminous_flux")) {
                luminous_flux = get_numeric<float>(); 
            }
            else if (match_string("sample_count")) {
                light.sample_count = get_numeric<int>();
            }
            else
                check(false, "unknown diffuse rectangular light attribute [%.*s", (int)get_current_token_string().size(), get_current_token_string().data());
        }

        float radiant_flux_per_wavelength = luminous_flux / (683.f * CIE_Y_integral); // [W/m]
        float radiant_exitance_per_wavelength = Pi * radiant_flux_per_wavelength; // [M/m]
        Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(radiant_exitance_per_wavelength);
        Vector3 xyz = s.emission_spectrum_to_XYZ();
        light.emitted_radiance = XYZ_to_sRGB(xyz);

        project.diffuse_rectangular_lights.emplace_back(std::move(light));
    }

    void parse_instance_object() {
        CHECK(token.type == JSMN_OBJECT);
        const int num_fields = token.size;
        next_token();

        bool has_transform = false;
        YAR_Instance instance;
        for (int i = 0; i < num_fields; i++) {
            if (match_string("geometry_name")) {
                instance.geometry_name = get_string();
            }
            else if (match_string("translation")) {
                Vector3 translation;
                get_fixed_numeric_array(3, &translation.x);
                instance.transform = translate(Matrix3x4::identity, translation);
                has_transform = true;
            }
            else 
                check(false, "unknown instance attribute [%.e*s]", (int)get_current_token_string().size(), get_current_token_string().data());
        }
        CHECK(!instance.geometry_name.empty());
        CHECK(has_transform);
        project.instances.push_back(instance);

    }
};
} // namespace

#undef CHECK

YAR_Project parse_yar_file(const std::string& yar_file_path) {
    ASSERT(get_extension(yar_file_path) == ".yar");

    std::string content = read_text_file(yar_file_path);
    YAR_Project project;
    Parser parser(content, project);
    try {
        parser.parse();
    } catch (const Parser::Error& parser_error) {
        error("Failed to parse yar project file [%s]: %s", yar_file_path.c_str(), parser_error.description.c_str());
    }

    // The scene path, as defined in the yar file, is either an absolute path or a 
    // relative path. If it is a relative path then we can't use it directly in the
    // program for file operations because it's relative to the yar file's parent
    // directory and not to the current working directory. The following code modifies
    // relative scene path to be either an absolute path or to be relative to the
    // current working directory.
    if (project.scene_path.is_relative()) {
        project.scene_path = fs::path(yar_file_path).parent_path() / project.scene_path;
    }

    return project;
}
