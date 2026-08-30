#include "std.h"
#include "lib/common.h"

// Flushes so that a crash later in the test does not hide the failure messages.
#define CHECK(cond) do { if (!(cond)) { printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); fflush(stdout); failures++; } } while (0)

// True when the characters live inside the String object (small string optimization).
static bool stored_inside(const String& s)
{
    const char* p = s.data();
    const char* object = (const char*)&s;
    return p >= object && p < object + sizeof(String);
}

void test_string()
{
    printf("Testing String (sizeof(String)=%zu)...\n", sizeof(String));
    fflush(stdout);
    int failures = 0;

    // Basics.
    {
        String empty;
        CHECK(empty.size() == 0 && empty.data()[0] == 0);
        CHECK(empty == "" && empty == String());

        String s = "hello";
        CHECK(s.size() == 5 && strcmp(s.data(), "hello") == 0);
        CHECK(s == "hello" && s != "hellp" && "hello" == s);
        CHECK(s != "hell" && s != "hello!");

        String t("abcdef", 3);
        CHECK(t.size() == 3 && t == "abc" && t.data()[3] == 0);
    }

    // Copy, move, assignment.
    {
        String copy;
        {
            String original = "original";
            copy = original;
            CHECK(copy == original);
            String moved = std::move(original);
            CHECK(moved == "original" && original.size() == 0 && original == "");
            String moved2;
            moved2 = std::move(moved);
            CHECK(moved2 == "original" && moved.size() == 0);
        }
        CHECK(copy == "original"); // outlives the source

        String& alias = copy;
        copy = alias; // self assignment
        CHECK(copy == "original");
    }

    // Small and heap strings around the inline limit.
    {
        const uint32_t limit = String::max_small;
        std::vector<char> text(64, 'q');
        text.push_back(0);
        for (uint32_t n : { 0u, 1u, 7u, 15u, 23u, 31u, 32u, 40u, 63u }) {
            String s(text.data(), n);
            CHECK(s.size() == n && s.data()[n] == 0 && memcmp(s.data(), text.data(), n) == 0);
            CHECK(stored_inside(s) == (n <= limit));

            String c = s;                  // copy
            CHECK(c == s && c.size() == n && c.data()[n] == 0);
            CHECK(stored_inside(c) == stored_inside(s));
            if (!stored_inside(s) && n > 0)
                CHECK(c.data() != s.data()); // heap copies own their characters

            const char* chars_before_move = c.data();
            String m = std::move(c);       // move
            CHECK(m == s && c.size() == 0 && c == "");
            CHECK((m.data() == chars_before_move) == !stored_inside(m)); // a heap string keeps its pointer, a small one is copied
        }

        String from_null((const char*)nullptr, 0); // empty range without a buffer
        CHECK(from_null.empty() && from_null == "");

        String embedded("a\0b", 3);
        CHECK(embedded.size() == 3 && embedded != "a" && embedded == String("a\0b", 3));

        String source = "span";
        Span<const char> span = source;    // Span's container constructor accepts a String lvalue
        CHECK(span.data == source.data() && span.size == source.size());
    }

    // Ordering and map keys.
    {
        CHECK(String("abc") < String("abd"));
        CHECK(String("ab") < String("abc"));
        CHECK(!(String("abc") < String("abc")));

        std::map<String, int> map;
        map[String("one")] = 1;
        map[String("two")] = 2;
        map[String("one")] = 11;
        CHECK(map.size() == 2 && map[String("one")] == 11);
    }

    // Formatting, including results longer than string_printf's stack buffer.
    {
        CHECK(string_printf("%s_%04d.%s", "tile", 7, "exr") == "tile_0007.exr");
        CHECK(string_printf("%s", "") == "");

        std::vector<char> big(3000, 'x');
        big.push_back(0);
        String long_string = string_printf("%s", big.data());
        CHECK(long_string.size() == 3000 && long_string.data()[2999] == 'x' && long_string.data()[3000] == 0);
    }

    // Helpers from common.h.
    {
        CHECK(to_lower("MiXed Case") == "mixed case");
        CHECK(get_extension("dir/scene.YAR") == ".yar");
        CHECK(get_extension("noext") == "");
    }

    // Strings created on worker threads, read and destroyed on the main thread.
    {
        std::vector<std::vector<String>> results(8);
        std::vector<std::jthread> threads;
        for (int t = 0; t < 8; t++) {
            threads.emplace_back([t, &results] {
                for (int i = 0; i < 5000; i++)
                    results[t].push_back(i % 2 ? string_printf("shared %d", i) : string_printf("thread %d item %d", t, i));
            });
        }
        threads.clear(); // joins
        CHECK(results[3][1] == "shared 1" && results[3][2] == "thread 3 item 2");
        CHECK(results[0][1] == results[7][1]);
    }

    if (failures)
        error("test_string: %d checks failed", failures);
    printf("String tests passed\n");
}

//
// Benchmark: String against std::string, construction from const char*, copy and move,
// for lengths on both sides of the inline limits (15, 23, 31), and concurrent construction.
//
static size_t benchmark_sink; // printed at the end, so the work feeding it is observable

// Opaque consumer: the object escapes into a function the optimizer cannot see through, so it
// has to be constructed for real. The release build uses whole program optimization.
#ifdef _MSC_VER
#define BENCHMARK_NOINLINE __declspec(noinline)
#else
#define BENCHMARK_NOINLINE __attribute__((noinline))
#endif
template <typename T>
BENCHMARK_NOINLINE static void benchmark_keep(const T& s)
{
    benchmark_sink += (size_t)s.data()[0] + s.size();
}

template <typename F>
static double benchmark_ns_per_op(int n, F f)
{
    Timestamp t;
    f();
    return (double)elapsed_nanoseconds(t) / n;
}

void benchmark_string()
{
    printf("String benchmark: sizeof(String)=%zu, sizeof(std::string)=%zu\n",
        sizeof(String), sizeof(std::string));

    const int N = 1'000'000; // operations per measurement
    const int K = 1024;      // distinct strings per length, so that the source text is not one hot line
    printf("%-6s %-22s %-22s %-22s\n", "chars", "construct std/String", "copy std/String", "move std/String");
    for (int length : { 7, 15, 23, 31, 47, 63 }) {
        std::vector<std::string> pool(K);
        for (int i = 0; i < K; i++) {
            pool[i] = std::to_string(1000000 + i);
            while ((int)pool[i].size() < length)
                pool[i] += (char)('a' + pool[i].size() % 26);
            pool[i].resize(length);
        }
        double construct_std = benchmark_ns_per_op(N, [&] { for (int i = 0; i < N; i++) { std::string s(pool[i % K].c_str()); benchmark_keep(s); } });
        double construct_our = benchmark_ns_per_op(N, [&] { for (int i = 0; i < N; i++) { String s(pool[i % K].c_str()); benchmark_keep(s); } });

        std::string std_source = pool[0];
        String our_source = pool[0].c_str();
        // Copies land in pre-sized vectors and every one is consumed afterwards, so none can be dropped.
        std::vector<std::string> std_copies(N);
        std::vector<String> our_copies(N);
        double copy_std = benchmark_ns_per_op(N, [&] { for (int i = 0; i < N; i++) std_copies[i] = std_source; });
        double copy_our = benchmark_ns_per_op(N, [&] { for (int i = 0; i < N; i++) our_copies[i] = our_source; });
        for (int i = 0; i < N; i++) {
            benchmark_keep(std_copies[i]);
            benchmark_keep(our_copies[i]);
        }

        std::string std_a = pool[1], std_b = pool[2];
        String our_a = pool[1].c_str(), our_b = pool[2].c_str();
        double move_std = benchmark_ns_per_op(3 * N, [&] { for (int i = 0; i < N; i++) { std::string t = std::move(std_a); std_a = std::move(std_b); std_b = std::move(t); } benchmark_keep(std_a); });
        double move_our = benchmark_ns_per_op(3 * N, [&] { for (int i = 0; i < N; i++) { String t = std::move(our_a); our_a = std::move(our_b); our_b = std::move(t); } benchmark_keep(our_a); });

        printf("%-6d %7.1f / %-12.1f %7.1f / %-12.1f %7.1f / %-12.1f\n", length,
            construct_std, construct_our, copy_std, copy_our, move_std, move_our);
    }

    // Eight threads constructing 23 character strings concurrently (ns per string, wall clock).
    {
        std::vector<std::string> pool(K);
        for (int i = 0; i < K; i++)
            pool[i] = "some/path/name_" + std::to_string(10000000 + i);
        auto run = [&](auto construct) {
            std::vector<size_t> results(8); // one slot per thread, no shared writes
            std::vector<std::jthread> threads;
            for (int t = 0; t < 8; t++)
                threads.emplace_back([&, t] { size_t local = 0; for (int i = t; i < N; i += 8) local += construct(pool[i % K].c_str()); results[t] = local; });
            threads.clear(); // joins
            for (size_t r : results)
                benchmark_sink += r;
        };
        double threads_std = benchmark_ns_per_op(N, [&] { run([](const char* s) { std::string x(s); benchmark_keep(x); return x.size(); }); });
        double threads_our = benchmark_ns_per_op(N, [&] { run([](const char* s) { String x(s); benchmark_keep(x); return (size_t)x.size(); }); });
        printf("8 threads constructing 23 char strings: std %.1f ns/op, String %.1f ns/op\n", threads_std, threads_our);
    }
    printf("(checksum %zu)\n", benchmark_sink);
}
