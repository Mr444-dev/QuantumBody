#include "quantumbody/scenes.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

struct Options {
    std::string scene{"stack"};
    std::string trajectory;
    std::string csv;
    std::uint32_t steps{600};
    std::uint32_t seed{7};
    std::uint32_t count{240};
    std::uint32_t iterations{10};
    std::uint32_t recordEvery{4};
    double dt{1.0 / 120.0};
    bool help{};
};

template<class T>
T parseInteger(std::string_view value, std::string_view option) {
    T out{};
    const auto [ptr, ec] = std::from_chars(value.data(), value.data() + value.size(), out);
    if (ec != std::errc{} || ptr != value.data() + value.size())
        throw std::invalid_argument("invalid integer for " + std::string(option));
    return out;
}

double parseDouble(std::string_view value, std::string_view option) {
    std::string text(value);
    std::size_t used{};
    const double out = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(out) || out <= 0.0)
        throw std::invalid_argument("expected a positive finite number for " + std::string(option));
    return out;
}

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        auto next = [&]() -> std::string_view {
            if (++i >= argc) throw std::invalid_argument("missing value after " + std::string(arg));
            return argv[i];
        };
        if (arg == "--help" || arg == "-h") o.help = true;
        else if (arg == "--scene") o.scene = next();
        else if (arg == "--steps") o.steps = parseInteger<std::uint32_t>(next(), arg);
        else if (arg == "--seed") o.seed = parseInteger<std::uint32_t>(next(), arg);
        else if (arg == "--count") o.count = parseInteger<std::uint32_t>(next(), arg);
        else if (arg == "--iterations") o.iterations = parseInteger<std::uint32_t>(next(), arg);
        else if (arg == "--record-every") o.recordEvery = parseInteger<std::uint32_t>(next(), arg);
        else if (arg == "--dt") o.dt = parseDouble(next(), arg);
        else if (arg == "--trajectory") o.trajectory = next();
        else if (arg == "--csv") o.csv = next();
        else throw std::invalid_argument("unknown option: " + std::string(arg));
    }
    if (o.steps == 0 || o.count == 0 || o.iterations == 0 || o.recordEvery == 0)
        throw std::invalid_argument("steps, count, iterations, and record-every must be greater than zero");
    return o;
}

void printHelp() {
    std::cout << "QuantumBody - small 3D rigid-body physics sandbox\n"
              << "Usage: quantumbody [options]\n"
              << "  --scene stack|swarm|drop|failure-lab   Scene to simulate (default: stack)\n"
              << "  --steps N                             Fixed simulation steps (default: 600)\n"
              << "  --dt SECONDS                          Fixed step (default: 1/120)\n"
              << "  --count N                             Swarm particle count (default: 240)\n"
              << "  --seed N                              Reproducible scene seed (default: 7)\n"
              << "  --iterations N                        Sequential-impulse iterations (default: 10)\n"
              << "  --trajectory FILE                     Write replay JSON for the viewer\n"
              << "  --record-every N                      Save every Nth simulation step (default: 4)\n"
              << "  --csv FILE                            Write per-step timing and contact CSV\n";
}

void writePose(std::ostream& out, const qb::RigidBody& body) {
    out << '[' << body.position.x << ',' << body.position.y << ',' << body.position.z << ','
        << body.orientation.x << ',' << body.orientation.y << ',' << body.orientation.z << ',' << body.orientation.w << ']';
}

void writeTrajectory(const Options& options) {
    std::ofstream out(options.trajectory);
    if (!out) throw std::runtime_error("cannot open trajectory output: " + options.trajectory);
    qb::WorldSettings settings;
    settings.solverIterations = options.iterations;
    qb::World world = qb::makeScene(options.scene, options.seed, options.count, settings);
    out << std::setprecision(9) << "{\"version\":1,\"scene\":\"" << options.scene << "\",\"dt\":" << options.dt << ",\"bodies\":[";
    for (std::size_t i = 0; i < world.bodies().size(); ++i) {
        if (i) out << ',';
        const auto& body = world.bodies()[i];
        out << "{\"id\":" << body.id << ",\"type\":\"" << (std::holds_alternative<qb::Sphere>(body.shape) ? "sphere" : "box") << "\",\"static\":" << (body.isStatic ? "true" : "false") << ",\"size\":[";
        if (const auto* sphere = std::get_if<qb::Sphere>(&body.shape)) out << sphere->radius << ',' << sphere->radius << ',' << sphere->radius;
        else { const auto e = std::get<qb::Box>(body.shape).halfExtents; out << e.x << ',' << e.y << ',' << e.z; }
        out << "]}";
    }
    out << "],\"frames\":[";
    auto writeFrame = [&](double time) {
        out << "{\"t\":" << time << ",\"poses\":[";
        for (std::size_t i = 0; i < world.bodies().size(); ++i) { if (i) out << ','; writePose(out, world.bodies()[i]); }
        out << "]}";
    };
    writeFrame(0.0);
    for (std::uint32_t step = 1; step <= options.steps; ++step) {
        world.step(options.dt);
        if (step % options.recordEvery == 0 || step == options.steps) { out << ','; writeFrame(step * options.dt); }
    }
    out << "]}\n";
    if (!out) throw std::runtime_error("failed writing trajectory output");
}

double percentile(std::vector<double> values, double p) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const std::size_t i = static_cast<std::size_t>(std::ceil(p * static_cast<double>(values.size()))) - 1;
    return values[std::min(i, values.size() - 1)];
}

void run(const Options& o) {
    qb::WorldSettings settings;
    settings.solverIterations = o.iterations;
    qb::World world = qb::makeScene(o.scene, o.seed, o.count, settings);
    std::ofstream csv;
    if (!o.csv.empty()) {
        csv.open(o.csv);
        if (!csv) throw std::runtime_error("cannot open CSV output: " + o.csv);
        csv << "step,simulated_seconds,step_ms,broadphase_candidates,contacts\n";
    }
    std::vector<double> samples;
    samples.reserve(o.steps);
    std::uint64_t candidateTotal = 0, contactTotal = 0;
    for (std::uint32_t i = 0; i < o.steps; ++i) {
        world.step(o.dt);
        const auto s = world.stats();
        samples.push_back(s.stepMilliseconds);
        candidateTotal += s.broadphaseCandidates;
        contactTotal += s.contacts;
        if (csv) csv << i + 1 << ',' << (i + 1) * o.dt << ',' << s.stepMilliseconds << ',' << s.broadphaseCandidates << ',' << s.contacts << '\n';
    }
    if (!o.trajectory.empty()) writeTrajectory(o);
    const double totalMs = std::accumulate(samples.begin(), samples.end(), 0.0);
    std::cout << std::fixed << std::setprecision(3)
              << "{\"scene\":\"" << o.scene << "\",\"seed\":" << o.seed
              << ",\"bodies\":" << world.bodies().size() << ",\"steps\":" << o.steps
              << ",\"simulated_seconds\":" << o.steps * o.dt
              << ",\"wall_seconds\":" << totalMs / 1000.0
              << ",\"steps_per_second\":" << (totalMs > 0.0 ? o.steps * 1000.0 / totalMs : 0.0)
              << ",\"step_ms_p50\":" << percentile(samples, 0.50)
              << ",\"step_ms_p95\":" << percentile(samples, 0.95)
              << ",\"step_ms_p99\":" << percentile(samples, 0.99)
              << ",\"broadphase_candidates_total\":" << candidateTotal
              << ",\"contacts_total\":" << contactTotal << "}\n";
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseArgs(argc, argv);
        if (options.help) { printHelp(); return 0; }
        run(options);
    } catch (const std::exception& error) {
        std::cerr << "quantumbody: " << error.what() << '\n';
        return 2;
    }
}
