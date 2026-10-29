
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <ws2tcpip.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#pragma comment(lib, "ws2_32.lib")

// ------------------------------------------------------------
// Configuration
// ------------------------------------------------------------

constexpr char DEFAULT_CSV_FILE[] = "fallback.csv";
constexpr char DESTINATION_IP[] = "127.0.0.1";
constexpr uint16_t DESTINATION_PORT = 9001;

// Based on the timestamp increments observed in your CSV.
// This is configurable, not a verified LPMS specification.
constexpr double CSV_TICKS_PER_SECOND = 500.0;
constexpr double PLAYBACK_SPEED = 1.0;
constexpr double DEFAULT_INTERVAL_SECONDS = 0.01;

using Clock = std::chrono::steady_clock;

#pragma pack(push, 1)
struct ImuPacket 
{
    float qw, qx, qy, qz;
    float wx, wy, wz;
    float ax, ay, az;
    uint32_t timestamp;
};
#pragma pack(pop)

static_assert(sizeof(ImuPacket) == 44, "ImuPacket must be exactly 44 bytes");

struct CsvSample 
{
    ImuPacket packet{};
    double intervalToNext = DEFAULT_INTERVAL_SECONDS;
};

// ------------------------------------------------------------
// CSV parsing
// ------------------------------------------------------------

std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");

    if (first == std::string::npos) {
        return "";
    }

    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    // Suitable for the numeric CSV format exported by LPMS.
    std::vector<std::string> fields;
    std::string current;

    for (char c : line) 
    {
        if (c == ',') 
        {
            fields.push_back(trim(current));
            current.clear();
        }
        else 
        {
            current += c;
        }
    }

    fields.push_back(trim(current));
    return fields;
}

using ColumnMap = std::unordered_map<std::string, size_t>;

const std::string& fieldByName(const std::vector<std::string>& fields, const ColumnMap& columns, const std::string& name) 
{
    const auto it = columns.find(name);

    if (it == columns.end() || it->second >= fields.size()) 
    {
        throw std::runtime_error("Missing field: " + name);
    }

    return fields[it->second];
}

float readFloat(const std::vector<std::string>& fields, const ColumnMap& columns, const std::string& name) 
{
    const std::string& text = fieldByName(fields, columns, name);

    size_t used = 0;
    const float value = std::stof(text, &used);

    if (used != text.size() || !std::isfinite(value)) 
    {
        throw std::runtime_error("Invalid value: " + name);
    }

    return value;
}

uint32_t readTimestamp(const std::vector<std::string>& fields, const ColumnMap& columns) {

    const std::string& text = fieldByName(fields, columns, "ts");

    size_t used = 0;
    const auto value = std::stoull(text, &used);

    if (used != text.size() || value > (std::numeric_limits<uint32_t>::max)()) 
    {
        throw std::runtime_error("Invalid timestamp");
    }

    return static_cast<uint32_t>(value);
}

std::vector<CsvSample> loadCsv(const std::string& filename) 
{
    std::ifstream file(filename);

    if (!file) 
    {
        throw std::runtime_error("Cannot open CSV: " + filename);
    }

    std::string line;

    if (!std::getline(file, line)) 
    {
        throw std::runtime_error("CSV is empty");
    }

    const auto headers = splitCsvLine(line);
    ColumnMap columns;

    for (size_t i = 0; i < headers.size(); ++i) 
    {
        columns[headers[i]] = i;
    }

    const std::array<std::string, 11> required = {
        "ts",
        "QuatW", "QuatX", "QuatY", "QuatZ",
        "AngVelX(dps)", "AngVelY(dps)", "AngVelZ(dps)",
        "LinAccX(g)", "LinAccY(g)", "LinAccZ(g)"
    };

    for (const auto& name : required) 
    {
        if (columns.find(name) == columns.end()) 
        {
            throw std::runtime_error("Required CSV column missing: " + name);
        }
    }

    std::vector<CsvSample> samples;
    size_t lineNumber = 1;

    while (std::getline(file, line)) 
    {
        ++lineNumber;

        if (trim(line).empty()) 
        {
            continue;
        }

        try 
        {
            const auto fields = splitCsvLine(line);

            CsvSample sample{};
            ImuPacket& p = sample.packet;

            p.timestamp = readTimestamp(fields, columns);

            p.qw = readFloat(fields, columns, "QuatW");
            p.qx = readFloat(fields, columns, "QuatX");
            p.qy = readFloat(fields, columns, "QuatY");
            p.qz = readFloat(fields, columns, "QuatZ");

            p.wx = readFloat(fields, columns, "AngVelX(dps)");
            p.wy = readFloat(fields, columns, "AngVelY(dps)");
            p.wz = readFloat(fields, columns, "AngVelZ(dps)");

            p.ax = readFloat(fields, columns, "LinAccX(g)");
            p.ay = readFloat(fields, columns, "LinAccY(g)");
            p.az = readFloat(fields, columns, "LinAccZ(g)");

            samples.push_back(sample);
        }
        catch (const std::exception& e) 
        {
            std::cerr << "[CSV] Skipping line "<< lineNumber << ": " << e.what() << '\n';
        }
    }

    if (samples.empty()) 
    {
        throw std::runtime_error("No valid CSV samples found");
    }

    // Calculate the interval FROM each sample TO the next.
    for (size_t i = 0; i + 1 < samples.size(); ++i) 
    {
        const uint32_t current = samples[i].packet.timestamp;
        const uint32_t next = samples[i + 1].packet.timestamp;

        // Unsigned subtraction supports uint32 timestamp wraparound.
        const uint32_t delta = next - current;

        if (delta > 0 && delta <= 50000) 
        {
            samples[i].intervalToNext = static_cast<double>(delta) / CSV_TICKS_PER_SECOND;
        }
    }

    // At the loop boundary, use the first recorded interval.
    if (samples.size() > 1) 
    {
        samples.back().intervalToNext = samples.front().intervalToNext;
    }

    return samples;
}

// ------------------------------------------------------------
// Main
// ------------------------------------------------------------

int main(int argc, char* argv[]) 
{
    const std::string filename = argc > 1 ? argv[1] : DEFAULT_CSV_FILE;

    try 
    {
        std::cout << "LPMS IMU CSV Player\n";
        std::cout << "Loading: " << filename << '\n';

        const auto samples = loadCsv(filename);

        std::cout << "Loaded " << samples.size() << " samples\n";

        WSADATA wsa{};

        const int startupResult = WSAStartup(MAKEWORD(2, 2), &wsa);

        if (startupResult != 0) 
        {
            throw std::runtime_error("WSAStartup failed: " +  std::to_string(startupResult));
        }

        SOCKET udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        if (udp == INVALID_SOCKET) 
        {
            const int error = WSAGetLastError();
            WSACleanup();

            throw std::runtime_error("Cannot create UDP socket: " + std::to_string(error));
        }

        sockaddr_in destination{};
        destination.sin_family = AF_INET;
        destination.sin_port = htons(DESTINATION_PORT);

        if (inet_pton(AF_INET, DESTINATION_IP, &destination.sin_addr) != 1) {

            closesocket(udp);
            WSACleanup();
            throw std::runtime_error("Invalid destination IP");
        }

        std::cout << "Sending to "
            << DESTINATION_IP << ":"
            << DESTINATION_PORT << '\n';

        std::cout << "Playback speed: "
            << PLAYBACK_SPEED << "x\n";

        std::cout << "Looping playback. Press Ctrl+C to stop.\n\n";

        size_t index = 0;
        uint64_t sentCount = 0;

        auto nextSendTime = Clock::now();

        while (true) 
        {
            std::this_thread::sleep_until(nextSendTime);

            const CsvSample& sample = samples[index];

            const int sent = sendto(udp, reinterpret_cast<const char*>(&sample.packet), sizeof(sample.packet), 0, reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));

            if (sent == SOCKET_ERROR) 
            {
                std::cerr << "[UDP] Send failed: " << WSAGetLastError() << '\n';
                break;
            }

            ++sentCount;

            if (sentCount % 1000 == 0) 
            {
                std::cout << "[CSV] Sent " << sentCount << " packets\n";
            }

            const double interval = sample.intervalToNext / PLAYBACK_SPEED;

            nextSendTime += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(interval));

            // If playback falls behind significantly, resynchronize
            // rather than transmitting a burst of old packets.
            if (nextSendTime + std::chrono::seconds(1) < Clock::now()) 
            {
                nextSendTime = Clock::now();
            }

            ++index;

            if (index >= samples.size()) 
            {
                index = 0;
                std::cout << "[CSV] Restarting playback loop\n";
            }
        }

        closesocket(udp);
        WSACleanup();

        return EXIT_FAILURE;
    }
    catch (const std::exception& e) 
    {
        std::cerr << "[FATAL] " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}
