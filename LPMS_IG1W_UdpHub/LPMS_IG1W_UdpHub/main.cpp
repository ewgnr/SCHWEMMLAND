#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#pragma comment(lib, "ws2_32.lib")

// ------------------------------------------------------------
// Configuration
// ------------------------------------------------------------

constexpr uint16_t INPUT_PORT = 9001;

constexpr std::array<uint16_t, 4> OUTPUT_PORTS = 
{
    9002, 9003, 9004, 9005
};

// Must match the middleware and visualizers exactly.
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

// ------------------------------------------------------------
// Main
// ------------------------------------------------------------

int main() 
{
    WSADATA wsa{};

    const int wsaResult = WSAStartup(MAKEWORD(2, 2), &wsa);

    if (wsaResult != 0) 
    {
        std::cerr << "[FATAL] WSAStartup failed: " << wsaResult << '\n';
        
        return EXIT_FAILURE;
    }

    SOCKET inputSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (inputSocket == INVALID_SOCKET) 
    {
        std::cerr << "[FATAL] Cannot create input socket: " << WSAGetLastError() << '\n'; 
        
        WSACleanup();
        
        return EXIT_FAILURE;
    }

    sockaddr_in inputAddress{};
    inputAddress.sin_family = AF_INET;
    inputAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    inputAddress.sin_port = htons(INPUT_PORT);

    if (bind(inputSocket, reinterpret_cast<const sockaddr*>(&inputAddress), sizeof(inputAddress)) == SOCKET_ERROR) 
    {
        std::cerr << "[FATAL] Cannot bind port " << INPUT_PORT << ": " << WSAGetLastError() << '\n';

        closesocket(inputSocket);
        WSACleanup();

        return EXIT_FAILURE;
    }

    // One output socket sends copies to all four visualizers.
    SOCKET outputSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (outputSocket == INVALID_SOCKET) 
    {
        std::cerr << "[FATAL] Cannot create output socket: " << WSAGetLastError() << '\n';

        closesocket(inputSocket);
        WSACleanup();
        
        return EXIT_FAILURE;
    }

    sockaddr_in destination{};
    destination.sin_family = AF_INET;

    if (inet_pton(AF_INET, "127.0.0.1", &destination.sin_addr) != 1) 
    {
        std::cerr << "[FATAL] Cannot configure destination address\n";

        closesocket(outputSocket);
        closesocket(inputSocket);
        WSACleanup();
   
        return EXIT_FAILURE;
    }

    std::cout << "\nLPMS IMU UDP Adapter started\n";
    std::cout << "Input:  127.0.0.1 / port " << INPUT_PORT << '\n';
    std::cout << "Output: 127.0.0.1 / ports ";

    for (size_t i = 0; i < OUTPUT_PORTS.size(); ++i) 
    {
        std::cout << OUTPUT_PORTS[i];

        if (i + 1 < OUTPUT_PORTS.size()) 
        {
            std::cout << ", ";
        }
    }

    std::cout << "\nWaiting for packets...\n\n";

    uint64_t packetCount = 0;

    while (true) 
    {
        ImuPacket packet{};

        const int received = recvfrom(
            inputSocket,
            reinterpret_cast<char*>(&packet),
            sizeof(packet),
            0,
            nullptr,
            nullptr);

        if (received == SOCKET_ERROR) 
        {
            std::cerr << "[UDP] Receive error: " << WSAGetLastError() << '\n';
            
            break;
        }

        // Forward only complete packets matching the protocol.
        if (received != sizeof(ImuPacket)) 
        {
            std::cerr << "[UDP] Ignoring unexpected datagram size: " << received << " bytes\n";
            
            continue;
        }

        // Forward the original bytes unchanged to each visualizer.
        for (uint16_t port : OUTPUT_PORTS) 
        {
            destination.sin_port = htons(port);

            const int sent = sendto(outputSocket, reinterpret_cast<const char*>(&packet), sizeof(packet), 0, reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));

            if (sent == SOCKET_ERROR) 
            {
                std::cerr << "[UDP] Send to port " << port  << " failed: " << WSAGetLastError() << '\n';
            }
        }

        ++packetCount;

        if (packetCount % 1000 == 0) 
        {
            std::cout << "[UDP] Forwarded " << packetCount << " packets\n";
        }
    }

    closesocket(outputSocket);
    closesocket(inputSocket);
    WSACleanup();

    return EXIT_SUCCESS;
}
