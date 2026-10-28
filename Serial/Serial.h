#ifndef TROMAC_SERIAL_H_
#define TROMAC_SERIAL_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace TRoMaC
{

constexpr uint8_t kFrameHeader0 = 0xA5;
constexpr uint8_t kFrameHeader1 = 0x5A;
constexpr uint8_t kVisionCommand = 0x01;
constexpr uint8_t kFrameTail = 0x15;
constexpr std::size_t kMaxRxPayloadSize = 32;

#pragma pack(push, 1)
// 下位机回传的云台绝对角度，顺序与 measure[2] 一致：yaw、pitch。
struct GimbalFeedback
{
    float yaw_angle;
    float pitch_angle;
};

// Command 0x01 payload. Floats are serialized as little-endian IEEE-754.
struct VisionData
{
    uint8_t Gimbal_Ctrl;
    float pitch_angle;
    float yaw_angle;
    float pitch_delta_angle;
    float yaw_delta_angle;
    uint8_t fric;
    uint8_t Auto_Push;
    uint8_t Push_flag;
};

#pragma pack(pop)

static_assert(sizeof(float) == 4, "Protocol requires a 4-byte float");
static_assert(sizeof(GimbalFeedback) == 8, "GimbalFeedback protocol size changed");
static_assert(sizeof(VisionData) == 20, "VisionData protocol size changed");

enum class RxParseState : uint8_t
{
    Header0,
    Header1,
    Command,
    Length,
    Data,
    CrcLow,
    CrcHigh,
    Tail
};

// Common receive-frame parser and raw payload storage.
class RxBase
{
public:
    RxBase() noexcept;
    RxBase(const RxBase&) = default;
    RxBase& operator=(const RxBase&) = default;

    bool consume(uint8_t byte);
    bool ready() const noexcept;
    uint8_t command() const noexcept;
    std::size_t payloadSize() const noexcept;
    const std::array<uint8_t, kMaxRxPayloadSize>& payload() const noexcept;
    void clear() noexcept;

private:
    void resetParser() noexcept;
    bool finishFrame(uint8_t tailByte);

    RxParseState state_;
    std::array<uint8_t, kMaxRxPayloadSize> payload_;
    std::array<uint8_t, 4 + kMaxRxPayloadSize> crcBytes_;
    uint8_t command_;
    uint8_t payloadSize_;
    std::size_t payloadIndex_;
    uint8_t crcLow_;
    uint8_t crcHigh_;
    bool ready_;
};

// Common transmit-frame construction: header, length, CRC and tail.
class TxBase
{
public:
    const uint8_t* bytes() const noexcept;
    std::size_t size() const noexcept;

protected:
    TxBase() noexcept;
    void begin(uint8_t command, std::size_t payloadSize) noexcept;
    uint8_t* payload() noexcept;
    void finish() noexcept;
    bool writeTo(int fd) const noexcept;

private:
    std::array<uint8_t, 4 + kMaxRxPayloadSize + 3> bytes_;
    std::size_t size_;
};

class TxCommand01 final : private TxBase
{
public:
    explicit TxCommand01(const VisionData& data) noexcept;
    std::size_t size() const noexcept;
    bool writeTo(int fd) const noexcept;
};

class Uart
{
public:
    Uart() noexcept;
    ~Uart() noexcept;

    Uart(const Uart&) = delete;
    Uart& operator=(const Uart&) = delete;
    Uart(Uart&& other) noexcept;
    Uart& operator=(Uart&& other) noexcept;

    void Close() noexcept;
    bool IsOpen() const noexcept;
    const std::string& devicePath() const noexcept;

    int open_port(int comport);
    int set_opt(int fd, int speed, int bits, char parity, int stopBits);
    bool Open(int comport, int speed);
    bool checkSerial();
    bool ReadData();

    bool send(const VisionData& data);
    bool ProtocolParse(uint8_t byte);

    const RxBase& lastRxFrame() const noexcept;
    bool tryGetGimbalFeedback(GimbalFeedback& output) const noexcept;

private:
    int serialId_;
    int speed_;
    std::string devicePath_;
    RxBase rx_;
};

} // namespace TRoMaC

#endif // TROMAC_SERIAL_H_
