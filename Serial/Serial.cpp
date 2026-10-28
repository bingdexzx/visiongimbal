#include "Serial.h"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#include <utility>

#include <fcntl.h>

namespace TRoMaC
{
namespace
{
speed_t baudRate(int value) noexcept
{
    switch (value)
    {
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 921600: return B921600;
        case 1000000: return B1000000;
        default: return 0;
    }
}

uint16_t crc16Modbus(const uint8_t* bytes, std::size_t size) noexcept
{
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1U) != 0
                ? static_cast<uint16_t>((crc >> 1) ^ 0xA001U)
                : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

bool writeAll(int fd, const uint8_t* bytes, std::size_t size) noexcept
{
    if (fd < 0 || (bytes == nullptr && size != 0))
    {
        return false;
    }

    std::size_t offset = 0;
    while (offset < size)
    {
        const ssize_t count = write(fd, bytes + offset, size - offset);
        if (count > 0)
        {
            offset += static_cast<std::size_t>(count);
        }
        else if (count < 0 && errno == EINTR)
        {
            continue;
        }
        else
        {
            return false;
        }
    }
    return true;
}

void writeFloatLittleEndian(uint8_t* destination, float value) noexcept
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    destination[0] = static_cast<uint8_t>(bits & 0xFFU);
    destination[1] = static_cast<uint8_t>((bits >> 8) & 0xFFU);
    destination[2] = static_cast<uint8_t>((bits >> 16) & 0xFFU);
    destination[3] = static_cast<uint8_t>(bits >> 24);
}

float readFloatLittleEndian(const uint8_t* source) noexcept
{
    const uint32_t bits = static_cast<uint32_t>(source[0]) |
                          (static_cast<uint32_t>(source[1]) << 8) |
                          (static_cast<uint32_t>(source[2]) << 16) |
                          (static_cast<uint32_t>(source[3]) << 24);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

int openDevice(int comport, const char*& path) noexcept
{
    switch (comport)
    {
        case 1: path = "/dev/ttyUSB_vision"; break;
        case 2: path = "/dev/ttyS1"; break;
        case 3: path = "/dev/ttyS2"; break;
        default: path = nullptr; return -1;
    }
    return open(path, O_RDWR | O_NOCTTY | O_NDELAY);
}
} // namespace

RxBase::RxBase() noexcept
    : state_(RxParseState::Header0),
      payload_{},
      crcBytes_{},
      command_(0),
      payloadSize_(0),
      payloadIndex_(0),
      crcLow_(0),
      crcHigh_(0),
      ready_(false)
{
}

bool RxBase::consume(uint8_t byte)
{
    switch (state_)
    {
        case RxParseState::Header0:
            if (byte == kFrameHeader0)
            {
                ready_ = false;
                crcBytes_[0] = byte;
                state_ = RxParseState::Header1;
            }
            break;

        case RxParseState::Header1:
            if (byte == kFrameHeader1)
            {
                crcBytes_[1] = byte;
                state_ = RxParseState::Command;
            }
            else if (byte == kFrameHeader0)
            {
                ready_ = false;
                crcBytes_[0] = byte;
            }
            else
            {
                resetParser();
            }
            break;

        case RxParseState::Command:
            command_ = byte;
            crcBytes_[2] = byte;
            state_ = RxParseState::Length;
            break;

        case RxParseState::Length:
            if (byte > kMaxRxPayloadSize)
            {
                std::cerr << "接收错误：载荷长度超过最大值 32: "
                          << static_cast<int>(byte) << '\n';
                resetParser();
                break;
            }
            payloadSize_ = byte;
            crcBytes_[3] = byte;
            payloadIndex_ = 0;
            state_ = byte == 0 ? RxParseState::CrcLow : RxParseState::Data;
            break;

        case RxParseState::Data:
            payload_[payloadIndex_] = byte;
            crcBytes_[4 + payloadIndex_] = byte;
            ++payloadIndex_;
            if (payloadIndex_ == payloadSize_)
            {
                state_ = RxParseState::CrcLow;
            }
            break;

        case RxParseState::CrcLow:
            crcLow_ = byte;
            state_ = RxParseState::CrcHigh;
            break;

        case RxParseState::CrcHigh:
            state_ = RxParseState::Tail;
            crcHigh_ = byte;
            break;

        case RxParseState::Tail:
            return finishFrame(byte);
    }
    return false;
}

bool RxBase::finishFrame(uint8_t tailByte)
{
    const uint16_t receivedCrc = static_cast<uint16_t>(crcLow_) |
        static_cast<uint16_t>(crcHigh_ << 8);
    const uint16_t calculatedCrc = crc16Modbus(
        crcBytes_.data(), 4 + static_cast<std::size_t>(payloadSize_));
    const bool valid = tailByte == kFrameTail && receivedCrc == calculatedCrc;

    if (!valid)
    {
        if (tailByte != kFrameTail)
        {
            std::cerr << "接收错误：帧尾错误，收到 0x"
                      << std::hex << static_cast<int>(tailByte)
                      << "，期望 0x" << static_cast<int>(kFrameTail)
                      << std::dec << '\n';
        }
        else
        {
            std::cerr << "接收错误：CRC 校验失败，收到 0x"
                      << std::hex << receivedCrc
                      << "，计算得到 0x" << calculatedCrc
                      << std::dec << '\n';
        }
    }
    ready_ = valid;
    resetParser();
    return valid;
}

bool RxBase::ready() const noexcept
{
    return ready_;
}

uint8_t RxBase::command() const noexcept
{
    return command_;
}

std::size_t RxBase::payloadSize() const noexcept
{
    return payloadSize_;
}

const std::array<uint8_t, kMaxRxPayloadSize>& RxBase::payload() const noexcept
{
    return payload_;
}

void RxBase::clear() noexcept
{
    payload_.fill(0);
    crcBytes_.fill(0);
    command_ = 0;
    payloadSize_ = 0;
    ready_ = false;
    resetParser();
}

void RxBase::resetParser() noexcept
{
    state_ = RxParseState::Header0;
    payloadIndex_ = 0;
    crcLow_ = 0;
    crcHigh_ = 0;
}

TxBase::TxBase() noexcept : bytes_{}, size_(0)
{
}

void TxBase::begin(uint8_t command, std::size_t payloadSize) noexcept
{
    bytes_.fill(0);
    size_ = 4 + payloadSize + 3;
    bytes_[0] = kFrameHeader0;
    bytes_[1] = kFrameHeader1;
    bytes_[2] = command;
    bytes_[3] = static_cast<uint8_t>(payloadSize);
}

uint8_t* TxBase::payload() noexcept
{
    return bytes_.data() + 4;
}

void TxBase::finish() noexcept
{
    const std::size_t crcOffset = size_ - 3;
    const uint16_t crc = crc16Modbus(bytes_.data(), crcOffset);
    bytes_[crcOffset] = static_cast<uint8_t>(crc & 0xFFU);
    bytes_[crcOffset + 1] = static_cast<uint8_t>(crc >> 8);
    bytes_[crcOffset + 2] = kFrameTail;
}

const uint8_t* TxBase::bytes() const noexcept
{
    return bytes_.data();
}

std::size_t TxBase::size() const noexcept
{
    return size_;
}

bool TxBase::writeTo(int fd) const noexcept
{
    return writeAll(fd, bytes(), size());
}

TxCommand01::TxCommand01(const VisionData& data) noexcept
{
    begin(kVisionCommand, sizeof(VisionData));
    uint8_t* const raw = payload();
    raw[0] = data.Gimbal_Ctrl;
    writeFloatLittleEndian(raw + 1, data.pitch_angle);
    writeFloatLittleEndian(raw + 5, data.yaw_angle);
    writeFloatLittleEndian(raw + 9, data.pitch_delta_angle);
    writeFloatLittleEndian(raw + 13, data.yaw_delta_angle);
    raw[17] = data.fric;
    raw[18] = data.Auto_Push;
    raw[19] = data.Push_flag;
    finish();
}

bool TxCommand01::writeTo(int fd) const noexcept
{
    return TxBase::writeTo(fd);
}

std::size_t TxCommand01::size() const noexcept
{
    return TxBase::size();
}

Uart::Uart() noexcept
    : serialId_(-1), speed_(0), devicePath_(), rx_{}
{
}

Uart::~Uart() noexcept
{
    Close();
}

Uart::Uart(Uart&& other) noexcept
    : serialId_(other.serialId_),
      speed_(other.speed_),
      devicePath_(std::move(other.devicePath_)),
      rx_(other.rx_)
{
    other.serialId_ = -1;
    other.speed_ = 0;
    other.rx_.clear();
}

Uart& Uart::operator=(Uart&& other) noexcept
{
    if (this != &other)
    {
        Close();
        serialId_ = other.serialId_;
        speed_ = other.speed_;
        devicePath_ = std::move(other.devicePath_);
        rx_ = other.rx_;
        other.serialId_ = -1;
        other.speed_ = 0;
        other.rx_.clear();
    }
    return *this;
}

void Uart::Close() noexcept
{
    if (serialId_ >= 0)
    {
        close(serialId_);
        serialId_ = -1;
    }
    speed_ = 0;
    devicePath_.clear();
    rx_.clear();
}

bool Uart::IsOpen() const noexcept
{
    return serialId_ >= 0;
}

const std::string& Uart::devicePath() const noexcept
{
    return devicePath_;
}

int Uart::open_port(int comport)
{
    const char* path = nullptr;
    const int fd = openDevice(comport, path);
    if (fd < 0)
    {
        if (path == nullptr)
        {
            std::cerr << "无效串口编号: " << comport << '\n';
        }
        else
        {
            std::cerr << "无法打开串口 " << path << ": "
                      << std::strerror(errno) << '\n';
        }
        return -1;
    }

    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        std::cerr << "无法设置串口为阻塞模式: " << std::strerror(errno) << '\n';
        close(fd);
        return -1;
    }
    devicePath_ = path;
    return fd;
}

int Uart::set_opt(int fd, int baud, int bits, char parity, int stopBits)
{
    if (fd < 0 || bits != 8 || (stopBits != 1 && stopBits != 2))
    {
        return -1;
    }

    const speed_t rate = baudRate(baud);
    if (rate == 0)
    {
        return -1;
    }

    termios options{};
    if (tcgetattr(fd, &options) != 0)
    {
        return -1;
    }

    cfmakeraw(&options);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CSTOPB;
    if (stopBits == 2)
    {
        options.c_cflag |= CSTOPB;
    }

    options.c_cflag &= ~(PARENB | PARODD);
    if (parity == 'e' || parity == 'E')
    {
        options.c_cflag |= PARENB;
    }
    else if (parity == 'o' || parity == 'O')
    {
        options.c_cflag |= PARENB | PARODD;
    }
    else if (parity != 'n' && parity != 'N')
    {
        return -1;
    }

    cfsetispeed(&options, rate);
    cfsetospeed(&options, rate);
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    return tcsetattr(fd, TCSANOW, &options) == 0 ? 0 : -1;
}

bool Uart::Open(int comport, int baud)
{
    Close();
    const int fd = open_port(comport);
    if (fd < 0)
    {
        return false;
    }
    if (set_opt(fd, baud, 8, 'N', 1) != 0)
    {
        close(fd);
        devicePath_.clear();
        return false;
    }
    serialId_ = fd;
    speed_ = baud;
    return true;
}

bool Uart::checkSerial()
{
    if (!IsOpen())
    {
        return false;
    }
    termios options{};
    if (tcgetattr(serialId_, &options) == 0)
    {
        return true;
    }
    Close();
    return false;
}

bool Uart::ReadData()
{
    if (!IsOpen())
    {
        return false;
    }

    uint8_t bytes[64]{};
    ssize_t count;
    do
    {
        count = read(serialId_, bytes, sizeof(bytes));
    } while (count < 0 && errno == EINTR);

    if (count <= 0)
    {
        return false;
    }

    bool receivedFrame = false;
    for (ssize_t i = 0; i < count; ++i)
    {
        receivedFrame = ProtocolParse(bytes[i]) || receivedFrame;
    }
    return receivedFrame;
}

bool Uart::send(const VisionData& data)
{
    const TxCommand01 frame(data);
    return frame.writeTo(serialId_);
}

bool Uart::ProtocolParse(uint8_t byte)
{
    return rx_.consume(byte);
}

const RxBase& Uart::lastRxFrame() const noexcept
{
    return rx_;
}

bool Uart::tryGetGimbalFeedback(GimbalFeedback& output) const noexcept
{
    if (!rx_.ready() || rx_.command() != kVisionCommand ||
        rx_.payloadSize() != sizeof(GimbalFeedback))
    {
        return false;
    }

    const auto& payload = rx_.payload();
    output.yaw_angle = readFloatLittleEndian(payload.data());
    output.pitch_angle = readFloatLittleEndian(payload.data() + sizeof(float));
    return true;
}

} // namespace TRoMaC
