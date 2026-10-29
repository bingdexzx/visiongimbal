// “禁止修改”是例程基础设施；自己的代码写在“需要编写”区域。

/*---------禁止修改：已有头文件----------*/
#include "Serial.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <chrono>
#include <cmath>
#include <iostream>

#include <mutex>
#include <thread>
#include <atomic>
/*------------------------------------*/



/*---------可自行添加：算法需要的头文件----------*/
#include <cstdio>
#include <string>
#include <vector>
/*---------添加区域结束----------*/



/*---------------------可修改：相机和串口开关参数--------------------*/

// 相机编号
// 当前 /dev/video0 是图像节点。
// /dev/video1 是元数据节点，不是第二台相机。
int cameraIndex = 0;

// 图像分辨率
// 本例使用 MJPG，驱动支持以下宽×高组合（不能随意混搭）：
// 1280×720、800×600、640×480、352×288、320×240、176×144。
int width = 1280;
int height = 720;

// 请求帧率
// 上述 MJPG 分辨率均支持 25、30、60 fps；YUYV 仅列出 10 fps。
// 长曝光、USB 带宽等因素仍可能使实际帧率降低。
int fps = 30;

// 串口开关
// 没接下位机时改成 false，只调图像。
bool useSerial = true;

// 串口编号：1 = /dev/ttyUSB_vision，2 = /dev/ttyS1，3 = /dev/ttyS2。
// 使用其他设备路径时，修改 Serial/Serial.cpp 中的 openDevice。
int serialPort = 1;

// 波特率，必须与下位机一致。
int serialBaudRate = 230400;

/*---------参数修改区域结束----------*/



/*---------可修改：相机画面参数（改完重新编译并运行）----------*/

// 【参数来源】
// 2026-10-24 从当前 USB Camera（uvcvideo，/dev/video0）查询，未改动相机设置。
// 以下“默认”是驱动报告的默认值，不是程序选用的调参值。
// 换相机后要重新查询，复查命令：
//   v4l2-ctl -d /dev/video0 --list-ctrls-menus --list-formats-ext
//
// 范围使用 OpenCV 4.6 V4L2 默认的原始数值模式。
// 不要开启环境变量 OPENCV_VIDEOIO_V4L_RANGE_NORMALIZED。


// 【缓冲与延迟】

// 缓冲帧数
// 范围：OpenCV 后端允许 1～10 帧，默认 4；实际分配受驱动限制。
// 作用：设为 1 可以减少排队的旧画面。
// 注意：缓冲不是图像控制项，不能用 --list-ctrls 查询范围。
int cameraBufferSize = 1;


// 【曝光与亮度】

// 曝光模式
// 可选：1 = 手动曝光；3 = 光圈优先自动曝光。
// 默认：3；这台相机不支持 0、2。
int cameraAutoExposure = 1;   // （需调参）

// 曝光时间（仅手动曝光时有效）
// 范围：1～10000，步长 1，默认 166。
// 单位：100 us，即 19 = 1.9 ms，166 = 16.6 ms。
// 实际时间范围：0.1～1000 ms。
// 作用：增大通常更亮，但运动拖影更明显，过长还可能降低实际帧率。
int cameraExposure = 19;   // （需调参）

// 亮度
// 范围：-64～64，步长 1，默认 0。
// 作用：调整画面整体明暗。
int cameraBrightness = 18;   // （需调参）

// 对比度
// 范围：0～100，步长 1，默认 35。
// 作用：调整明暗区域的差别。
int cameraContrast = 50;   // （需调参）

// 伽马
// 范围：100～500，步长 1，默认 300。
// 作用：调整中间亮度层次。
int cameraGamma = 266;   // （需调参）

// 逆光补偿
// 范围：0～2，步长 1，默认 0。
// 作用：改善逆光环境下的主体亮度。
int cameraBacklight = 2;   // （需调参）


// 【颜色与白平衡】

// 饱和度
// 范围：0～100，步长 1，默认 64。
// 作用：调整颜色浓淡，会影响 HSV 阈值。
int cameraSaturation = 100;   // （需调参）

// 色调
// 范围：-180～180，步长 1，默认 0。
// 作用：改变画面颜色，通常保持 0。
int cameraHue = 0;

// 自动白平衡
// 可选：false / 0 = 关闭；true / 1 = 开启。
// 默认：开启。
bool cameraAutoWhiteBalance = true;   // （需调参）

// 手动白平衡色温
// 范围：2800～6500 K，步长 10 K，默认 4600 K。
// 注意：必须先关闭自动白平衡；开启时驱动将此项标记为 inactive。
int cameraWhiteBalance = 4600;   // （需调参）


// 【边缘细节】

// 锐度
// 范围：0～100，步长 1，默认 80。
// 作用：增强边缘；过大可能放大噪点、产生假边缘。
int cameraSharpness = 100;   // （需调参）


// 以上是驱动声明的范围，未逐项实测效果。

/*---------相机画面参数修改区域结束----------*/



/*---------需要编写：自己的全局变量和算法参数----------*/
// 在这里添加 HSV 阈值,计时起点等。

// 识别到的目标
struct BladeTarget
{
    cv::Point2f center;          // 框中心
    float radius = 0.0F;         // 框长边的一半
    double score = 0.0;          // 框的面积 调试用
    bool valid = false;          // 这帧有没有找到
};

// 调试开关
static bool kShowDebug = true;    // 调试窗口 按 d 关
static bool kEnableFire = false;  // 开火总开关 调参时一定关着

// 瞄准点 打完看偏哪边就往反方向挪
static double kAimU = width * 0.5;   // （需调参）
static double kAimV = height * 0.5;   // （需调参）

// 颜色 用 any 可能锁到对面
enum class TargetColor
{
    Red,
    Blue,
    Any
};
static TargetColor kTargetColor = TargetColor::Any;   // （需调参）

// 红色 H 跨两头 分两段
static cv::Scalar red_ranges_low[2]  = {cv::Scalar(0, 130, 137),   cv::Scalar(11, 255, 255)};   // （需调参）
static cv::Scalar red_ranges_high[2] = {cv::Scalar(168, 130, 137), cv::Scalar(179, 255, 255)};   // （需调参）

// 蓝色一段
static cv::Scalar blue_low  = cv::Scalar(100, 120, 80);   // （需调参）
static cv::Scalar blue_high = cv::Scalar(130, 255, 255);   // （需调参）

// 形态学核
static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

// 丢几帧还算有目标
static int    kHoldFrames = 8;

// 像素转角度
static double kFocalPx = 700.0;   // （需调参）
static double kYawSign = 1.0;       // 图像往右 yaw 变大填 1 变小填 -1（需调参）
static double kPitchSign = 1.0;     // 图像往上 pitch 变大填 1 变小填 -1（需调参）
static double kYawBias = 0.0;       // 固定的角度偏差（需调参）
static double kPitchBias = 0.0;     // 弹道下坠 远了要加（需调参）
static double kGain = 0.8;          // 抖就调小 慢就调大（需调参）
static double kMaxStep = 6.0;       // 没反馈时一帧最多转多少度

// 开火
static double kAimTolPx = 12.0;         // 差多少像素算对准（需调参）
static double kAimTolDegMoving = 4.0;   // 打运动目标要放宽 不然永远不开火（需调参）
static int    kAimFrames = 4;           // 连着几帧对上才打（需调参）
static int    kShotIntervalMs = 800;    // 两发之间隔多久（需调参）
static int    kMaxShots = 10;           // 一场就 10 发

// 运动模型 1 不动 2 匀速圆周 3 正弦变速
enum class TargetMotion
{
    Static,
    CircleUniform,
    CircleSine
};
static TargetMotion kTargetMotion = TargetMotion::Static;   // （需调参）

// 提前量 不动填 0 动了从 0.08 试
static double kPredictAheadSec = 0.0;   // （需调参）

static double kMaxYawRate = 120.0;       // 直线兜底的速度上限 度每秒
static double kMaxPitchRate = 60.0;
static double kMaxOmega = 8.0;           // 角速度上限 弧度每秒
static double kMaxAlpha = 20.0;          // 角加速度上限 只给正弦模式
static double kCircleHistorySec = 3.0;   // 拟合圆心用多久的历史
static double kPhaseWindowSec = 0.5;     // 拟合角度用多长的窗口
static double kMinArcRad = 0.8;          // 至少转过这么多才算圆
static double kJumpDeg = 12.0;           // 一下跳这么多就是换目标了

// 调参窗口的值 滑块存的是加偏移的
struct TuneState
{
    // 相机窗口
    int autoExposure = 1;    // 0 自动 1 手动
    int exposure     = 19;
    int brightness   = 82;   // 实际亮度 等于这个减 64
    int contrast     = 50;
    int saturation   = 100;
    int gamma        = 166;  // 实际伽马 等于这个加 100
    int sharpness    = 100;
    int backlight    = 2;
    int autoWB       = 1;
    int wbTemp       = 1800; // 实际色温 等于这个加 2800

    // 算法窗口
    int redH         = 0;
    int redS         = 130;
    int redV         = 137;
    int blueH        = 100;
    int blueH2       = 130;
    int blueS        = 120;
    int blueV        = 80;
    int gain         = 80;    // 实际增益 等于这个除 100
    int aimU         = 640;
    int aimV         = 360;
    int aimTol       = 12;
    int aimFrames    = 4;
    int shotInterval = 800;
    int leadMs       = 0;     // 实际提前量 等于这个除 1000
};
static TuneState g_tune;
static bool g_camCreated = false;    // 相机窗口建过没有
static bool g_algoCreated = false;   // 算法窗口建过没有

// 跨帧要记住的
static BladeTarget g_lastTarget;
static cv::Point2f g_filtered;                // 滤波后的点
static bool g_hasFilter = false;
static int  g_lostFrames = 100000;
static int  g_aimedFrames = 0;
static int  g_shotsFired = 0;
static int  g_frameIndex = 0;
static std::chrono::steady_clock::time_point g_lastShotAt =
    std::chrono::steady_clock::now() - std::chrono::seconds(10);   // 开机就能打

/*---------全局变量编写区域结束----------*/



/*---------禁止修改：全局通信数据和收发线程----------*/
TRoMaC::Uart uart;
TRoMaC::VisionData sendData{};        // 只由主线程填写，修改这个变量即可。
TRoMaC::VisionData latestSendData{};  // 填好后复制到这里，供发送线程读取。
std::mutex sendMutex;
bool sendReady = false;              // 第一份数据填好之前不发送。
std::chrono::steady_clock::time_point sendUpdatedAt;
std::atomic<bool> sendFailed(false);
TRoMaC::GimbalFeedback receiveData{}; // 接收线程更新，读取时要加锁。
std::mutex receiveMutex;             // 保证 yaw、pitch 总是来自同一份数据。
bool hasFeedback = false;
std::chrono::steady_clock::time_point receivedAt;
std::atomic<bool> running(true);     // 主线程、发送线程、接收线程共用的退出开关。


// 独立发送线程：读取最近一次完整结果，每次循环等待 10 ms。
void sendLoop()
{
    while (running.load())
    {
        TRoMaC::VisionData data{};
        bool ready;
        {
            std::lock_guard<std::mutex> lock(sendMutex);
            data = latestSendData;
            // 超过 250 ms 没有新的图像处理结果时暂停发送，避免反复发送旧指令。
            ready = sendReady && std::chrono::steady_clock::now() - sendUpdatedAt <
                                     std::chrono::milliseconds(250);
        } // 复制完就解锁，不在锁内等待串口发送。
        if (ready && !uart.send(data))
        {
            std::cerr << "串口发送失败。\n";
            sendFailed.store(true);
            running.store(false);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// 独立接收线程：等待串口数据不会拖慢相机画面。
void receiveLoop()
{
    while (running.load())
    {
        if (!uart.ReadData())
            continue;

        // 下位机回传命令 0x01，两个 float 的顺序是 yaw、pitch，单位为度。
        TRoMaC::GimbalFeedback data{};
        if (uart.tryGetGimbalFeedback(data) &&
            std::isfinite(data.yaw_angle) && std::isfinite(data.pitch_angle))
        {
            std::lock_guard<std::mutex> lock(receiveMutex);
            receiveData = data;
            receivedAt = std::chrono::steady_clock::now();
            hasFeedback = true;
        } // 离开这对大括号，自动解锁。
    }
}
/*---------禁止修改区域结束----------*/



/*---------需要编写：自己的算法函数（按需添加）----------*/
// 在这里定义圆环识别、弹道解算等函数，再到主循环的动手区调用。

static double clampValue(double value, double low, double high)
{
    return value < low ? low : (value > high ? high : value);
}

// 像素差转角度 单位是度
static double pixelToAngle(double pixels, double focalPx)
{
    return std::atan(pixels / focalPx) * 180.0 / CV_PI;
}

// 运动模型 用云台绝对角 跟云台怎么转无关

struct MotionSample
{
    double t;       // 秒
    double yaw;     // 度
    double pitch;   // 度
};

// 拟合角速度
static bool fitAngularVelocity(const std::vector<MotionSample>& samples,
                               double& vyaw, double& vpitch)
{
    const std::size_t n = samples.size();
    if (n < 2)
    {
        return false;
    }
    const double t0 = samples.back().t;
    double st = 0.0, sy = 0.0, sp = 0.0, sty = 0.0, stp = 0.0, stt = 0.0;
    for (std::size_t i = 0; i < n; ++i)
    {
        const double t = samples[i].t - t0;
        st += t;
        sy += samples[i].yaw;
        sp += samples[i].pitch;
        sty += t * samples[i].yaw;
        stp += t * samples[i].pitch;
        stt += t * t;
    }
    const double dn = static_cast<double>(n);
    const double den = dn * stt - st * st;
    if (std::fabs(den) < 1e-9)
    {
        return false;
    }
    vyaw = (dn * sty - st * sy) / den;
    vpitch = (dn * stp - st * sp) / den;
    return true;
}

// 基类 丢帧跳变在这里处理
class ITargetModel
{
public:
    virtual ~ITargetModel() {}

    virtual const char* name() const = 0;
    virtual void reset() = 0;

    // 预测 aheadSec 秒后在哪
    virtual bool predict(double aheadSec, double& yaw, double& pitch) const = 0;

    // 圆心学到没有
    virtual bool centered() const { return true; }

    // 主循环每帧喂一次
    void update(double t, double yaw, double pitch, bool valid)
    {
        if (!valid)
        {
            if (missing_ < 1000000)
            {
                ++missing_;
            }
            if (missing_ > kHoldFrames && hasLast_)
            {
                reset();
                hasLast_ = false;
            }
            return;
        }

        if (hasLast_)
        {
            // 跳变就重置
            const double dt = t - lastT_;
            const bool gap = dt > 0.15;
            const bool jump = std::hypot(yaw - lastYaw_, pitch - lastPitch_) > kJumpDeg;
            if (gap || jump)
            {
                reset();
            }
        }

        missing_ = 0;
        lastT_ = t;
        lastYaw_ = yaw;
        lastPitch_ = pitch;
        hasLast_ = true;
        onSample(t, yaw, pitch);
    }

protected:
    virtual void onSample(double t, double yaw, double pitch) = 0;

    bool hasLast_ = false;
    double lastT_ = 0.0;
    double lastYaw_ = 0.0;
    double lastPitch_ = 0.0;
    int missing_ = 1000000;
};

// 模型 1 不动 本轮用这个
class StaticModel : public ITargetModel
{
public:
    const char* name() const override { return "static"; }

    void reset() override { valid_ = false; }

    bool predict(double, double& yaw, double& pitch) const override
    {
        if (!valid_)
        {
            return false;
        }
        yaw = yaw_;
        pitch = pitch_;
        return true;
    }

protected:
    void onSample(double, double yaw, double pitch) override
    {
        yaw_ = yaw;
        pitch_ = pitch;
        valid_ = true;
    }

private:
    double yaw_ = 0.0;
    double pitch_ = 0.0;
    bool valid_ = false;
};

// 绕圆心转 三种模式只差角度怎么变

// 二次拟合
static bool fitPoly2(const std::vector<double>& ts, const std::vector<double>& th,
                     double& a, double& b, double& c)
{
    const std::size_t n = ts.size();
    if (n < 5)
    {
        return false;
    }
    const double t0 = ts.front();
    double S0 = 0.0, S1 = 0.0, S2 = 0.0, S3 = 0.0, S4 = 0.0;
    double T0 = 0.0, T1 = 0.0, T2 = 0.0;
    for (std::size_t i = 0; i < n; ++i)
    {
        const double t = ts[i] - t0;
        const double t2 = t * t;
        S0 += 1.0;
        S1 += t;
        S2 += t2;
        S3 += t2 * t;
        S4 += t2 * t2;
        T0 += th[i];
        T1 += t * th[i];
        T2 += t2 * th[i];
    }
    const double det = S0 * (S2 * S4 - S3 * S3)
                     - S1 * (S1 * S4 - S3 * S2)
                     + S2 * (S1 * S3 - S2 * S2);
    if (std::fabs(det) < 1e-12)
    {
        return false;
    }
    a = (T0 * (S2 * S4 - S3 * S3) - S1 * (T1 * S4 - S3 * T2) + S2 * (T1 * S3 - S2 * T2)) / det;
    b = (S0 * (T1 * S4 - S3 * T2) - T0 * (S1 * S4 - S3 * S2) + S2 * (S1 * T2 - S2 * T1)) / det;
    c = (S0 * (S2 * T2 - T1 * S3) - S1 * (S1 * T2 - T1 * S2) + T0 * (S1 * S3 - S2 * S2)) / det;
    return true;
}

// 一次拟合取斜率
static bool fitLine(const std::vector<double>& ts, const std::vector<double>& th, double& slope)
{
    const std::size_t n = ts.size();
    if (n < 3)
    {
        return false;
    }
    const double t0 = ts.front();
    double st = 0.0, sq = 0.0, sth = 0.0, stth = 0.0;
    for (std::size_t i = 0; i < n; ++i)
    {
        const double t = ts[i] - t0;
        st += t;
        sq += t * t;
        sth += th[i];
        stth += t * th[i];
    }
    const double dn = static_cast<double>(n);
    const double den = dn * sq - st * st;
    if (std::fabs(den) < 1e-12)
    {
        return false;
    }
    slope = (dn * stth - st * sth) / den;
    return true;
}

class CircleModelBase : public ITargetModel
{
public:
    void reset() override
    {
        samples_.clear();
        centerValid_ = false;
        cy_ = 0.0;
        cp_ = 0.0;
        radius_ = 0.0;
    }

    bool centered() const override { return centerValid_; }
    double radius() const { return radius_; }

    bool predict(double aheadSec, double& yaw, double& pitch) const override
    {
        if (samples_.empty())
        {
            return false;
        }

        std::vector<double> ts;
        std::vector<double> th;
        buildPhaseSeries(ts, th);

        if (!centerValid_ || ts.size() < 4)
        {
            return predictLinear(aheadSec, yaw, pitch);   // 圆心还没学到 先用直线顶
        }

        double theta = 0.0;
        if (!predictTheta(ts, th, aheadSec, theta))
        {
            return predictLinear(aheadSec, yaw, pitch);
        }

        // 绕实测点转 不用拟合半径
        const MotionSample& last = samples_.back();
        const double dTheta = theta - th.back();
        const double cs = std::cos(dTheta);
        const double sn = std::sin(dTheta);
        const double vx = last.yaw - cy_;
        const double vy = last.pitch - cp_;
        yaw = cy_ + cs * vx - sn * vy;
        pitch = cp_ + sn * vx + cs * vy;
        return true;
    }

protected:
    void onSample(double t, double yaw, double pitch) override
    {
        MotionSample s;
        s.t = t;
        s.yaw = yaw;
        s.pitch = pitch;
        samples_.push_back(s);
        while (samples_.size() > 1 && t - samples_.front().t > kCircleHistorySec)
        {
            samples_.erase(samples_.begin());
        }
        if (samples_.size() > 400)
        {
            samples_.erase(samples_.begin());
        }

        // 每 30 帧重算圆心
        ++refitCounter_;
        if (!centerValid_ || refitCounter_ >= 30)
        {
            refitCounter_ = 0;
            fitCenter();
        }
    }

    // 预测角度
    virtual bool predictTheta(const std::vector<double>& ts,
                              const std::vector<double>& th,
                              double aheadSec, double& theta) const = 0;

    // 取最近样本算角度
    void buildPhaseSeries(std::vector<double>& ts, std::vector<double>& th) const
    {
        ts.clear();
        th.clear();
        if (!centerValid_ || samples_.empty())
        {
            return;
        }
        const double tEnd = samples_.back().t;
        double prev = 0.0;
        bool first = true;
        for (std::size_t i = 0; i < samples_.size(); ++i)
        {
            if (tEnd - samples_[i].t > kPhaseWindowSec)
            {
                continue;
            }
            double a = std::atan2(samples_[i].pitch - cp_, samples_[i].yaw - cy_);
            if (!first)
            {
                // 展开角度
                while (a - prev > CV_PI)  a -= 2.0 * CV_PI;
                while (a - prev < -CV_PI) a += 2.0 * CV_PI;
            }
            ts.push_back(samples_[i].t);
            th.push_back(a);
            prev = a;
            first = false;
        }
    }

    // 还没圆心 先直线顶
    bool predictLinear(double aheadSec, double& yaw, double& pitch) const
    {
        if (samples_.empty())
        {
            return false;
        }
        const MotionSample& last = samples_.back();
        yaw = last.yaw;
        pitch = last.pitch;

        std::vector<MotionSample> recent;
        const double tEnd = last.t;
        for (std::size_t i = 0; i < samples_.size(); ++i)
        {
            if (tEnd - samples_[i].t <= kPhaseWindowSec)
            {
                recent.push_back(samples_[i]);
            }
        }

        double vy = 0.0, vp = 0.0;
        if (!fitAngularVelocity(recent, vy, vp))
        {
            return true;
        }
        const double h = clampValue(aheadSec, 0.0, 0.5);
        yaw = last.yaw + clampValue(vy, -kMaxYawRate, kMaxYawRate) * h;
        pitch = last.pitch + clampValue(vp, -kMaxPitchRate, kMaxPitchRate) * h;
        return true;
    }

    // 圆拟合 再检查真转过没有
    void fitCenter()
    {
        const std::size_t n = samples_.size();
        if (n < 12)
        {
            return;   // 点太少 拟合出来全是噪声
        }

        double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
        double sz = 0.0, sxz = 0.0, syz = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            const double x = samples_[i].yaw;
            const double y = samples_[i].pitch;
            const double z = x * x + y * y;
            sx += x;  sy += y;
            sxx += x * x;  syy += y * y;  sxy += x * y;
            sz += z;  sxz += x * z;  syz += y * z;
        }
        const double dn = static_cast<double>(n);
        const double det = sxx * (syy * dn - sy * sy)
                         - sxy * (sxy * dn - sy * sx)
                         + sx * (sxy * sy - syy * sx);
        if (std::fabs(det) < 1e-9)
        {
            return;
        }

        const double a = (sxz * (syy * dn - sy * sy)
                        - sxy * (syz * dn - sy * sz)
                        + sx * (syz * sy - syy * sz)) / det;
        const double b = (sxx * (syz * dn - sy * sz)
                        - sxz * (sxy * dn - sy * sx)
                        + sx * (sxy * sz - syz * sx)) / det;
        const double c = (sxx * (syy * sz - syz * sy)
                        - sxy * (sxy * sz - syz * sx)
                        + sxz * (sxy * sy - syy * sx)) / det;

        const double cy = a * 0.5;
        const double cp = b * 0.5;
        const double r2 = c + cy * cy + cp * cp;
        if (r2 <= 0.0)
        {
            return;
        }
        const double radius = std::sqrt(r2);
        if (radius < 0.3 || radius > 60.0)
        {
            return;
        }

        double th0 = 0.0, thLast = 0.0, prev = 0.0;
        bool first = true;
        for (std::size_t i = 0; i < n; ++i)
        {
            double t = std::atan2(samples_[i].pitch - cp, samples_[i].yaw - cy);
            if (!first)
            {
                while (t - prev > CV_PI)  t -= 2.0 * CV_PI;
                while (t - prev < -CV_PI) t += 2.0 * CV_PI;
            }
            else
            {
                th0 = t;
                first = false;
            }
            prev = t;
            thLast = t;
        }
        if (std::fabs(thLast - th0) < kMinArcRad)
        {
            return;   // 转得太少 还是用直线吧
        }

        cy_ = cy;
        cp_ = cp;
        radius_ = radius;
        centerValid_ = true;
    }

    double cy_ = 0.0;
    double cp_ = 0.0;
    double radius_ = 0.0;
    bool centerValid_ = false;
    int refitCounter_ = 0;
    std::vector<MotionSample> samples_;
};

// 模型 2 绕着圆心匀速转
class UniformCircleModel : public CircleModelBase
{
public:
    const char* name() const override { return "circle-uniform"; }

protected:
    bool predictTheta(const std::vector<double>& ts, const std::vector<double>& th,
                      double aheadSec, double& theta) const override
    {
        double w = 0.0;
        if (!fitLine(ts, th, w))
        {
            return false;
        }
        w = clampValue(w, -kMaxOmega, kMaxOmega);
        const double h = clampValue(aheadSec, 0.0, 0.5);
        theta = th.back() + w * h;      // 角速度不变 角度就是一条直线
        return true;
    }
};

// 模型 3 角速度按正弦变
class SineCircleModel : public CircleModelBase
{
public:
    const char* name() const override { return "circle-sine"; }

protected:
    bool predictTheta(const std::vector<double>& ts, const std::vector<double>& th,
                      double aheadSec, double& theta) const override
    {
        double a = 0.0, b = 0.0, c = 0.0;
        if (!fitPoly2(ts, th, a, b, c))
        {
            return false;
        }
        (void)a;
        const double tl = ts.back() - ts.front();      // 窗口末尾相对开头
        const double omega = clampValue(b + 2.0 * c * tl, -kMaxOmega, kMaxOmega);
        const double alpha = clampValue(2.0 * c, -kMaxAlpha, kMaxAlpha);
        const double h = clampValue(aheadSec, 0.0, 0.5);
        theta = th.back() + omega * h + 0.5 * alpha * h * h;
        return true;
    }
};

// 三个模型各一个 按枚举挑
static ITargetModel& modelStatic()
{
    static StaticModel instance;
    return instance;
}

static ITargetModel& modelUniform()
{
    static UniformCircleModel instance;
    return instance;
}

static ITargetModel& modelSine()
{
    static SineCircleModel instance;
    return instance;
}

static ITargetModel& targetModel()
{
    switch (kTargetMotion)
    {
        case TargetMotion::CircleUniform: return modelUniform();
        case TargetMotion::CircleSine:    return modelSine();
        case TargetMotion::Static:
        default:                          return modelStatic();
    }
}

static void resetAllModels()
{
    modelStatic().reset();
    modelUniform().reset();
    modelSine().reset();
}

// 算时间用的 秒
static double secondsNow()
{
    static const std::chrono::steady_clock::time_point origin =
        std::chrono::steady_clock::now();
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - origin).count();
}


// 圆环检测
namespace
{
using namespace cv;

// 按当前颜色生成二值图
void buildColorMask(const Mat& hsv, Mat& mask)
{
    mask = Mat::zeros(hsv.size(), CV_8U);

    if (kTargetColor == TargetColor::Red || kTargetColor == TargetColor::Any)
    {
        // 红的 H 跨 0 分两段
        Mat red1, red2;
        inRange(hsv, red_ranges_low[0], red_ranges_low[1], red1);
        inRange(hsv, red_ranges_high[0], red_ranges_high[1], red2);
        bitwise_or(red1, red2, mask);
    }

    if (kTargetColor == TargetColor::Blue || kTargetColor == TargetColor::Any)
    {
        Mat blue;
        inRange(hsv, blue_low, blue_high, blue);
        bitwise_or(mask, blue, mask);
    }
}

// 屏幕上显示颜色模式用
const char* targetColorName()
{
    switch (kTargetColor)
    {
        case TargetColor::Red:  return "red";
        case TargetColor::Blue: return "blue";
        case TargetColor::Any:
        default:                return "any";
    }
}

// 调参窗口 分两个 一个管相机 一个管算法

// 相机窗口
void initCamWindow()
{
    namedWindow("Tune Cam", WINDOW_NORMAL);
    createTrackbar("autoExp 0auto 1manual", "Tune Cam", &g_tune.autoExposure, 1);
    createTrackbar("exposure 100us",        "Tune Cam", &g_tune.exposure,     10000);
    createTrackbar("brightness",            "Tune Cam", &g_tune.brightness,   128);
    createTrackbar("contrast",              "Tune Cam", &g_tune.contrast,     100);
    createTrackbar("saturation",            "Tune Cam", &g_tune.saturation,   100);
    createTrackbar("gamma",                 "Tune Cam", &g_tune.gamma,        400);
    createTrackbar("sharpness",             "Tune Cam", &g_tune.sharpness,    100);
    createTrackbar("backlight",             "Tune Cam", &g_tune.backlight,    2);
    createTrackbar("autoWB 0off 1on",       "Tune Cam", &g_tune.autoWB,       1);
    createTrackbar("wbTemp K",              "Tune Cam", &g_tune.wbTemp,       3700);
}

// 算法窗口
void initAlgoWindow()
{
    // 先把代码里当前的值搬到滑块上
    g_tune.gain = static_cast<int>(kGain * 100.0 + 0.5);
    g_tune.aimU = static_cast<int>(kAimU + 0.5);
    g_tune.aimV = static_cast<int>(kAimV + 0.5);
    g_tune.aimTol = static_cast<int>(kAimTolPx + 0.5);
    g_tune.aimFrames = kAimFrames;
    g_tune.shotInterval = kShotIntervalMs;
    g_tune.leadMs = static_cast<int>(kPredictAheadSec * 1000.0 + 0.5);

    namedWindow("Tune Algo", WINDOW_NORMAL);
    createTrackbar("red H",        "Tune Algo", &g_tune.redH,         179);
    createTrackbar("red S",        "Tune Algo", &g_tune.redS,         255);
    createTrackbar("red V",        "Tune Algo", &g_tune.redV,         255);
    createTrackbar("blue H low",   "Tune Algo", &g_tune.blueH,        179);
    createTrackbar("blue H high",  "Tune Algo", &g_tune.blueH2,       179);
    createTrackbar("blue S",       "Tune Algo", &g_tune.blueS,        255);
    createTrackbar("blue V",       "Tune Algo", &g_tune.blueV,        255);
    createTrackbar("gain x0.01",   "Tune Algo", &g_tune.gain,         200);
    createTrackbar("aim X",        "Tune Algo", &g_tune.aimU,         width);
    createTrackbar("aim Y",        "Tune Algo", &g_tune.aimV,         height);
    createTrackbar("aim tol px",   "Tune Algo", &g_tune.aimTol,       200);
    createTrackbar("aim frames",   "Tune Algo", &g_tune.aimFrames,    30);
    createTrackbar("shot gap ms",  "Tune Algo", &g_tune.shotInterval, 3000);
    createTrackbar("lead ms",      "Tune Algo", &g_tune.leadMs,       500);
}

// 两块板子上写字 显示实际值
void showCamPanel()
{
    static Mat panel(200, 400, CV_8U, Scalar(0));
    panel.setTo(0);

    char buf[160];
    std::snprintf(buf, sizeof(buf), "exp=%d  bright=%d  gamma=%d  wb=%dK",
                  g_tune.exposure, g_tune.brightness - 64,
                  g_tune.gamma + 100, g_tune.wbTemp + 2800);
    putText(panel, buf, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.55, Scalar(255), 1);
    putText(panel, "d close   s save   q quit",
            Point(10, 70), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(180), 1);
    putText(panel, "copy values back into the source",
            Point(10, 105), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(150), 1);

    imshow("Tune Cam", panel);
}

void showAlgoPanel()
{
    static Mat panel(200, 430, CV_8U, Scalar(0));
    panel.setTo(0);

    char buf[160];
    std::snprintf(buf, sizeof(buf), "red H>%d S>%d V>%d",
                  g_tune.redH, g_tune.redS, g_tune.redV);
    putText(panel, buf, Point(10, 26), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 1);

    std::snprintf(buf, sizeof(buf), "blue H %d~%d S>%d V>%d",
                  g_tune.blueH, g_tune.blueH2, g_tune.blueS, g_tune.blueV);
    putText(panel, buf, Point(10, 54), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 1);

    std::snprintf(buf, sizeof(buf), "gain=%.2f  aim=%d,%d  tol=%d",
                  g_tune.gain / 100.0, g_tune.aimU, g_tune.aimV, g_tune.aimTol);
    putText(panel, buf, Point(10, 82), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 1);

    std::snprintf(buf, sizeof(buf), "frames=%d  gap=%dms  lead=%dms",
                  g_tune.aimFrames, g_tune.shotInterval, g_tune.leadMs);
    putText(panel, buf, Point(10, 110), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 1);

    imshow("Tune Algo", panel);
}

// 相机滑块写到相机
void applyTune(VideoCapture& camera)
{
    static TuneState last;
    static bool initialized = false;
    if (!initialized)
    {
        last = g_tune;          // 第一帧只记一下 不要重复设置
        initialized = true;
        return;
    }

    const bool expChanged = (g_tune.autoExposure != last.autoExposure) ||
                            (g_tune.exposure != last.exposure);
    if (expChanged)
    {
        // 1 是手动 3 是自动
        camera.set(CAP_PROP_AUTO_EXPOSURE, g_tune.autoExposure ? 1 : 3);
        camera.set(CAP_PROP_EXPOSURE, g_tune.exposure);
    }
    if (g_tune.brightness != last.brightness)
        camera.set(CAP_PROP_BRIGHTNESS, g_tune.brightness - 64);
    if (g_tune.contrast != last.contrast)
        camera.set(CAP_PROP_CONTRAST, g_tune.contrast);
    if (g_tune.saturation != last.saturation)
        camera.set(CAP_PROP_SATURATION, g_tune.saturation);
    if (g_tune.gamma != last.gamma)
        camera.set(CAP_PROP_GAMMA, g_tune.gamma + 100);
    if (g_tune.sharpness != last.sharpness)
        camera.set(CAP_PROP_SHARPNESS, g_tune.sharpness);
    if (g_tune.backlight != last.backlight)
        camera.set(CAP_PROP_BACKLIGHT, g_tune.backlight);
    if (g_tune.autoWB != last.autoWB || g_tune.wbTemp != last.wbTemp)
    {
        camera.set(CAP_PROP_AUTO_WB, g_tune.autoWB ? 1 : 0);
        camera.set(CAP_PROP_WB_TEMPERATURE, g_tune.wbTemp + 2800);
    }

    last = g_tune;
}

// 算法滑块写到算法
void applyAlgo()
{
    static TuneState last;
    static bool initialized = false;
    if (!initialized)
    {
        last = g_tune;
        initialized = true;
        return;
    }

    // 改全局阈值
    if (g_tune.redH != last.redH || g_tune.redS != last.redS || g_tune.redV != last.redV)
    {
        red_ranges_low[0]  = Scalar(g_tune.redH, g_tune.redS, g_tune.redV);
        red_ranges_high[0] = Scalar(168,         g_tune.redS, g_tune.redV);
    }
    if (g_tune.blueH != last.blueH || g_tune.blueH2 != last.blueH2 ||
        g_tune.blueS != last.blueS || g_tune.blueV != last.blueV)
    {
        blue_low  = Scalar(g_tune.blueH,  g_tune.blueS, g_tune.blueV);
        blue_high = Scalar(g_tune.blueH2, 255,          255);
    }

    // 控制参数
    if (g_tune.gain != last.gain)
        kGain = g_tune.gain / 100.0;
    if (g_tune.aimU != last.aimU)
        kAimU = g_tune.aimU;
    if (g_tune.aimV != last.aimV)
        kAimV = g_tune.aimV;
    if (g_tune.aimTol != last.aimTol)
        kAimTolPx = g_tune.aimTol;
    if (g_tune.aimFrames != last.aimFrames)
        kAimFrames = (g_tune.aimFrames < 1) ? 1 : g_tune.aimFrames;   // 至少 1 帧 不然一直开火
    if (g_tune.shotInterval != last.shotInterval)
        kShotIntervalMs = g_tune.shotInterval;
    if (g_tune.leadMs != last.leadMs)
        kPredictAheadSec = g_tune.leadMs / 1000.0;

    last = g_tune;
}

std::vector<Rect> detectColorRing(const Mat& frame, Mat& maskOut)
{
    std::vector<Rect> targetBoxs;

    Mat hsv, mask0;
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;

    // BGR to HSV
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // 按当前颜色取二值图
    buildColorMask(hsv, mask0);

    // 闭开去噪
    morphologyEx(mask0, mask0, MORPH_CLOSE, kernel);
    morphologyEx(mask0, mask0, MORPH_OPEN, kernel);
    maskOut = mask0;   // 这里不开窗口 二值图拿回主循环显示

    // 两层轮廓
    findContours(mask0.clone(), contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    for (int cnt = 0; cnt < static_cast<int>(contours.size()); ++cnt)
    {
        // 太小的不要
        if (contourArea(contours[cnt]) < 80)
        {
            continue;
        }
        // 只要最外层的
        if (hierarchy[cnt][3] != -1)
        {
            continue;
        }
        // 没内孔的不是环
        if (hierarchy[cnt][2] == -1)
        {
            continue;
        }

        // 取最大子轮廓当内孔
        int next = hierarchy[cnt][2];
        double inAera = 0;
        int in_cnt = -1;
        while (next != -1)
        {
            if (contourArea(contours[next]) < 90)
            {
                next = hierarchy[next][0];
                continue;
            }
            if (contourArea(contours[next]) > inAera)
            {
                inAera = contourArea(contours[next]);
                in_cnt = next;
            }
            next = hierarchy[next][0];
        }
        if (inAera < 80 || in_cnt < 0)
        {
            continue;
        }

        // 外圈圆度
        double experi = arcLength(contours[cnt], true);
        double exArea = contourArea(contours[cnt]);
        if (experi <= 1e-6)
        {
            continue;
        }
        double circularity = 4.0 * CV_PI * exArea / (experi * experi);
        if (circularity < 0.85 || circularity > 1.18)
        {
            continue;
        }

        // 内孔也卡一遍
        double inPeri = arcLength(contours[in_cnt], true);
        if (inPeri <= 1e-6)
        {
            continue;
        }
        double inCircularity = 4.0 * CV_PI * inAera / (inPeri * inPeri);
        if (inCircularity < 0.85 || inCircularity > 1.18)
        {
            continue;
        }

        // 内外形状相似
        if (matchShapes(contours[cnt], contours[in_cnt], CONTOURS_MATCH_I1, 0) > 0.2)
        {
            continue;
        }

        // 同心
        Point2f oc, ic;
        float orad = 0, irad = 0;
        minEnclosingCircle(contours[cnt], oc, orad);
        minEnclosingCircle(contours[in_cnt], ic, irad);
        if (orad <= 1e-6)
        {
            continue;
        }
        double dx = oc.x - ic.x, dy = oc.y - ic.y;
        double offset = std::sqrt(dx * dx + dy * dy) / orad;
        if (offset > 0.2)
        {
            continue;
        }

        // 粗细
        double rRatio = static_cast<double>(irad) / orad;
        if (rRatio < 0.6 || rRatio > 0.9)
        {
            continue;
        }

        Rect Box = boundingRect(contours[cnt]);

        // 长宽比
        double rate = static_cast<double>(Box.width) / Box.height;
        if (rate > 1.2 || rate < 0.8)
        {
            continue;
        }

        targetBoxs.push_back(Box);
    }

    return targetBoxs;
}

// 多个环挑一个
BladeTarget pickTarget(const std::vector<Rect>& boxes, const Point2f& reference)
{
    BladeTarget best;
    double bestDist = 0.0;
    for (std::size_t i = 0; i < boxes.size(); ++i)
    {
        const Point2f c(boxes[i].x + boxes[i].width * 0.5F,
                        boxes[i].y + boxes[i].height * 0.5F);
        const double d = std::hypot(c.x - reference.x, c.y - reference.y);
        if (!best.valid || d < bestDist)
        {
            best.center = c;
            best.radius = (boxes[i].width > boxes[i].height ? boxes[i].width : boxes[i].height) * 0.5F;
            best.score = static_cast<double>(boxes[i].area());
            best.valid = true;
            bestDist = d;
        }
    }
    return best;
}

} // namespace

/*---------算法函数编写区域结束----------*/



/*---------禁止修改：主函数入口和相机初始化----------*/
int main()
{
    // ======== 1. 调参数：画面清楚 =========
    cv::VideoCapture camera(cameraIndex, cv::CAP_V4L2);
    if (!camera.isOpened())
    {
        std::cerr << "相机打开失败，请检查 cameraIndex。\n";
        return 1;
    }
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    camera.set(cv::CAP_PROP_FPS, fps);
    /*---------禁止修改区域结束----------*/



    /*---------禁止修改：应用全局相机参数----------*/
    // 调参只改上面的全局变量，这里负责把参数写入相机。
    if (!camera.set(cv::CAP_PROP_BUFFERSIZE, cameraBufferSize))
        std::cerr << "相机不支持设置缓冲帧数，需核对实际图像延迟。\n";
    camera.set(cv::CAP_PROP_BRIGHTNESS, cameraBrightness);
    camera.set(cv::CAP_PROP_CONTRAST, cameraContrast);
    camera.set(cv::CAP_PROP_SATURATION, cameraSaturation);
    camera.set(cv::CAP_PROP_HUE, cameraHue);
    // 先设置曝光模式，再设置曝光值，避免在自动模式下写入手动参数。
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, cameraAutoExposure);
    camera.set(cv::CAP_PROP_EXPOSURE, cameraExposure);
    camera.set(cv::CAP_PROP_SHARPNESS, cameraSharpness);
    camera.set(cv::CAP_PROP_GAMMA, cameraGamma);
    camera.set(cv::CAP_PROP_BACKLIGHT, cameraBacklight);
    // 先选择白平衡模式；开启自动模式时，驱动可能忽略下面的色温值。
    camera.set(cv::CAP_PROP_AUTO_WB, cameraAutoWhiteBalance);
    camera.set(cv::CAP_PROP_WB_TEMPERATURE, cameraWhiteBalance);
    /*---------禁止修改区域结束----------*/



    /*---------禁止修改：串口初始化、取图和反馈读取----------*/
    // 使用上方的全局串口参数。
    if (useSerial && !uart.Open(serialPort, serialBaudRate))
    {
        std::cerr << "串口打开失败，本次只显示图像。\n";
        useSerial = false;
    }

    cv::namedWindow("Camera", cv::WINDOW_NORMAL);
    cv::Mat image;
    int exitCode = 0;
    std::thread sender;
    std::thread receiver;
    if (useSerial)
    {
        receiver = std::thread(receiveLoop);
        sender = std::thread(sendLoop);
    }

    while (running.load())
    {
        // ===== 2. 取图：image 就是这一帧的 BGR 图像 =====
        camera >> image;
        if (image.empty())
        {
            std::cerr << "没有读到图像。\n";
            exitCode = 1;
            break;
        }
        const auto frameReceivedAt = std::chrono::steady_clock::now();

        // ===== 3. 复制接收结果：只在复制时加锁，算法放在锁外 =====
        TRoMaC::GimbalFeedback feedback{};
        bool feedbackValid;
        {
            std::lock_guard<std::mutex> lock(receiveMutex);
            feedback = receiveData;
            // 超过 250 ms 没更新，就不再把旧角度当成有效反馈。
            feedbackValid = hasFeedback &&
                std::chrono::steady_clock::now() - receivedAt < std::chrono::milliseconds(250);
        }
        // 只在调试时打印 确认串口通没通
        if (feedbackValid && kShowDebug)
            std::cout << "yaw=" << feedback.yaw_angle
                      << " pitch=" << feedback.pitch_angle << '\n';
        // feedbackValid 为 true 才能使用 feedback，角度单位为度。

        // 每帧先清零，避免丢失目标后继续发送上一帧的角度增量/推球标志。
        sendData = TRoMaC::VisionData{};
        /*---------禁止修改区域结束----------*/

        /*---------需要编写：图像识别、控制计算和发送数据----------*/
        // 输入：image 是当前图像；feedback 是角度反馈，仅 feedbackValid 为 true 时可用。
        // 输出：修改 sendData 的字段；不用自己调用串口发送函数。

        // 下面先设默认值，再在本区域内根据识别和计算结果覆盖它们。
        sendData.Gimbal_Ctrl = 1;          // 2：增量控制；1：绝对角度控制。
        sendData.pitch_delta_angle = 0.0F; // 增量模式：实测角度 + 此增量，单位度。
        sendData.yaw_delta_angle = 0.0F;
        sendData.pitch_angle = 20.0F;       // 绝对模式：要到达的角度，单位度。
        sendData.yaw_angle = 0.0F;
        sendData.fric = 1;
        sendData.Auto_Push = 0;
        sendData.Push_flag = 0;

        // 找环
        cv::Mat mask;
        std::vector<cv::Rect> rings = detectColorRing(image, mask);
        // 参考点
        const cv::Point2f reference = g_hasFilter
            ? g_filtered
            : cv::Point2f(static_cast<float>(kAimU), static_cast<float>(kAimV));
        BladeTarget target = pickTarget(rings, reference);

        if (target.valid)
        {
            if (!g_hasFilter || g_lostFrames > kHoldFrames)
            {
                g_filtered = target.center;   // 刚重新找到就直接贴上去
                g_hasFilter = true;
            }
            else
            {
                // 低通
                g_filtered.x = 0.7F * g_filtered.x + 0.3F * target.center.x;
                g_filtered.y = 0.7F * g_filtered.y + 0.3F * target.center.y;
            }
            g_lastTarget = target;
            g_lastTarget.center = g_filtered;
            g_lostFrames = 0;
        }
        else if (g_lostFrames < 100000)
        {
            ++g_lostFrames;
        }
        const bool tracking = g_hasFilter && g_lostFrames <= kHoldFrames;

        // 换算成绝对角
        double du = 0.0;
        double dv = 0.0;
        if (tracking)
        {
            du = g_lastTarget.center.x - kAimU;
            dv = g_lastTarget.center.y - kAimV;
        }
        const double yawError = pixelToAngle(du, kFocalPx) * kYawSign;
        const double pitchError = -pixelToAngle(dv, kFocalPx) * kPitchSign;

        const double obsYawAbs = feedback.yaw_angle + yawError;
        const double obsPitchAbs = feedback.pitch_angle + pitchError;

        // 喂模型拿预测
        ITargetModel& model = targetModel();
        double aimYawAbs = obsYawAbs;
        double aimPitchAbs = obsPitchAbs;
        if (feedbackValid)
        {
            // 传 target.valid 不是 tracking
            model.update(secondsNow(), obsYawAbs, obsPitchAbs, target.valid);
            if (tracking)
            {
                if (!model.predict(kPredictAheadSec, aimYawAbs, aimPitchAbs))
                {
                    aimYawAbs = obsYawAbs;       // 模型算不出来就打当前位置
                    aimPitchAbs = obsPitchAbs;
                }
            }
        }

        // 给云台下指令
        if (tracking)
        {
            if (feedbackValid)
            {
                sendData.Gimbal_Ctrl = 1;
                sendData.yaw_angle = static_cast<float>(
                    feedback.yaw_angle + kGain * (aimYawAbs - feedback.yaw_angle) + kYawBias);
                sendData.pitch_angle = static_cast<float>(
                    feedback.pitch_angle + kGain * (aimPitchAbs - feedback.pitch_angle) + kPitchBias);
            }
            else
            {
                // 没反馈发增量
                sendData.Gimbal_Ctrl = 2;
                sendData.yaw_delta_angle = static_cast<float>(
                    clampValue(kGain * yawError, -kMaxStep, kMaxStep));
                sendData.pitch_delta_angle = static_cast<float>(
                    clampValue(kGain * pitchError, -kMaxStep, kMaxStep));
            }
        }
        else if (feedbackValid)
        {
            // 没目标就停在原地
            sendData.Gimbal_Ctrl = 1;
            sendData.yaw_angle = feedback.yaw_angle;
            sendData.pitch_angle = feedback.pitch_angle;
        }

        // 对准了就开火
        const auto now = std::chrono::steady_clock::now();

        // 门限 运动目标要放宽
        double tolDeg = pixelToAngle(kAimTolPx, kFocalPx);
        if (kTargetMotion != TargetMotion::Static && kPredictAheadSec > 0.0 &&
            kAimTolDegMoving > tolDeg)
        {
            tolDeg = kAimTolDegMoving;   // 只有真在打提前量才放宽
        }
        double fireYaw = yawError;
        double firePitch = pitchError;
        if (feedbackValid)
        {
            fireYaw = aimYawAbs - feedback.yaw_angle;
            firePitch = aimPitchAbs - feedback.pitch_angle;
        }
        const bool aimed = tracking &&
            std::fabs(fireYaw) <= tolDeg && std::fabs(firePitch) <= tolDeg;
        g_aimedFrames = aimed ? (g_aimedFrames + 1) : 0;

        const auto sinceLastShot = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - g_lastShotAt).count();
        if (kEnableFire && g_shotsFired < kMaxShots &&
            g_aimedFrames >= kAimFrames && sinceLastShot >= kShotIntervalMs)
        {
            // 开火
            sendData.fric = 1;
            sendData.Auto_Push = 1;
            sendData.Push_flag = 1;
            ++g_shotsFired;
            g_aimedFrames = 0;
            g_lastShotAt = now;
            std::cout << "[FIRE] 第 " << g_shotsFired << " 发"
                      << " du=" << du << " dv=" << dv << '\n';
        }

        // 画图
        cv::circle(image, cv::Point(static_cast<int>(kAimU), static_cast<int>(kAimV)),
                   6, cv::Scalar(0, 255, 0), 2);
        if (tracking)
        {
            const cv::Point center(static_cast<int>(g_lastTarget.center.x),
                                   static_cast<int>(g_lastTarget.center.y));
            const cv::Scalar color = aimed ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            cv::circle(image, center, static_cast<int>(g_lastTarget.radius), color, 2);
            cv::line(image, center,
                     cv::Point(static_cast<int>(kAimU), static_cast<int>(kAimV)),
                     cv::Scalar(0, 255, 255), 1);
            cv::putText(image,
                        "du=" + std::to_string(static_cast<int>(du)) +
                            " dv=" + std::to_string(static_cast<int>(dv)) +
                            " yaw=" + std::to_string(static_cast<int>(yawError)) +
                            " pitch=" + std::to_string(static_cast<int>(pitchError)),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 255), 2);
        }
        else
        {
            cv::putText(image, "NO TARGET", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
        }
        cv::putText(image, "shots=" + std::to_string(g_shotsFired) +
                              (kEnableFire ? " FIRE ON" : " FIRE OFF"),
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    kEnableFire ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
        cv::putText(image, std::string("model: ") + model.name() +
                              (model.centered() ? "" : "(no-circle)") +
                              "  lead: " + std::to_string(kPredictAheadSec) +
                              "s  color: " + targetColorName(),
                    cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 0), 2);

        if (kShowDebug)
        {
            // 瞄准点和接受圈
            const cv::Point aimPoint(static_cast<int>(kAimU), static_cast<int>(kAimV));
            cv::drawMarker(image, aimPoint, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 1);
            cv::circle(image, aimPoint, static_cast<int>(kAimTolPx),
                       cv::Scalar(0, 255, 0), 1);
            cv::putText(image, "AIM", aimPoint + cv::Point(14, -12),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);

            // 预测点 角度反算回像素
            if (tracking && feedbackValid && kPredictAheadSec > 0.0 &&
                kTargetMotion != TargetMotion::Static)
            {
                const double dYawDeg   = aimYawAbs   - feedback.yaw_angle;
                const double dPitchDeg = aimPitchAbs - feedback.pitch_angle;
                const double duPred = kFocalPx * std::tan(dYawDeg   * CV_PI / 180.0) * kYawSign;
                const double dvPred = -kFocalPx * std::tan(dPitchDeg * CV_PI / 180.0) * kPitchSign;
                const cv::Point pred(static_cast<int>(kAimU + duPred),
                                     static_cast<int>(kAimV + dvPred));

                // 提前量
                cv::line(image,
                         cv::Point(static_cast<int>(g_lastTarget.center.x),
                                   static_cast<int>(g_lastTarget.center.y)),
                         pred, cv::Scalar(255, 255, 0), 1);
                cv::drawMarker(image, pred, cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 16, 1);
                cv::putText(image, "PRED", pred + cv::Point(10, -10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 0, 255), 1);
            }

            // 图例
            const int lx = image.cols - 216;
            const int ly = image.rows - 96;
            cv::rectangle(image, cv::Rect(lx - 10, ly - 10, 216, 96),
                          cv::Scalar(0, 0, 0), cv::FILLED);
            cv::circle(image, cv::Point(lx + 4, ly + 6), 4, cv::Scalar(0, 255, 0), -1);
            cv::putText(image, "AIM   impact point",
                        cv::Point(lx + 16, ly + 11), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                        cv::Scalar(0, 255, 0), 1);
            cv::circle(image, cv::Point(lx + 4, ly + 30), 4, cv::Scalar(0, 0, 255), -1);
            cv::putText(image, "TGT   detected ring",
                        cv::Point(lx + 16, ly + 35), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                        cv::Scalar(0, 0, 255), 1);
            cv::circle(image, cv::Point(lx + 4, ly + 54), 4, cv::Scalar(255, 0, 255), -1);
            cv::putText(image, "PRED  predicted point",
                        cv::Point(lx + 16, ly + 59), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                        cv::Scalar(255, 0, 255), 1);
            cv::putText(image, "yellow=error  cyan=lead",
                        cv::Point(lx + 4, ly + 80), cv::FONT_HERSHEY_SIMPLEX, 0.38,
                        cv::Scalar(200, 200, 200), 1);

            // 帧率 框数
            static std::chrono::steady_clock::time_point prevFrameAt = frameReceivedAt;
            const double frameDt =
                std::chrono::duration<double>(frameReceivedAt - prevFrameAt).count();
            prevFrameAt = frameReceivedAt;
            static double fps = 0.0;
            if (frameDt > 1e-6)
            {
                fps = (fps <= 0.0) ? (1.0 / frameDt) : (0.9 * fps + 0.1 / frameDt);
            }
            cv::putText(image,
                        "fps=" + std::to_string(static_cast<int>(fps + 0.5)) +
                            "  rings=" + std::to_string(static_cast<int>(rings.size())) +
                            "  off=" + std::to_string(static_cast<int>(du)) + "," +
                            std::to_string(static_cast<int>(dv)),
                        cv::Point(10, image.rows - 14), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 255, 255), 1);

            // 两个窗口的滑块都只建一次
            if (!g_camCreated)
            {
                initCamWindow();
                g_camCreated = true;
            }
            if (!g_algoCreated)
            {
                initAlgoWindow();
                g_algoCreated = true;
            }
            applyTune(camera);
            applyAlgo();
            showCamPanel();
            showAlgoPanel();
            cv::imshow("Mask", mask);
        }

        cv::imshow("Camera", image);
        int key = cv::waitKey(1); // 也负责刷新窗口，不能删掉。
        if (key == 'q' || key == 'Q' || key == 27)
        {
            break;
        }
        if (key == 's' || key == 'S')       // 按 s 存图 回去慢慢看
        {
            const std::string index = std::to_string(g_frameIndex);
            cv::imwrite("cap_" + index + ".png", image);
            cv::imwrite("cap_" + index + "_mask.png", mask);
            std::cout << "已保存 cap_" << index << ".png\n";
            ++g_frameIndex;
        }
        if (key == 'd' || key == 'D')       // 按 d 开关调试窗口
        {
            kShowDebug = !kShowDebug;
            if (!kShowDebug)
            {
                cv::destroyWindow("Mask");
                if (g_camCreated)
                {
                    cv::destroyWindow("Tune Cam");   // 滑块跟着窗口一起没了
                    g_camCreated = false;
                }
                if (g_algoCreated)
                {
                    cv::destroyWindow("Tune Algo");
                    g_algoCreated = false;
                }
            }
        }
        if (key == 'f' || key == 'F')       // 按 f 开关开火
        {
            kEnableFire = !kEnableFire;
            std::cout << "开火开关: " << (kEnableFire ? "ON" : "OFF") << '\n';
        }
        if (key == 'c' || key == 'C')       // 按 c 循环切 红 蓝 都认
        {
            kTargetColor = (kTargetColor == TargetColor::Red)  ? TargetColor::Blue
                         : (kTargetColor == TargetColor::Blue) ? TargetColor::Any
                                                               : TargetColor::Red;
            std::cout << "目标颜色: " << targetColorName() << '\n';
        }
        if (key >= '1' && key <= '3')       // 1 不动 2 匀速圆周 3 正弦变速
        {
            kTargetMotion = (key == '1') ? TargetMotion::Static
                          : (key == '2') ? TargetMotion::CircleUniform
                                         : TargetMotion::CircleSine;
            resetAllModels();
            std::cout << "运动模型: " << targetModel().name() << '\n';
        }
        /*---------图像识别和控制编写区域结束----------*/

        /*---------禁止修改：发布发送数据、退出清理----------*/
        // ===== 4. 发布完整结果，由独立发送线程负责串口发送 =====
        // 主线程只改 sendData；加锁复制后，发送线程才能看到这一整份结果。
        if (useSerial)
        {
            std::lock_guard<std::mutex> lock(sendMutex);
            latestSendData = sendData;
            sendUpdatedAt = frameReceivedAt;
            sendReady = true;
        }
    }

    // 先通知两个通信线程退出并等它们结束，再关闭串口。
    running.store(false);
    if (sender.joinable())
        sender.join();
    if (receiver.joinable())
        receiver.join();
    if (sendFailed.load())
        exitCode = 1;
    uart.Close();
    camera.release();
    cv::destroyAllWindows();
    return exitCode;
}
/*---------禁止修改区域结束----------*/
