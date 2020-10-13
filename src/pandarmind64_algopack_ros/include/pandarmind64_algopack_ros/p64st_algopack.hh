#ifndef _____P64ST_ALGOPACK_HH___
#define _____P64ST_ALGOPACK_HH___

#include <cstdint>
#include <array>
#include <tuple>
#include <functional>
#include <cassert>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <QJsonObject>
#include <QJsonArray>

namespace hesai {

using Clock = std::chrono::system_clock;
using Time = Clock::time_point;
using Period = Clock::period;
using Duration = Clock::duration;

#pragma pack(push, 1)
struct xyz_t {
    float x, y, z;
    xyz_t(float x = 0, float y = 0, float z = 0)
        : x(x)
        , y(y)
        , z(z)
    {}
    xyz_t(Eigen::Vector3f&& vec3)
        : x(vec3[0])
        , y(vec3[1])
        , z(vec3[2])
    {}
    inline operator Eigen::Vector3f() const { return Eigen::Vector3f(x, y, z); }
    inline xyz_t operator+(const Eigen::Vector3f& vec3) const { return { x + vec3[0], y + vec3[1], z + vec3[2] }; }
    inline xyz_t operator-(const Eigen::Vector3f& vec3) const { return { x - vec3[0], y - vec3[1], z - vec3[2] }; }
    inline bool operator==(const xyz_t& other) const { return this->x == other.x && this->y == other.y && this->z == other.z; }
};
// static_assert(sizeof(xyz_t) == 3 * sizeof(float));
struct obj_t {
    xyz_t center;
    xyz_t front, front1, front2;
    xyz_t up1, up11, up12;
    xyz_t up2, up21, up22;
    xyz_t up3, up31, up32;
    xyz_t up4, up41, up42;
    xyz_t down1, down11, down12;
    xyz_t down2, down21, down22;
    xyz_t down3, down31, down32;
    xyz_t down4, down41, down42;
    obj_t() {
        std::memset(this, 0, sizeof(obj_t));
    }
    obj_t(std::array<xyz_t, 28>&& arr) {
        // static_assert(sizeof(std::array<xyz_t, 28>) == sizeof(std::array<xyz_t, 28>));
        *(std::array<xyz_t, 28>*)this = arr;
    }
};
struct sub_anno_t {
    uint8_t type = -1;
    Eigen::Vector3f front{ 0,0,0 }, left{ 0,0,0 }, up{ 0,0,0 };
    float w = 0, h = 0, l = 0;
};
// static_assert(sizeof(obj_t) == sizeof(std::array<xyz_t, 28>));
struct xyzw_t : xyz_t {
    float w = 0;
    xyzw_t(float x = 0, float y = 0, float z = 0, float w = 1)
        : xyz_t(x, y, z)
        , w(1)
    {}
    xyzw_t(Eigen::Quaternionf&& quat)
        : w(quat.w())
        , xyz_t(quat.x(), quat.y(), quat.z())
    {}
    inline operator Eigen::Quaternionf() const {
        return Eigen::Quaternionf(w, x, y, z);
    }
};
// static_assert(sizeof(xyzw_t) == 4 * sizeof(float));
struct STAlgopack {
public:
    struct Header {
        uint8_t magic_num[2];
        uint8_t minor_version;
        uint8_t major_version;
    };
    static constexpr uint8_t magic_num[2] = { 0x32, 0x23 };
public:
    inline static bool isValid(const uint8_t* pkt, uint32_t pkt_sz) {
        return
            pkt_sz > sizeof(STAlgopack::Header)
            && ((const Header*)pkt)->magic_num[0] == magic_num[0]
            && ((const Header*)pkt)->magic_num[1] == magic_num[1]
        ;
    }
};
// static_assert(sizeof(STAlgopack::Header) == 4);
struct STAlgopackV2_0 : STAlgopack {
public:
    struct Header : STAlgopack::Header {
        uint32_t timestamp_begin;
        uint8_t utc_begin[6];
        uint32_t timestamp_end;
        uint8_t utc_end[6];
        uint8_t return_code;
        uint8_t info_type;
        uint32_t data_length;
        uint8_t packet_total_count;
        uint8_t packet_count;
        float detection_center_x;
        float detection_center_y;
        float translation_matrix[3][4];
    };
    static constexpr uint32_t initiation_codon = 0xffffffff;
    struct ObjectMeta {
        uint8_t type;
        uint8_t count;
        uint8_t size;               // object struct size
    };
    struct Object {
        uint32_t id;
        uint64_t timestamp;
        xyzw_t pos_quat;
        xyz_t lwh;
        xyz_t center;
        xyz_t relative_velocity;
        xyz_t absolute_velocity;
        xyz_t acceleration;
        float location_cov[9];       //!< position uncertainty matrix
        float velocity_cov[9];       //!< velocity uncertainty matrix
        float acceleration_cov[9];   //!< acceleration uncertainty matrix
        uint8_t tracking_confidence;    //!< confidence of detection 0(low)~5(height)
        uint8_t is_detection;           //!< whether it is really the target of
        uint8_t reserved[2];
        inline Time parseTime() const {
            using namespace std::chrono_literals;
            return Time() + std::chrono::duration_cast<Duration>(timestamp * 1ns);
        }
        inline std::tuple<obj_t, sub_anno_t> objectCuboid(uint8_t type) const {
            auto R = pos_quat.operator Eigen::Quaternionf().toRotationMatrix();
            Eigen::Vector3f front = R * Eigen::Vector3f{ 1, 0, 0 };
            Eigen::Vector3f left = R * Eigen::Vector3f{ 0, 1, 0 };
            Eigen::Vector3f up = R * Eigen::Vector3f{ 0, 0, 1 };
            Eigen::Vector3f arrow_front = R * Eigen::Vector3f{ absolute_velocity.x, absolute_velocity.y, absolute_velocity.z };
            Eigen::Vector3f arrow_left = up.cross(arrow_front) / 4;
            auto l = lwh.x, w = lwh.y, h = lwh.z;
            return std::make_tuple(
                obj_t({
                    center,
                    center + l * front, center + l * 3 / 4 * front + w / 4 * left, center + l * 3 / 4 * front - w / 4 * left, 
                    center + l / 2 * front - w / 2 * left + h / 2 * up, center + l / 2 * front - w / 4 * left + h / 2 * up, center + l / 4 * front - w / 2 * left + h / 2 * up,
                    center + l / 2 * front + w / 2 * left + h / 2 * up, center + l / 2 * front + w / 4 * left + h / 2 * up, center + l / 4 * front + w / 2 * left + h / 2 * up,
                    center - l / 2 * front + w / 2 * left + h / 2 * up, center - l / 2 * front + w / 4 * left + h / 2 * up, center - l / 4 * front + w / 2 * left + h / 2 * up,
                    center - l / 2 * front - w / 2 * left + h / 2 * up, center - l / 2 * front - w / 4 * left + h / 2 * up, center - l / 4 * front - w / 2 * left + h / 2 * up,
                    center + l / 2 * front - w / 2 * left - h / 2 * up, center + l / 2 * front - w / 4 * left - h / 2 * up, center + l / 4 * front - w / 2 * left - h / 2 * up,
                    center + l / 2 * front + w / 2 * left - h / 2 * up, center + l / 2 * front + w / 4 * left - h / 2 * up, center + l / 4 * front + w / 2 * left - h / 2 * up,
                    center - l / 2 * front + w / 2 * left - h / 2 * up, center - l / 2 * front + w / 4 * left - h / 2 * up, center - l / 4 * front + w / 2 * left - h / 2 * up,
                    center - l / 2 * front - w / 2 * left - h / 2 * up, center - l / 2 * front - w / 4 * left - h / 2 * up, center - l / 4 * front - w / 2 * left - h / 2 * up,
                }),
                sub_anno_t({
                    type, front, left, up,
                    w, h, l
                })
            );
        }
    };
    static constexpr uint32_t termination_codon = 0xeeeeeeee;
    struct Tail {
        uint32_t crc_check;
    };
public:
     inline static QString objectType(uint8_t type) {
        switch (type)
        {
        case 0:
            return "Car";
        case 1:
            return "Trucks";
        case 2:
            return "Cyclist";
        case 3:
            return "Pedestrian";
        default:
            return "";
        }
    }
    inline static bool isValid(const uint8_t* pkt, uint32_t pkt_sz) {
        return 
            pkt_sz > sizeof(Header) + sizeof(Tail) + sizeof(uint32_t) * 2
            && *(const uint32_t*)(pkt + sizeof(Header)) == initiation_codon
            && *(const uint32_t*)(pkt + pkt_sz - (sizeof(Tail) + sizeof(uint32_t))) == termination_codon
            && ((const Header*)pkt)->data_length == pkt_sz - (sizeof(Header) + sizeof(Tail) + sizeof(uint32_t) * 2)
        ;
    }
    inline static void iterate(const uint8_t* pkt, uint32_t pkt_sz, std::function<void(uint8_t, const Object&)> callback) {
        auto begin = pkt + sizeof(Header) + sizeof(uint32_t);
        for (
            auto ptr = begin;
            ptr - begin < ((const Header*)pkt)->data_length - sizeof(ObjectMeta);

        ) {
            auto object_meta = (const ObjectMeta*)ptr;
            assert(object_meta->size * 4 == sizeof(Object));
            ptr += sizeof(ObjectMeta);
            for (auto i = 0; i < object_meta->count; i++) {
                auto object = (const Object*)(ptr + i * 4 * object_meta->size);
                callback(
                    object_meta->type,
                    *object
                );
            }
            ptr += object_meta->count * 4 * object_meta->size;
        }
    }
};
// static_assert(sizeof(STAlgopackV2_0::Header) == 88);
// static_assert(sizeof(STAlgopackV2_0::ObjectMeta) == 3);
// static_assert(sizeof(STAlgopackV2_0::Object) == 200);
// static_assert(sizeof(STAlgopackV2_0::Tail) == 4);
struct STAlgopackV2_1 : STAlgopackV2_0 {
public:
    struct Header : STAlgopackV2_0::Header {
        int32_t point_count;
        int32_t algo_cost_ms;
        int32_t post_proc_ms;
        int32_t send_proc_ms;
        int32_t total_ms;
    };
public:
    inline static bool isValid(const uint8_t* pkt, uint32_t pkt_sz) {
        return
            pkt_sz > sizeof(Header) + sizeof(Tail) + sizeof(uint32_t) * 2
            && *(const uint32_t*)(pkt + sizeof(Header)) == initiation_codon
            && *(const uint32_t*)(pkt + pkt_sz - (sizeof(Tail) + sizeof(uint32_t))) == termination_codon
            && ((const Header*)pkt)->data_length == pkt_sz - (sizeof(Header) + sizeof(Tail) + sizeof(uint32_t) * 2)
        ;
    }
    inline static void iterate(const uint8_t* pkt, uint32_t pkt_sz, std::function<void(uint8_t, const Object&)> callback) {
        auto begin = pkt + sizeof(Header) + sizeof(uint32_t);
        for (
            auto ptr = begin;
            ptr - begin < ((const Header*)pkt)->data_length - sizeof(ObjectMeta);

            ) {
            auto object_meta = (const ObjectMeta*)ptr;
            assert(object_meta->size * 4 == sizeof(Object));
            ptr += sizeof(ObjectMeta);
            for (auto i = 0; i < object_meta->count; i++) {
                auto object = (const Object*)(ptr + i * 4 * object_meta->size);
                callback(
                    object_meta->type,
                    *object
                );
            }
            ptr += object_meta->count * 4 * object_meta->size;
        }
    }
    inline static QJsonObject parseAlgoProcessProfile(const uint8_t* pkt, uint32_t pkt_sz) {
        QJsonObject algo_process_profile;
        auto header = (const Header*)pkt;
        algo_process_profile["packet_count"] = header->packet_total_count;
        algo_process_profile["packet_num"] = header->packet_count;
        algo_process_profile["point_count"] = header->point_count;
        algo_process_profile["algo_cost_ms"] = header->algo_cost_ms;
        algo_process_profile["post_proc_ms"] = header->post_proc_ms;
        algo_process_profile["send_proc_ms"] = header->send_proc_ms;
        algo_process_profile["total_ms"] = header->total_ms;
        QJsonArray obj_counts{ 0,0,0,0,0 };
        auto begin = pkt + sizeof(Header) + sizeof(uint32_t);
        for (
            auto ptr = begin;
            ptr - begin < ((const Header*)pkt)->data_length - sizeof(ObjectMeta);

            ) {
            auto object_meta = (const ObjectMeta*)ptr;
            assert(object_meta->size * 4 == sizeof(Object));
            assert(object_meta->type <= 4);
            obj_counts[object_meta->type] = obj_counts[object_meta->type].toInt() + object_meta->count;
            ptr += sizeof(ObjectMeta);
            ptr += object_meta->count * 4 * object_meta->size;
        }
        algo_process_profile["obj_counts"] = obj_counts;
        return algo_process_profile;
    }
};
// static_assert(sizeof(STAlgopackV2_1::Header) == 108);
#pragma pack(pop)

}

#endif // !_____P64ST_ALGOPACK_HH___
