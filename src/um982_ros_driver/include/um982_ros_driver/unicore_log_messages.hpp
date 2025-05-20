#ifndef ROVER_AUTONOMY_UNICOREMSGS_HPP
#define ROVER_AUTONOMY_UNICOREMSGS_HPP

#include <array>

namespace UnicoreLogMsgs {
  #pragma pack(push, 1)  // Ensure no padding is added

  const std::array<uint8_t, 3> SYNC_BYTES = {0xAA, 0x44, 0xB5};
  constexpr size_t UNICORE_HEADER_LEN = 24;

  constexpr uint16_t UNIHEADING_MSG_ID = 972;
  constexpr size_t UNIHEADING_LOG_LEN = UNICORE_HEADER_LEN + 48;

  constexpr uint16_t AGRIC_MSG_ID = 11276;
  constexpr size_t AGRIC_LOG_LEN = UNICORE_HEADER_LEN + 232;

  constexpr uint16_t PVTSLN_MSG_ID = 1021;
  constexpr size_t PVTSLN_LOG_LEN = UNICORE_HEADER_LEN + 228;

  constexpr uint16_t RTKSTATUS_MSG_ID = 509;
  constexpr size_t RTKSTATUS_LOG_LEN = UNICORE_HEADER_LEN + 60;

  constexpr size_t MIN_LOG_LEN = UNIHEADING_LOG_LEN;

  enum class Model : uint32_t {
    UNKNOWN = 0,
    UB4B0   = 1,
    UM4B0   = 2,
    UM480   = 3,
    UM440   = 4,
    UM482   = 5,
    UM982   = 17
  };

  enum class PosVelType: uint32_t {
    NO_SOL = 0,
    FIXED_POS = 1,
    FIXED_HEIGHT = 2,
    DOPPLER_VEL = 8,
    SINGLE = 16,
    PSRDIFF = 17,
    SBAS = 18,
    L1_FLOAT = 32,
    IONOFREE_FLOAT = 33,
    NARROW_FLOAT = 34,
    L1_INT = 48,
    WIDE_INT = 49,
    NARROW_INT = 50,
    INS = 52,
    INS_PSRSP = 53,
    INS_PSRDIFF = 54,
    INS_RTKFLOAT = 55,
    INS_RTKFIXED = 56,
    PPP_CONVERGING = 68,
    PPP = 69
  };

  enum class SolutionStatus: uint32_t {
    SOL_COMPUTED = 0,
    INSUFF_OBS = 1,
    NO_CONVERGENCE = 2,
    COV_TRACE_HIGH = 3
  };

  enum class RTKStatus_CalcStatus : uint32_t {
    NO_DATA = 0,
    INSUFF_OBS_SRC = 1,
    HIGH_LATENCY = 2,
    ACTIVE_IONOSPH = 3,
    INSUFF_OBS_ROV = 4,
    RTK_GOOD = 5
  };

  // Binary Header: Usually 24 Bytes
  struct UnicoreBinaryHeader {
    uint8_t  sync1;         // 0xAA
    uint8_t  sync2;         // 0x44
    uint8_t  sync3;         // 0xB5
    uint8_t  cpu_idle;      // CPU idle 0–100
    uint16_t message_id;    // Little-endian
    uint16_t message_length;
    uint8_t  time_ref;
    uint8_t  time_status;
    uint16_t week_number;
    uint32_t ms_of_week;
    uint32_t version;
    uint8_t  reserved;
    uint8_t  leap_seconds;
    uint16_t delay_ms;
  };

  // VERSION, ID: 37 (x0025)
  struct VersionLOG {
    UnicoreBinaryHeader header;

    Model model;                 // Offset: H, Size: 4
    char sw_version[33];         // Offset: H+4
    char auth[129];              // Offset: H+37
    char psn[66];                // Offset: H+166
    char efuse_id[33];           // Offset: H+232
    char compile_time[43];       // Offset: H+265
    uint32_t crc;                // Offset: H+308
  };

  // AGRIC, ID: 11276
  struct AGRIC {
    // Header
    UnicoreBinaryHeader header;

    // H+0
    char gnss[4];                // Just says "GNSS"
    uint8_t length;             // Payload length (0xE8 = 232)

    // H+5: UTC Time
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    // H+11: Status
    uint8_t pos_type;
    uint8_t heading_status;
    uint8_t num_gps;
    uint8_t num_bds;
    uint8_t num_glo;

    // H+16: Baseline N/E/U
    float baseline_n;
    float baseline_e;
    float baseline_u;

    // H+28: Std dev of baseline
    float baseline_n_std;
    float baseline_e_std;
    float baseline_u_std;

    // H+40: Attitude
    float heading;
    float pitch;
    float roll;

    // H+52: Speed & Velocity
    float speed;
    float vel_n;
    float vel_e;
    float vel_u;

    // H+68: Velocity Std Dev
    float std_vel_n;
    float std_vel_e;
    float std_vel_u;

    // H+80: WGS84 Position
    double lat;
    double lon;
    double alt;

    // H+104: ECEF Position
    double ecef_x;
    double ecef_y;
    double ecef_z;

    // H+128: WGS84 Std Dev
    float std_lat;
    float std_lon;
    float std_alt;

    // H+140: ECEF Std Dev
    float std_ecef_x;
    float std_ecef_y;
    float std_ecef_z;

    // H+152: Base station
    double base_lat;
    double base_lon;
    double base_alt;

    // H+176: Slave antenna
    double slave_lat;
    double slave_lon;
    double slave_alt;

    // H+200: GPS Time (ms of week)
    int32_t gps_time_ms;

    // H+204: Differential age
    float diff_age;

    // H+208: Direction and Undulation
    float speed_heading;
    float undulation;

    // H+216: Reserved floats
    float reserved_float_1;
    float reserved_float_2;

    // H+224: Galileo + speed type
    uint8_t num_galileo;
    uint8_t speed_type;

    // H+226: Reserved chars
    uint8_t reserved_char_1;
    uint8_t reserved_char_2;

    // H+228: CRC
    uint32_t crc;  // 32-bit CRC.
    // H + 232 Including CRC

  }; // AGRIC


  struct AGRICShort {
    // H+80: WGS84 Position
    double lat;
    double lon;
    double alt;

    // H+128: WGS84 Std Dev
    float lat_sigma;
    float lon_sigma;
    float alt_sigma;

    // H+40: Attitude
    float heading;
    float pitch;
    float roll;

    // H+52: Speed & Velocity
    float speed;
    float vel_n;
    float vel_e;
    float vel_u;

    // H+68: Velocity Std Dev
    float vel_n_sigma;
    float vel_e_sigma;
    float vel_u_sigma;

    // H+208: Direction and Undulation
    float speed_heading;
    float undulation;
  };


  /* PVTSLN, ID: 1021, H + 228 = 252 Bytes */
  struct PVTSLN {
    UnicoreBinaryHeader header;

    uint32_t bestpos_type;
    float bestpos_hgt;
    double bestpos_lat;
    double bestpos_lon;
    float bestpos_hgtstd;
    float bestpos_latstd;
    float bestpos_lonstd;
    float bestpos_diffage;

    uint32_t psrpos_type;
    float psrpos_hgt;
    double psrpos_lat;
    double psrpos_lon;

    float undulation;

    uint8_t bestpos_svs;
    uint8_t bestpos_solnsvs;
    uint8_t psrpos_svs;
    uint8_t psrpos_solnsvs;

    double psrvel_north;
    double psrvel_east;
    double psrvel_ground;

    uint32_t heading_type;
    float baseline;
    float heading_deg;
    float pitch_deg;

    uint8_t heading_trackedsvs;
    uint8_t heading_solnsvs;
    uint8_t heading_ggl1;
    uint8_t heading_ggl12;

    float gdop;
    float pdop;
    float hdop;
    float htdop;
    float tdop;
    float cutoff;

    uint16_t prn_num;
    uint16_t prn_list[41];

    uint32_t crc;  // CRC at H+224
  };

  struct PVTSLNShort {
    uint32_t bestpos_type;
    float bestpos_hgt;
    double bestpos_lat;
    double bestpos_lon;
    float bestpos_hgtstd;
    float bestpos_latstd;
    float bestpos_lonstd;

    double psrvel_north;
    double psrvel_east;
    double psrvel_ground;

    uint32_t heading_type;
    float baseline;
    float heading_deg;
    float pitch_deg;
  };

  /* UNIHEADING ID: 972, H + 48 = 72 bytes */
  struct UNIHEADING {
    UnicoreBinaryHeader header;

    uint32_t sol_stat;           // H
    uint32_t pos_type;           // H+4
    float baseline_length;       // H+8
    float heading;               // H+12
    float pitch;                 // H+16
    float reserved1;             // H+20
    float hdg_stddev;            // H+24
    float pitch_stddev;          // H+28
    char stn_id[4];              // H+32

    uint8_t num_svs;             // H+36
    uint8_t num_solnsvs;         // H+37
    uint8_t num_obs_above_elev;  // H+38
    uint8_t num_multi_l2;        // H+39
    uint8_t reserved2;           // H+40
    uint8_t ext_sol_status;      // H+41
    uint8_t sig_mask_gal_bds3;   // H+42
    uint8_t sig_mask_gps_glon_bds2; // H+43

    uint32_t crc;                // H+44
  };

  struct UNIHEADINGShort {
    uint32_t sol_status;
    uint32_t pos_type;
    float baseline_len;
    float heading;
    float heading_sigma;
    float pitch;
    float pitch_sigma;
  };

  struct RTKSTATUS {
    UnicoreBinaryHeader header;
    uint32_t gpsSource;        // H+0
    uint32_t reserved1;        // H+4
    uint32_t bdsSource1;       // H+8
    uint32_t bdsSource2;       // H+12
    uint32_t reserved2;        // H+16
    uint32_t gloSource;        // H+20
    uint32_t reserved3;        // H+24
    uint32_t galSource1;       // H+28
    uint32_t galSource2;       // H+32
    uint32_t qzssSource;       // H+36
    uint32_t reserved4;        // H+40
    uint32_t positionType;     // H+44
    uint32_t calcStatus;       // H+48
    uint8_t  ionDetected;      // H+52
    uint8_t  dualRtkFlag;      // H+53
    uint8_t  adrNumber;        // H+54
    uint8_t  reserved5;        // H+55
    uint32_t crc;              // H+56
    char     terminator[2];    // H+60
  };

  #pragma pack(pop)
} // namespace UnicoreLogMsgs



namespace RoverMsgs {
  enum class PositionType : uint8_t {
    Invalid = 0,
    Single = 1,
    DGPS = 2,
    FloatRTK = 4,
    FixedRTK = 5,
    SBAS = 6,
    PPP = 8,
    Unknown = 255
  };


  struct Position3D {
    double latitude_deg;       // degrees
    double longitude_deg;      // degrees
    double altitude_m;         // meters

    double std_latitude_m;     // meters
    double std_longitude_m;    // meters
    double std_altitude_m;     // meters

  };

  struct Attitude3D {
    double roll_deg;           // degrees
    double pitch_deg;          // degrees
    double yaw_deg;            // degrees

    double std_roll_deg;       // degrees
    double std_pitch_deg;      // degrees
    double std_yaw_deg;        // degrees
  };



  struct Pose {
    Position3D pos;
    Attitude3D att;
    uint32_t gps_week;
    double seconds_of_week;    // fractional seconds since beginning of week
  };

  struct Velocity {
    double vel_north;    // m/s (e.g., forward)
    double vel_east;     // m/s (e.g., lateral)
    double vel_up;       // m/s (e.g., vertical)

    double yaw_rate_dps;   // deg/s

    double std_vel_north;
    double std_vel_east;
    double std_vel_up;
    double std_yaw_rate_dps;

    uint32_t gps_week;
    double seconds_of_week;
  };

  struct GpsStatus {
    PositionType pos_type;
    bool rtk_fix;
    bool heading_fix;
    uint8_t num_sats_used;

    float differential_age_sec;  // Age of corrections, if reported
    float cpu_idle_percent;

    uint32_t gps_week;
    double seconds_of_week;
  };
}

#endif // ROVER_AUTONOMY_UNICOREMSGS_HPP
