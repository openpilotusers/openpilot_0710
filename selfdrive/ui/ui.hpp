#pragma once
#include "messaging.hpp"

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define NANOVG_GL3_IMPLEMENTATION
#define nvgCreate nvgCreateGL3
#else
#include <GLES3/gl3.h>
#include <EGL/egl.h>
#define NANOVG_GLES3_IMPLEMENTATION
#define nvgCreate nvgCreateGLES3
#endif

#include <atomic>
#include <map>
#include <string>
#include <sstream>

#include "nanovg.h"

#include "common/mat.h"
#include "common/visionipc.h"
#include "common/visionimg.h"
#include "common/framebuffer.h"
#include "common/modeldata.h"
#include "common/params.h"
#include "sound.hpp"

//#include "cereal/gen/c/arne182.capnp.h"

#define COLOR_BLACK nvgRGBA(0, 0, 0, 255)
#define COLOR_BLACK_ALPHA(x) nvgRGBA(0, 0, 0, x)
#define COLOR_WHITE nvgRGBA(255, 255, 255, 255)
#define COLOR_WHITE_ALPHA(x) nvgRGBA(255, 255, 255, x)
#define COLOR_YELLOW nvgRGBA(218, 202, 37, 255)
#define COLOR_RED nvgRGBA(201, 34, 49, 255)
#define COLOR_OCHRE nvgRGBA(218, 111, 37, 255)
#define COLOR_OCHRE_ALPHA(x) nvgRGBA(218, 111, 37, x)
#define COLOR_GREEN nvgRGBA(0, 255, 0, 255)
#define COLOR_GREEN_ALPHA(x) nvgRGBA(0, 255, 0, x)
#define COLOR_ORANGE nvgRGBA(255, 175, 3, 255)
#define COLOR_ORANGE_ALPHA(x) nvgRGBA(255, 175, 3, x)
#define COLOR_RED_ALPHA(x) nvgRGBA(201, 34, 49, x)
#define COLOR_YELLOW_ALPHA(x) nvgRGBA(218, 202, 37, x)
#define UI_BUF_COUNT 4

typedef struct Rect {
  int x, y, w, h;
  int centerX() const { return x + w / 2; }
  int right() const { return x + w; }
  int bottom() const { return y + h; }
  bool ptInRect(int px, int py) const {
    return px >= x && px < (x + w) && py >= y && py < (y + h);
  }
} Rect;

const int sbr_w = 300;
const int bdr_s = 30;
const int bdr_is = 30;
const int vwp_h = 1080;
const int header_h = 420;
const int footer_h = 280;
const int footer_y = vwp_h-bdr_s-footer_h;
const Rect settings_btn = {50, 35, 200, 117};
const Rect home_btn = {60, 1080 - 180 - 40, 180, 180};

const int UI_FREQ = 20;   // Hz

const int MODEL_PATH_MAX_VERTICES_CNT = 98;
const int MODEL_LANE_PATH_CNT = 2;
const int TRACK_POINTS_MAX_CNT = 50 * 2;

const int SET_SPEED_NA = 255;

typedef enum NetStatus {
  NET_CONNECTED,
  NET_DISCONNECTED,
  NET_ERROR,
} NetStatus;

typedef enum UIStatus {
  STATUS_OFFROAD,
  STATUS_DISENGAGED,
  STATUS_ENGAGED,
  STATUS_WARNING,
  STATUS_ALERT,
} UIStatus;

static std::map<UIStatus, NVGcolor> bg_colors = {
  {STATUS_OFFROAD, nvgRGBA(0x0, 0x0, 0x0, 0xf1)},
  {STATUS_DISENGAGED, nvgRGBA(0x0, 0x0, 0x0, 0xc8)},
  {STATUS_ENGAGED, nvgRGBA(0x01, 0x50, 0x01, 0xf1)},
  {STATUS_WARNING, nvgRGBA(0x80, 0x80, 0x80, 0xf1)},
  {STATUS_ALERT, nvgRGBA(0xC9, 0x22, 0x31, 0xf1)},
};

typedef struct UIScene {

  bool mlButtonEnabled;

  float mpc_x[50];
  float mpc_y[50];

  mat4 extrinsic_matrix;      // Last row is 0 so we can use mat4.
  bool world_objects_visible;

  //dev ui
  float speedlimit;
  bool speedlimit_valid;
  float speedlimitaheaddistance;
  bool speedlimitahead_valid;
  bool map_valid;
  float gpsAccuracy;
  int lead_status;
  int lead_status2;
  float lead_d_rel, lead_y_rel, lead_v_rel;
  float lead_d_rel2, lead_y_rel2, lead_v_rel2;
  int engaged;
  //bool brakeLights;
  bool leftBlinker;
  //bool steerOverride;
  bool rightBlinker;
  int blinker_blinkingrate;
  //std::string ipAddr;
  //float output_scale;
  //cereal::CarState::GearShifter gear;
  bool rightblindspot;
  bool leftblindspot;

  bool is_rhd;
  bool frontview;
  bool uilayout_sidebarcollapsed;
  // responsive layout
  Rect viz_rect;
  int ui_viz_ro;

  std::string alert_text1;
  std::string alert_text2;
  std::string alertTextMsg1;
  std::string alertTextMsg2;
  std::string alert_type;
  cereal::ControlsState::AlertSize alert_size;
  float awareness_status;

  bool  brakePress;
  float angleSteers;
  bool brakeLights;
  float angleSteersDes;
  bool recording;
  float gpsAccuracyUblox;
  float altitudeUblox;
  bool steerOverride;
  float output_scale;
  float steeringTorqueEps;
  float aEgo;
  float cpuTemp;
  int cpuPerc;
  float steerRatio;
  float curvature;

  float cpu0Temp;
  int blindspot_blinkingrate = 120;
  int car_valid_status_changed = 0;
  int batteryPercent;
  bool batteryCharging;
  char batteryStatus[64];
  char ipAddr[20];
  int fanSpeed;
  float tpmsPressureFl;
  float tpmsPressureFr;
  float tpmsPressureRl;
  float tpmsPressureRr;
  int lateralControlMethod;
  float radarDistance;
  int long_plan_source;

  cereal::HealthData::HwType hwType;
  int satelliteCount;
  NetStatus athenaStatus;

  cereal::ThermalData::Reader thermal;
  cereal::RadarState::LeadData::Reader lead_data[2];
  cereal::ControlsState::Reader controls_state;
  cereal::DriverState::Reader driver_state;
  cereal::DMonitoringState::Reader dmonitoring_state;
  cereal::ModelData::Reader model;
  float left_lane_points[MODEL_PATH_DISTANCE];
  float path_points[MODEL_PATH_DISTANCE];
  float right_lane_points[MODEL_PATH_DISTANCE];
  cereal::CarState::GearShifter getGearShifter;
  cereal::PathPlan::Reader path_plan;

  struct _LiveParams
  {
    float gyroBias;
    float angleOffset;
    float angleOffsetAverage;
    float stiffnessFactor;
    float steerRatio;
    float yawRate;
    float posenetSpeed;
  } liveParams;

  struct _PathPlan
  {
    float laneWidth;
    float steerRatio;
    float steerActuatorDelay;
    float steerRateCost;
    int standstillElapsedTime = 0;

    float cProb;
    float lProb;
    float rProb;

    float angleOffset;

    float lPoly;
    float rPoly;
  } pathPlan;
} UIScene;

typedef struct {
  float x, y;
} vertex_data;

typedef struct {
  vertex_data v[MODEL_PATH_MAX_VERTICES_CNT];
  int cnt;
} model_path_vertices_data;

typedef struct {
  vertex_data v[TRACK_POINTS_MAX_CNT];
  int cnt;
} track_vertices_data;


typedef struct UIState {
  // framebuffer
  FramebufferState *fb;
  int fb_w, fb_h;

  // NVG
  NVGcontext *vg;

  // fonts and images
  int font_sans_regular;
  int font_sans_semibold;
  int font_sans_bold;
  int img_wheel;
  int img_hands_on_wheel;
  int img_turn;
  int img_face;
  int img_button_settings;
  int img_button_home;
  int img_battery;
  int img_battery_charging;
  int img_network[6];
  int img_map;
  int img_speed;
  int img_car_left;
  int img_car_right;

  SubMaster *sm;
  PubMaster *pm;

  Sound *sound;
  UIStatus status;
  UIScene scene;
  cereal::UiLayoutState::App active_app;

  // vision state
  bool vision_connected;
  VisionStream stream;

  // graphics
  GLuint frame_program;
  GLuint frame_texs[UI_BUF_COUNT];
  EGLImageKHR khr[UI_BUF_COUNT];
  void *priv_hnds[UI_BUF_COUNT];

  GLint frame_pos_loc, frame_texcoord_loc;
  GLint frame_texture_loc, frame_transform_loc;
  GLuint frame_vao[2], frame_vbo[2], frame_ibo[2];
  mat4 rear_frame_mat, front_frame_mat;

  // device state
  bool awake;
  int awake_timeout;
  float light_sensor, accel_sensor, gyro_sensor;

  bool started;
  bool ignition;
  bool is_metric;
  bool longitudinal_control;
  bool limit_set_speed;
  bool is_ego_over_limit;
  float speed_lim_off;
  
  int is_OpenpilotViewEnabled;
  int lateral_control;
  int lat_mode;
  int acc_mode;

  int nOpkrAutoScreenOff;
  int nOpkrUIBrightness;
  int nOpkrUIVolumeBoost;
  int nDebugUi1;
  int nDebugUi2;
  int nOpkrBlindSpotDetect;
  uint64_t last_athena_ping;
  uint64_t started_frame;

  bool alert_blinked;
  float alert_blinking_alpha;

  track_vertices_data track_vertices[2];
  model_path_vertices_data model_path_vertices[MODEL_LANE_PATH_CNT * 2];

  Rect video_rect;
} UIState;

void ui_init(UIState *s);
void sa_init(UIState *s, bool full_init);
void ui_update(UIState *s);

int write_param_float(float param, const char* param_name, bool persistent_param = false);
template <class T>
int read_param(T* param, const char *param_name, bool persistent_param = false){
  T param_orig = *param;
  char *value;
  size_t sz;

  int result = Params(persistent_param).read_db_value(param_name, &value, &sz);
  if (result == 0){
    std::string s = std::string(value, sz); // value is not null terminated
    free(value);

    // Parse result
    std::istringstream iss(s);
    iss >> *param;

    // Restore original value if parsing failed
    if (iss.fail()) {
      *param = param_orig;
      result = -1;
    }
  }
  return result;
}
