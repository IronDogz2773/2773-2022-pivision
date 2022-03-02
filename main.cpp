// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "cameraserver/CameraServer.h"

#include "ballpipeline.h"
#include "retro.h"

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
       "switched cameras": [
           {
               "name": <virtual camera name>
               "key": <network table key used for selection>
               // if NT value is a string, it's treated as a name
               // if NT value is a double, it's treated as an integer index
           }
       ]
   }
 */

static const char* configFile = "/boot/frc.json";

namespace {

unsigned int team;
bool server = false;

struct CameraConfig {
  std::string name;
  std::string path;
  wpi::json config;
  wpi::json streamConfig;
};

struct SwitchedCameraConfig {
  std::string name;
  std::string key;
};

std::vector<CameraConfig> cameraConfigs;
std::vector<SwitchedCameraConfig> switchedCameraConfigs;
std::vector<cs::VideoSource> cameras;

wpi::raw_ostream& ParseError() {
  return wpi::errs() << "config error in '" << configFile << "': ";
}

bool ReadCameraConfig(const wpi::json& config) {
  CameraConfig c;

  // name
  try {
    c.name = config.at("name").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read camera name: " << e.what() << '\n';
    return false;
  }

  // path
  try {
    c.path = config.at("path").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "camera '" << c.name
                 << "': could not read path: " << e.what() << '\n';
    return false;
  }

  // stream properties
  if (config.count("stream") != 0) c.streamConfig = config.at("stream");

  c.config = config;

  cameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadSwitchedCameraConfig(const wpi::json& config) {
  SwitchedCameraConfig c;

  // name
  try {
    c.name = config.at("name").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read switched camera name: " << e.what() << '\n';
    return false;
  }

  // key
  try {
    c.key = config.at("key").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "switched camera '" << c.name
                 << "': could not read key: " << e.what() << '\n';
    return false;
  }

  switchedCameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadConfig() {
  // open config file
  std::error_code ec;
  wpi::raw_fd_istream is(configFile, ec);
  if (ec) {
    wpi::errs() << "could not open '" << configFile << "': " << ec.message()
                << '\n';
    return false;
  }

  // parse file
  wpi::json j;
  try {
    j = wpi::json::parse(is);
  } catch (const wpi::json::parse_error& e) {
    ParseError() << "byte " << e.byte << ": " << e.what() << '\n';
    return false;
  }

  // top level must be an object
  if (!j.is_object()) {
    ParseError() << "must be JSON object\n";
    return false;
  }

  // team number
  try {
    team = j.at("team").get<unsigned int>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read team number: " << e.what() << '\n';
    return false;
  }

  // ntmode (optional)
  if (j.count("ntmode") != 0) {
    try {
      auto str = j.at("ntmode").get<std::string>();
      wpi::StringRef s(str);
      if (s.equals_lower("client")) {
        server = false;
      } else if (s.equals_lower("server")) {
        server = true;
      } else {
        ParseError() << "could not understand ntmode value '" << str << "'\n";
      }
    } catch (const wpi::json::exception& e) {
      ParseError() << "could not read ntmode: " << e.what() << '\n';
    }
  }

  // cameras
  try {
    for (auto&& camera : j.at("cameras")) {
      if (!ReadCameraConfig(camera)) return false;
    }
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read cameras: " << e.what() << '\n';
    return false;
  }

  // switched cameras (optional)
  if (j.count("switched cameras") != 0) {
    try {
      for (auto&& camera : j.at("switched cameras")) {
        if (!ReadSwitchedCameraConfig(camera)) return false;
      }
    } catch (const wpi::json::exception& e) {
      ParseError() << "could not read switched cameras: " << e.what() << '\n';
      return false;
    }
  }

  return true;
}

cs::UsbCamera StartCamera(const CameraConfig& config) {
  wpi::outs() << "Starting camera '" << config.name << "' on " << config.path
              << '\n';
  auto inst = frc::CameraServer::GetInstance();
  cs::UsbCamera camera{config.name, config.path};
  auto server = inst->StartAutomaticCapture(camera);

  camera.SetConfigJson(config.config);
  camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

  if (config.streamConfig.is_object())
    server.SetConfigJson(config.streamConfig);

  return camera;
}

cs::MjpegServer StartSwitchedCamera(const SwitchedCameraConfig& config) {
  wpi::outs() << "Starting switched camera '" << config.name << "' on "
              << config.key << '\n';
  auto server =
      frc::CameraServer::GetInstance()->AddSwitchedCamera(config.name);

  nt::NetworkTableInstance::GetDefault()
      .GetEntry(config.key)
      .AddListener(
          [server](const auto& event) mutable {
            if (event.value->IsDouble()) {
              int i = event.value->GetDouble();
              if (i >= 0 && i < cameras.size()) server.SetSource(cameras[i]);
            } else if (event.value->IsString()) {
              auto str = event.value->GetString();
              for (int i = 0; i < cameraConfigs.size(); ++i) {
                if (str == cameraConfigs[i].name) {
                  server.SetSource(cameras[i]);
                  break;
                }
              }
            }
          },
          NT_NOTIFY_IMMEDIATE | NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

  return server;
}
}

int main(int argc, char* argv[]) {
  if (argc >= 2) configFile = argv[1];

  // read configuration
  if (!ReadConfig()) return EXIT_FAILURE;

  // start NetworkTables
  auto ntinst = nt::NetworkTableInstance::GetDefault();
  if (server) {
    wpi::outs() << "Setting up NetworkTables server\n";
    ntinst.StartServer();
  } else {
    wpi::outs() << "Setting up NetworkTables client for team " << team << '\n';
    ntinst.StartClientTeam(team);
    ntinst.StartDSClient();
  }

  // start cameras
  for (const auto& config : cameraConfigs)
    cameras.emplace_back(StartCamera(config));

  // start switched cameras
  for (const auto& config : switchedCameraConfigs) StartSwitchedCamera(config);

  // start image processing on camera 0 if present
  if (cameras.size() >= 1) {
    std::thread([&] {
      auto table = ntinst.GetTable("pivision");
      nt::NetworkTableEntry red_1_x = table->GetEntry("red_1_x");
      // nt::NetworkTableEntry red_1_y = table->GetEntry("red_1_y");
      nt::NetworkTableEntry red_1_distance = table->GetEntry("red_1_distance");
      nt::NetworkTableEntry red_1_present = table->GetEntry("red_1_present");
      nt::NetworkTableEntry blue_1_x = table->GetEntry("blue_1_x");
      // nt::NetworkTableEntry blue_1_y = table->GetEntry("blue_1_y");
      nt::NetworkTableEntry blue_1_distance = table->GetEntry("blue_1_distance");
      nt::NetworkTableEntry blue_1_present = table->GetEntry("blue_1_present");
      // nt::NetworkTableEntry red_2_x = table->GetEntry("red_2_x");
      // nt::NetworkTableEntry red_2_y = table->GetEntry("red_2_y");
      // nt::NetworkTableEntry red_2_present = table->GetEntry("red_2_present");
      // nt::NetworkTableEntry blue_2_x = table->GetEntry("blue_2_x");
      // nt::NetworkTableEntry blue_2_y = table->GetEntry("blue_2_y");
      // nt::NetworkTableEntry blue_2_present = table->GetEntry("blue_2_present");
      // nt::NetworkTableEntry red_3_x = table->GetEntry("red_3_x");
      // nt::NetworkTableEntry red_3_y = table->GetEntry("red_3_y");
      // nt::NetworkTableEntry red_3_present = table->GetEntry("red_3_present");
      // nt::NetworkTableEntry blue_3_x = table->GetEntry("blue_3_x");
      // nt::NetworkTableEntry blue_3_y = table->GetEntry("blue_3_y");
      // nt::NetworkTableEntry blue_3_present = table->GetEntry("blue_3_present");
      int i, j, k;
      int max[3];
      int max_index[3];
      double key;
      bool temp_bool;
      double res_x = cameraConfigs[0].config.at("width").get<double>();
      double res_y = cameraConfigs[0].config.at("height").get<double>();
      double midpoint_x = res_x / 4.0;
      double midpoint_y = res_y / 4.0;
      std::vector<cv::KeyPoint> temp_red, temp_blue;
      frc::VisionRunner<grip::ballpipeline> runner(cameras[0], new grip::ballpipeline(),
                                           [&](grip::ballpipeline& pipeline) 
      {
        temp_red = *(pipeline.GetFindBlobs1Output());
        if(temp_red.size() > 0)
        {
          for(i = 0; i < (3 <  temp_red.size() ? 3 : temp_red.size()); i++)
          {
            max[i] = 0;
            for(j = 0; j < temp_red.size(); j++)
            {
              if(temp_red[j].size > max[i])
              {
                if(i > 0)
                {
                  temp_bool = true;
                  for(k = 0; k < i; k++)
                  {
                    if(max_index[k] = j)
                      temp_bool = false;
                  }
                  if(temp_bool)
                  {
                    max[i] = temp_red[j].size;
                    max_index[i] = j;
                  }
                }
                else
                {
                  max[i] = temp_red[j].size;
                  max_index[i] = j;
                }
              }
            }
          }
          red_1_x.SetDouble(33.4762782961 * (temp_red[max_index[0]].pt.x - midpoint_x) / midpoint_x);
          // red_1_y.SetDouble(33.4762782961 * (temp_red[max_index[0]].pt.y - midpoint_x) / midpoint_x);
          red_1_distance.SetDouble(0.209375*res_x/temp_red[max_index[0]].size);
          red_1_present.SetBoolean(true);
          std::cout << "red: " << 33.4762782961 * (temp_red[max_index[0]].pt.x - midpoint_x) / midpoint_x << std::endl;

          if(temp_red.size() > 1)
          {
            // red_2_x.SetDouble(temp_red[max_index[1]].pt.x);
            // red_2_y.SetDouble(temp_red[max_index[1]].pt.y);
            // red_2_present.SetBoolean(true);
            std::cout << "RED 2: " << temp_red[max_index[1]].pt.x << ", " << temp_red[max_index[1]].pt.y << std::endl;
          }
          else
            ;// red_2_present.SetBoolean(false);

          if(temp_red.size() > 2)
          {
            // red_3_x.SetDouble(temp_red[max_index[2]].pt.x);
            // red_3_y.SetDouble(temp_red[max_index[2]].pt.y);
            // red_3_present.SetBoolean(true);
            std::cout << "RED 3: " << temp_red[max_index[2]].pt.x << ", " << temp_red[max_index[2]].pt.y << std::endl;
          }
          else
            ;// red_3_present.SetBoolean(false);
        }
        else
          red_1_present.SetBoolean(false);
        temp_blue = *(pipeline.GetFindBlobs0Output());
        if(temp_blue.size() > 0)
        {
          for(i = 0; i < (3 <  temp_blue.size() ? 3 : temp_blue.size()); i++)
          {
            max[i] = 0;
            for(j = 0; j < temp_blue.size(); j++)
            {
              if(temp_blue[j].size > max[i])
              {
                if(i > 0)
                {
                  temp_bool = true;
                  for(k = 0; k < i; k++)
                  {
                    if(max_index[k] = j)
                      temp_bool = false;
                  }
                  if(temp_bool)
                  {
                    max[i] = temp_blue[j].size;
                    max_index[i] = j;
                  }
                }
                else
                {
                  max[i] = temp_blue[j].size;
                  max_index[i] = j;
                }
              }
            }
          }
          blue_1_x.SetDouble(33.4762782961 * (temp_blue[max_index[0]].pt.x - midpoint_x) / midpoint_x);
          // blue_1_y.SetDouble(temp_blue[max_index[0]].pt.y);
          blue_1_distance.SetDouble(0.209375*res_x/temp_blue[max_index[0]].size);
          blue_1_present.SetBoolean(true);
          std::cout << "\nblue 1: " << temp_blue[max_index[0]].pt.x << ", " << temp_blue[max_index[0]].pt.y << std::endl;
          if(temp_blue.size() > 1)
          {
            // blue_2_x.SetDouble(temp_blue[max_index[1]].pt.x);
            // blue_2_y.SetDouble(temp_blue[max_index[1]].pt.y);
            // blue_2_present.SetBoolean(true);
            std::cout << "blue 2: " << temp_blue[max_index[1]].pt.x << ", " << temp_blue[max_index[1]].pt.y << std::endl;
          }
          else
            ;// blue_2_present.SetBoolean(false);

          if(temp_blue.size() > 2)
          {
            // blue_3_x.SetDouble(temp_blue[max_index[2]].pt.x);
            // blue_3_y.SetDouble(temp_blue[max_index[2]].pt.y);
            // blue_3_present.SetBoolean(true);
            std::cout << "blue 3: " << temp_blue[max_index[2]].pt.x << ", " << temp_blue[max_index[2]].pt.y << std::endl;
          }
          else
            ;// blue_3_present.SetBoolean(false);
        }
        else
          blue_1_present.SetBoolean(false);
      });
      runner.RunForever();
    }).detach();
     if (cameras.size() >= 2) {
    std::thread([&] {
      // auto table = ntinst.GetTable("pivision");
      // nt::NetworkTableEntry retro_average_x = table->GetEntry("retro_average_x");
      // nt::NetworkTableEntry retro_average_y = table->GetEntry("retro_average_y");
      int i;
      int sum_x, sum_y;
      std::vector<cv::KeyPoint> temp_blobs;
      frc::VisionRunner<grip::retro> runner(cameras[1], new grip::retro(),
                                           [&](grip::retro& pipeline) 
      {
        temp_blobs = *(pipeline.GetFindBlobsOutput());
        if(temp_blobs.size() > 0)
        {
          sum_x = 0;
          sum_y = 0;
          for(i = 0; i < temp_blobs.size(); i++)
          {
            sum_x += temp_blobs[i].pt.x;
            sum_y += temp_blobs[i].pt.y;
          }
          // retro_average_x.SetDouble(sum_x / temp_blobs.size());
          // retro_average_y.SetDouble(sum_y / temp_blobs.size());
          std::cout << "RETRO AVG: " << sum_x / temp_blobs.size() << ", " << sum_y / temp_blobs.size() << std::endl;
        }
      });
      runner.RunForever();
    }).detach();
    
  }

  // loop forever
  for (;;) std::this_thread::sleep_for(std::chrono::seconds(10));
}
}
