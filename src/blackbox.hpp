/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef OPENDLV_SIM_CFSD18_COGNITION_BLACKBOX_HPP
#define OPENDLV_SIM_CFSD18_COGNITION_BLACKBOX_HPP

#include <opendlv-standard-message-set.hpp>
#include <cluon-complete.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <thread>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <list>
#include <vector>
#include <algorithm>
#include <string>
#include "neat/neat.h"
#include "neat/network.h"
#include "neat/population.h"
#include "neat/organism.h"
#include "neat/genome.h"
#include "neat/species.h"

class BlackBox {
 public:
  BlackBox(std::map<std::string, std::string>);
  BlackBox(BlackBox const &) = delete;
  BlackBox &operator=(BlackBox const &) = delete;
  virtual ~BlackBox();
  virtual void nextContainer(cluon::data::Envelope &);

 private:
  void setUp();
  void tearDown();

  void generateSurfaces(Eigen::ArrayXXf, Eigen::ArrayXXf, Eigen::ArrayXXf globalLocation, float yaw);
  void readMap(std::string filename);
  void run(Eigen::ArrayXXf globalLocation, float yaw);
  Eigen::ArrayXXf simConeDetectorSlam(Eigen::ArrayXXf globalMap, Eigen::ArrayXXf location, float heading, int nConesInFakeSlam);
  void vehicleModel(float steeringAngle, float prevSteerAngle,float accelerationRequest, float vx,float vy, float yawRate, float dt);
  float magicFormula(float const &a_slipAngle, float const &a_forceZ,
      float const &a_frictionCoefficient, float const &a_cAlpha, float const &a_c,
      float const &a_e);

  uint16_t m_cid;
  float m_maxSteering;
  float m_maxAcceleration;
  float m_maxDeceleration;
  float m_dt;
  float m_timeLimit;
  std::string m_mapfilename;
  std::string m_pathFile;
  std::string m_XYFile;
  std::string m_CrashFile;
  std::string m_SpeedFile;
  NEAT::Network *m_net;
  Eigen::ArrayXXf m_leftCones;
  Eigen::ArrayXXf m_rightCones;
  Eigen::ArrayXXf m_smallCones;
  Eigen::ArrayXXf m_bigCones;
  bool m_orangeVisibleInSlam;
  float m_prevSteerAngle;
  Eigen::VectorXf m_kinematicState;
  std::mutex m_stateMutex;
  int m_step;
  int m_step2;
  bool m_OT;
  bool m_crash;
  int m_nOffTrack;
  int m_nHitRight;
  int m_nHitLeft;
  Eigen::Vector2f m_crashLocation;
  float m_distanceTraveledAlongPath;
  std::vector<float> m_globalPath;
  int m_lastClosestPointIndex;
  int m_crashCount;
  int m_startIndex;
  float m_pathLength;

  const double DEG2RAD = 0.017453292522222; // PI/180.0

};


#endif
