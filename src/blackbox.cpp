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

#include <iostream>
#include <cstdlib>
#include "blackbox.hpp"

BlackBox::BlackBox(std::map<std::string, std::string> commandlineArguments) :
 m_cid{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))}
, m_maxSteering{(commandlineArguments["maxSteering"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxSteering"]))) : (25.0f)}
, m_maxAcceleration{(commandlineArguments["maxAcceleration"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxAcceleration"]))) : (5.0f)}
, m_maxDeceleration{(commandlineArguments["maxDeceleration"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxDeceleration"]))) : (5.0f)}
, m_dt{(commandlineArguments["dt"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["dt"]))) : (0.05f)}
, m_timeLimit{(commandlineArguments["timeLimit"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["timeLimit"]))) : (100.0f)}
, m_mapfilename{(commandlineArguments["mapfilename"].size() != 0) ? (commandlineArguments["mapfilename"]) : ("track1.csv")}
, m_pathFile{(commandlineArguments["pathFile"].size() != 0) ? (commandlineArguments["pathFile"]) : ("path1.csv")}
, m_XYFile{(commandlineArguments["XYFile"].size() != 0) ? (commandlineArguments["XYFile"]) : ("XY_blackboxcomplete_0x.csv")}
, m_CrashFile{(commandlineArguments["CrashFile"].size() != 0) ? (commandlineArguments["CrashFile"]) : ("Crash_blackboxcomplete_0x.csv")}
, m_SpeedFile{(commandlineArguments["SpeedFile"].size() != 0) ? (commandlineArguments["SpeedFile"]) : ("Speed_blackboxcomplete_0x.csv")}
, m_net()
, m_leftCones()
, m_rightCones()
, m_smallCones()
, m_bigCones()
, m_orangeVisibleInSlam()
, m_prevSteerAngle{0.0f}
, m_kinematicState{}
, m_stateMutex()
, m_step{0}
, m_step2{0}
, m_OT{false}
, m_crash{false}
, m_nOffTrack{0}
, m_nHitRight{0}
, m_nHitLeft{0}
, m_crashLocation{}
, m_distanceTraveledAlongPath{0.0f}
, m_globalPath{}
, m_lastClosestPointIndex{0}
, m_crashCount{0}
, m_startIndex{0}
, m_pathLength{0}
{
  m_crashLocation(0) = -1000.0f;
  m_crashLocation(1) = -1000.0f;
  m_kinematicState = Eigen::VectorXf::Zero(6);
  m_kinematicState(0) = 0.01f;
  std::cout<<"crashLocation:" <<m_crashLocation<<std::endl;
  std::cout<<"kinState:" <<m_kinematicState<<std::endl;
  setUp();
}

BlackBox::~BlackBox()
{
}

void BlackBox::setUp()
{
  std::cout << "Setting up blackbox with maxSteering = " << m_maxSteering << ", maxAcceleration = " << m_maxAcceleration << ", maxDeceleration = " << m_maxDeceleration << std::endl;
  std::cout << "Using Map: " << m_mapfilename <<std::endl;
  NEAT::Population *pop=0;
  NEAT::Genome *start_genome;
  char curword[20];
  int id;
  int pop_size = 1;

  //Read in the start Genome
  std::string const filename = "cfsdstartgenes";
  std::string const HOME = "/opt/opendlv.data/";
  std::string infile = HOME + filename;

  std::ifstream iFile(infile);
  iFile>>curword;
  iFile>>id;
  std::cout<<"Reading in Genome id "<<id<<std::endl;
  start_genome=new NEAT::Genome(id,iFile);
  iFile.close();
  pop=new NEAT::Population(start_genome,pop_size);
  pop->verify();

  std::vector<NEAT::Organism*>::iterator curorg;
  for(curorg=(pop->organisms).begin();curorg!=(pop->organisms).end();++curorg) {
    NEAT::Organism *org = *curorg;
    NEAT::Network *net;
    net=org->net;
    m_net = net;
  }
  std::cout << "Network extracted" << std::endl;

  BlackBox::readMap(m_mapfilename);

  //Read path
  std::string line, word;
  float x, y, angle;
  std::ifstream pathfile("/opt/opendlv.data/"+m_pathFile, std::ifstream::in);
  //std::ifstream pathfile;
  //pathfile.open();

  if(pathfile.is_open())
  {
    while(getline(pathfile,line))
    {
      std::stringstream strstr(line);

      getline(strstr,word,',');
      x = std::stof(word);
      getline(strstr,word,',');
      y = std::stof(word);
      getline(strstr,word,',');
      angle = std::stof(word);
      m_globalPath.push_back(x);
      m_globalPath.push_back(y);
      m_globalPath.push_back(angle);
    } // End of while
    pathfile.close();
  } // End of if
  std::cout<<"globalPath size:" <<m_globalPath.size()<<std::endl;
  uint32_t idx1=0;
  uint32_t idx2=idx1+3;
    for (uint32_t i = 0; i<m_globalPath.size()/3; i++){
      m_pathLength += sqrtf(powf(m_globalPath[idx2]-m_globalPath[idx1],2)+powf(m_globalPath[idx2+1]-m_globalPath[idx1+1],2));
      idx1 +=3;
      idx2 +=3;
    }
    std::cout<<"pathLength:" <<m_pathLength<<std::endl;
} // End of setUp

void BlackBox::tearDown()
{
}

void BlackBox::readMap(std::string filename)
{
  int leftCounter = 0;
  int rightCounter = 0;
  int smallCounter = 0;
  int bigCounter = 0;

  std::string line, word;
  std::string const HOME = "/opt/opendlv.data/";
  std::string infile = HOME + filename;

  std::ifstream file(infile, std::ifstream::in);

  if(file.is_open())
  {
    while(getline(file,line))
    {
      std::stringstream strstr(line);

      getline(strstr,word,',');
      getline(strstr,word,',');
      getline(strstr,word,',');

      if(word.compare("1") == 0){leftCounter = leftCounter+1;}
      else if(word.compare("2") == 0){rightCounter = rightCounter+1;}
      else if(word.compare("3") == 0){smallCounter = smallCounter+1;}
      else if(word.compare("4") == 0){bigCounter = bigCounter+1;}
      else{std::cout << "ERROR in BlackBox::simDetectCone while counting types. Not a valid cone type." << std::endl;}
    } // End of while

    file.close();
  } // End of if

  Eigen::ArrayXXf tmpLeftCones(leftCounter,2);
  Eigen::ArrayXXf tmpRightCones(rightCounter,2);
  Eigen::ArrayXXf tmpSmallCones(smallCounter,2);
  Eigen::ArrayXXf tmpBigCones(bigCounter,2);
  float x, y;
  leftCounter = 0;
  rightCounter = 0;
  smallCounter = 0;
  bigCounter = 0;
  std::ifstream myFile(infile, std::ifstream::in);

  if(myFile.is_open())
  {
    while(getline(myFile,line))
    {
      std::stringstream strstr(line);

      getline(strstr,word,',');
      x = std::stof(word);
      getline(strstr,word,',');
      y = std::stof(word);

      getline(strstr,word,',');

      if(word.compare("1") == 0)
      {
        tmpLeftCones(leftCounter,0) = x;
        tmpLeftCones(leftCounter,1) = y;
        leftCounter = leftCounter+1;
      }
      else if(word.compare("2") == 0)
      {
        tmpRightCones(rightCounter,0) = x;
        tmpRightCones(rightCounter,1) = y;
        rightCounter = rightCounter+1;
      }
      else if(word.compare("3") == 0)
      {
        tmpSmallCones(smallCounter,0) = x;
        tmpSmallCones(smallCounter,1) = y;
        smallCounter = smallCounter+1;
      }
      else if(word.compare("4") == 0)
      {
        tmpBigCones(bigCounter,0) = x;
        tmpBigCones(bigCounter,1) = y;
        bigCounter = bigCounter+1;
      }
      else{std::cout << "ERROR in BlackBox::simDetectCone while storing cones. Not a valid cone type." << std::endl;}
    } // End of while

    myFile.close();
  } // End of if

  m_leftCones = tmpLeftCones;
  m_rightCones = tmpRightCones;
  m_smallCones = tmpSmallCones;
  m_bigCones = tmpBigCones;
} // End of readMap

void BlackBox::nextContainer(cluon::data::Envelope &a_container)
{
  if (a_container.dataType() == opendlv::sim::Frame::ID())
  {
    auto frame =  cluon::extractMessage<opendlv::sim::Frame>(std::move(a_container));
    float x = frame.x();
    float y = frame.y();
    float yaw = frame.yaw();
    Eigen::ArrayXXf globalLocation(1,2);
    globalLocation(0,0) = x;
    globalLocation(0,1) = y;
    BlackBox::run(globalLocation, yaw);
  }
}

void BlackBox::run(Eigen::ArrayXXf globalLocation, float yaw){
  Eigen::ArrayXXf detectedConesLeft, detectedConesRight, detectedConesSmall, detectedConesBig;
  int nConesFakeSlam = 5;
    detectedConesLeft = BlackBox::simConeDetectorSlam(m_leftCones, globalLocation, yaw, nConesFakeSlam);
    detectedConesRight = BlackBox::simConeDetectorSlam(m_rightCones, globalLocation, yaw, nConesFakeSlam);
    if(m_orangeVisibleInSlam)
    {
      // If the orange cones are set to visible in the detection they will be transformed into local coordinates and stored
      m_orangeVisibleInSlam = false;
      Eigen::MatrixXf rotationMatrix(2,2);
      rotationMatrix << std::cos(-yaw),-std::sin(-yaw),
                        std::sin(-yaw),std::cos(-yaw);

      Eigen::ArrayXXf tmpLocationSmall(m_smallCones.rows(),2);
      (tmpLocationSmall.col(0)).fill(globalLocation(0));
      (tmpLocationSmall.col(1)).fill(globalLocation(1));
      Eigen::ArrayXXf tmpLocationBig(m_bigCones.rows(),2);
      (tmpLocationBig.col(0)).fill(globalLocation(0));
      (tmpLocationBig.col(1)).fill(globalLocation(1));

      detectedConesSmall = ((rotationMatrix*(((m_smallCones-tmpLocationSmall).matrix()).transpose())).transpose()).array();
      detectedConesBig = ((rotationMatrix*(((m_bigCones-tmpLocationBig).matrix()).transpose())).transpose()).array();
    }
    else
    {
      // Otherwise no orange cones are stored
      detectedConesSmall.resize(0,2);
      detectedConesBig.resize(0,2);
    } // End of else

  BlackBox::generateSurfaces(detectedConesLeft, detectedConesRight, globalLocation, yaw);
}

Eigen::ArrayXXf BlackBox::simConeDetectorSlam(Eigen::ArrayXXf globalMap, Eigen::ArrayXXf location, float heading, int nConesInFakeSlam)
{
  // Input: Positions of cones and vehicle, heading angle, detection ranges forward and to the side
  // Output: Local coordinates of the upcoming cones

  int nCones = globalMap.rows();
  Eigen::MatrixXf rotationMatrix(2,2);
  rotationMatrix << std::cos(-heading),-std::sin(-heading),
                    std::sin(-heading),std::cos(-heading);
  Eigen::ArrayXXf tmpLocation(nCones,2);
  (tmpLocation.col(0)).fill(location(0));
  (tmpLocation.col(1)).fill(location(1));

  // Convert to local coordinates
  Eigen::ArrayXXf localMap = ((rotationMatrix*(((globalMap-tmpLocation).matrix()).transpose())).transpose()).array();

  float shortestDist = std::numeric_limits<float>::infinity();
  float tmpDist;
  int closestConeIndex = -1;

  // Find the closest cone. It will be the first in the returned sequence.
  for(int i = 0; i < nCones; i = i+1)
  {
    tmpDist = ((localMap.row(i)).matrix()).norm();
    if(tmpDist < shortestDist && tmpDist > 0)
    {
      shortestDist = tmpDist;
      closestConeIndex = i;
    } // End of if
  } // End of for

  if(closestConeIndex != -1)
  {
    Eigen::VectorXi indices;

    // If more than the existing cones are requested, send all existing cones
    if(nConesInFakeSlam >= nCones)
    {
      // If the first cone is closest, no wrap-around is needed
      if(closestConeIndex == 0)
      {
        indices = Eigen::VectorXi::LinSpaced(nCones,0,nCones-1);
      }
      else
      {
        Eigen::VectorXi firstPart = Eigen::VectorXi::LinSpaced(nCones-closestConeIndex,closestConeIndex,nCones-1);
        Eigen::VectorXi secondPart = Eigen::VectorXi::LinSpaced(closestConeIndex,0,closestConeIndex-1);
        indices.resize(firstPart.size()+secondPart.size());
        indices.topRows(firstPart.size()) = firstPart;
        indices.bottomRows(secondPart.size()) = secondPart;
      } // End of else
    }
    // If the sequence should contain both the end and beginning of the track, do wrap-around
    else if(closestConeIndex + nConesInFakeSlam > nCones)
    {
      Eigen::VectorXi firstPart = Eigen::VectorXi::LinSpaced(nCones-closestConeIndex,closestConeIndex,nCones-1);
      Eigen::VectorXi secondPart = Eigen::VectorXi::LinSpaced(nConesInFakeSlam-(nCones-closestConeIndex),0,nConesInFakeSlam-(nCones-closestConeIndex)-1);
      indices.resize(firstPart.size()+secondPart.size());
      indices.topRows(firstPart.size()) = firstPart;
      indices.bottomRows(secondPart.size()) =secondPart;
    }
    // Otherwise simply take the closest and the following cones
    else
    {
      indices = Eigen::VectorXi::LinSpaced(nConesInFakeSlam,closestConeIndex,closestConeIndex+nConesInFakeSlam-1);
    }

    // Sort the cones according to the order set above
    Eigen::ArrayXXf detectedCones(indices.size(),2);
    for(int i = 0; i < indices.size(); i = i+1)
    {
      detectedCones.row(i) = localMap.row(indices(i));
    }

    // If the first cones of the track is visible, the orange cones are set as visible as well
    if(indices.minCoeff() == 0)
    {
      m_orangeVisibleInSlam = true;
    }

    return detectedCones;

  }
  // If no closest cone was found, the returned array is empty
  else
  {
    std::cout << "Error: No cone found in fake slam detection" << std::endl;
    Eigen::ArrayXXf detectedCones(0,2);

    return detectedCones;
  } // End of else
} // End of simConeDetectorSlam


void BlackBox::generateSurfaces(Eigen::ArrayXXf sideLeft, Eigen::ArrayXXf sideRight, Eigen::ArrayXXf globalLocation, float yaw){
  if(sideLeft.rows()==5 && sideRight.rows()==5){

    Eigen::ArrayXXd leftSide = sideLeft.cast <double> ();
    Eigen::ArrayXXd rightSide = sideRight.cast <double> ();

    double in[24];  //Input loading array
    double out1;
    double out2;
    float vx, vy, yawRate;
    std::vector<NEAT::NNode*>::iterator out_iter;

    in[0] = 1; // Bias
    {
      std::unique_lock<std::mutex> lockState(m_stateMutex);
      in[1] = static_cast<double>(m_kinematicState(0));
      in[2] = static_cast<double>(m_kinematicState(1));
      in[3] = static_cast<double>(m_kinematicState(2));
      vx = m_kinematicState(0);
      vy = m_kinematicState(1);
      yawRate = m_kinematicState(2);
    }
    in[4] = leftSide(0,0);
    in[5] = leftSide(0,1);
    in[6] = leftSide(1,0);
    in[7] = leftSide(1,1);
    in[8] = leftSide(2,0);
    in[9] = leftSide(2,1);
    in[10] = leftSide(3,0);
    in[11] = leftSide(3,1);
    in[12] = leftSide(4,0);
    in[13] = leftSide(4,1);
    in[14] = rightSide(0,0);
    in[15] = rightSide(0,1);
    in[16] = rightSide(1,0);
    in[17] = rightSide(1,1);
    in[18] = rightSide(2,0);
    in[19] = rightSide(2,1);
    in[20] = rightSide(3,0);
    in[21] = rightSide(3,1);
    in[22] = rightSide(4,0);
    in[23] = rightSide(4,1);

    m_net->load_sensors(in);

    //Activate the net
    //If it loops, exit returning only fitness of 1 step
    if(m_net->activate())
    {
      //std::cout << "NET ACTIVATED" << std::endl;
      out_iter=m_net->outputs.begin();
      out1=(*out_iter)->activation;
      ++out_iter;
      out2=(*out_iter)->activation;
    }
    //std::cout << "OUTS: " << out1*2-1 << " and " << out2*2-1 << std::endl;

    float maxSteer = m_maxSteering;
    float maxAcc = m_maxAcceleration;
    float maxDec = m_maxDeceleration;

    // Send messages
    cluon::OD4Session od4{m_cid,[](auto){}};
    float steeringAngle = maxSteer*3.14159265f/180.0f*(out1*2-1);
    //Send for Ui TODO: Remove
    opendlv::logic::action::AimPoint o4;
    o4.azimuthAngle(steeringAngle);
    o4.distance(2.0f);
    od4.send(o4);

    float acc = maxAcc*(out2*2-1);
    vehicleModel(steeringAngle, m_prevSteerAngle, acc, vx, vy, yawRate, m_dt);
    m_prevSteerAngle = steeringAngle;
    opendlv::sim::KinematicState kinematicState;
    kinematicState.vx(m_kinematicState(0));
    kinematicState.vy(m_kinematicState(1));
    kinematicState.yawRate(m_kinematicState(2));
    kinematicState.vz(0.0);
    kinematicState.rollRate(0.0);
    kinematicState.pitchRate(0.0);
    od4.send(kinematicState);



    /*--------------SAVE DATA---------------*/
    m_step++;
    m_step2++;
    int closestPointIndex;
    std::string reason;
    if (m_step*m_dt>m_timeLimit) {
      reason = "Cleared timeLimit";
      m_crash = true;
    }

    // Find closestPointIndex and calculate vehicle offset
    Eigen::Vector2f tmpPoint;
    Eigen::Vector2f vehicleLocation(2,1);
    vehicleLocation(0) = globalLocation(0,0);
    vehicleLocation(1) = globalLocation(0,1);
    float vehicleOffset = std::numeric_limits<float>::infinity();
    for(uint32_t j = 0; j < m_globalPath.size()/3; j++)
    {
        tmpPoint << m_globalPath[j*3],
                    m_globalPath[j*3+1];
        float tmpDist = (vehicleLocation-tmpPoint).norm();
        if(tmpDist < vehicleOffset)
        {
          vehicleOffset = tmpDist;
          closestPointIndex = j*3;
        } // End of if
    } // End of for
    if (fabs(vehicleOffset)>3.0f&&((vehicleLocation-m_crashLocation).norm()>3.0f)){ // if going far of track, break
      std::cout<<"I'M OFF TRACK"<<std::endl;
      reason = "Went off track";
      m_crashLocation = vehicleLocation;
      m_crash = true;
    }
  if (leftSide.rows()>0 && rightSide.rows() >0) {
    if (!m_OT) {
    float D = (leftSide.matrix().row(0)-rightSide.matrix().row(0)).norm()+1.5f;
    if (leftSide.matrix().row(0).norm()>D || rightSide.matrix().row(0).norm()>D){ // if going far of track, break
      std::cout<<"I'M OFF TRACK"<<std::endl;
      reason = "Went off track";
      m_crashLocation = vehicleLocation;
      m_OT = true;
      m_nOffTrack++;
      m_crash = true;
    }
    }
  }
  if (leftSide.rows()>1 && rightSide.rows()>1) {
    if (((leftSide(0,0)<1.0f) && (leftSide(0,0)>-1.0f))&&((leftSide(0,1)<0.65f) && (leftSide(0,1)>-0.65f))&&((vehicleLocation-m_crashLocation).norm()>3.0f)) {
      std::cout<<"I HIT A CONE"<<std::endl;
      reason = "Hit a left side cone";
      m_crashLocation = vehicleLocation;
      m_nHitLeft++;
      m_crash = true; // if hitting a cone, break

    }
    if (((rightSide(0,0)<1.0f) && (rightSide(0,0)>-1.0f))&&((rightSide(0,1)<0.65f) && (rightSide(0,1)>-0.65f))&&((vehicleLocation-m_crashLocation).norm()>3.0f)) {
      std::cout<<"I HIT A CONE"<<std::endl;
      reason = "Hit a right side cone";
      m_crashLocation = vehicleLocation;
      m_nHitRight++;
      m_crash = true; // if hitting a cone, break

    }
  }
    if(m_crash){
      //std::cout<<"CRASH"<<std::endl;

      std::ofstream tmpFile;
      tmpFile.open("/opt/opendlv.data/"+m_CrashFile,std::ios_base::app);
      tmpFile<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< "<<std::endl;
      tmpFile<<"m_crashCount: "<<m_crashCount<<std::endl;
      tmpFile<<"Reason: "<<reason<<std::endl;
      tmpFile<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< "<<std::endl;
      tmpFile<<"Time since last crash: "<<m_step*m_dt<<std::endl;
      tmpFile<<"Timesince start: "<<m_step2*m_dt<<std::endl;
      tmpFile<<"m_startIndex: "<<m_startIndex<<std::endl;
      tmpFile<<"endIndex: "<<closestPointIndex<<std::endl;
      tmpFile<<"m_m_distanceTraveledAlongPath: "<<m_distanceTraveledAlongPath<<std::endl;
      tmpFile<<"Start -> x: "<<m_globalPath[m_startIndex]<<" y: "<< m_globalPath[m_startIndex+1] <<" yaw: "<<m_globalPath[m_startIndex+2]<<std::endl;
      tmpFile<<"Stop -> x: "<<vehicleLocation(0)<<" y: "<< vehicleLocation(1) <<" yaw: "<<yaw<<std::endl;
      tmpFile<<"Calculated average vx: "<<m_distanceTraveledAlongPath/(m_step*m_dt)*3.6f<<" km/h"<<std::endl;
      tmpFile<<"--------------------------------- "<<std::endl;
      tmpFile.close();
      m_crashCount++;
      m_crash = false;
      m_step = 0;
      m_distanceTraveledAlongPath = 0.0f;
      m_startIndex = closestPointIndex;
    }


    //Save path
    std::ofstream xyFile;
    xyFile.open("/opt/opendlv.data/"+m_XYFile,std::ios_base::app);
    xyFile<<vehicleLocation(0)<<","<<vehicleLocation(1)<<std::endl;
    xyFile.close();

    // Save velocities
    if (m_kinematicState.size()>5) {
      std::ofstream speedFile;
      speedFile.open("/opt/opendlv.data/"+m_SpeedFile,std::ios_base::app);
      speedFile<<m_kinematicState(0)<<","<<m_kinematicState(1)<<","<<m_kinematicState(2)<<","<<m_kinematicState(3)<<","<<m_kinematicState(4)<<","<<m_kinematicState(5)<<std::endl;
      speedFile.close();
    }




    uint32_t index1=m_lastClosestPointIndex;
    uint32_t index2=index1+3;
    float diff = std::abs(closestPointIndex-m_lastClosestPointIndex);

    if(diff>100.0f){
      std::ofstream tmpFile;
      tmpFile.open("/opt/opendlv.data/"+m_CrashFile,std::ios_base::app);
      tmpFile<<"################################# "<<std::endl;
      tmpFile<<"Total time: "<<m_step2*m_dt<<std::endl;
      if (m_nOffTrack>0) {
        tmpFile<<"Total time with penalties: DNF"<<std::endl;
      } else{
        tmpFile<<"Total time with penalties: "<<m_step2*m_dt+2.0f*(m_nHitLeft+m_nHitRight)<<std::endl;
      }
      tmpFile<<"Total length of path: "<<m_pathLength<<std::endl;
      tmpFile<<"Total crashes this run: "<<"Hit left: "<<m_nHitLeft<<" | Hit right: "<<m_nHitRight<<" | Off track: "<< m_nOffTrack<<std::endl;
      tmpFile<<"Calculated average vx: "<<m_pathLength/(m_step2*m_dt)*3.6f<<" km/h"<<std::endl;
      tmpFile.close();
    }



    if (index2>m_globalPath.size()-3)
      index2 = 0;
    if (diff>=3 && ((m_lastClosestPointIndex < closestPointIndex && diff<100.0f) || (m_lastClosestPointIndex > closestPointIndex && diff>100.0f)) ) {
      //std::cout<<"Enter count loop with index1: "<<index1<<" index2: "<<index2<<" closestPointIndex: "<<closestPointIndex<<"\n";
      while (static_cast<int>(index1) != closestPointIndex){
        m_distanceTraveledAlongPath += sqrtf(powf(m_globalPath[index2]-m_globalPath[index1],2)+powf(m_globalPath[index2+1]-m_globalPath[index1+1],2));
        //std::cout<<"Count in while: "<<m_distanceTraveledAlongPath<<"\n";
        index1 +=3;
        index2 +=3;
        if (index1>m_globalPath.size()-3){
          //std::cout<<"index1 = 0 -> closestPointIndex = "<< closestPointIndex<<"\n";
          index1 = 0;
        }

        else if (index2>m_globalPath.size()-3){
          //std::cout<<"index2 = 0 -> closestPointIndex = "<< closestPointIndex<<"\n";
          index2 = 0;
        }
      }
    }
    m_lastClosestPointIndex = closestPointIndex;























  }
  else
  {
  std::cout << "Not enough cones to run network" << std::endl;
  }
} // End of generateSurfaces

void BlackBox::vehicleModel(float steeringAngle, float prevSteerAngle,float accelerationRequest, float vx,float vy, float yawRate, float dt) {

  const float g=9.81;
  const float mass = 188.0;
  const float momentOfInertiaZ = 105.0;
  const float length = 1.53;
  const float frontToCog = 0.765;
  const float rearToCog = length-frontToCog;
  const float frictionCoefficient = 0.9;
  const float magicFormulaCAlpha = 25229.0;
  const float magicFormulaC = 1.0;
  const float magicFormulaE = -2.0;
  if(vx <=0){
    vx = 0.01f;
  }
  if (std::abs(steeringAngle-prevSteerAngle)/dt>(80.0f*3.14159265f/180.0f)){
    if (steeringAngle > prevSteerAngle) {
      steeringAngle = dt*80.0f*3.14159265f/180.0f + prevSteerAngle;
    }
    else{
      steeringAngle = -dt*80.0f*3.14159265f/180.0f + prevSteerAngle;
    }
  }

  float slipAngleFront = steeringAngle - std::atan(
      (vy + frontToCog * yawRate) / std::abs(vx));
  float slipAngleRear = -std::atan((vy - rearToCog * yawRate) /
      std::abs(vx));

  float forceFrontZ = mass * g * (frontToCog / (frontToCog + length));
  float forceRearZ = mass * g * (length / (frontToCog + length));

  float forceFrontY = magicFormula(slipAngleFront, forceFrontZ,
      frictionCoefficient, magicFormulaCAlpha, magicFormulaC, magicFormulaE);
  float forceRearY = magicFormula(slipAngleRear, forceRearZ,
      frictionCoefficient, magicFormulaCAlpha, magicFormulaC, magicFormulaE);

  float rollResistance;
  if (vx>0) {rollResistance = -9.81*0.02;}
  else if (vx<0){rollResistance = 9.81*0.02;}
  else {rollResistance = 0.0;}

  float vxDot = accelerationRequest - std::sin(steeringAngle)*forceFrontY/mass + yawRate * vy + rollResistance;

  float vyDot =
    (forceFrontY * std::cos(steeringAngle) + forceRearY) / mass -
    yawRate * vx;

  float yawRateDot = (frontToCog * forceFrontY *
      std::cos(steeringAngle) - rearToCog * forceRearY) /
    momentOfInertiaZ;

  if ((vx+=vxDot * dt)<0.0f)
    vx=0.0f;
  else if ((vx+=vxDot * dt)>30.0f)
    vx=30.0f;
  else
    vx += vxDot * dt;

  vy += vyDot * dt;
  yawRate += yawRateDot * dt;
  {
    std::unique_lock<std::mutex> lockState(m_stateMutex);
    m_kinematicState(0) = vx;
    m_kinematicState(1) = vy;
    m_kinematicState(2) = yawRate;
    m_kinematicState(3) = vxDot;
    m_kinematicState(4) = vyDot;
    m_kinematicState(5) = yawRateDot;
  }

} // End vehicleModel

float BlackBox::magicFormula(float const &a_slipAngle, float const &a_forceZ,
    float const &a_frictionCoefficient, float const &a_cAlpha, float const &a_c,
    float const &a_e)
{
  float const b = a_cAlpha / (a_c * a_frictionCoefficient * a_forceZ);
  float const forceY = a_frictionCoefficient * a_forceZ * std::sin(a_c *
     std::atan(b * a_slipAngle - a_e * (b * a_slipAngle - std::atan(b * a_slipAngle))));
  return forceY;
}// End magicFormula
