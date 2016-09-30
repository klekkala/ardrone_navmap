 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "EstimationNode.h"
#include "ros/ros.h"
#include "PTAMWrapper.h"
#include "MapView.h"
#include "boost/thread.hpp"
#include "ORB_SLAM/src/Tracking.h"
#include "ORB_SLAM/src/FramePublisher.h"
#include "ORB_SLAM/src/MapPublisher.h"
#include "ORB_SLAM/src/Map.h"
#include "ORB_SLAM/src/LocalMapping.h"
#include "ORB_SLAM/src/LoopClosing.h"
#include "ORB_SLAM/src/KeyFrameDatabase.h"
#include "ORB_SLAM/src/ORBVocabulary.h"

// this global var is used in getMS(ros::Time t) to convert to a consistent integer timestamp used internally pretty much everywhere.
// kind of an artifact from Windows-Version, where only that was available / used.
// Modified
unsigned int ros_header_timestamp_base = 0;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "drone_stateestimation");


  ROS_INFO("Started TUM ArDrone Stateestimation Node.");

/*
  const string strVocFile = "/home/kiran/catkin_ws/src/Vocabulary/ORBvoc.txt";
  const string strSettingsFile = "/home/kiran/catkin_ws/src/ardrone_navmap/camcalib/ardrone2_default.yaml";

  



    //Create Frame Publisher for image_view
    ORB_SLAM::FramePublisher FramePub;

    //Load ORB Vocabulary
   /* Old version to load vocabulary using cv::FileStorage
    string strVocFile = ros::package::getPath("ORB_SLAM")+"/"+argv[1];
    cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;
    cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
    if(!fsVoc.isOpened())
    {
        cerr << endl << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
        ros::shutdown();
        return 1;
    }
    ORB_SLAM::ORBVocabulary Vocabulary;
    Vocabulary.load(fsVoc);
    */
    
    // New version to load vocabulary from text file "Data/ORBvoc.txt". 
    // If you have an own .yml vocabulary, use the function
    // saveToTextFile in Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h
    
    /*
    ORB_SLAM::ORBVocabulary Vocabulary;
    bool bVocLoad = Vocabulary.loadFromTextFile(strVocFile);
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

    //Create the map
    ORB_SLAM::Map World;

    FramePub.SetMap(&World);

    //Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher MapPub(&World);

    //Initialize the Tracking Thread and launch
    ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, &MapPub, &World, strSettingsFile);
    boost::thread trackingThread(&ORB_SLAM::Tracking::Run,&Tracker);

    Tracker.SetKeyFrameDatabase(&Database);

    //Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping LocalMapper(&World);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run,&LocalMapper);

    //Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);

    //Set pointers between threads
    Tracker.SetLocalMapper(&LocalMapper);
    Tracker.SetLoopClosing(&LoopCloser);

    LocalMapper.SetTracker(&Tracker);
    LocalMapper.SetLoopCloser(&LoopCloser);

    LoopCloser.SetTracker(&Tracker);
    LoopCloser.SetLocalMapper(&LocalMapper);*/

  EstimationNode estimator;

  dynamic_reconfigure::Server<tum_ardrone::StateestimationParamsConfig> srv;
  dynamic_reconfigure::Server<tum_ardrone::StateestimationParamsConfig>::CallbackType f;
  f = boost::bind(&EstimationNode::dynConfCb, &estimator, _1, _2);
  srv.setCallback(f);

  estimator.ptamWrapper->startSystem();
  estimator.mapView->startSystem();

  estimator.Loop();

  estimator.mapView->stopSystem();
  estimator.ptamWrapper->stopSystem();

  return 0;
}
