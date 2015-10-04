/**
* @author Percy
*/

#ifndef SOUNDPROCESSING_H
#define SOUNDPROCESSING_H
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#define PI M_PI /* pi to machine precision, defined in math.h */
#define TWOPI (2.0*PI)

#include <string>
#include <rttools/rttime.h>

#include <boost/shared_ptr.hpp>
#include <alvalue/alvalue.h>
#include <alproxies/almemoryproxy.h>
#include <alaudio/alsoundextractor.h>
#include <alcommon/almodule.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>

using namespace AL;

namespace AL
{
  class ALBroker;
}

class ALSoundProcessing : public ALSoundExtractor
{

public:

  ALSoundProcessing(boost::shared_ptr<ALBroker> pBroker, std::string pName);
  virtual ~ALSoundProcessing();

  void init();

public:
  void process(const int & nbOfChannels,
               const int & nbrOfSamplesByChannel,
               const AL_SOUND_FORMAT * buffer,
               const ALValue & timeStamp);

  void fft(double data[], int nn);


private:
  ALTextToSpeechProxy ttsProxy;
  ALMotionProxy motion;
  int restcount;
  int validsoundcount;
  int validwhistlecount;
  AL::ALValue jointName;
  AL::ALValue stiffness;
  AL::ALValue time;
  AL::ALValue targetAngles;
  AL::ALValue targetTimes;
  bool isAbsolute;

};
#endif

