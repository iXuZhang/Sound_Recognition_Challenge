

#include "alsoundprocessing.h"
#include <alcommon/alproxy.h>
#include <iostream>

ALSoundProcessing::ALSoundProcessing(boost::shared_ptr<ALBroker> pBroker,
                                     std::string pName)
  : ALSoundExtractor(pBroker, pName)
{
  setModuleDescription("This module processes the data collected by the "
                       "microphones and output in the ALMemory the RMS power "
                       "of each of the four channels.");
}

void ALSoundProcessing::init()
{

  audioDevice->callVoid("setClientPreferences",
                        getName(),                //Name of this module
                        16000,                    //48000 Hz requested
                        (int)LEFTCHANNEL,         //4 Channels requested
                        1                         //Deinterleaving requested
                        );
  restcount = 0;
  validsoundcount = 0;
  validwhistlecount = 0;
  jointName1 = "LShoulderPitch";
  jointName2 = "RShoulderPitch";
  stiffness = 1.0f;
  time = 1.0f;
  isAbsolute = true;
  motion.stiffnessInterpolation(jointName1, stiffness, time);
  motion.stiffnessInterpolation(jointName2, stiffness, time);

  std::fstream cfgFile;
  cfgFile.open("/home/nao/Config/sound.cfg");
  if( !cfgFile)  
  {  
      std::cout<<"can not open cfg file!"<<std::endl; 
  }
  char tmp[100];
  int K[20];
  int i;
  i = 0;
  while(!cfgFile.eof())  
  {  
      tmp[0] = '\0';
      cfgFile.getline(tmp,100,' '); 
      K[i] = atoi(tmp);
      cfgFile.getline(tmp,100); 
      std::cout<<"In sound Recognition Challenge,  K = "<<K[i]<<std::endl;
      i ++;
  }  
  cfgFile.close();

  isCout = K[0];
  validsoundcountValue = K[1];
  validwhistlecountValue = K[2];
  fmaxmax1 = K[3];
  fmaxmin1 = K[4];
  fmaxmax2 = K[5];
  fmaxmin2 = K[6];
  fmax2max = K[7];
  fmax2min = K[8];
  whistleDifference = K[9];
  whistlefmaxmax = K[10];
  whistlefmaxmin = K[11];
  soundmaxValue = K[12]*1000000000;
  whistlemaxValue = K[13]*1000000000;

  ttsProxy.say("Detection Begin");
  startDetection();
}

ALSoundProcessing::~ALSoundProcessing()
{
  stiffness = 0.0f;
  time = 1.0f;
  motion.stiffnessInterpolation(jointName1, stiffness, time);
  motion.stiffnessInterpolation(jointName2, stiffness, time);
  ttsProxy.say("Detection Stop");
  stopDetection();
}

void ALSoundProcessing::fft(double data[], int nn)
{
    int n, mmax, m, j, istep, i;
    double wtemp, wr, wpr, wpi, wi, theta;
    double tempr, tempi;
    n = nn << 1;
    j = 1;
    for (i = 1; i < n; i += 2) {
        if (j > i) {
            tempr = data[j]; data[j] = data[i]; data[i] = tempr;
            tempr = data[j+1]; data[j+1] = data[i+1]; data[i+1] = tempr;
        }
        m = n >> 1;
        while (m >= 2 && j > m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }
    mmax = 2;
    while (n > mmax) {
        istep = 2*mmax;
        theta = TWOPI/mmax;
        wtemp = sin(0.5*theta);
        wpr = -2.0*wtemp*wtemp;
        wpi = sin(theta);
        wr = 1.0;
        wi = 0.0;
        for (m = 1; m < mmax; m += 2) {
            for (i = m; i <= n; i += istep) {
                j =i + mmax;
                tempr = wr*data[j] - wi*data[j+1];
                tempi = wr*data[j+1] + wi*data[j];
                data[j] = data[i] - tempr;
                data[j+1] = data[i+1] - tempi;
                data[i] += tempr;
                data[i+1] += tempi;
            }
            wr = (wtemp = wr)*wpr - wi*wpi + wr;
            wi = wi*wpr + wtemp*wpi + wi;
        }
        mmax = istep;
    }
}

void ALSoundProcessing::process(const int & nbOfChannels,
                                const int & nbOfSamplesByChannel,
                                const AL_SOUND_FORMAT * buffer,
                                const ALValue & timeStamp)
{
    if (restcount == 0)
    {

        clock_t startTime, finishTime;
        double totalTime;
        startTime = clock();

        double data[4100];
        int a;
        int nn;
        int fmax;
        int fmax2;
        long double Rmax;
        long double Rmax2;
        long double R;
        bool soundvalid;
        bool soundvalid2;
        bool whistlevalid;

        for(a = 0; a <= 2048; a++)
        {
            data[2*a+1] = buffer[1*a];
            data[2*a] = 0;
        }

        nn = 2048;
        fft(data, nn);

        R = 0;
        Rmax =0;
        Rmax2 = 0;

        for(a = 1; a < 100; a++)
        {

            R = data[2*a+1]*data[2*a+1] + data[2*a+2]*data[2*a+2];
            if (R > Rmax)
            {
                Rmax = R;
                fmax = a;
            }
            else if(R >Rmax2)
            {
                Rmax2 = R;
                fmax2 = a;
            }
        }
        for(a = 300; a < 1024; a++)
        {

            R = data[2*a+1]*data[2*a+1] + data[2*a+2]*data[2*a+2];
            if (R > Rmax)
            {
                Rmax = R;
                fmax = a;
            }
            else if(R >Rmax2)
            {
                Rmax2 = R;
                fmax2 = a;
            }
        }
        if(isCout)
        {
            std::cout<< "Rmax = "<< Rmax <<std::endl;
            std::cout<< "fmax = "<< fmax <<std::endl;
            std::cout<< "Rmax2 = "<< Rmax2 <<std::endl;
            std::cout<< "fmax2 = "<< fmax2 <<std::endl;
        }

        soundvalid = ((fmax > fmaxmin1)&&(fmax < fmaxmax1)) || ((fmax < fmaxmax2)&&(fmax > fmaxmin2));
        soundvalid2 = ((fmax2 > fmaxmin1)&&(fmax2 < fmaxmax1)) || ((fmax2 < fmaxmax2)&&(fmax2 > fmaxmin2))||((fmax2 < fmax2max)&&(fmax2 > fmax2min));
        whistlevalid = ( (fmax - fmax2) < whistleDifference )&&( whistlefmaxmin< fmax)&&(fmax < whistlefmaxmax) && (whistlefmaxmin < fmax2)&&(fmax2 < whistlefmaxmax);

        if(soundvalid && soundvalid2 && Rmax > soundmaxValue)
            {
                validwhistlecount = 0;
                validsoundcount = validsoundcount +1;
                if(validsoundcount == validsoundcountValue)
                {
                    restcount = 5;
                    validsoundcount = 0;
                    //ttsProxy.say("Sound Detected");

                    targetAngles = AL::ALValue::array(1.3f, -0.0f, -0.0f, 1.3f);
                    targetTimes = AL::ALValue::array(0.5f, 1.0f, 4.0f, 4.5f);
                    motion.angleInterpolation(jointName1, targetAngles, targetTimes, isAbsolute);
                }
            }
        else if(whistlevalid && Rmax > whistlemaxValue)
            {
                validsoundcount = 0;
                validwhistlecount = validwhistlecount +1;
                if(validwhistlecount == validwhistlecountValue)
                {
                    restcount = 10;
                    validwhistlecount = 0;
                    //ttsProxy.say("whistle Detected");

                    targetAngles = AL::ALValue::array(1.3f, -0.0f, -0.0f, 1.3f);
                    targetTimes = AL::ALValue::array(0.5f, 1.0f, 4.0f, 4.5f);
                    motion.angleInterpolation(jointName2, targetAngles, targetTimes, isAbsolute);
                }
            }
        else
            {
                validsoundcount = 0;
                validwhistlecount = 0;

            }

        finishTime = clock();
        totalTime = (double)(finishTime - startTime);

    }
    else
    {
        restcount = restcount - 1;
        if(isCout)
        {
          std::cout<< "restcount = "<< restcount <<std::endl;
        }

    }

}
