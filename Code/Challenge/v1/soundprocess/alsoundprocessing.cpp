

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
  jointName = "RShoulderPitch";
  stiffness = 1.0f;
  time = 1.0f;
  isAbsolute = true;
  motion.stiffnessInterpolation(jointName, stiffness, time);
  ttsProxy.say("Detection Begin");
  startDetection();
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

ALSoundProcessing::~ALSoundProcessing()
{
  stiffness = 0.0f;
  time = 1.0f;
  motion.stiffnessInterpolation(jointName, stiffness, time);
  ttsProxy.say("Detection Stop");
  stopDetection();
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

        double data[4098];
        int a;
        int nn;
        int fmax;
        int fmax2;
        double Rmax;
        double Rmax2;
        double R;
        bool soundvalid;
        bool soundvalid2;
        bool whistlevalid;

        for(a = 0; a < 2048; a++)
        {
            data[2*a+1] = buffer[1*a];
        }

        nn = 2048;
        fft(data, nn);

        Rmax =0;
        Rmax2 = 0;
        for(a = 1; a < 1024; a++)
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
        std::cout<< "Rmax = "<< Rmax <<std::endl;
        std::cout<< "fmax = "<< fmax <<std::endl;
        std::cout<< "Rmax2 = "<< Rmax2 <<std::endl;
        std::cout<< "fmax2 = "<< fmax2 <<std::endl;

        soundvalid = ((fmax >= 39)&&(fmax < 44)) || ((fmax < 28)&&(fmax > 24));
        soundvalid2 = (fmax2 >= 24)&&(fmax2 < 150);
        whistlevalid = ( (fmax - fmax2) <= 12 )&&( 400< fmax)&&(fmax < 450) && (400 < fmax2)&&(fmax2 < 450);

        if(soundvalid && soundvalid2 && Rmax > 3000000000)
            {
                validwhistlecount = 0;
                validsoundcount = validsoundcount +1;
                if(validsoundcount == 2)
                {
                    qi::os::sleep(1);
                    restcount = 20;
                    validsoundcount = 0;
                    ttsProxy.say("Sound Detected");

                    targetAngles = AL::ALValue::array(1.3f, -0.0f, -0.0f, 1.3f);
                    targetTimes = AL::ALValue::array(1.0f, 2.0f, 8.0f, 10.0f);
                    motion.angleInterpolation(jointName, targetAngles, targetTimes, isAbsolute);
                }
            }
        else if(whistlevalid && Rmax > 1000000000000)
            {
                validsoundcount = 0;
                validwhistlecount = validwhistlecount +1;
                if(validwhistlecount == 6)
                {
                    qi::os::sleep(1);
                    restcount = 20;
                    validwhistlecount = 0;
                    ttsProxy.say("whistle Detected");

                    targetAngles = AL::ALValue::array(1.0f, -0.5f, -0.5f, 1.0f);
                    targetTimes = AL::ALValue::array(1.0f, 2.0f, 8.0f, 10.0f);
                    motion.angleInterpolation(jointName, targetAngles, targetTimes, isAbsolute);
                }
            }
        else
            {
                validsoundcount = 0;
                validwhistlecount = 0;

            }

        finishTime = clock();
        totalTime = (double)(finishTime - startTime);
        //std::cout<< "totalTime = "<< totalTime <<std::endl;

    }
    else
    {
        restcount = restcount - 1;
        std::cout<< "restcount = "<< restcount <<std::endl;
    }

}
