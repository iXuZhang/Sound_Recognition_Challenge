

#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>
#include <alcommon/alproxy.h>
#include "alsoundprocessing.h"

#include <time.h>

#ifdef SOUNDPROCESSING_IS_REMOTE
# define ALCALL
#else
# ifdef _WIN32
#  define ALCALL __declspec(dllexport)
# else
#  define ALCALL
# endif
#endif


#ifdef SOUNDPROCESSING_IS_REMOTE


int main(int argc, char *argv[] )
{
    int pport = 9559;
    std::string pip = "192.168.5.12";

    setlocale(LC_NUMERIC, "C");
    const std::string brokerName = "mybroker";
    int brokerPort = 54000;
    const std::string brokerIp = "0.0.0.0";
    boost::shared_ptr<AL::ALBroker> broker;
    try
    {
      broker = AL::ALBroker::createBroker(
          brokerName,
          brokerIp,
          brokerPort,
          pip,
          pport,
          0

        );
    }
    catch(...)
    {
      std::cerr << "Fail to connect broker to: "
                << pip
                << ":"
                << pport
                << std::endl;

      AL::ALBrokerManager::getInstance()->killAllBroker();
      AL::ALBrokerManager::kill();

      return 1;
    }

    AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(broker);
    AL::ALModule::createModule<ALSoundProcessing>(broker,"ALSoundProcessing");

    qi::os::sleep(3600);

    return 0;
}

#endif

