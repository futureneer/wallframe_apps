#ifndef cpp_example_app_h
#define cpp_example_app_h 

#include <modulair_core/modulair_core.h>
#include <modulair_core/modulair_app_base.h>

namespace modulair{

	class ExampleApp : public ModulairAppBase{
  public:
    ExampleApp(QString app_name, ros::NodeHandle nh, int event_deque_size);
    ~ExampleApp(){};
		bool build();
		bool start();
    bool stop();
    bool pause();
    bool resume();
	};
}

#endif

