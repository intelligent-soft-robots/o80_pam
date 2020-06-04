#pragma once

#include <memory>
#include "pam_interface/configuration.hpp"
#include "pam_interface/dummy/driver.hpp"
#include "pam_interface/real/driver.hpp"
#include "o80_pam/standalone.hpp"

#define SEGMENT_ID_SUFFIX "_o80_pam"
#define OBJECT_ID "o80_pam"

namespace o80_pam
{

  typename std::shared_ptr<Standalone<QUEUE_SIZE,NB_ACTUATORS>> StandalonePtr;

  namespace internal
  {
  
    template<class Driver>
    StandalonePtr get_standalone(std::string type,
				 int id,
				 const pam_interface::Configuration<NB_DOFS> &config)
    {
      std::string segment_id = std::string(SEGMENT_ID_SUFFIX)
	+ type +std::to_string(id);
      std::string object_id = std::string(OBJECT_ID)+std::to_string(id);
    
      std::shared_ptr<Driver> driver =
	std::make_shared<Driver>(config)
	return std::make_shared<Standalone<QUEUE_SIZE,NB_ACTUATORS>>(driver,
								     config.server_frequency,
								     segment_id,
								     object_id);
    
    }

    template<int QUEUE_SIZE, int NB_DOFS, class Driver>
    void run_standalone(std::string type,
			int id,
			const pam_interface::Configuration<NB_DOFS> &config,
			bool bursting,
			std::atomic<bool> &external_running)
    {
      StandalonePtr standalone =
	internal::get_standalone<pam_interface::RealDriver>(type,
							    id,config);
      standalone->start();

      bool internal_running = true;
      
      while ( external_running && internal_running)
	{
	  internal_running = standalone->spin(bursting);
	}

      standalone->stop();
    }
    
    
  }

  
  template<int QUEUE_SIZE, int NB_DOFS>
  StandalonePtr get_dummy(int id,
			  const pam_interface::Configuration<NB_DOFS> &config)
  {
    return internal::get_standalone<pam_interface::DummyDriver>("_dummy_",
								id,config);
  }

  template<int QUEUE_SIZE, int NB_DOFS>
  StandalonePtr get_real(int id,
			 const pam_interface::Configuration<NB_DOFS> &config)
  {
    return internal::get_standalone<pam_interface::RealDriver>("_real_",
								id,config);
  }

  template<int QUEUE_SIZE, int NB_DOFS>
  void run_dummy_standalone(int id,
			    const pam_interface::Configuration<NB_DOFS> &config,
			    bool bursting,
			    std::atomic<bool> &running)
  {
    internal::run_standalone<QUEUE_SIZE,
			     NB_DOFS,
			     pam_interface::DummyDriver>("_dummy_",
							 id,
							 config,
							 bursting,
							 running);
  }

  template<int QUEUE_SIZE, int NB_DOFS>
  void run_real_standalone(int id,
			   const pam_interface::Configuration<NB_DOFS> &config,
			   bool bursting,
			   std::atomic<bool> &running)
  {
    internal::run_standalone<QUEUE_SIZE,
			     NB_DOFS,
			     pam_interface::RealDriver>("_real_",
							id,
							config,
							bursting,
							running);
  }

  
};
