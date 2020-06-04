
#include "synchronizer/leader.hpp"
#include "o80_pam/dummy_robot.hpp"

int main()
{

  std::cout << "1\n";
  synchronizer::Leader leader(std::string(DUMMY_PAM_SEGMENT_ID)+std::string("_synchronizer"));
  std::cout << "2\n";
  leader.start_sync();
  std::cout << "3\n";
  
}
