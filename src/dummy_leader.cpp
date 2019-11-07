
#include "synchronizer/leader.hpp"
#include "O8O_pam/dummy_robot.hpp"

int main()
{

  std::cout << "1\n";
  synchronizer::Leader leader(std::string(DUMMY_PAM_SEGMENT_ID)+std::string("_synchronizer"));
  std::cout << "2\n";
  usleep(3000000);
  std::cout << "3\n";
  leader.start_sync();
  std::cout << "4\n";

  for(int i=0;i<4;i++)
    {
      std::cout << "5\n";
      usleep(3000000);
      leader.pulse();
    }
  
}
