
#include "o80_pam/dummy_robot.hpp"
#include "synchronizer/follower.hpp"

int main()
{
    synchronizer::Follower follower(
        std::string(DUMMY_PAM_SEGMENT_ID) + std::string("_synchronizer"), 100);

    int count = 0;

    while (true)
    {
        usleep(20000);

        count++;
        for (int c = 0; c < count; c++)
        {
            std::cout << "-";
        }
        if (count > 9) count = 0;
        std::cout << "\n";

        follower.pulse();
    }
}
