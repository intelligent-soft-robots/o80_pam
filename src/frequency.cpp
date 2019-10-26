
#include "O8O_pam/frequency.hpp"

namespace O8O_pam
{

class Frequency
{
public:
  template<class Archive>
  void serialize(Archive &archive)
  {
    archive( value );
  }
  double value;
};

}
