#include <sstream>
#include <vector>

class InputParser{
public:
  InputParser (int &argc, char **argv)
  {
    for (int i=1; i < argc; ++i)
      this->tokens.push_back(std::string(argv[i]));
  }

  template<typename T>
  bool getCmdOption(const std::string &option, T& ret) const
  {
    std::stringstream ss;
    auto itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
    if (itr != this->tokens.end() && ++itr != this->tokens.end())
    {
      ss << *itr;
      ss >> ret;
      return true;
    }
    return false;
  }

  bool cmdOptionExists(const std::string &option) const
  {
    return std::find(this->tokens.begin(), this->tokens.end(), option)
        != this->tokens.end();
  }

private:
  std::vector <std::string> tokens;
};
