#include <dlfcn.h>
#include <string>

class LibLoader
{
  private:
    void *handle = nullptr;

  public:
    void Load(std::string libPath);
    void *GetFunction(std::string funcName);
    void Close();
    ~LibLoader();
};
