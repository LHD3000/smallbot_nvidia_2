#include "lib_loader.hpp"
#include <stdexcept>

void LibLoader::Load(std::string libPath)
{
    if (handle)
    {
        Close();
    }
    handle = dlopen(libPath.c_str(), RTLD_LAZY);
    if (!handle)
    {
        throw std::runtime_error(dlerror());
    }
}

void *LibLoader::GetFunction(std::string funcName)
{
    void *func = dlsym(handle, funcName.c_str());
    const char *dlsym_error = dlerror();
    if (dlsym_error)
    {
        throw std::runtime_error("Cannot load symbol '" + funcName + "': " + std::string(dlsym_error));
    }
    return func;
}

void LibLoader::Close()
{
    if (handle)
    {
        dlclose(handle);
        handle = nullptr;
    }
}

LibLoader::~LibLoader()
{
    Close();
}