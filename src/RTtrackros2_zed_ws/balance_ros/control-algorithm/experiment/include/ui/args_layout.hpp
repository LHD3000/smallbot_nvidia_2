#pragma once
#include "ui/args_inspector.hpp"
class ArgsLayout
{
  public:
    virtual void CreateLayout(ArgsInspector &args_inspector){
        args_inspector.AddLabel("none");
    }

    virtual void PhraseArgs(const std::map<std::string, ArgsInspector::Arg> &args) {
        (void)args;
    }
};