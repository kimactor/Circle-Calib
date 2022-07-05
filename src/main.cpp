#include <iostream>

#include "process.h"
#include "logger.h"
int main(int argc, char **argv) {
  if (argc >= 2) {
    std::cout << "Received " << argc - 1 << "folder" << std::endl;
    for (int i = 1; i < argc; ++i) {
      std::cout << "Start dealing with folder : " << argv[1]<< std::endl;
      Calibration::Process dodo(argv[i]);
      dodo.compute();
      std::cout << "Finished";
    }
  }
  return 1;
}
