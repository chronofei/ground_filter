#ifndef GROUND_FILTER_TOOLS_TIC_TOC_HPP_
#define GROUND_FILTER_TOOLS_TIC_TOC_HPP_

// C/C++
#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib>
#include <chrono>

namespace ground_filter
{

class TicToc
{
public:
  TicToc()
  {
    tic();
  }

  void toc( std::string about_task )
  {
    std::cout.precision(3); // 10 for sec, 3 for ms 
    std::cout << about_task << ": " << toc() << " msec." << std::endl;
  }

  double toc()
  {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

private:
  void tic()
  {
      start = std::chrono::system_clock::now();
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
    
}; // end class TicToc

}  // end namespace ground_filter

#endif