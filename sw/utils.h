#ifndef UTILS_H
#define UTILS_H
#include <iostream>


template <typename T>
inline void print(const T head){
  // std::cout<<
  std::cout << std::setprecision(4)<<head << std::endl;
}

template<typename T, typename... Ts>
inline void print(const T head, const Ts... tail){
  std::cout << head;
  print(tail...);
  // std::cout<<std::endl;
}

#endif