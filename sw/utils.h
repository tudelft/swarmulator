#ifndef UTILS_H
#define UTILS_H
#include <iostream>
template <typename T>
inline void print(T head){
  std::cout << head << std::endl;
}

template<typename T, typename... Ts>
inline void print(T head, Ts... tail){
  std::cout << head;
  print(tail...);
  // std::cout<<std::endl;
}

#endif