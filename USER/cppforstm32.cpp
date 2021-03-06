/*! @file cppforstm32.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief Support for printf to USART2 on STM32 platform
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */
#include "cppforstm32.h"
#include "usart.h"

#ifdef DYNAMIC_MEMORY
void * operator new (size_t size)
{
  if(NULL == size)
  {
#ifdef DEBUG
    printf("Error! Size is zero");
#endif//DEBUG
    return NULL;
  }
  void *p = malloc(size);
#ifdef DEBUG
  if(p == 0)
  printf("Lack Memory!");
#endif//DEBUG
  return p;
}

void * operator new [](size_t size)
{
  return operator new(size);
}

void operator delete (void * pointer)
{
  if(NULL != pointer)
  {
    free(pointer);
  }
}

void operator delete[](void * pointer)
{
  operator delete(pointer);
}
#endif //DYNAMIC_MEMORY
