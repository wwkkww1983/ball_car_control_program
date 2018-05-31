/*the file is written by dr.ye*/

/* Define to prevent ursive inclusion -------------------------------------*/
#ifndef __ANGLE_QUEUE_H
#define __ANGLE_QUEUE_H

#include <stdlib.h>
#include <stdint.h>

//typedef unsigned char QUEUE_TYPE;//8 

typedef struct
{
  unsigned int angle;
  uint16_t time;
}ANGLEQUEUE_TYPE;


	int anglequeue_enqueue(ANGLEQUEUE_TYPE value);

	int anglequeue_dequeue(ANGLEQUEUE_TYPE * value) ; 


 
  int anglequeue_is_empty(void);
  
  /**
  * @brief  if_full return true if the queue full,otherwise false 
  *         
  * @param  None 
  * @retval None 
  * @note 
  */
  
  int anglequeue_is_full(void);
  
#endif