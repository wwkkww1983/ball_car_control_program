/*the file is written by dr.ye*/

/* Define to prevent ursive inclusion -------------------------------------*/
#ifndef __SPEED_QUEUE_H
#define __SPEED_QUEUE_H

#include <stdlib.h>

//typedef unsigned char QUEUE_TYPE;//8 

typedef struct
{
  unsigned int speed;
	
}SPEEDQUEUE_TYPE;

/**
  * @brief  create queue 
  *          
  * @param  size the max number of the queue that could store
  * @retval None
  * @note the function is only suitably for the queue that creat dynamically
  */
  void speed_create_queue(size_t size);
  
  /**
  * @brief  destroy_queue
  *         
  * @param  None
  * @retval None
  * @note the function is only suitably for the queue that creat dynamically
  */
  
  void speed_destroy_queue(void);
  
  /**
  * @brief  insert 
  *         
  * @param  value the element what is needed to be added
  * @retval None
  * @note the function is only suitably for the queue that creat dynamically
  */
  
  int speedqueue_enqueue(SPEEDQUEUE_TYPE value);
  
 
  int speedqueue_dequeue(SPEEDQUEUE_TYPE *value);
  
  /**
  * @brief  is_empty return true if queue empty,otherwise false 
  *         
  * @param  None 
  * @retval None 
  * @note 
  */
  
  int speedqueue_is_empty(void);
  
  /**
  * @brief  if_full return true if the queue full,otherwise false 
  *         
  * @param  None 
  * @retval None 
  * @note 
  */
  
  int speedqueue_is_full(void);
  
#endif