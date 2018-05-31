/**
  ****************************************************************************** 
  * @作者    叶以亮  
  * @版本    V1.0.0
  * @日期    8-9-2017
  * @概要    a Queue that make through static array,the array size can change by define again 
  *            this is a circular Queue!!
  ******************************************************************************
  * @注意事项
  ******************************************************************************  
  */ 
	
#include "speedqueue.h"
#include <stdio.h>
#include <stdlib.h>
//#include <assert.h>
#include "stm32f10x_conf.h"
#include <string.h>
/*eive Queue*/

#define ARRAY_SIZE  100
/*-------------*/
   
/*
 * the array that stores elements && the pointer that point to the head as well as the tail of the array 
 */
 /* Queue */
   static  SPEEDQUEUE_TYPE Queue[ARRAY_SIZE];
   static  size_t front =0;
   static  size_t tail =0;
 /*---------------*/
  
 /**
  * @brief  insert 
  *         
  * @param  value the element what is needed to be added
  * @retval None
  * @note the function is only suitably for the Queue that creat dynamically
  */
  
	 int speedqueue_enqueue(SPEEDQUEUE_TYPE value)
  {
		if(speedqueue_is_full())
				return -1 ; 
    Queue[ front ].speed = value.speed;
		front = (front + 1) % ARRAY_SIZE; 	
				return 0 ; 
  }
  
	int speedqueue_dequeue(SPEEDQUEUE_TYPE *value)
	{
		if(speedqueue_is_empty())
				return -1 ; 
		value->speed = Queue[tail].speed; 		
		tail = (tail + 1) % ARRAY_SIZE; 		
		return 0; 
	}
	
//  void speedqueue_insert(SPEEDQUEUE_TYPE value)
//  {
//  	//assert( !is_full());
//		assert_param(!speedqueue_is_full());
//  	rear = (rear +1)%ARRAY_SIZE;
//    Queue[ rear ].speed = value.speed;
//		//printf("%d %d\r\n",Queue[ rear ].speed,rear);
//		///strcpy(Queue[ rear ].str,value.str);
//		
//  }
//  
//  
//  void speedqueue_delete(void)
//  {
//      //assert(!is_empty());
//		  assert_param(!speedqueue_is_empty());
//      front = (front +1)%ARRAY_SIZE;
//  }
  
 
  //return 1  empty
	//return 0  noempty
  int speedqueue_is_empty(void)
  {
       return tail == front ; 
  }
  /**
  * @brief  if_full return true if the Queue full,otherwise false 
  *         
  * @param  None 
  * @retval None 
  * @note 
  */
	//return 1  full
	//return 0  no full
  
  int speedqueue_is_full(void)
  {
  	return (front+1)%ARRAY_SIZE == tail ; 
  }
	
	
//  SPEEDQUEUE_TYPE speedqueue_first (void)
//  {
//      //assert(!is_empty());
//		  assert_param(!speedqueue_is_empty());
//		  //printf("%d\r\n",front);
//      return Queue[front];
//  }
//  
//  
//  int speedqueue_is_empty(void)
//  {
//       return ((rear +1)%ARRAY_SIZE) == front;
//  }
//	
//  
//  int speedqueue_is_full(void)
//  {
//  	return (rear +2)%ARRAY_SIZE == front;
//  }