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
	
#include "anglequeue.h"
#include <stdio.h>
#include <stdlib.h>
//#include <assert.h>
#include "stm32f10x_conf.h"
#include <string.h>
/*eive Queue*/

#define ARRAY_SIZE 200
/*-------------*/
   
/*
 * the array that stores elements && the pointer that point to the head as well as the tail of the array 
 */
 /* Queue */
  static  ANGLEQUEUE_TYPE Queue[ARRAY_SIZE];
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
  
	
	
  int anglequeue_enqueue(ANGLEQUEUE_TYPE value)
  {
		if(anglequeue_is_full())
				return -1 ; 
    Queue[ front ].angle = value.angle;
		Queue[ front ].time = value.time;
		front = (front + 1) % ARRAY_SIZE ; 
		
				return 0 ; 
  }
  
	int anglequeue_dequeue(ANGLEQUEUE_TYPE *value)
	{
		if(anglequeue_is_empty())
				return -1 ; 
		value->angle = Queue[tail].angle ; 
		value->time  = Queue[tail].time ; 
		
		tail = (tail + 1) % ARRAY_SIZE ; 
		
		return 0 ; 
	}
  
  /**
  * @brief  is_empty return true if Queue empty,otherwise false 
  *         
  * @param  None 
  * @retval None 
  * @note 
  */
  //return 1  empty
	//return 0  noempty
  int anglequeue_is_empty(void)
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
  
  int anglequeue_is_full(void)
  {
  	return (front+1)%ARRAY_SIZE == tail  ; 
  }
	