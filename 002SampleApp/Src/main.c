/**
 * @file main.c
 * @author Mohamed Ali Haoufa
 * @brief 
 * @version 0.1
 * @date 2023-09-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
// Complete the following function.
//  solved : 11/02/2023

int numbers[]={30,23,39,19,239,22};
int someData = 90;

void calculate_the_maximum(int n, int k)
{
  // Write your code here.
  /* You will be given an integer , and a k threshold,
  for each number i from 1 through n
  find the maximum value of the logical and,or,xor when compared against all integer n that are greater than i. consider a value only if the comparaison returns a result less then k.
      510 448
      446
      447
      447
  */
  int andmaxcomp = 0, ormaxcomp = 0, xormaxcomp = 0, andop, orop, xorop;
  int compt1 = 0, compt2 = 0, compt3 = 0;

  for (int i = 1; i < n; i++)
  {
    int a = i;
    for (int b = i + 1; b <= n; b++)
    {
      andop = a & (b);
      orop = a | (b);
      xorop = a ^ (b);

      if ((andop < k) && (andop > andmaxcomp))
      {
        andmaxcomp = andop;
        compt1++;
      }
      else if (compt1 == 0)
      {
        andmaxcomp = 0;
      }

      if ((orop < k) && (orop > ormaxcomp))
      {
        ormaxcomp = orop;
        compt2++;
      }
      else if (compt2 == 0)
      {
        ormaxcomp = 0;
      }

      if ((xorop < k) && (xorop > xormaxcomp))
      {
        compt3++;
        xormaxcomp = xorop;
      }
      else if (compt3 == 0)
      {
        xormaxcomp = 0;
      }
    }
  }
  // printf("results of the and: %d\n",andmaxcomp);
  // printf("results of the or: %d\n",ormaxcomp);
  // printf("results of the xor: %d\n",xormaxcomp);

  printf(" Hello World HAHAHA :\n");
  printf("%d\n", andmaxcomp);
  printf("%d\n", ormaxcomp);
  printf("%d\n", xormaxcomp);
  numbers[2]=26;
  someData = 10;

  /* this will result to an "Instruction fault exception or/ invalid op-code exception" :
  void (* jump_addr) (void); // function pointer
  jump_addr= (void *) 0x20000009; // pointer point to RAM address
  jump_addr(); // jump to execute in RAM address*/

}

int main(void)
{
  int n=12, k=3;
  //scanf("%d %d", &n, &k);
  calculate_the_maximum(n, k);

  /* Loop forever */
  for(;;);
}
