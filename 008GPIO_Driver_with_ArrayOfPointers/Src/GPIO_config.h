/*
 * GPIO_config.h
 *
 *  Created on: Sep 16, 2023
 *      Author: mohamed
 */

#ifndef GPIO_CONFIG_H_
#define GPIO_CONFIG_H_

#define ENABLE          1           /**< Enable macro */
#define DISABLE         0           /**< Disable macro */
#define SET             ENABLE      /**< Set macro */
#define RESET           DISABLE     /**< Reset macro */
#define GPIO_PIN_SET    SET         /**< GPIO Pin set macro */
#define GPIO_PIN_RESET  RESET       /**< GPIO Pin reset macro */
#define FLAG_SET        SET         /**< Flag set macro */
#define FLAG_RESET      RESET       /**< Flag reset macro */

 typedef enum{

    Pin_No_0 =0,
    Pin_No_1 =1,
    Pin_No_2 =2,
    Pin_No_3 =3,
    Pin_No_4 =4,
    Pin_No_5 =5,
    Pin_No_6 =6,
    Pin_No_7 =7,
    Pin_No_8 =8,
    Pin_No_9 =9,
    Pin_No_10 =10,
    Pin_No_11 =11,
    Pin_No_12 =12,
    Pin_No_13 =13,
    Pin_No_14 =14,
    Pin_No_15 =15,

}GPIO_ConfigurePinNum_t;

typedef enum{
    PortA =0,
    PortB =1,
    PortC =2,
    PortD =3,
    PortE =4,
    PortF =5,
    PortG =6,
    PortH =7,
    PortI =8,

    Port_Indices=9
}GPIO_PortNumIndexArr_t;

#endif /* GPIO_CONFIG_H_ */
