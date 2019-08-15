#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

#include "lwip/err.h"
#include "lwip/netif.h"

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */
#define NETIF_MTU								      ( 1500 )

#define NETIF_IN_TASK_STACK_SIZE			( 1024 )
#define NETIF_IN_TASK_PRIORITY			  ( 3 )

#define NETIF_OUT_TASK_STACK_SIZE			( 1024 )
#define NETIF_OUT_TASK_PRIORITY			  ( 3 )

/* USER CODE END 0 */


/* Exported functions ------------------------------------------------------- */
err_t ethernetif_init(struct netif *netif);

err_t ethernetif_input(struct netif *netif);
void ethernetif_output( void *argument );
void ethernetif_update_config(struct netif *netif);
void ethernetif_notify_conn_changed(struct netif *netif);

//u32_t sys_jiffies(void);
//u32_t sys_now(void);

#endif //_ETHERNETIF_H__
