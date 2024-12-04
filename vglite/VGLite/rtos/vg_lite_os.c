#include <zephyr/kernel.h>
#include "vg_lite_os.h"
// #include "FreeRTOS.h"
// #include "semphr.h"
#if !defined(VG_DRIVER_SINGLE_THREAD)
// #include "task.h"
// #include "queue.h"
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */
#include "vg_lite_hw.h"
#include "vg_lite_hal.h"
/* If bit31 is activated this indicates a bus error */
#define IS_AXI_BUS_ERR(x) ((x) & (1U << 31))
#if !defined(VG_DRIVER_SINGLE_THREAD)
#define ISR_WAIT_TIME 0x1FFFF
#define MAX_MUTEX_TIME 10
#define TASK_WAIT_TIME 20
#define configTICK_RATE_HZ    ( ( uint32_t ) 1000)
#define portTICK_PERIOD_MS    ( ( uint32_t ) 1000 / configTICK_RATE_HZ )
/* command queue task parameter */
#define QUEUE_TASK_NAME "queue_task"
#ifndef QUEUE_TASK_PRIO
#define QUEUE_TASK_PRIO (2)
#endif /* QUEUE_TASK_PRIO */
#define QUEUE_TASK_SIZE 8096
#define QUEUE_LENGTH 8
#define MAX_QUEUE_WAIT_NUM 10
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TURE
#define TURE 1
#endif
K_THREAD_STACK_DEFINE(my_stack_area, QUEUE_TASK_SIZE);
typedef struct vg_lite_queue
{
    uint32_t cmd_physical;
    uint32_t cmd_offset;
    uint32_t cmd_size;
    vg_lite_os_async_event_t *event;
} vg_lite_queue_t;
typedef struct vg_lite_os
{
    struct k_thread task_hanlde;
    struct k_msgq queue_handlestructer;
} vg_lite_os_t;

K_MSGQ_DEFINE(queue_handle, sizeof(vg_lite_queue_t), 10, 1);

static struct k_mutex mutex;
static vg_lite_os_t os_obj = {0};
struct k_sem semaphore[8];
struct k_sem command_semaphore;
uint32_t curContext;
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */
struct k_sem int_queue;
volatile uint32_t int_flags;

// K_SEM_DEFINE(command_semaphore, 0, 30);
void __attribute__((weak)) vg_lite_bus_error_handler()
{
    /*
     * Default implementation of the bus error handler does nothing. Application
     * should override this handler if it requires to be notified when a bus
     * error event occurs.
     */
    return;
}
#if !defined(VG_DRIVER_SINGLE_THREAD)
/* command queue function */
void command_queue(void *parameters)
{
    vg_lite_queue_t *peek_queue;
    uint32_t even_got;
    long ret;
   
    k_sem_init(&command_semaphore, 0, 30);    

    while (1)
    {
        even_got = 0;
        if (k_msgq_num_used_get(&queue_handle) != 0)
        {
            ret = k_msgq_get(&queue_handle, peek_queue, K_NO_WAIT);
            if (ret == 0)
            {
                vg_lite_hal_poke(VG_LITE_HW_CMDBUF_ADDRESS, peek_queue->cmd_physical + peek_queue->cmd_offset);
                vg_lite_hal_poke(VG_LITE_HW_CMDBUF_SIZE, (peek_queue->cmd_size + 7) / 8);
                if (vg_lite_hal_wait_interrupt(ISR_WAIT_TIME, (uint32_t)~0, &even_got))
                {
                    peek_queue->event->signal = VG_LITE_HW_FINISHED;
                }
                else
                {
                    /* wait timeout */
                    peek_queue->event->signal = VG_LITE_IDLE;
                }

                if (k_sem_count_get(&semaphore[peek_queue->event->semaphore_id]))
                {
                    k_sem_give(&semaphore[peek_queue->event->semaphore_id]);
                }
            }
        }
        else
        {
            k_msleep(10);
        }
    }
}

__thread void *tlsGlob;
int32_t vg_lite_os_set_tls(void *tls)
{
    if (tls == NULL)
        return VG_LITE_INVALID_ARGUMENT;

    tlsGlob = tls;
    return VG_LITE_SUCCESS;
}
void *vg_lite_os_get_tls()
{
    return tlsGlob;
}
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */
void *vg_lite_os_malloc(uint32_t size)
{
    return k_malloc(size);
}
void vg_lite_os_free(void *memory)
{
    k_free(memory);
}
#if !defined(VG_DRIVER_SINGLE_THREAD)
void vg_lite_os_reset_tls()
{
    tlsGlob = NULL;
}
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */
void vg_lite_os_sleep(uint32_t msec)
{
    k_msleep(msec);
}
int32_t vg_lite_os_initialize(void)
{
#if !defined(VG_DRIVER_SINGLE_THREAD)
    static int task_number = 0;
    k_tid_t my_tid;
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */

    k_sem_init(&int_queue, 0, 1);
    int_flags = 0;

    k_mutex_init(&mutex);

    my_tid = k_thread_create(&os_obj.task_hanlde, my_stack_area,
                             K_THREAD_STACK_SIZEOF(my_stack_area),
                             command_queue,
                             NULL, NULL, NULL,
                             QUEUE_TASK_PRIO, 0, K_NO_WAIT);

    k_thread_name_set(my_tid, QUEUE_TASK_NAME);
    // struct k_timer my_timer;
    // k_timer_init(&my_timer, my_timer_handler, NULL);
    // k_timer_start(&my_timer, K_MSEC(30), K_MSEC(30));

    return VG_LITE_SUCCESS;
}
void vg_lite_os_deinitialize(void)
{
    // do nothing...
}
#if !defined(VG_DRIVER_SINGLE_THREAD)
int32_t vg_lite_os_lock()
{
    if (&mutex == NULL)
    {
        return VG_LITE_NOT_SUPPORT;
    }

    if (k_mutex_lock(&mutex, K_MSEC(5)) == 1)
    {
        return VG_LITE_MULTI_THREAD_FAIL;
    }
    return VG_LITE_SUCCESS;
}
int32_t vg_lite_os_unlock()
{
    if (k_mutex_unlock(&mutex) == 1)
        return VG_LITE_MULTI_THREAD_FAIL;
    return VG_LITE_SUCCESS;
}
int32_t vg_lite_os_submit(uint32_t context, uint32_t physical, uint32_t offset, uint32_t size, vg_lite_os_async_event_t *event)
{
    vg_lite_queue_t *queue_node;
    if (&queue_handle == NULL)
        return VG_LITE_NOT_SUPPORT;
    queue_node = (vg_lite_queue_t *)vg_lite_os_malloc(sizeof(vg_lite_queue_t));
    if (queue_node == NULL)
        return VG_LITE_MULTI_THREAD_FAIL;
    queue_node->cmd_physical = physical;
    queue_node->cmd_offset = offset;
    queue_node->cmd_size = size;
    queue_node->event = event;
    /* Current command buffer has been sent to the command queue. */
    event->signal = VG_LITE_IN_QUEUE;

    int ret = k_msgq_put(&queue_handle, queue_node, K_MSEC(5));

    if (queue_node == NULL)
    {
        return VG_LITE_MULTI_THREAD_FAIL;
    }
    curContext = context;

    if (vg_lite_os_wait_event(event) == VG_LITE_SUCCESS)
    {
        k_sem_give(&command_semaphore);
        return VG_LITE_SUCCESS;
    }
    return VG_LITE_MULTI_THREAD_FAIL;
}
int32_t vg_lite_os_wait(uint32_t timeout, vg_lite_os_async_event_t *event)
{
    if (k_sem_take(&semaphore[event->semaphore_id], K_MSEC(5)) == 0)
    {
        if (event->signal == VG_LITE_HW_FINISHED)
        {
            k_sem_give(&semaphore[event->semaphore_id]);
            return VG_LITE_SUCCESS;
        }
        else
        {
            k_sem_give(&semaphore[event->semaphore_id]);
            return VG_LITE_TIMEOUT;
        }
    }
    return VG_LITE_TIMEOUT;
}
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */
void vg_lite_os_IRQHandler(void)
{
    uint32_t flags = vg_lite_hal_peek(VG_LITE_INTR_STATUS);
    long xHigherPriorityTaskWoken = 0;

    if (flags)
    {
        /* Combine with current interrupt flags. */
        int_flags |= flags;
        /* Wake up any waiters. */
        if (&int_queue)
        {            
            k_sem_give(&int_queue);
            if(xHigherPriorityTaskWoken != 0 )
            {
                k_yield();
            }
        }
    }
}
int32_t vg_lite_os_wait_interrupt(uint32_t timeout, uint32_t mask, uint32_t *value)
{
#if _BAREMETAL
    uint32_t int_status = 0;
    int_status = vg_lite_hal_peek(VG_LITE_INTR_STATUS);
    (void)value;
    while (int_status == 0)
    {
        int_status = vg_lite_hal_peek(VG_LITE_INTR_STATUS);
        usleep(1);
    }
    if (IS_AXI_BUS_ERR(*value))
    {
        vg_lite_bus_error_handler();
    }
    return 1;
#else /*for rt500*/
    if (&int_queue)
    {
        k_sem_take(&int_queue, K_MSEC(5));
        if (value != NULL)
        {
            *value = int_flags & mask;
            if (IS_AXI_BUS_ERR(*value))
            {
                vg_lite_bus_error_handler();
            }
        }
        int_flags = 0;
        return 1;
    }
    return 0;
#endif
}
#if !defined(VG_DRIVER_SINGLE_THREAD)
int32_t vg_lite_os_init_event(vg_lite_os_async_event_t *event,
                              uint32_t semaphore_id,
                              int32_t state)
{
    if (event->semaphore_id >= TASK_LENGTH)
    {
        return VG_LITE_INVALID_ARGUMENT;
    }

    if (k_sem_init(&semaphore[semaphore_id], 0, 1))
    {
        return VG_LITE_OUT_OF_MEMORY;
    }

    k_sem_give(&semaphore[semaphore_id]);
    event->semaphore_id = semaphore_id;
    event->signal = state;
    return VG_LITE_SUCCESS;
}
int32_t vg_lite_os_delete_event(vg_lite_os_async_event_t *event)
{
    if (event->semaphore_id >= TASK_LENGTH)
        return VG_LITE_INVALID_ARGUMENT;

    return VG_LITE_SUCCESS;
}
int32_t vg_lite_os_wait_event(vg_lite_os_async_event_t *event)
{
    if (event->semaphore_id >= TASK_LENGTH)
        return VG_LITE_INVALID_ARGUMENT;
    if(k_sem_take(&semaphore[event->semaphore_id], K_MSEC(5)) != 0)
    {
        return VG_LITE_MULTI_THREAD_FAIL;
    }
    return VG_LITE_SUCCESS;
}
int32_t vg_lite_os_signal_event(vg_lite_os_async_event_t *event)
{
    if (event->semaphore_id >= TASK_LENGTH)
        return VG_LITE_INVALID_ARGUMENT;
    k_sem_give(&semaphore[event->semaphore_id]);
    return VG_LITE_SUCCESS;
}
int8_t vg_lite_os_query_context_switch(uint32_t context)
{
    if (!curContext || curContext == context)
        return FALSE;
    return TURE;
}
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */