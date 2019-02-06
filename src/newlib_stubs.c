//#include <errno.h>
//#include <sys/stat.h>
//#include <sys/times.h>
//#include <sys/unistd.h>
//#include <stdint.h>
//#include "libopencmsis/core_cm3.h"
#include "newlib_stubs.h"
//#include <stm32f10x.h>

//#include "hw_config.h"
/* define compiler specific symbols */

//#if defined ( __CC_ARM   )
//  #define __ASM            __asm                                      /*!< asm keyword for ARM Compiler          */
//  #define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler       */
//
//#elif defined ( __ICCARM__ )
//  #define __ASM           __asm                                       /*!< asm keyword for IAR Compiler          */
//  #define __INLINE        inline                                      /*!< inline keyword for IAR Compiler. Only avaiable in High optimization mode! */
//
//#elif defined   (  __GNUC__  )
//  #define __ASM            __asm                                      /*!< asm keyword for GNU Compiler          */
//  #define __INLINE         inline                                     /*!< inline keyword for GNU Compiler       */
//
//#elif defined   (  __TASKING__  )
//  #define __ASM            __asm                                      /*!< asm keyword for TASKING Compiler      */
//  #define __INLINE         inline                                     /*!< inline keyword for TASKING Compiler   */
//
//#endif

//#if defined ( __CC_ARM   ) /*------------------RealView Compiler -----------------*/
//extern uint32_t __get_MSP(void);
/**
 * @brief  Return the Main Stack Pointer
 *
 * @return Main Stack Pointer
 *
 * Return the current value of the MSP (main stack pointer)
 * Cortex processor register
 */
//__ASM uint32_t __get_MSP(void)
//{
//  mrs r0, msp
//  bx lr
//}
/*
__INLINE uint32_t __get_MSP(void)
{
//	register bool old;
//	__asm__ __volatile__ ("MRS %0, FAULTMASK"  : "=r" (old));
  register uint32_t __regMainStackPointer;
  __asm__ __volatile__ ("mrs %0, msp" : "=r" (__regMainStackPointer));
  return(__regMainStackPointer);
}
*/
//#endif


#define STDOUT_USART USART1
#define STDERR_USART USART1
#define STDIN_USART USART1

// errno
#undef errno
extern int errno;

/*
 Переменные среды - пустой список.
 */
char *__env[1] = { 0 };
char **environ = __env;

int _write(int file, char *ptr, int len);

// exit - экстренный выход. В качестве выхода - зацикливаемся.
void _exit(int status)
{
    while (1);
}

// close - закрытие файла - возвращаем ошибку 
int _close(int file)
{
    return -1;
}
/*
 execve - передача управления новому процессу - процессов нет -> возвращаем ошибку.
 */
//int _execve(char *name, char **argv, char **env)
//{
//    errno = ENOMEM;
//    return -1;
//}

/*
 fork = создание нового процесса
 */
//int _fork()
//{
//    errno = EAGAIN;
//    return -1;
//}

/*
 fstat - состояние открытого файла
 */
//int _fstat(int file, struct stat *st)
//{
//    st->st_mode = S_IFCHR;
//    return 0;
//}

/*
 getpid - получить ID текущего процесса
 */

int _getpid()
{
    return 1;
}

/*
 isatty - является ли файл терминалом.
 */
//int _isatty(int file)
//{
//    switch (file)
//    {
//    case STDOUT_FILENO:
//    case STDERR_FILENO:
//    case STDIN_FILENO:
//        return 1;
//    default:
//        //errno = ENOTTY;
//        errno = EBADF;
//        return 0;
//    }
//}

/*
 kill - послать сигнал процессу
 */
//int _kill(int pid, int sig)
//{
//    errno = EINVAL;
//    return (-1);
//}

/*
 link - устанвить новое имя для существующего файла.
 */

//int _link(char *old, char *new)
//{
//    errno = EMLINK;
//    return -1;
//}

/*
 lseek - установить позицию в файле
 */
//int _lseek(int file, int ptr, int dir)
//{
//    return 0;
//}

/*
 sbrk - увеличить размер области данных, использутся для malloc
 */
//caddr_t _sbrk(int incr)
/*
void * _sbrk(int incr)
{
//    extern char _end;
    extern char     _ebss;
//    extern char _ebss;
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0)
    {
//        heap_end = &_end;
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

//    char * stack = (char*) __get_MSP();
    char * stack = (char*) __asm__("MSP");

    if (heap_end + incr > stack)
    {
//        _write(STDERR_FILENO, "Heap and stack collision\n", 25);
//        errno = ENOMEM;
//        return (caddr_t) -1;
    	return (void *)-1;
        //abort ();
    }

    heap_end += incr;
//    return (caddr_t) prev_heap_end;
    return (void *) prev_heap_end;

}
*/

/*
 read - чтение из файла, у нас пока для чтения есть только stdin
 */

//int _read(int file, char *ptr, int len)
//{
//    int n;
//    int num = 0;
//    switch (file)
//    {
//    case STDIN_FILENO:
//        for (n = 0; n < len; n++)
//        {
////            while (USART_GetFlagStatus(STDIN_USART, USART_FLAG_RXNE) == RESET);
////            char c = (char) (USART_ReceiveData(STDIN_USART) & (uint16_t) 0x01FF);
////            *ptr++ = c;
//            num++;
//        }
//        break;
//    default:
//        errno = EBADF;
//        return -1;
//    }
//    return num;
//}

/*
 stat - состояние открытого файла.
 */

//int _stat(const char *filepath, struct stat *st)
//{
//    st->st_mode = S_IFCHR;
//    return 0;
//}

/*
 times - временная информация о процессе (сколько тиков: системных, процессорных и т.д.)
 */

//clock_t _times(struct tms *buf)
//{
//    return -1;
//}

/*
 unlink - удалить имя файла.
 */
//int _unlink(char *name)
//{
//    errno = ENOENT;
//    return -1;
//}

/*
 wait - ожидания дочерних процессов
 */
//int _wait(int *status)
//{
//    errno = ECHILD;
//    return -1;
//}

/*
 write - запись в файл - у нас есть только stderr/stdout
 */
//int _write(int file, char *ptr, int len)
//{
//    int n;
//    switch (file)
//    {
//    case STDOUT_FILENO: /*stdout*/
//        for (n = 0; n < len; n++)
//        {
////        	USB_Send_Data((uint8_t) * ptr++);
//        }
//        break;
//    case STDERR_FILENO: /* stderr */
//        for (n = 0; n < len; n++)
//        {
////        	USB_Send_Data((uint8_t) * ptr++);
//        }
//        break;
//    default:
//        errno = EBADF;
//        return -1;
//    }
//    return len;
//}
