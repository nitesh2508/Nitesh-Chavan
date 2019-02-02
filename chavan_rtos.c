// RTOS Framework - Fall 2018
// J Losh

// Student Name:
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// 02_rtos.c   Single-file with your project code
// (xx is a unique number that will be issued in class)
// Please do not include .intvecs section in your code submissions
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
uint8_t fieldposition[20];
uint8_t fieldcount = 0;
uint8_t i = 0;
uint8_t count=0;
//uint32_t temp;
uint32_t Stack_address;
struct semaphore *temp;
bool flag = false;
//bool string = true;
bool pre = false;
char s[80];
char buffer[50];
bool PI_ON=true;
uint8_t checkcommand=0;


// REQUIRED: correct these bitbanding references for the off-board LEDs
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4))) // off-board red LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4))) // off-board orange LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) // off-board yellow LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) // off-board green LED
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define PUSH_BUTTON1  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 7*4)))
#define PUSH_BUTTON2  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON3  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON4  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON5  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char name[16];
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;
    void *new_pid;// used to uniquely identify thread
    void *sp;                      // location of stack pointer for thread
    uint8_t priority;              // 0=highest, 15=lowest
    uint8_t currentPriority;       // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    uint16_t taskcount;
    uint16_t skipCount;
    void *semaphore;     // pointer to the semaphore that is blocking the thread
    void *currentSemaphore;
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }

    NVIC_ST_CTRL_R |= 0x00000007;
    NVIC_ST_RELOAD_R = 0x00009C3F;
    NVIC_ST_CURRENT_R |= 0x00000000;
    // REQUIRED: initialize systick for 1ms system timer
}

void rtosStart()
{

    // REQUIRED: add code to call the first task to be run
    pre = true;
    _fn fn;
    taskCurrent = rtosScheduler();
    Stack_address = getsp();
    setsp(tcb[taskCurrent].sp);
    fn = (_fn) tcb[taskCurrent].pid;
    (*fn)();
    // Add code to initialize the SP with tcb[task_current].sp;
}

store_values()
{
    __asm ( " MOV R4, R0  ");
    __asm ( " MOV R5, R1  ");
    __asm ( " MOV R6, R2  ");
}

bool createThread(_fn fn, char name[], int priority)
{
     store_values();
    __asm ( " SVC # 06 ");
}

uint32_t getsp()
{
    __asm("  MOV  R0, R13");
}
void setsp(uint32_t X)
{
    __asm(" MOV R13, R0" );
    //stack address to Stack pointer register
    __asm(" BX LR ");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm ( " SVC # 05 ");
}
// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}

struct semaphore* createSemaphore(uint8_t count,char name1[16])
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
       strcpy( pSemaphore->name,name1);
    }
    return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    //tcb[taskCurrent].state= STATE_READY;
    __asm ( " SVC # 01 ");
    // push registers, call scheduler, pop registers, return to new function

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm ( " SVC # 02 ");
    // push registers, set state to delayed, store timeout, call scheduler, pop registers,
    // return to new function (separate unrun or ready processing)
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm ( " SVC # 03 ");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm ( " SVC # 04 ");
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
   // uint8_t i;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
      if (flag)
      {
      task++;
      if(task >= MAX_TASKS)
      task = 0;
      if (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN)
      {
      if(tcb[task].skipCount==0)
      {
      ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
      tcb[task].skipCount= tcb[task].currentPriority;
      }
      else {
          tcb[task].skipCount--;
          }
     }
  }
      else
      {
         task++;
         if (task >= MAX_TASKS)
           task = 0;
         ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
       }
    }
    return task;
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{

    for (i = 0; i < 10; i++)
    {
        if ((tcb[i].state == STATE_DELAYED))
        {
            tcb[i].ticks--;

            if (tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }

        }
    }
    if(pre)
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    __asm (" PUSH {R4, R5, R6,R7,R8, R9, R10 ,R11} ");
    (tcb[taskCurrent].sp) = getsp();
    setsp(Stack_address);
    taskCurrent = rtosScheduler();
    //setsp(tcb[taskCurrent].sp);
    //(tcb[taskCurrent].sp)=getsp();
    //(tcb[taskCurrent].sp )=setsp();
    //__asm (" POP {R11}, {R10}, {R9},{R8},{R7}, {R6}, {R5} ,{R4} ");
    //if(tcb[i].state =STATE_READY)
    setsp(tcb[taskCurrent].sp);
    if (tcb[taskCurrent].state == STATE_UNRUN)
    {
        stack[taskCurrent][255] = 0x01000000;
        stack[taskCurrent][254] = tcb[taskCurrent].pid;
        stack[taskCurrent][253] = 1;
        stack[taskCurrent][252] = 1;
        stack[taskCurrent][251] = 1;
        stack[taskCurrent][250] = 1;
        stack[taskCurrent][249] = 1;
        stack[taskCurrent][248] = 1;
        stack[taskCurrent][247] = 0xFFFFFFF9;
        stack[taskCurrent][246] = 1;
        stack[taskCurrent][245] = 1;
        stack[taskCurrent][244] = 1;
        stack[taskCurrent][243] = 1;
        stack[taskCurrent][242] = 1;
        stack[taskCurrent][241] = 1;
        stack[taskCurrent][240] = 1;
        stack[taskCurrent][239] = 1;
        stack[taskCurrent][238] = 1;
        __asm (" SUB R13, #0x44 ");
        tcb[taskCurrent].state = STATE_READY;
        tcb[taskCurrent].sp = &stack[taskCurrent][238];
        setsp(tcb[taskCurrent].sp);
    }
    __asm (" POP {R4, R5, R6,R7,R8, R9, R10 ,R11} ");

}
uint32_t getSVC_N()
{
    //__asm (" ADD R13,#8" );
    __asm (" MOV R0 ,R13" );
    // __asm (" ADD R0 ,#24" );
    //__asm (" LDR R0,[R0] ");
    __asm (" ADD R0,#0x40 " );
    __asm (" LDR R0,[R0] " );
    __asm (" SUB R0, #2 ");
    __asm (" LDR R0,[R0] " );
    __asm (" AND R0, #0x00FF ");
}
uint32_t gettime()
 {
 __asm("    MOV R0, R13");
 __asm (" ADD R0 , #0x38 ");
 __asm (" LDR R0 ,[R0] ");
 }
uint32_t get_R0()
{
    /*__asm (" MOV R0, R13 ");
     __asm (" ADD R0 , #48  ");
     __asm (" LDR R0, [R0] ");*/

}
uint16_t r0_value()
      {
          __asm (" MOV R0, R4 ");
       }
uint16_t r1_value()
      {
          __asm (" MOV R0, R5 ");
       }
uint16_t r2_value()
      {
          __asm (" MOV R0, R6 ");
       }
// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives

void svCallIsr()
{
    temp = get_R0();
    uint8_t n;
    n = getSVC_N();

    switch (n)
    {
    case 1:
    {
        tcb[taskCurrent].state = STATE_READY;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    }
    case 2:
    {
        tcb[taskCurrent].ticks = temp;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    }
    case 3:
    {
        //if (temp->count > 0)
        //temp = get_R0();
        if ((temp->count) == 0)
        {
            temp->processQueue[temp->queueSize] = tcb[taskCurrent].pid;
            temp->queueSize++;
            tcb[taskCurrent].state = STATE_BLOCKED;
             //NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            if (PI_ON)
            {
                 int z;
                    for(z=0;z<10;z++)
                    {
                       //tcb[taskCurrent].semaphore=temp;
                       if(temp ==tcb[z].semaphore && tcb[z].state != STATE_BLOCKED)
                       if(tcb[z].currentPriority>tcb[taskCurrent].currentPriority)
                       tcb[z].currentPriority=tcb[taskCurrent].currentPriority;
                    }
            }
        }
        else{
            temp->count--;
            tcb[taskCurrent].semaphore=temp;
            }
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    }
    case 4:
    {
        uint8_t x;
        uint32_t z;
        uint8_t v;
        tcb[taskCurrent].semaphore=0;
        temp->count++;
        if (temp->queueSize > 0)
        {
            //tcb[taskCurrent].pid = temp -> processQueue[0];
            temp->count--;
            z = temp->processQueue[0];
            for (x = 0; x < 10; x++)
            {
                if (tcb[x].pid == z)
                {
                    temp->processQueue[0] = 0;
                    //temp->count--;
                    temp->queueSize--;
                    tcb[x].state = STATE_READY;
                }

            }
          for (x = 0; x< temp->queueSize; x++)
             {
             temp->processQueue[x]= temp->processQueue[x+1];
             }
          for (v=0;v<10;v++)
          {
              if(tcb[v].semaphore == temp && tcb[v].currentPriority!=tcb[v].priority)
              tcb[v].currentPriority= tcb[v].priority;
          }

        }
        else temp -> queueSize ++;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        //tcb[taskCurrent].state = STATE_READY;
        break;
    }
    case 5:
    {
        uint8_t x;
        uint32_t *z =temp;
        for (x = 0; x < 10; x++)
                    {
                        if (tcb[x].pid == z)
                        {
                            tcb[x].state = STATE_INVALID;
                            tcb[x].pid=0;
                            taskCount--;
                        }
                    }
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;
    }
    case 6:
    {
        _fn fn;
        fn =r0_value();
        char *name=r1_value();
        int priority=r2_value();
        bool ok = false;
            uint8_t i = 0;
            bool found = false;
            // REQUIRED: store the thread name
            // add task if room in task list
            if (taskCount < MAX_TASKS)
            {
                // make sure fn not already in list (prevent reentrancy)
                while (!found && (i < MAX_TASKS))
                {
                    found = (tcb[i++].pid == fn);
                }
                if (!found)
                {
                    // find first available tcb record
                    i = 0;
                    while (tcb[i].state != STATE_INVALID)
                    {
                        i++;
                    }
                    tcb[i].state = STATE_UNRUN;
                    tcb[i].pid = fn;
                    tcb[i].new_pid = fn;
                    tcb[i].sp = &stack[i][255];
                    tcb[i].priority = priority;
                    tcb[i].currentPriority = priority;
                    strcpy(tcb[i].name,name);
                   // tcb[i].name = name;
                    // increment task count
                    taskCount++;
                    ok = true;
                }
            }
            // REQUIRED: allow tasks switches again
            return ok;
    }
}
}

void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
    yield();
    return UART0_DR_R & 0xFF;
}
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB
            | SYSCTL_RCGC2_GPIOF;

    // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs & 5 pushbuttons,
    GPIO_PORTF_DIR_R = 0x04;  // bit 2 is output, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x04;
    //GPIO_PORTF_PUR_R = 0x04;  // enable internal pull-up for push button
    // orange ,green ,Yellow  and pushbutton 3 , 4 and 5 .
    GPIO_PORTA_DIR_R = 0xE0;  // bit  is output, other pins are inputs
    GPIO_PORTA_DR2R_R = 0xE0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = 0xFC;
    GPIO_PORTA_PUR_R = 0x1C;  // enable internal pull-up for push button
    // red and push button 6 & 7
    GPIO_PORTB_DIR_R = 0x10;  // bit 3 is output, other pins are inputs
    GPIO_PORTB_DR2R_R = 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R = 0xD0;
    GPIO_PORTB_PUR_R = 0xC0;  // enable internal pull-up for push button

    //  uart intialization
    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                         // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                       // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");
    // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");
    // 6
    __asm("             CBZ  R1, WMS_DONE1");
    // 5+1*3
    __asm("             NOP");
    // 5
    __asm("             B    WMS_LOOP1");
    // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");
    // 1
    __asm("             CBZ  R0, WMS_DONE0");
    // 1
    __asm("             B    WMS_LOOP0");
    // 1*3
    __asm("WMS_DONE0:");
    // ---
    // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    waitMicrosecond(100);
    if (PUSH_BUTTON1 == 0)
        return 1;
    if (PUSH_BUTTON2 == 0)
      return 2;
    if (PUSH_BUTTON3 == 0)
        return 4;
    if (PUSH_BUTTON4 == 0)
        return 8;
    if (PUSH_BUTTON5 == 0)
        return 16;
    else return 0;
}
// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while (true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}
void flash4Hz()
{
    while (true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while (true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(1000);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while (true)
    {
        wait(resource);
        for (i = 0; i < 4000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while (true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            createThread(flash4Hz, "Flash4Hz", 0);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while (true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}



void uncooperative()
{
    while (true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void important()
{
    while (true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}
bool iscommand(char*Verb[], uint8_t minArgs)
{
    if ((strcmp(&s[fieldposition[0]], Verb) == 0) && (fieldcount >= minArgs))
    {
        return true;
    }
    else
    {
        return false;
    }
}
uint32_t getnumber(uint16_t numb)
{
    uint32_t x;
    x= atoi(&s[fieldposition[numb]]);
    return x;
}
void uart()
{
   // int gh=0;
    putsUart0("   ENTER COMMAND   ");

    while (count<=81)
    {
        s[count]=getcUart0();

    //    putcUart0(s[i]);
        if (s[count]==13 || s[count]==10)
        {
            s[count]='\0';
            putsUart0("\r\n");
            break;
        }
        else if (s[count]<32)
        { if (count>=0)
            count--;

        }
        else if(s[count]>=32)
        {
            s[count]=tolower(s[count]);
            count++;
        }
    }

    //putsUart0(s);
    fieldcount=0;
    uint8_t strlength= strlen(s);
    if (s[0]==46||s[0]==45 ||s[0]==38||(s[0]>47 && s[0]<58))
    {
    fieldposition[fieldcount]=0;
    fieldcount++;
    }
    else if((s[0]>96 && s[0]<123))
    {
      fieldposition[fieldcount]=0;
      fieldcount++;
    }
    else {
        s[0]='\0';
    }
    for(i=1;i<=strlength;i++)
    {
        if (!((s[i-1]>96&&s[i-1]<123)||(s[i-1]>47&&s[i-1]<58)||(s[i-1]==46||s[i-1]==45||s[i-1]==38)))
        {
            if (s[i]==46||(s[i]>47 && s[i]<58||s[i]==45||s[i]==38))
            {

            fieldposition[fieldcount]=i;
            fieldcount++;
            //c=s[i];
            }
            else if((s[i]>96 && s[i]<123))

            {

              fieldposition[fieldcount]=i;
              fieldcount++;
              //c=s[i];
            }else {
                   s[i]='\0';
                   //c=s[i];
            }
                   }
        if (!((s[i]>96&&s[i]<123)||(s[i]>47&&s[i]<58)||(s[i]==46||s[i]==45||s[i]==38)))
        {
            s[i]='\0';
          //  c=s[i];
        }
    }
}
void shell()
{
    while(true)
    {
uart();
checkcommand=0;
if(iscommand("setpriority",1))
   {
        if(strcmp(&s[fieldposition[1]],"on") == 0)
           {
            putsUart0(" TRUE PRIORITY ON  ");
            putsUart0("\r\n");
            flag=true;
            }
        else if(strcmp(&s[fieldposition[1]],"off") == 0)
        {
            flag =false;
            putsUart0("  TRUE PRIORITY OFF ");
            putsUart0("\r\n");
        }
        checkcommand=1;
     }
if(iscommand("setpreemption",1))
    {
        if(strcmp(&s[fieldposition[1]],"on") == 0)
           {
            putsUart0(" TRUE PREMPTION ON  ");
            putsUart0("\r\n");
            pre = true;
            }
        else if(strcmp(&s[fieldposition[1]],"off") == 0)
        {
            pre =false;
            putsUart0("  TRUE PREMPTION OFF ");
            putsUart0("\r\n");
        }
        checkcommand=1;
     }
if(iscommand("ps",0))
{
      //uint16_t pid_name1;
      //char name2[16];
     int r;
     putsUart0("TASK NAME");
     putsUart0("\t");
     putsUart0("PID NAME");
     putsUart0("\r\n");
     for(r=0;r<10;r++)
       {

           putsUart0(tcb[r].name);
           if(strcmp(&tcb[r].name,"idle")==0||strcmp(&tcb[r].name,"oneshot")==0 || strcmp(&tcb[r].name,"uncoop")==0 || strcmp(&tcb[r].name,"shell")==0)
           putsUart0("\t\t");
           else putsUart0("\t");
           uint16_t pid_name = tcb[r].pid;
           ltoa(pid_name,buffer);
           putsUart0(buffer);
           putsUart0("\r\n");
       }
     checkcommand=1;
  }
if(iscommand("ipcs",0))
{
    checkcommand=1;
    int check;
    int j;
    putsUart0 (" SEMAPHORE NAME");
    putsUart0 ("\r\n");

    for (j=0;j<5;j++)
    {
           putsUart0 (semaphores[j].name);
           putsUart0 ("\r\n");
     }
           putsUart0 ("Blocked Tasks");
           for (check=0;check<10;check++)
           {
            if(tcb[check].state == STATE_BLOCKED )
            putsUart0 (tcb[check].name);
            putsUart0 ("\r\n");
           }

}
if(strcmp(&s[fieldposition[1]],"&") == 0)
  {
    int t;
    for( t=0;t<10;t++)
    {
    if(strcmp(&s[fieldposition[0]],tcb[t].name) == 0)
    {
    _fn new_fn = tcb[t].new_pid;
    char *name1 = tcb[t].name;
    uint8_t new_prior = tcb[t].priority;
    createThread(new_fn,name1,new_prior);
    }
  }
    checkcommand=1;
  }
if(iscommand("pidof",1))
 {
    int n;
    //char *name1 = getnumber(1);
    for( n=0;n<10;n++)
    {
        if(strcmp(&s[fieldposition[1]],tcb[n].name) == 0)
        {
         uint32_t pid_val= tcb[n].pid;
         ltoa(pid_val,buffer);
         putsUart0(buffer);
         putsUart0("\r\n");
        }
       }
    checkcommand=1;
 }
if(iscommand("kill",1))
 {
     //uint8_t x;
     putsUart0(" PID  ENTERED  ");
     uint32_t Pid = getnumber(1);
     //if(strcmp(&s[fieldposition[1]],"on") == 0)
     uint32_t Pid1= Pid;
     destroyThread(Pid1);
     checkcommand=1;
 }
if(iscommand("reboot",0))
      {
         ResetISR();
         checkcommand=1;
      }
if(iscommand("pi",1))
  {
    if(strcmp(&s[fieldposition[1]],"on") == 0)
               {
                putsUart0(" TRUE PI ON  ");
                putsUart0("\r\n");
                PI_ON = true;
                }
            else if(strcmp(&s[fieldposition[1]],"off") == 0)
            {
                PI_ON =false;
                putsUart0("  TRUE PI OFF ");
                putsUart0("\r\n");
            }
    checkcommand=1;
      }
if (checkcommand==1)
{
    putsUart0 (" VALID COMMAND ");
    putsUart0("\r\n");
}
else if (checkcommand==0)
{
    putsUart0 (" NOT VALID COMMAND ");
    putsUart0("\r\n");
}
int clear=0;
 for (clear=0;clear<81;clear++)
{
    s[clear]=0;
}
for(clear=0;clear<10;clear++)
{
    fieldposition[clear]=0;
    fieldcount=0;
    count=0;
}
    }
//string=false;
 }

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    rtosInit();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1,"keyPresses");
    keyReleased = createSemaphore(0,"keyReleased");
    flashReq = createSemaphore(5,"flashReq");
    resource = createSemaphore(1,"resource");

    // Add required idle process
    ok = createThread(idle, "idle",15); // Add other processes
    ok &= createThread(lengthyFn, "lengthyfn", 12);
    ok &= createThread(flash4Hz, "flash4hz", 4);
    ok &= createThread(oneshot, "oneshot", 4);
    ok &= createThread(readKeys, "readkeys", 12);
    ok &= createThread(debounce, "debounce", 12);
    ok &= createThread(important, "important", 0);
    ok &= createThread(uncooperative, "uncoop", 10);
    ok &= createThread(shell, "shell", 8);

    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}
