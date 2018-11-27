
#include "cmsis_os.h"  // CMSIS RTOS header file
#include "Board_LED.h"
#include "UART_driver.h"
#include "stdint.h"                     // data type definitions
#include "stdio.h"                      // file I/O functions
#include "rl_usb.h"                     // Keil.MDK-Pro::USB:CORE
#include "rl_fs.h"                      // Keil.MDK-Pro::File System:CORE
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"
#include "math.h"
#include "arm_math.h" // header for DSP library
#include <stdio.h>

// LED constants
#define LED_Green   0
#define LED_Orange  1
#define LED_Red     2
#define LED_Blue    3

#define Show_Files_char "1"
#define Receive_Name_char "4"
#define Play_File_char "5"
#define Pause_File_char "6"

#define NUM_CHAN	2 // number of audio channels
#define NUM_POINTS 1024 // number of points per channel
#define BUF_LEN NUM_CHAN*NUM_POINTS // length of the audio buffer
/* buffer used for audio play */
int16_t Audio_Buffer[BUF_LEN];
int16_t Audio_Buffer2[BUF_LEN];
char rx_name[20];
int paused;
int selected;

enum commands{
	ListFiles,
	SendComplete,
	SendFiles,
	PlaySong,
	SongEnd,
	PauseSong
};


////////////////////////////////////////////////
// State Machine definitions
enum state{
  NoState,
  Idle,
  List,
	Play,
	Pause
};

enum Selected_Commands{
	Song_Selected
};

//////////////////////////////////////////////////////////
void Control (void const *argument); // thread function
osThreadId tid_Control; // thread id
osThreadDef (Control, osPriorityNormal, 1, 0); // thread object

void FS (void const *arg);    // thread function                       // function prototype for Thread_1
osThreadId tid_FS;
osThreadDef (FS, osPriorityNormal, 1, 0);       // define FS object

osSemaphoreDef (SEM0);    // Declare semaphore
osSemaphoreId  (SEM0_id); // Semaphore ID

// Command queue from Rx_Command to Controller
osMessageQId mid_CMDQueue; // message queue for commands to Thread
osMessageQDef (CMDQueue, 1, uint32_t); // message queue object

// Command queue from Controller to FS Thread
osMessageQId mid_Command_FSQueue; // message queue for commands to Thread
osMessageQDef (Command_FSQueue, 1, uint32_t); // message queue object

// Queue to communicate DMA with Thread_1
osMessageQId mid_DMAQueue; // message queue for commands to Thread
osMessageQDef (DMAQueue, 1, uint32_t); // message queue object

// Command queue from Controller Selected_Thread to tell the system a song has been selected
osMessageQId mid_Selected_Queue; // message queue for commands to Thread
osMessageQDef (Selected_Queue, 1, uint32_t); // message queue object

// UART receive thread
void Rx_Command (void const *argument);  // thread function
osThreadId tid_RX_Command;  // thread id
osThreadDef (Rx_Command, osPriorityNormal, 1, 0); // thread object

// WAVE file header format
typedef struct WAVHEADER {
	unsigned char riff[4];						// RIFF string
	uint32_t overall_size;				// overall size of file in bytes
	unsigned char wave[4];						// WAVE string
	unsigned char fmt_chunk_marker[4];		// fmt string with trailing null char
	uint32_t length_of_fmt;					// length of the format data
	uint16_t format_type;					// format type. 1-PCM, 3- IEEE float, 6 - 8bit A law, 7 - 8bit mu law
	uint16_t channels;						// no.of channels
	uint32_t sample_rate;					// sampling rate (blocks per second)
	uint32_t byterate;						// SampleRate * NumChannels * BitsPerSample/8
	uint16_t block_align;					// NumChannels * BitsPerSample/8
	uint16_t bits_per_sample;				// bits per sample, 8- 8bits, 16- 16 bits etc
	unsigned char data_chunk_header [4];		// DATA string or FLLR string
	uint32_t data_size;						// NumSamples * NumChannels * BitsPerSample/8 - size of the next chunk that will be read
} WAVHEADER;

// pointer to file type for files on USB device
FILE *f;

void Process_Event(uint16_t event){
  static uint16_t   Current_State = NoState; // Current state of the SM
	osEvent evt; // Receive message object
	
  switch(Current_State){
    case NoState:
			paused = 0;
      // Next State
      Current_State = Idle;
      // Exit actions
      // Transition actions
      // Idle entry actions
      LED_On(LED_Red);
    
      break;
    case Idle:
      if(event == ListFiles){
        Current_State = List;
        // Exit actions
        LED_Off(LED_Red);
        // Transition actions
        // List entry actions
        LED_On(LED_Blue);
				osMessagePut(mid_Command_FSQueue,SendFiles,osWaitForever);
      }
      break;
    case List:
      if(event == SendComplete){
        Current_State = Pause;
        // Exit actions
        LED_Off(LED_Blue);
        // Transition actions
        // Idle entry actions
        LED_On(LED_Red);
      }
      break;
		case Pause:
			if(event == PlaySong) {
				if (!selected){
					evt = osMessageGet(mid_Selected_Queue, osWaitForever);
					if (evt.status == osEventMessage) { // check for valid message
						selected = 1;
					}
				}
				Current_State = Play;
				// Exit actions
				LED_Off(LED_Red);
				// Transition actions
				// Play entry actions
				LED_On(LED_Green);
				if(paused) {
					paused = 0;
					BSP_AUDIO_OUT_Resume();
				}
				else {
					BSP_AUDIO_OUT_Stop(0);
					osMessagePut(mid_Command_FSQueue,PlaySong,osWaitForever);
				}
			}
			break;
		case Play:
			if(event == SongEnd){
				Current_State = Pause;
				// Exit actions
				LED_Off(LED_Green);
				// Transition actions
				// Idle entry actions
				LED_On(LED_Red);
			}
			else if(event == PauseSong) {
				Current_State = Pause;
				// Exit Actions
				LED_Off(LED_Green);
				// Transition actions
				// Idle entry actions
				LED_On(LED_Red);
				BSP_AUDIO_OUT_Pause();
				paused = 1;
			}
    default:
      break;
  } // end case(Current_State)
} // Process_Event


void Init_Thread (void) {
	selected = 0;
	
   LED_Initialize(); // Initialize the LEDs
   UART_Init(); // Initialize the UART
	
	SEM0_id = osSemaphoreCreate(osSemaphore(SEM0),0); // Create Semaphore with 0 tokens
	
  // Create queues
   mid_CMDQueue = osMessageCreate (osMessageQ(CMDQueue), NULL);  // create msg queue
  if (!mid_CMDQueue)return; // Message Queue object not created, handle failure
  mid_Command_FSQueue = osMessageCreate (osMessageQ(Command_FSQueue), NULL);  // create msg queue
  if (!mid_Command_FSQueue)return; // Message Queue object not created, handle failure
		mid_DMAQueue = osMessageCreate (osMessageQ(DMAQueue), NULL);  // create msg queue
  if (!mid_DMAQueue)return; // Message Queue object not created, handle failure
		mid_Selected_Queue = osMessageCreate (osMessageQ(Selected_Queue), NULL);  // create msg queue
  if (!mid_Selected_Queue)return; // Message Queue object not created, handle failure
	
  // Create threads
   tid_RX_Command = osThreadCreate (osThread(Rx_Command), NULL);
  if (!tid_RX_Command) return;
   tid_Control = osThreadCreate (osThread(Control), NULL);
  if (!tid_Control) return;
	 tid_FS = osThreadCreate (osThread(FS), NULL);
  if (!tid_FS) return;
  }

// Thread function
void Control(void const *arg){
  osEvent evt; // Receive message object
  Process_Event(0); // Initialize the State Machine
   while(1){
    evt = osMessageGet (mid_CMDQueue, osWaitForever); // receive command
      if (evt.status == osEventMessage) { // check for valid message
      Process_Event(evt.value.v); // Process event
    }
   }
}

void Rx_Command (void const *argument){ 
	char rx_char[2]={0,0};
		 //int stop = 0;
	 
   while(1){
      UART_receive(rx_char, 1); // Wait for command from PC GUI
    // Check for the type of character received
      if(!strcmp(rx_char,Show_Files_char)){
         // Show Files Received
        osMessagePut (mid_CMDQueue, ListFiles, osWaitForever);
      } // end if
			else if(!strcmp(rx_char,Receive_Name_char)){
				// Receive Name Received
				UART_receivestring(rx_name,20);
				paused = 0;
				osMessagePut(mid_Selected_Queue, Song_Selected, 0);
			} // end else if
			else if(!strcmp(rx_char, Play_File_char)){
         // Trigger1 received
        osMessagePut (mid_CMDQueue, PlaySong, osWaitForever);
      } 
			else if(!strcmp(rx_char, Pause_File_char)){
         // Trigger1 received
        osMessagePut (mid_CMDQueue, PauseSong, osWaitForever);
      } 
		}
} // end Rx_Command

void FS(void const *arg){
  osEvent evt; // Receive message object
  
	WAVHEADER header;
	usbStatus ustatus; // USB driver status variable
	uint8_t drivenum = 0; // Using U0: drive number
	char *drive_name = "U0:"; // USB drive name
	fsStatus fstatus; // file system status variable
//	static FILE *f;
	char *StartFileList_msg = "2\n";
	char *EndFileList_msg = "3\n";
	fsFileInfo info;
	static uint8_t rtrn = 0;
	int buffer = 1; // index to hold the current buffer number
	int end; // index used to stop the buffers from filling
	//char *tx_buf = "\n";
	//char *file_name;
	//int len;
	
	ustatus = USBH_Initialize (drivenum); // initialize the USB Host
	if (ustatus == usbOK){
		// loop until the device is OK, may be delay from Initialize
		ustatus = USBH_Device_GetStatus (drivenum); // get the status of the USB device
		while(ustatus != usbOK){
			ustatus = USBH_Device_GetStatus (drivenum); // get the status of the USB device
		}
		// initialize the drive
		fstatus = finit (drive_name);
		if (fstatus != fsOK){
			// handle the error, finit didn't work
		} // end if
		// Mount the drive
		fstatus = fmount (drive_name);
		if (fstatus != fsOK){
				// handle the error, fmount didn't work
		} // end if 
		// file system and drive are good to go
	}
	
	rtrn = BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 0x46, 44100);
	if (rtrn != AUDIO_OK)return;
		
	while(1){
		evt = osMessageGet (mid_Command_FSQueue, osWaitForever); // wait for message
		if (evt.status == osEventMessage) { // check for valid message
			if( evt.value.v == SendFiles){
				// SendFiles received
				UART_send(StartFileList_msg,2); // Send start string
				info.fileID = 0;
				 while (ffind ("*.wav", &info) == fsOK) {
					UART_send(info.name, strlen(info.name));
					UART_send("\n",1);
					}
				//UART_send(file_name,len);
				//UART_send("\n",1);
					
				//fclose(f);
				UART_send(EndFileList_msg,2); // Send end string
				osMessagePut(mid_CMDQueue, SendComplete,osWaitForever);
			} // if( evt.value.v == SendFiles)
			else if( evt.value.v == PlaySong) {	
				//fclose(f);
				buffer = 1;
				end = 0;
				BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_OFF);
				
				f = fopen (rx_name,"r");

			// generate data for the audio buffer
			if(fread(&Audio_Buffer, 2, BUF_LEN, f) < BUF_LEN) {
				fclose(f);
				BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_ON);
				osMessagePut(mid_CMDQueue, SongEnd, osWaitForever);
				end = 1;
			}
			
			// Start the audio player
			BSP_AUDIO_OUT_Play((uint16_t *)Audio_Buffer, BUF_LEN);
			
			while(!end) {
				if(buffer == 1) {
					buffer = 2;
					if(fread(&Audio_Buffer2, 2, BUF_LEN, f) < BUF_LEN) {
						fclose(f);
						BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_ON);
						osMessagePut(mid_CMDQueue, SongEnd,osWaitForever);
						end = 1;
					}
				}
				else{
					buffer = 1;
					if(fread(&Audio_Buffer, 2, BUF_LEN, f) < BUF_LEN) {
						fclose(f);
						BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_ON);
						osMessagePut(mid_CMDQueue, SongEnd, osWaitForever);
						end = 1;
					}
				}
				
				osMessagePut (mid_DMAQueue, buffer, osWaitForever);
				osSemaphoreWait(SEM0_id, osWaitForever);
			} // while(!end)
			} // else if (evt.value.v == PlaySong)
		} // if( evt.status == osEventMessage)
	} // while(1)
} // FS
 
/* User Callbacks: user has to implement these functions if they are needed. */
/* This function is called when the requested data has been completely transferred. */
void    BSP_AUDIO_OUT_TransferComplete_CallBack(void){
	osEvent evt; // Receive Message object
	evt = osMessageGet (mid_DMAQueue, 0);
	if (evt.status == osEventMessage) {
		osSemaphoreRelease(SEM0_id);
		
		if (evt.value.v == 2){
				BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)Audio_Buffer2, BUF_LEN);
		}
		else if (evt.value.v == 1){
				BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)Audio_Buffer, BUF_LEN);
		}
	}
	
}

/* This function is called when half of the requested buffer has been transferred. */
void    BSP_AUDIO_OUT_HalfTransfer_CallBack(void){
}

/* This function is called when an Interrupt due to transfer error or peripheral
   error occurs. */
void    BSP_AUDIO_OUT_Error_CallBack(void){
		while(1){
		}
}

