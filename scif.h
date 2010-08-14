
#include <asm/ioctl.h>

/* SCI driver modes */
struct sci_modes
{
    int emv2000;
    int dma;
    int man_act;
    int rw_mode;
};

/* SCI communication parameters */
struct sci_parameters
{
    unsigned char T;			//Protocol type
    unsigned long fs;			//The subsequent frequency during subsequent transmission
    unsigned long ETU;			//Elementary Time Unit
    unsigned long WWT;			//Work Waiting Time
    unsigned long CWT;			//Char Waiting Time
    unsigned long BWT;			//Block Waiting Time
    unsigned long EGT;
    unsigned long clock_stop_polarity;
    unsigned char check;
    unsigned char P;			//Programming voltage
    unsigned char I;			//Maximum programming current
    unsigned char U;
};


#define SCI_IOW_MAGIC			's'


#define IOCTL_SET_RESET			_IOW(SCI_IOW_MAGIC, 1,  unsigned long)
#define IOCTL_SET_MODES			_IOW(SCI_IOW_MAGIC, 2,  struct sci_modes)
#define IOCTL_GET_MODES			_IOW(SCI_IOW_MAGIC, 3,  struct sci_modes)
#define IOCTL_SET_PARAMETERS		_IOW(SCI_IOW_MAGIC, 4,  struct sci_parameters)
#define IOCTL_GET_PARAMETERS		_IOW(SCI_IOW_MAGIC, 5,  struct sci_parameters)
#define IOCTL_SET_CLOCK_START		_IOW(SCI_IOW_MAGIC, 6,  unsigned long)
#define IOCTL_SET_CLOCK_STOP		_IOW(SCI_IOW_MAGIC, 7,  unsigned long)
#define IOCTL_GET_IS_CARD_PRESENT	_IOW(SCI_IOW_MAGIC, 8,  unsigned long)
#define IOCTL_GET_IS_CARD_ACTIVATED	_IOW(SCI_IOW_MAGIC, 9,  unsigned long)
#define IOCTL_SET_DEACTIVATE		_IOW(SCI_IOW_MAGIC, 10, unsigned long)
#define IOCTL_SET_ATR_READY		_IOW(SCI_IOW_MAGIC, 11, unsigned long)
#define IOCTL_GET_ATR_STATUS		_IOW(SCI_IOW_MAGIC, 12, unsigned long)
#define IOCTL_DUMP_REGS			_IOW(SCI_IOW_MAGIC, 20, unsigned long)

