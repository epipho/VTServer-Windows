/*
 * Virtual tape driver - copyright 1998 Warren Toomey	wkt@cs.adfa.edu.au
 *
 * $Revision: 2.3.1.5 $
 * $Date: 2001/04/04 02:57:28 $
 *
 * Windows stuff added 2001/5/23 Jonathan Engdahl engdahl@cle.ab.com
 * Extended block number added 2001/10/4 Jonathan Engdahl engdahl@cle.ab.com
 *
 * This program sits on a serial line, receives `tape' commands from a
 * PDP-11, and returns the results of those command to the other machine.
 * It was designed to allow 7th Edition UNIX to be installed on to a
 * machine without a tape drive.
 *
 * Much of the functionality of the tape protocol has been designed but
 * not yet implemented.
 *
 * Commands look like the following:
 *
 *	  +----+----+-----+------+----+----+---+----+
 *	  | 31 | 42 | cmd | rec# | blk|num | ck|sum |
 *	  +----+----+-----+------+----+----+---+----+
 *
 * Each box is an octet. The block number is a 16-bit value, with the
 * low-order octet first. Any data that is transmitted (in either direction)
 * comes as 512 octets between the block number and the checksum. The
 * checksum is a bitwise-XOR of octet pairs, excluding the checksum itself.
 * I.e checksum octet 1 holds 31 XOR cmd XOR blklo [ XOR odd data octets ], and
 * checksum octet 2 holds 42 XOR rec# XOR blkhi [ XOR even data octets ].
 *
 * A write command from the client has 512 octets of data. Similarly, a read
 * command from the server to the client has 512 octets of data. 
 *
 *
 * The Protocol
 * ------------
 *
 * The protocol is stateless. Commands are read, zeroread, quickread, write,
 * open and close.
 *
 * The record number holds the fictitious tape record which is requested.
 * The server keeps one disk file per record. The block number holds the
 * 512-octet block number within the record.
 *
 * Assumptions: a read on a record without a previous open implies the open.
 * A command on a new record will close any previously opened record.
 * There is only one outstanding client request at any time.
 *
 * The client sends a command to the server. The server returns the command,
 * possibly with 512 octets of data, to the client. The top four bits of the
 * command octet hold the return error value. All bits off indicates no error.
 * If an error occurred, including EOF, no data octets are returned on a read
 * command.
 *
 * If the client receives a garbled return command, it will resend the command.
 * Therefore, the server must cope with this.
 *
 * The exception command to all of this is the QUICK read command. Here,
 * the server responds with one octet followed by 512 data octets. If the
 * octet is zero, data will be sent. Otherwise, the octet contains an
 * error value in the top four bits (including EOF), and no data is sent.
 * There are no command replies or checksums. This makes it useful only
 * to load one record by hand at bootstrap.

 * If the client requests a ZEROREAD, and if the server detects that the
 * block requested is all zeroes, then the returned message has cmd=ZEROREAD
 * and _no_ data octets. However, if any octet in the block non-zero, then
 * the server sends back a cmd=READ message with the 512-octets of data.
 *
 */

#include <sys/types.h>
#include <stdio.h>
#include <fcntl.h>
#ifdef _MSC_VER
#define _WIN32_WINDOWS 0x0410	/* this work for Win98 and up only */
#define WINVER 0x0400
#include <io.h>			/* include files unique to Windows */	
#include <windows.h>
#else
#include <termios.h>		/* include files unique to UNIX */
#include <unistd.h>
#endif
#include <stdlib.h>
#include <string.h>
char *strerror(int errno);

/* Commands sent in both directions */
struct vtcmd {
  unsigned char hdr1;		/* Header, 31 followed by 42 (decimal) */
  unsigned char hdr2;
  unsigned char cmd;		/* Command, one of VTC_XXX below */
  				/* Error comes back in top 4 bits */
  unsigned char record;		/* Record we're accessing */
  unsigned char blklo;		/* Block number, in lo/hi format */
  unsigned char blkhi;
  unsigned char sum0;		/* 16-bit checksum */
  unsigned char sum1;		/* 16-bit checksum */
};

/* Header octets */
#define VT_HDR1		31
#define VT_HDR2		42

/* Commands available */
#define VTC_QUICK	0	/* Quick read, no cksum sent */
#define VTC_OPEN	1	/* Open the requested record */
#define VTC_CLOSE	2	/* Close the requested record */
#define VTC_READ	3	/* Read requested block from record */
#define VTC_WRITE	4	/* Write requested block from record */
#define VTC_ZEROREAD	6	/* Zero read, return no data if all zeroes */

/* Errors returned */
#define VTE_NOREC	1	/* No such record available */
#define VTE_OPEN	2	/* Can't open requested block */
#define VTE_CLOSE	3	/* Can't close requested block */
#define VTE_READ	4	/* Can't read requested block */
#define VTE_WRITE	5	/* Can't write requested block */
#define VTE_NOCMD	6	/* No such command */
#define VTE_EOF		7	/* End of file: no blocks left to read */

#define BLKSIZE		512

/* Static things */
extern int errno;		/* Error from system calls etc. */
struct vtcmd vtcmd;		/* Command from client */
struct vtcmd vtreply;		/* Reply to client */
char inbuf[BLKSIZE];		/* Input buffer */
char *port = NULL;		/* Device for serial port */
int rate=38400;
#ifdef _MSC_VER
HANDLE portfd;
#else
int portfd;			/* File descriptor for the port */
#endif
int ttyfd=0;			/* File descriptor for the console */
int recfd = -1;			/* File descriptor for the in-use record */
int lastrec = -2;		/* Last record used */
char *recname[256];		/* Up to 256 records on the tape */
long block;			/* the block number (variable length field in protocol) */

#ifndef _MSC_VER		/* I/O routines uniqie to UNIX */
struct termios oldterm;		/* Original terminal settings */

/* define port and console I/O */
#define readP(a,b,c) read(a,b,c)
#define writeP(a,b,c) write(a,b,c)
#define readC(a,b,c) read(a,b,c)
#define writeC(a,b,c) write(a,b,c)

#define BINARY 0
#else				/* I/O routines unique to Windows */

HANDLE hStdout, hStdin;		/* input and output handles for console */
DWORD TTMode, OldTTMode; 
OVERLAPPED ovin;		/* overlapped I/O structures, contains event handle */
OVERLAPPED ovout;
INPUT_RECORD pinput;		/* for peeking at console input */
unsigned char Rbuf;
int Rsize;
int Xsize;
int peeksize;
int scratchsize;
int firstRead=1;			/* flag to detect first call to port I/O */
int firstWrite=1;

/*
   read from the comm port
   This routine actually only reads one byte at a time.
   All calls to read in the original UNIX code were one byte reads.

   The wait w/ infinite timeout is not robust communications -- but we assume a reliable connection.
*/

void readP(HANDLE a,char *b, int c)
	{
	if(!firstRead)WaitForSingleObject(ovin.hEvent,INFINITE); /* wait for I/O complete */
	firstRead=0;
	if(b)*b = Rbuf;						/* get the character */
	if(ReadFile(a,&Rbuf,1,&Rsize,&ovin)==0)			/* post a new read-and-continue */
		{
		DWORD sts;

		sts = GetLastError();
		if(sts!=ERROR_IO_PENDING)			/* this error code is what is expected for a read-and-continue */
			{
			printf("port read error = %i\n",sts);
			}
		}
	}


/*
   write to the comm port
   this must used overlapped I/O, since the read side does
*/
void writeP(HANDLE a,char *b,int c)
	{
	static char buf[1024];

	if(!firstWrite)WaitForSingleObject(ovout.hEvent,INFINITE); /* wait for completion of previous write */
	firstWrite=0;
	memcpy(buf,b,c);
	if(WriteFile(a,&buf,c,&Xsize,&ovout)==0)			/* write new data */
		{
		DWORD sts;

		sts = GetLastError();
		if(sts!=ERROR_IO_PENDING)			/* expected, OK */
			{
			printf("port write error = %i\n",sts);
			}
		}
	}

/* read from the console */
/* all reads are one byte each, but accept size parameter in order to look like "read" */

void readC(HANDLE a,char *b,int c)
	{
	int nch;
				/* Windows is giving us raw keystrokes */
	for(nch=1;nch;nch--)	/* this nch business is to skip over presses of shift and arrow keys, which are two byte sequences, the first of which is zero (not sure about this) */
		{
		ReadFile(hStdin,b,c,&scratchsize,0);
		if(*b==0)nch=3;				/* ignore arrow keys and such */
		}
	}

/* write to the console */
#define writeC(a,b,c) WriteFile(hStdout,b,c,&scratchsize,0)

#define BINARY _O_BINARY

#endif

unsigned char sum0,sum1;

#define read0(a,b,c)					\
	do{						\
	readP(a,b,c);					\
	sum0 ^= *b;					\
	}while(0)

#define read1(a,b,c)					\
	do{						\
	readP(a,b,c);					\
	sum1 ^= *b;					\
	}while(0)

#define write0(a,b,c)					\
	do{						\
	writeP(a,b,c);					\
	sum0 ^= *b;					\
	}while(0)

#define write1(a,b,c)					\
	do{						\
	writeP(a,b,c);					\
	sum1 ^= *b;					\
	}while(0)


/* This array holds the bootstrap code, which we can enter via ODT
 * at the BOOTSTART address.
 */
#define BOOTSTACK 0130000
#define BOOTSTART 0140000
int bootcode[]= {
	0010706, 005003, 0012701, 0177560, 012704, 0140106, 0112400, 0100406,
	0105761, 000004, 0100375, 0110061, 000006, 0000770, 0005267, 0000052,
	0004767, 000030, 0001403, 0012703, 006400, 0005007, 0012702, 0001000,
	0004767, 000010, 0110023, 0005302, 001373, 0000746, 0105711, 0100376,
	0116100, 000002, 0000207, 0025037, 000000, 0000000, 0177777
};
int havesentbootcode=1;		/* Don't send it unless user asks on cmd line */


/* Get a command from the client.
 * If a command is received, returns 1,
 * otherwise return 0.
 */
int get_command(struct vtcmd *v)
{
  int i,loc;
  char ch,bootbuf[40];

  sum0 = 0;
  sum1 = 0;

  /* Get a valid command from the client */
  read0(portfd, &v->hdr1, 1);

  /* Send down the bootstrap code to ODT if we see an @ sign */
  if ((havesentbootcode==0) && (v->hdr1 == '@')) {
    writeC(ttyfd,&v->hdr1,1);
    for (i=0,loc=BOOTSTART;i<(sizeof(bootcode)/sizeof(int));i++,loc+=2) {
	sprintf(bootbuf, "%06o/", loc);
	writeP(portfd, bootbuf, strlen(bootbuf));

	/* wait for current value to print */
	while (1) { readP(portfd, &ch, 1); writeC(ttyfd,&ch,1); if (ch==' ') break; }
	sprintf(bootbuf, "%06o\r", bootcode[i]);
	writeP(portfd, bootbuf, strlen(bootbuf));

	/* and suck up any characters sent from ODT */
	while (1) { readP(portfd, &ch, 1); writeC(ttyfd,&ch,1); if (ch=='@') break; }
    }
    sprintf(bootbuf, "r6/", loc);
    writeP(portfd, bootbuf, strlen(bootbuf));
    while (1) { readP(portfd, &ch, 1); writeC(ttyfd,&ch,1); if (ch==' ') break; }
    sprintf(bootbuf, "%06o\r", BOOTSTACK);
    writeP(portfd, bootbuf, strlen(bootbuf));
    while (1) { readP(portfd, &ch, 1); writeC(ttyfd,&ch,1); if (ch=='@') break; }
    sprintf(bootbuf, "r7/", loc);
    writeP(portfd, bootbuf, strlen(bootbuf));
    while (1) { readP(portfd, &ch, 1); writeC(ttyfd,&ch,1); if (ch==' ') break; }
    sprintf(bootbuf, "%06o\r", BOOTSTART);
    writeP(portfd, bootbuf, strlen(bootbuf));
    while (1) { readP(portfd, &ch, 1); writeC(ttyfd,&ch,1); if (ch=='@') break; }
    sprintf(bootbuf, "rs/", loc);
    writeP(portfd, bootbuf, strlen(bootbuf));
    while (1) { readP(portfd, &ch, 1); writeC(ttyfd,&ch,1); if (ch==' ') break; }
    sprintf(bootbuf, "%06o\r", 000340);
    writeP(portfd, bootbuf, strlen(bootbuf));
    while (1) { readP(portfd, &ch, 1); writeC(ttyfd,&ch,1); if (ch=='@') break; }
    sprintf(bootbuf, "p");
    writeP(portfd, bootbuf, strlen(bootbuf));
    while (1) { readP(portfd, &ch, 1); writeC(ttyfd,&ch,1); if (ch=='p') break; }
    havesentbootcode=1; return(0);
  }

  if (v->hdr1 != VT_HDR1) { v->hdr1&= 127; writeC(1,&v->hdr1, 1); return(0); }
  read1(portfd, &v->hdr2, 1);
  if (v->hdr2 != VT_HDR2) { v->hdr1&= 127; writeC(1,&v->hdr1, 1); v->hdr2&= 127; writeC(1,&v->hdr2, 1); return(0); }

  read0(portfd, &v->cmd, 1); read1(portfd, &v->record, 1);
  read0(portfd, &v->blklo, 1); read1(portfd, &v->blkhi, 1);
  block = v->blkhi<<8&0xff00 | v->blklo&0xff;
  if(block>=0xff00)
	{
	unsigned char tmp0,tmp1;

	read0(portfd, &tmp0, 1);
	read1(portfd, &tmp1, 1);
	block = tmp1<<16&0xff0000 | tmp0<<8&0xff00 | v->blklo&0xff;
	}


  /* All done if a quick read */
  if (v->cmd == VTC_QUICK) return(1);

  /* Retrieve the block if a WRITE cmd */
  if (v->cmd == VTC_WRITE) {
    for (i = 0; i < BLKSIZE; i++) {
      read0(portfd, &inbuf[i], 1); i++;
      read1(portfd, &inbuf[i], 1);
    }
  }

  /* Get the checksum */
  read0(portfd, &(v->sum0), 1);
  read1(portfd, &(v->sum1), 1);

  /* Try again on a bad checksum */
  if (sum0 | sum1)
    { fputc('e',stderr); return(0); }

  return(1);
}


/* Reply has been mostly initialised by do_command */
void send_reply()
{
  int i;

  if ((vtcmd.cmd)==VTC_QUICK) {     /* Only send buffer on a quick read */
    writeP(portfd, &vtreply.cmd, 1);
    if (vtreply.cmd!=VTC_QUICK) return;	/* Must have been an error */
    for (i=0; i< BLKSIZE; i++) writeP(portfd, &inbuf[i], 1);
    return;
  }

  sum0 = 0;
  sum1 = 0;

  /* Transmit the reply */
  write0(portfd, &vtreply.hdr1, 1);
  write1(portfd, &vtreply.hdr2, 1);
  write0(portfd, &vtreply.cmd, 1);
  write1(portfd, &vtreply.record, 1);
  if(block<0xff00)
	{
	unsigned char tmp;

	tmp = block;
	write0(portfd, &tmp, 1);
	tmp = block>>8;
	write1(portfd, &tmp, 1);
	}
  else
	{
	unsigned char tmp;

	tmp = block;
	write0(portfd, &tmp, 1);
	tmp = 0xff;
	write1(portfd, &tmp, 1);
	tmp = block>>8;
	write0(portfd, &tmp, 1);
	tmp = block>>16;
	write1(portfd, &tmp, 1);
	}

  if (vtreply.cmd == VTC_READ) {
    for (i = 0; i < BLKSIZE; i++)
	{
	write0(portfd, &inbuf[i], 1);
	i++;
	write1(portfd, &inbuf[i], 1);
	}
  }
  write0(portfd, &sum0, 1);
  write1(portfd, &sum1, 1);
}


#define seterror(x)	vtreply.cmd |= (x<<4);

/* Actually do the command sent to us */
void do_command()
{
  int record, i;
  long offset;

  /* First, copy the command to the reply */
  memcpy(&vtreply, &vtcmd, sizeof(vtcmd));

  record = vtcmd.record;
  offset = block * BLKSIZE;

  /* Open the record if not already open */
  if (record != lastrec) {
    if (recname[record] == NULL) {
	fprintf(stderr,"No such tape record %d\r\n",record);
	seterror(VTE_NOREC); return;
    }

    i = open(recname[record], O_RDWR | BINARY);
    if (i>=0) {
       fprintf(stderr,"\nOpened %s read-write\r\n ", recname[record]);
       goto afteropen;			/* yuk, a goto! */
    }
    i = open(recname[record], O_RDONLY | BINARY);
    if (i>=0) {
       fprintf(stderr,"\nOpened %s read-only\r\n ", recname[record]);
       goto afteropen;			/* yuk, a goto! */
    }
    i = open(recname[record], O_RDWR|O_CREAT|O_TRUNC | BINARY, 0600);
    if (i>=0) {
       fprintf(stderr,"\nOpened %s as a new file\r\n ", recname[record]);
       goto afteropen;			/* yuk, a goto! */
    }
    fprintf(stderr,"Cannot open %s: %s\r\n",recname[record], strerror(errno));
    seterror(VTE_NOREC); return;
 
afteropen:
    if (record != lastrec) close(recfd);
    recfd = i; lastrec = record;
  }

  switch (vtcmd.cmd) {
    case VTC_OPEN:  break;
    case VTC_CLOSE: close(recfd); lastrec = -1; break;

    case VTC_QUICK: vtreply.cmd=0;	/* No errors yet */
    case VTC_ZEROREAD: 
    case VTC_READ:  
		    i= lseek(recfd, offset, SEEK_SET);
      		    if (i==-1)
      		      { fprintf(stderr," EOF1\r\n"); seterror(VTE_EOF); return; }
   		    i = read(recfd, &inbuf, BLKSIZE);
      		    if (i == 0)
      		      { fprintf(stderr," EOF2\r\n"); seterror(VTE_EOF); return; }
      		    if (i == -1) { seterror(VTE_READ); return; }

				/* Determine if the entire block is zero */
		    if (vtcmd.cmd==VTC_ZEROREAD) {
		      for (i=0;i<BLKSIZE;i++) if (inbuf[i]!=0) break;
		      if (i==BLKSIZE) vtreply.cmd=VTC_ZEROREAD;
		      else vtreply.cmd=VTC_READ;
		    }

		    if (offset && (offset % 102400) == 0)
			fprintf(stderr,"\r\n%dK sent\r\n", offset/1024);
		    fputc('r',stderr);
      		    break;

    case VTC_WRITE: i= lseek(recfd, offset, SEEK_SET);
      		    if (i==-1)
      		      { fprintf(stderr," seek error\r\n");
			seterror(VTE_WRITE); return;
		      }
		    i = write(recfd, &inbuf, BLKSIZE);
      		    if (i < 1) { seterror(VTE_WRITE); return; }
		    if (offset && (offset % 102400) == 0)
			fprintf(stderr,"\r\n%dK received\r\n", offset/1024);
		    fputc('w',stderr);
      	 	    break;

    default:	    fputc('?',stderr);
   		    seterror(VTE_NOCMD);
  }
  fflush(stderr);
}

/* The configuration file is .vtrc. The first line holds the name
 * of the serial device. The following lines hold the filenames which
 * are the successive tape records. Lines starting with a hash are ignored.
 * Files are not opened unless they are referenced by a client's command.
 */
void read_config()
{
  FILE *in;
  char *c;
  int i, cnt = 0, donesystem=0;

  in = fopen(".vtrc", "r");
  if (in == NULL) {
    fprintf(stderr, "Error opening .vtrc config file: %s\n", strerror(errno));
    exit(1);
  }
  while (cnt != 256) {
    if (feof(in)) break;
    c = fgets(inbuf, BLKSIZE - 2, in);
    if (feof(in)) break;

    if (c == NULL) {
      fprintf(stderr, "Error reading .vtrc config file: %s\n", strerror(errno));
      exit(1);
    }
    if (inbuf[0] == '#') continue;

    inbuf[strlen(inbuf) - 1] = '\0';	/* Remove trailing newline */

    if (donesystem == 0) {
	fprintf(stderr,"Running command %s\n\n",inbuf);
	system(inbuf); donesystem=1; continue;
    }

    if (port == NULL) {
      port = (char *) malloc(strlen(inbuf) + 2);
      strcpy(port, inbuf); continue;
    }

    recname[cnt] = (char *) malloc(strlen(inbuf) + 2);
    strcpy(recname[cnt], inbuf); cnt++;
  }
  fprintf(stderr,"Tape records are:\n");
  for (i=0; i<cnt; i++) fprintf(stderr,"  %2d %s\n", i, recname[i]);
  fprintf(stderr,"\n");

  fclose(in);
}

#ifndef _MSC_VER
/* Use POSIX terminal commands to
 * set the serial line to raw mode.
 */
void setraw(int fd, char *portname, int dosave)
{
  struct termios t;

  /* Get the device's terminal attributes */
  if (tcgetattr(fd, &t) == -1) {
    fprintf(stderr, "Error getting %s attributes: %s\n",
						 portname, strerror(errno));
    exit(1);
  }
  if (dosave) memcpy(&oldterm,&t,sizeof(t));	/* Save the old settings */

  /* Set raw - code stolen from 4.4BSD libc/termios.c */
  t.c_iflag &= ~(IMAXBEL | IXOFF | INPCK | BRKINT | PARMRK | ISTRIP |
		 INLCR | IGNCR | ICRNL | IXON | IGNPAR);
  t.c_iflag |= IGNBRK;
  t.c_oflag &= ~OPOST;
  t.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | ICANON | ISIG | IEXTEN |
		 NOFLSH | TOSTOP | PENDIN);
  t.c_cflag &= ~(CSIZE | PARENB);
  t.c_cflag |= CS8 | CREAD;
  t.c_cc[VMIN] = 1;
  t.c_cc[VTIME] = 0;

  /* Set the device's terminal attributes */
  if (tcsetattr(fd, TCSANOW, &t) == -1) {
    fprintf(stderr, "Error setting %s attributes: %s\n",
						portname, strerror(errno));
    exit(1);
  }
}
#endif

/* Reset the terminal settings and
 * exit the process
 */
void termexit(int how)
{
#ifdef _MSC_VER
    SetConsoleMode(hStdin, OldTTMode);
#else
  tcsetattr(ttyfd, TCSANOW, &oldterm);
#endif
  exit(how);
}


/* Open the named port and set it to raw mode.
 * Someone else deals with such things as
 * baud rate, clocal and crtscts.
 */
void open_port()
{
#ifdef _MSC_VER
	/* The following is sort of like APL. If I have to explain it to you, you don't deserve to know. */
	/* Actually, I copied most of it from the examples, and I don't understand it that well myself */

	DCB dcb;
	struct _COMMTIMEOUTS TO = {MAXDWORD,MAXDWORD,1,2,1000};	/* NOTE -- timeouts are hard wired for 9600 baud or higher */

	fprintf(stderr,"Opening port %s .... ", port); fflush(stderr);

	portfd = CreateFile("COM1:",GENERIC_READ|GENERIC_WRITE,0,0,OPEN_EXISTING,FILE_FLAG_OVERLAPPED,0);
	if(portfd == INVALID_HANDLE_VALUE)
		{
		fprintf(stderr, "can't open COM1:");
		exit(1);
		}

	memset(&dcb, sizeof(dcb), 0);
	dcb.DCBlength = sizeof(dcb);
	if (!GetCommState(portfd, &dcb))
		{
		fprintf(stderr, "can't get CommState ");
		exit(1);
		}

        if(rate==4800)
		{
		dcb.BaudRate = CBR_4800;
		fprintf(stderr, "rate=4800 ");
		}
        else if(rate==9600)
		{
		dcb.BaudRate = CBR_9600;
		fprintf(stderr, "rate=9600 ");
		}
	else if(rate==19200)
		{
		dcb.BaudRate = CBR_19200;
		fprintf(stderr, "rate=19200 ");
		}
	else
		{
		dcb.BaudRate = CBR_38400;
		fprintf(stderr, "rate=38400 ");
		}

	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fParity = FALSE;

	if (!SetCommState(portfd, &dcb))
		{
		fprintf(stderr, "can't set CommState");
		exit(1);
		}

	SetCommTimeouts(portfd,&TO);

	/* init the overlapped I/O semaphores */
	memset(&ovin, sizeof(OVERLAPPED), 0);
	ovin.hEvent = CreateEvent(0,TRUE,0,0);

	memset(&ovout, sizeof(OVERLAPPED), 0);
	ovout.hEvent = CreateEvent(0,TRUE,0,0);

	readP(portfd,0,1);	/* do a dummy read to post the first overlapped read */

	fprintf(stderr,"Port open\n");
#else
  fprintf(stderr,"Opening port %s .... ", port); fflush(stderr);
  portfd = open(port, O_RDWR);
  if (portfd == -1) {
    fprintf(stderr, "Error opening device %s: %s\n", port, strerror(errno));
    exit(1);
  }
  fprintf(stderr,"Port open\n");
  setraw(portfd,port,0);
#endif
}

/* this is *so* ugly, but there weren't to many choices for adding Windows without touching the UNIX code */

void server_loop()
{
  char ch;
  int i;
  int esc_num=0;
  int in_tape_mode=0;		/* Are we in tape mode or console mode */
#ifdef _MSC_VER
  int nch=1;			/* variables unique to Windows */
#else
  fd_set fdset;
#endif

  ch='\r'; writeP(portfd,&ch,1);	/* Send a \r to wake ODT up if it is there */
#ifndef _MSC_VER
  FD_ZERO(&fdset);
#endif
  while (1) {
#ifndef _MSC_VER
    FD_SET(ttyfd, &fdset);
    FD_SET(portfd, &fdset);	/* Wait for chars in stdin or serial line */

    i=select(portfd+1, &fdset, NULL, NULL, NULL);
    if (i<1) continue;
#endif

				/* Console input */
#ifdef _MSC_VER
    if (PeekConsoleInput(hStdin,&pinput,1,&peeksize)
    && peeksize>0
    && ReadConsoleInput(hStdin,&pinput,1,&peeksize)
    && pinput.EventType==KEY_EVENT
    && pinput.Event.KeyEvent.bKeyDown) {
	ch = pinput.Event.KeyEvent.uChar.AsciiChar;
	if(--nch>0)continue;	/* this skips over input characters to be ignored */
	if(ch==0)		/* lead-in for arrow, shift down, etc */
		{
		nch=1;		/* skip junk */
		continue;
		}
				/* "}" missing on purpose -- keep reading... */
#else
    if (FD_ISSET(ttyfd, &fdset)) {
	readC(ttyfd,&ch,1);
#endif

	if (!in_tape_mode) {
	  if (ch==0x1b) esc_num++;	/* Exit when two ESCs consecutively */
	  else esc_num=0;
	  if (esc_num==2) termexit(0);
	  if(ch=='A'-64)havesentbootcode=0;
	  else
#ifdef _MSC_VER

	    if(ch=='B'-64)
		{
		HANDLE timer = CreateWaitableTimer(0,TRUE,0);
		LARGE_INTEGER t = {-1*10000000,-1};

		SetCommBreak(portfd);
		SetWaitableTimer(timer,&t,0,0,0,0);
		WaitForSingleObject(timer,INFINITE);
		ClearCommBreak(portfd);
		CloseHandle(timer);
		}
	  else
#endif
	    writeP(portfd,&ch,1);
	}
    }
				/* Get a command from the client */
#ifdef _MSC_VER
    if(WaitForSingleObject(ovin.hEvent,0)==WAIT_OBJECT_0) {
#else
    if (FD_ISSET(portfd, &fdset)) {
#endif
      if (get_command(&vtcmd)==0) { in_tape_mode=0; continue; }

      in_tape_mode=1;
      do_command();		/* Do the command */
      send_reply();		/* Send the reply */
    }
  }
}



int main(int argc, char *argv[])
{
  fprintf(stderr,"Virtual tape server, $Revision: 2.3.1.5 $ \n");


  if(argc>=2 && *argv[1]!='-')
	{
	rate=atoi(argv[1]);
	argv++;
	argc--;
	}

  if ((argc==2) && (!strcmp(argv[1], "-odt"))) havesentbootcode=0;

  read_config();
  open_port();

#ifdef _MSC_VER
    hStdin = GetStdHandle(STD_INPUT_HANDLE); 
    hStdout = GetStdHandle(STD_OUTPUT_HANDLE); 

    GetConsoleMode(hStdin, &OldTTMode); 
    TTMode = OldTTMode & ~(ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT | ENABLE_PROCESSED_INPUT); 
    SetConsoleMode(hStdin, TTMode);

#else  
  setraw(ttyfd,"standard input",1); 
#endif
  server_loop();
  exit(0);
}
