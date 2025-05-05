#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 7      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ )
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}

bool inWindow(int base, int seq) {
    int end = (base + WINDOWSIZE - 1) % SEQSPACE;
    if (base <= end) {
        return (seq >= base && seq <= end);
    } else {
        return (seq >= base || seq <= end);
    }
}
/********* Sender (A) variables and functions ************/
/*aaaaaaaaaaaaaseqspace instead of windowsize, as selective repeat must track each sequence number in the space*/
static struct pkt buffer[SEQSPACE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static bool acked[SEQSPACE];           /* Tracks which seqnums have been ACKed */
static float packet_timer[SEQSPACE];   /* stores time of each packet's last send */
static bool timer_active[SEQSPACE];    /* is timer active for each packet */
static int windowbase;                 /* SR window */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ )
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    windowlast = (windowlast + 1) % WINDOWSIZE;
    buffer[windowlast] = sendpkt;
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    packet_timer[sendpkt.seqnum] = 0.0;
    timer_active[sendpkt.seqnum] = true;

    /* start timer if first packet in window */
    bool timer_running = false;
    int j;
    for (j = 0; j < SEQSPACE; j++) {
      if (timer_active[j]) {
        timer_running = true;
          break;
      }
  }
  if (!timer_running) {
      starttimer(A, RTT);
  }

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked,  window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
    if (!IsCorrupted(packet)) {
        if (TRACE > 0)
            printf("----A: uncorrupted ACK %d is received\n", packet.acknum);

        total_ACKs_received++;

        int acknum = packet.acknum;


        if (inWindow(windowbase, acknum) && !acked[acknum]) {
            /* Mark the packet as acknowledged*/
            acked[acknum] = true;
            timer_active[acknum] = false;
            if (TRACE > 0)
              printf("----A: ACK %d is not a duplicate\n",packet.acknum);
            new_ACKs++;


            /* Slide window forward as long as the first packet is ACKed*/
            while (windowcount > 0 && acked[windowbase]) {
              timer_active[windowbase] = false;
              windowbase = (windowbase + 1) % SEQSPACE;
              windowcount--;
            }

            bool active;
            active = false;
            int i;
            for (i = 0; i < SEQSPACE; i++){
                if (timer_active[i]) active = true;
            }
            if (!active){
                stoptimer(A);
              }            
        } else {
            if (TRACE > 0)
                printf("----A: duplicate ACK received, do nothing!\n", acknum);
        }
    } else {
        if (TRACE > 0)
            printf("----A: corrupted ACK received, ignored\n");
    }
}

/* called when A's timer goes off */
void A_timerinterrupt(void) {
  bool any_timer_active;
  any_timer_active = false;
  int i;
  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  for (i = 0; i < SEQSPACE; i++) {
    if (timer_active[i] && !acked[i]) {
        /* Timeout detected — resend packet */
        
        packet_timer[i]+=1.0;
        if (packet_timer[i] >= RTT) {
          tolayer3(A, buffer[i]);
          packets_resent++;
          packet_timer[i] = 0.0;
          if (TRACE > 0){
            printf("----A: resending packet %d\n", (buffer[(windowfirst+i) % WINDOWSIZE]).seqnum);
          }
        }
      }

    }
    if (any_timer_active) {
      starttimer(A, 1.0);  /* Reschedule next check*/
    }
    
}




/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowbase = 0;
  windowcount = 0;
  int i; /*bbbbbbbbsame as before*/
  for (i = 0; i < SEQSPACE; i++) {
    acked[i] = false;
    timer_active[i] = false;
    packet_timer[i] = 0.0;
  }
}




/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt recv_buffer[SEQSPACE];  /* aaaaaaaaaaaaato store received packets*/
static bool received[SEQSPACE];           /* aaaaaaaaaaaaaatrack received sequence numbers*/

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt ackpkt;
  int i;
    /*aaaaaaaaif not corrupted*/
  if (!IsCorrupted(packet)) {
    int seq = packet.seqnum;

    /* aaaaaaaaaaif packet isnt received, IGNORING ORDER*/
    /*bbbbb added function to check if seq is in receiving window*/
    if (!received[seq] && inWindow(expectedseqnum,seq)) {
        /*aaaa buffer the packet and mark it received*/
        recv_buffer[seq] = packet;
        received[seq] = true;
        if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n",packet.seqnum);
    }

    /* aaaaaaaaasend ACK for this packet regardless of order*/
    ackpkt.acknum = seq;

    /* aaaaaaaadeliver in-order packets starting from expectedseqnum*/
    while (received[expectedseqnum]) {
      tolayer5(B, recv_buffer[expectedseqnum].payload);
      packets_received++; /*whoops*/
      received[expectedseqnum] = false; /*aaaaaaaaa mark as delivered*/
      expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
    }
  } else {
    /*aaaaaaaaaaaa corrupted, resend last ACK*/
    if (expectedseqnum == 0)
      ackpkt.acknum = SEQSPACE - 1;
    else
      ackpkt.acknum = expectedseqnum - 1;

    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
  }

  /*aaaaaaaaaaaaa fill the rest of the ACK packet fields*/
  ackpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;

  for (i = 0; i < 20; i++)
    ackpkt.payload[i] = '0';

  ackpkt.checksum = ComputeChecksum(ackpkt);
  tolayer3(B, ackpkt);
}



/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;
  /*aaaaaaaaensures that all spaces are correctly classified as empty before starting transfer*/
  int i;
  for (i = 0; i < SEQSPACE; i++) {
    received[i] = false;
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}
