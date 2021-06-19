#include "main.h"
#include "controllers.h"
#include "motor.h"
#include "pid.h"
#include "solver.h"
#include "stdlib.h"
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define TS 0.01 /* time step for simulation */

void report_err(const char *on_what)
{
  if (errno != 0)
  {
    fputs(strerror(errno), stderr);
    fputs(": ", stderr);
  }
  fputs(on_what, stderr);
  fputc('\n', stderr);
  exit(1);
}

int getctrl(char *buffer, float *ud, float *uq)
{
  char *control = strtok(buffer, ",");
  if (control == NULL)
  {
    return -1;
  }
  *ud = atof(control);
  control = strtok(NULL, ",");
  if (control == NULL)
  {
    return -1;
  }
  *uq = atof(control);
  return 0;
}

int main()
{
  /* motor parameters */
  float t;  /* time */
  float ud, uq; /* control signals */
  float u[N];   /* buffer for output */
  /* server parameters */
  int z;
  char *srvr_port = "9099";
  struct sockaddr_in adr_srvr;
  struct sockaddr_in adr_clnt;
  unsigned int len_inet;
  int s; /* socket file descriptor */
  int c; /* client file descriptor */
  int n; /* placeholder to store length of messages */
  char dtbuf[256];

  s = socket(PF_INET, SOCK_STREAM, 0);
  if (s == -1)
  {
    report_err("socket()");
  }
  memset(&adr_srvr, 0, sizeof(adr_srvr));
  adr_srvr.sin_family = AF_INET;
  adr_srvr.sin_port = htons(atoi(srvr_port));
  adr_srvr.sin_addr.s_addr = INADDR_ANY;

  len_inet = sizeof(adr_srvr);
  z = bind(s, (struct sockaddr *)&adr_srvr, len_inet);
  if (z == -1)
  {
    report_err("bind(2)");
  }
  z = listen(s, 10);
  if (z == -1)
  {
    report_err("listen()");
  }

  /* motor prep */
  write_header();
  motor_turn_on(u);
  for(int i = 0; i < 16; i++)
  {
      printf("%s", fig[i]);
  }
  /* 
     when a client connects, read out the control signals (ud,uq), and step 0.1
     send the id,iq currents, the input to verify, speed and rotor position 
summary: 
- client : send control
- server : get control, update motor, send observation 
*/
  while (1)
  {
    t = 0; /* reset motor sim to zero */
    /* wait for connection */
    len_inet = sizeof adr_clnt;
    c = accept(s, (struct sockaddr *)&adr_clnt, &len_inet);
    if (c == -1)
    {
      report_err("accept()");
    }
    /* sim loop */
    while (1)
    {
      n = recv(c, dtbuf, sizeof(dtbuf) - 1, 0);
      dtbuf[n] = '\0';
      printf("Input: %s\n", dtbuf);
      /* reset */
      if ((strncmp("X", dtbuf, 2) == 0))
      {
        printf("Reset request\n");
        t = 0;
        motor_turn_on(u);
      }
      /* shut down simulation */
      else if ((strncmp("exit", dtbuf, 5) == 0))
      {
        report_err("Exit signal()");
      }
      /* tokenize with strtok, get the (expected) float values */
      else if (getctrl(dtbuf, &ud, &uq) == 0)
      {
        u[VD] = ud;
        u[VQ] = uq;
      }
      else
      {
        printf("Invalid input\n");
        u[VD] = 0;
        u[VQ] = 0;
      }
      step(t, t + TS, u);
      t += TS;
      /* this should be somewhere else */
      if (u[THETA] > 360)
      {
          u[THETA] = u[THETA]-360;
      }
      n = (int)snprintf(dtbuf, sizeof(dtbuf), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", t, u[ID], u[IQ], u[VD], u[VQ], u[WR], u[THETA]);

      z = write(c, dtbuf, n);

      if (z == -1)
      {
        report_err("Write()\n");
      }
    }
    close(c);
  }

  return 0;
}
