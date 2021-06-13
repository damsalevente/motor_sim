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

#define TS 0.1 /* time step for simulation */

void report_err(const char *on_what)
{
  if(errno != 0)
  {
    fputs(strerror(errno), stderr);
    fputs(": ",stderr);
  }
  fputs(on_what, stderr);
  fputc('\n', stderr);
  exit(1);
}


int main() {
  /* motor parameters */
  float t; /* time */
  float ud, uq, ref_speed; /* control signals */
  float u[N]; /* buffer for output */
  /* server parameters */
  int z;
  char *srvr_addr = NULL;
  char *srvr_port = "9099";
  struct sockaddr_in adr_srvr;
  struct sockaddr_in adr_clnt;
  int len_inet;
  int s;
  int c;
  int n;
  time_t td;

  char dtbuf[256];
  srvr_addr = "127.0.0.1";

  s = socket(PF_INET, SOCK_STREAM,0);
  if(s == -1)
  {
    report_err("socket()");
  }
  memset(&adr_srvr, 0, sizeof(adr_srvr));
  adr_srvr.sin_family = AF_INET;
  adr_srvr.sin_port = htons(atoi(srvr_port));
  adr_srvr.sin_addr.s_addr = INADDR_ANY;

  len_inet = sizeof(adr_srvr);
  z = bind(s, (struct sockaddr *)&adr_srvr, len_inet);
  if(z == -1)
  {
    report_err("bind(2)");
  }
  z = listen(s,10);
  if(z == -1)
  {
    report_err("listen()");
  }

  /* motor prep */
  write_header();
  motor_turn_on(u);

  t = 0;
  /* 
  when a client connects, read out the control signals (ud,uq), and step 0.1
  send the id,iq currents, the input to verify, speed and rotor position 
  TODO: another option is to get only one connection, and continously play this loop: 
  client : send control
  server : get control, update motor, send observation 
  */
  while(1)
  {
    /* wait for connection */
    len_inet = sizeof adr_clnt;
    c = accept(s, (struct sockaddr *)&adr_clnt, &len_inet);
    if(c == -1)
    {
      report_err("accept()");
    }
    n = recv(c, dtbuf, sizeof(dtbuf) -1, 0);
    dtbuf[n] = '\0';
    /* tokenize with , to get the two float values */
    char *control = strtok(dtbuf, ",");
    ud = atof(control);
    control = strtok(NULL, ",");
    uq = atof(control); 
    u[VD] = ud;
    u[VQ] = uq;
    step(t, t + TS, u);
    t += TS;
    n = (int)snprintf(dtbuf,sizeof(dtbuf), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",t, u[ID],u[IQ],u[VD],u[VQ],u[WR],u[THETA]);
    z = write(c,dtbuf,n);
    if ( z == -1)
    {
      report_err("Write()");
    }
    close(c);
  }

  return 0;
}
