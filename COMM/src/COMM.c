#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/dispatch.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#define ONE_THOUSAND	1000
#define ONE_MILLION		1000000
#define COMMUNICATION_CHANNEL "COM"
typedef struct _pulse msg_header_t;

typedef struct communication_data {
    msg_header_t hdr;
    int id;
} data_t;

struct top_t{
	struct log_data_t *head;
	struct log_data_t *current;
};
struct log_data_t{
	char message[50];
	char time[20];
	struct log_data_t *next;
};

int *comm_channel_server(void *args){
	static int cycles = 0;
	static uint64_t start;
	uint64_t current;
	struct timespec tv;
	name_attach_t *attach;
	struct top_t *tptr=(struct top_t*)args;
	   data_t msg;
	   int rcvid;


	   /* Create a local name (/dev/name/local/...) */
	   if ((attach = name_attach(NULL, COMMUNICATION_CHANNEL, 0)) == NULL) {
		   return EXIT_FAILURE;
	   }
	   if (start == 0) {
	   		clock_gettime(CLOCK_MONOTONIC, &tv);
	   		start = tv.tv_sec * ONE_THOUSAND + tv.tv_nsec / ONE_MILLION;
	   	}
	   /* Do your MsgReceive's here now with the chid */
	   while (1) {
	//	   msg=(server_data_t)malloc(sizeof(server_data_t));
		   rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);

		   if (rcvid == -1) {/* Error condition, exit */
			   break;
		   }
		   /* name_open() sends a connect message, must EOK this */
		   if (msg.hdr.type == _IO_CONNECT ) {
			   MsgReply( rcvid, EOK, NULL, 0 );
			   continue;
		   }

		   /* Some other QNX IO message was received; reject it */
		   if (msg.hdr.type > _IO_BASE && msg.hdr.type <= _IO_MAX ) {
			   MsgError( rcvid, ENOSYS );
			   continue;
		   }

		   if (msg.hdr.type == 0x00) {
			  if (msg.hdr.subtype == 0x01) {
				  if(tptr->head == NULL){
				  		tptr->head=(struct log_data_t*)malloc(sizeof(struct log_data_t));
				  		tptr->head->next=NULL;
				  		tptr->current=tptr->head;
				  	}
				  	else{
				  		tptr->current->next=(struct log_data_t *)malloc(sizeof(struct log_data_t));
				  		tptr->current=tptr->current->next;
				  		tptr->current->next=NULL;

				  	}

				  char m[50]=": Sending Command to Plane ";
				  char id[5];
				  itoa(msg.id,id,10);
				  strcat(m,id);
				  strcpy(tptr->current->message,m);

				  clock_gettime(CLOCK_MONOTONIC, &tv);
				  current = tv.tv_sec * ONE_THOUSAND + tv.tv_nsec / ONE_MILLION;
				if (cycles > 0) {
					char time_s[20];
					int a=current-start;
					itoa(a,time_s,10);
					strcpy(tptr->current->time,time_s);
				}

				fprintf(stderr,"[COMMUNICATION] Sending Command to Plane %d\n",msg.id);
				cycles++;
			  }
		   }

		   MsgReply(rcvid, EOK, 0,0);

	   }

	   /* Remove the name from the space */
	   name_detach(attach, 0);

	   return EXIT_SUCCESS;
}

void write_log(struct log_data_t *t){
	FILE *fp;
	fp  = fopen ("logs.log", "a");
	fprintf(fp, "%s: %s\n",t->time,t->message);
	fclose(fp);
}
void add_space(){
	FILE *fp;
	fp  = fopen ("logs.log", "a");
	fprintf(fp, "------------------------------\n");
	fclose(fp);
}

static void timer_handler(int sig,siginfo_t *si,void *uc){


	struct top_t *tptr=si->si_value.sival_ptr;
	struct log_data_t *t=(struct log_data_t *)malloc(sizeof(struct log_data_t));
	t=tptr->head;






	while(t != NULL){

		write_log(t);
		t=t->next;
	}


	//start=current;

	add_space();
	tptr->head=NULL;
}
int start_periodic_timer(struct top_t *pdata,unsigned sec,unsigned msec,unsigned period) {
	struct itimerspec timer_spec;
	struct sigevent sigev;
	struct sigaction sa;
	timer_t timer;
	int res;
	const int sig=SIGRTMIN+5;


	//p->sig=SIGALRM;
	//next_sig++;
	sa.sa_flags=SA_SIGINFO;
	sa.sa_sigaction=timer_handler;
	sigemptyset(&sa.sa_mask);

	if(sigaction(sig,&sa,NULL) == -1){
		perror("sigaction error\n");
		return -1;
	}

	sigev.sigev_notify = SIGEV_SIGNAL;
	sigev.sigev_signo = sig;
	sigev.sigev_value.sival_ptr=pdata;

	/* create timer */
	res = timer_create(CLOCK_MONOTONIC, &sigev, &timer);

	if (res < 0) {
		perror("timer_reate error");
		return -1;

	}

	/* set timer parameters */

	timer_spec.it_value.tv_sec = sec;
	timer_spec.it_value.tv_nsec = msec * 1000000;
	timer_spec.it_interval.tv_sec = period;
	timer_spec.it_interval.tv_nsec = msec * 1000000;
	return timer_settime(timer, 0, &timer_spec, NULL);
//	return timer;
}
int main(void) {
	pthread_t comm_channel;
	struct top_t *tptr=(struct top_t*)malloc(sizeof(struct top_t));

	pthread_create(&comm_channel,NULL,&comm_channel_server,(void*)tptr);
	start_periodic_timer(tptr,10,0,10);
	pthread_join(comm_channel,NULL);
	return EXIT_SUCCESS;
}
