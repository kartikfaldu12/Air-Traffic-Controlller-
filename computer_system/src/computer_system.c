#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/dispatch.h>
#include <pthread.h>
#include <sys/time.h>
#include <time.h>
#include <signal.h>

#define SERVER_INIT_ATTACH_POINT "DFR"
#define RADAR_ATTACH_POINT "DFC"
#define CONSOLE_ATTACH_POINT_SERVER "SS"
#define CONSOLE_ATTACH_POINT_CLIENT "CC"
#define COMMUNICATION_ATTACH_POINT "COM"
typedef struct _pulse msg_header_t;

typedef struct server_data {
    msg_header_t hdr;
    int id;
	int posx,posy,posz;
	int spx,spy,spz;
	int time_s;
} server_data_t;

typedef struct client_data {
    msg_header_t hdr;
    int id;
	int posx,posy,posz;
	int spx,spy,spz;
	int time_s;
} client_data_t;

typedef struct oc_server_data {
    msg_header_t hdr;
    int code;
    int id_a;
    int id_b;
} oc_server_data_t;

typedef struct oc_client_data {
    msg_header_t hdr;
    int p_a;
    int p_b;
    int dist;
} oc_client_data_t;

typedef struct data_for_operator_console1{
	int num_of_planes;
}plane_ids;

typedef struct data_for_operator_console2{
	int id;
	int posx,posy,posz;
	int spx,spy,spz;
	int time_s;
}plane_info;

typedef struct comm_data{
	msg_header_t hdr;
	int id;
}comm_data_t;
struct top{

	int plane_count;
	int time_u;
	timer_t radar_timer;
	struct itimerspec *old_spec;
	struct plane *head;
	struct plane *current;
	struct planes_to_collide *c_head;
	struct planes_to_collide *c_current;
};
struct planes_to_collide{
	int p_a;
	int p_b;
	int dist;
	struct planes_to_collide *next;
};

struct plane{
	int id;
	int posx,posy,posz;
	int spx,spy,spz;
	int time_s;
	int en_route;
	timer_t timer;
	pthread_t thrd;
	struct plane *next;
};

static float find_sqrt(double x){
    double y;
    int p,square,c;

    /* find the surrounding perfect squares */
    p = 0;
    do
    {
        p++;
        square = (p+1) * (p+1);
    }
    while( x > square );

    /* process the root */
    y = (double)p;
    c = 0;
    while(c<10)
    {
        /* divide and average */
        y = (x/y + y)/2;
        /* test for success */
        if( y*y == x)
            return(y);
        c++;
    }
    return(y);
}

int *get_new_data(void *arg) {
   name_attach_t *attach;
   server_data_t msg;
   int rcvid;
   struct top *tptr=(struct top*)arg;

   /* Create a local name (/dev/name/local/...) */
   if ((attach = name_attach(NULL, SERVER_INIT_ATTACH_POINT, 0)) == NULL) {
       return EXIT_FAILURE;
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
	    	  add_plane_to_mem(tptr,&msg);
	    	  tptr->plane_count++;
	    	  run_plane(tptr->current);
	    	  fprintf(stderr,"[CONTROLLER] # of Planes in the Sky %d\n",tptr->plane_count);
	      }
	   }

       MsgReply(rcvid, EOK, 0,0);

   }

   /* Remove the name from the space */
   name_detach(attach, 0);

   return EXIT_SUCCESS;
}

static void timer_handler(int sig,siginfo_t *si,void *uc){
	struct plane *p=si->si_value.sival_ptr;
	p->posy+=p->spy;
	p->en_route=1;
//	fprintf(stderr,"Plane %d running at PosX %d\n",p->id,p->posx);
}

int *send_alert_to_operator(void* args){
	oc_client_data_t msg;
	int server_coid; //server connection ID.
	if ((server_coid = name_open(CONSOLE_ATTACH_POINT_CLIENT, 0)) == -1) {
		return EXIT_FAILURE;
	}
//
//	/* We would have pre-defined data to stuff here */
	msg.hdr.type = 0x00;
	msg.hdr.subtype = 0x01;
	struct top *tptr=(struct top*)args;
	struct planes_to_collide *t=(struct planes_to_collide *)malloc(sizeof(struct planes_to_collide));
	t=tptr->c_head;
	while(t->next != NULL){
		msg.p_a=t->p_a;
		msg.p_b=t->p_b;
		msg.dist=t->dist;
		if (MsgSend(server_coid, &msg, sizeof(msg), NULL,0) == -1) {
			 fprintf (stderr, "[COMPUTER] Error during MsgSend\n");
			perror (NULL);
			exit (EXIT_FAILURE);
		}
//
		t=t->next;
	}
//
//	/* Close the connection */
	name_close(server_coid);

	return EXIT_SUCCESS;
}
static void *detect_collision(void *args){
	struct top *tptr=(struct top*)(args);
	struct plane *i=(struct plane *)malloc(sizeof(struct plane));
	struct plane *j=(struct plane *)malloc(sizeof(struct plane));

	tptr->c_head=(struct planes_to_collide*)malloc(sizeof(struct planes_to_collide));
	tptr->c_head->next=NULL;
	tptr->c_current=tptr->c_head;
	i=tptr->head;
	while(i!=NULL){
		if(!i->en_route) { i=i->next; continue;}
		j=i->next;
		while(j!=NULL){
			if(!j->en_route) { j=j->next; continue;}
			int x=(i->posx - j->posx);
			int y=(i->posy - j->posy);
			int z=(i->posz - j->posz);
			if(abs(x) < 3000){
				if(abs(y) < 3000){
					if(abs(z) < 1000){
						tptr->c_current->p_a=i->id;
						tptr->c_current->p_b=j->id;
						tptr->c_current->dist=abs(y);
						tptr->c_current->next=(struct planes_to_collide*)malloc(sizeof(struct planes_to_collide));
						tptr->c_current=tptr->c_current->next;
						tptr->c_current->next=NULL;
					}
				}
			}
			j=j->next;
		}
		i=i->next;
	}
	if(tptr->c_current != tptr->c_head){
		pthread_t to_operator;
		pthread_create(&to_operator,NULL,&send_alert_to_operator,(void*)tptr);

	}
}

int send_data_to_radar(struct top *tptr){
	client_data_t msg;
	int server_coid; //server connection ID.

	if ((server_coid = name_open(RADAR_ATTACH_POINT, 0)) == -1) {
		return EXIT_FAILURE;
	}

	/* We would have pre-defined data to stuff here */
	msg.hdr.type = 0x00;
	msg.hdr.subtype = 0x01;

	struct plane *t=(struct plane *)malloc(sizeof(struct plane));
	t=tptr->head;
	while(t != NULL){
		if(t->en_route){
			msg.id=t->id;
			msg.posx=t->posx;
			msg.posy=t->posy;
			msg.posz=t->posz;
			msg.spx=t->spx;
			msg.spy=t->spy;
			msg.spz=t->spz;
			if (MsgSend(server_coid, &msg, sizeof(msg), NULL,0) == -1) {
				 fprintf (stderr, "[COMPUTER] Error during MsgSend\n");
				perror (NULL);
				exit (EXIT_FAILURE);
			}
		}
		t=t->next;
	}

	/* Close the connection */
	name_close(server_coid);

	pthread_t detect_collision_p;
	pthread_create(&detect_collision_p,NULL,&detect_collision,(void*)tptr);
	return EXIT_SUCCESS;
}


static void send_updates_to_radar(int sig,siginfo_t *si,void *uc){
	struct top *tptr=si->si_value.sival_ptr;
	send_data_to_radar(tptr);

}
static int start_radar_timer(struct top *tptr,unsigned sec,unsigned msec,unsigned period){
	struct sigevent sigev;
	struct sigaction sa;
	int res;
	const int sig=SIGRTMIN+1;


	//p->sig=SIGALRM;
	//next_sig++;
	sa.sa_flags=SA_SIGINFO;
	sa.sa_sigaction=send_updates_to_radar;
	sigemptyset(&sa.sa_mask);

	if(sigaction(sig,&sa,NULL) == -1){
		perror("sigaction error\n");
		return -1;
	}

	sigev.sigev_notify = SIGEV_SIGNAL;
	sigev.sigev_signo = sig;
	sigev.sigev_value.sival_ptr=(void *)tptr;

	/* create timer */
	res = timer_create(CLOCK_MONOTONIC, &sigev, &tptr->radar_timer);

	if (res < 0) {
		perror("timer_reate error");
		return -1;

	}

	/* set timer parameters */

	tptr->old_spec->it_value.tv_sec = sec;
	tptr->old_spec->it_value.tv_nsec = msec * 1000000;
	tptr->old_spec->it_interval.tv_sec = period;
	tptr->old_spec->it_interval.tv_nsec = msec * 1000000;
	return timer_settime(tptr->radar_timer, 0, tptr->old_spec, NULL);


}
static int start_periodic_timer(timer_t t,void (handler)(int,siginfo_t *,void *),int signal,void *pdata,unsigned sec,unsigned msec,unsigned period) {
	struct itimerspec *timer_spec;
	struct sigevent sigev;
	struct sigaction sa;
	int res;
	const int sig=signal;


	//p->sig=SIGALRM;
	//next_sig++;
	sa.sa_flags=SA_SIGINFO;
	sa.sa_sigaction=handler;
	sigemptyset(&sa.sa_mask);

	if(sigaction(sig,&sa,NULL) == -1){
		perror("sigaction error\n");
		return -1;
	}

	sigev.sigev_notify = SIGEV_SIGNAL;
	sigev.sigev_signo = sig;
	sigev.sigev_value.sival_ptr=pdata;

	/* create timer */
	res = timer_create(CLOCK_MONOTONIC, &sigev, &t);

	if (res < 0) {
		perror("timer_reate error");
		return -1;

	}

	/* set timer parameters */
	timer_spec=(struct itimerspec*)malloc(sizeof(struct itimerspec));
	timer_spec->it_value.tv_sec = sec;
	timer_spec->it_value.tv_nsec = msec * 1000000;
	timer_spec->it_interval.tv_sec = period;
	timer_spec->it_interval.tv_nsec = msec * 1000000;
	return timer_settime(t, 0, timer_spec, NULL);


}
static void vol(void *arg){
	struct plane *p=(struct plane *)arg;
	start_periodic_timer(p->timer,timer_handler,SIGRTMIN,p,(p->time_s+1),0,1);

}
void run_plane(struct plane *p){
	pthread_create(&p->thrd,NULL,vol,(void *)p);
}
void add_plane_to_mem(struct top *tptr,server_data_t *p){
	fprintf(stderr,"[COMPUTER] Adding Plane %d To Memory\n",p->id);
	if(tptr->head == NULL){
		tptr->head=(struct plane *)malloc(sizeof(struct plane));
		tptr->head->next=NULL;
		tptr->current=tptr->head;
	}
	else{
		tptr->current->next=(struct plane *)malloc(sizeof(struct plane));
		tptr->current=tptr->current->next;
		tptr->current->next=NULL;
	}
	tptr->current->id=p->id;
	tptr->current->posx=p->posx;
	tptr->current->posy=p->posy;
	tptr->current->posz=p->posz;
	tptr->current->spx=p->spx;
	tptr->current->spy=p->spy;
	tptr->current->spz=p->spz;
	tptr->current->time_s=p->time_s;
	tptr->current->en_route=0;

}
void display_all(struct top *tptr,char title[]){
	struct plane *t=(struct plane *)malloc(sizeof(struct plane));
	t=tptr->head;
	while(t != NULL){
		display_plane(t,title);
		t=t->next;
	}
}
void display_plane(struct plane *p,char s[]){
	//for(int i=0;i<tptr->disk_plane_count;i++){
		printf("%s Data for Plane %d: ",s,p->id);
		printf("X = %dft Y = %dft Z = %dft SpeedX = %dms SpeedY = %dms SpeedZ = %dms ",p->posx,p->posy,p->posz,p->spx,p->spy,p->spz);
		printf("Start Time %dsec\n",p->time_s);
	//}
}

int *send_command_to_comm(void *args){
	int *ocmsg=(int*)args;
	comm_data_t msg;
	int server_coid; //server connection ID.

	if ((server_coid = name_open(COMMUNICATION_ATTACH_POINT, 0)) == -1) {
		return EXIT_FAILURE;
	}

	/* We would have pre-defined data to stuff here */
	for(int i=0;i<2;i++){
		msg.hdr.type = 0x00;
		msg.hdr.subtype = 0x01;
		msg.id=ocmsg[i];
		if (MsgSend(server_coid, &msg, sizeof(msg), NULL,0) == -1) {
			 fprintf (stderr, "[COMPUTER] Error during MsgSend\n");
			perror (NULL);
			exit (EXIT_FAILURE);
		}
	}

	/* Close the connection */
	name_close(server_coid);
	return EXIT_SUCCESS;
}
void *answer_operator_console(void *args){
	struct top *tptr=(struct top *)args;
	name_attach_t *attach;
	oc_server_data_t msg;
	int rcvid;

	/* Create a local name (/dev/name/local/...) */
	if ((attach = name_attach(NULL, CONSOLE_ATTACH_POINT_SERVER, 0)) == NULL) {
	   return EXIT_FAILURE;
	}

	/* Do your MsgReceive's here now with the chid */
	while (1) {

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
			  if(msg.code == 0){
				  plane_ids *data=(struct plane_ids*)malloc(sizeof(plane_ids));
				  data->num_of_planes=tptr->plane_count;
				  MsgReply(rcvid, EOK, data,sizeof(plane_ids));
			  }
			  else if(msg.code == 1){
				  plane_info *data=(struct plane_info*)malloc(sizeof(plane_info));
				  struct plane *t=(struct plane*)malloc(sizeof(struct plane));
				  t=tptr->head;
				  int f=0;
				  while(t!=NULL){
					  if(msg.id_a == t->id){
						  f=1;
						  data->id=t->id;
						  data->posx=t->posx;
						  data->posy=t->posy;
						  data->posz=t->posz;
						  data->spx=t->spx;
						  data->spy=t->spy;
						  data->spz=t->spz;
						  data->time_s=t->time_s;
						  MsgReply(rcvid, EOK, data,sizeof(plane_info));
						  break;
					  }
					  t=t->next;

				  }
				  if(!f) {
					  MsgReply(rcvid, EOK, 0,0);
				  }
			  }
			  else if(msg.code == 2){
//				  int *arr=malloc(tptr->plane_count);
				  int arr[tptr->plane_count];
				  struct plane *t=(struct plane*)malloc(sizeof(struct plane));
				  t=tptr->head;
				  int i=0;
				  while(t!=NULL){
					  arr[i]=t->id;
					  t=t->next;
					  i+=1;
				  }
				  MsgReply(rcvid, EOK, arr,sizeof(arr));
			  }
			  else if(msg.code == 3){
				  fprintf(stderr,"Receive Command for Planes %d and %d\n",msg.id_a,msg.id_b);
				  int cmsg[2]={msg.id_a,msg.id_b};
				  pthread_t to_comm;
				  pthread_create(&to_comm,NULL,&send_command_to_comm,&cmsg);
			  }
			  else{
				  MsgReply(rcvid, EOK, 0,0);
			  }
		  }

	   }



	}

	/* Remove the name from the space */
	name_detach(attach, 0);

	return EXIT_SUCCESS;
}
void *get_val_from_usr(void *args){
	char *val;
	struct top *tptr=(struct top *)args;

	val=malloc(10);
	while(tptr->plane_count == 0){
		sleep(1);
	}
	tptr->old_spec=(struct itimerspec*)malloc(sizeof(struct itimerspec));
	start_radar_timer(tptr,5,0,5);
	printf("[CONTROLLER] Timer Value: %d.\n",tptr->time_u);

	while(1){
		fprintf(stderr,"\n[CONTROLLER] Enter New Value: ");
		scanf("%s",val);
		int t=strtol(val,(char **)NULL,10);
		if(t < 2) {
			fprintf(stderr,"[CONTROLLER] Please Enter Value Greater than 2");
			continue;
		}
		tptr->time_u=t;
		fprintf(stderr,"[CONTROLLER] New Update Timer Value: %d\n",tptr->time_u);

		struct itimerspec *new_spec=(struct itimerspec*)malloc(sizeof(struct itimerspec));
		new_spec->it_value.tv_sec = tptr->time_u;
		new_spec->it_value.tv_nsec = 0;
		new_spec->it_interval.tv_sec = tptr->time_u;
		new_spec->it_interval.tv_nsec = 0;
		timer_settime(tptr->radar_timer, 0, new_spec, tptr->old_spec);
		tptr->old_spec=new_spec;
//		start_periodic_timer(tptr->radar_timer,send_updates_to_radar,SIGALRM,tptr,tptr->time_u,0,tptr->time_u);
	}

}

int main(void) {
	pthread_t _init_data,console,operator_console;
	struct top *tptr=(struct top*)malloc(sizeof(struct top));

	tptr->plane_count=0;
	tptr->time_u=5;
	pthread_create(&_init_data,NULL,&get_new_data,(void*)tptr);
	pthread_create(&console,NULL,&get_val_from_usr,(void *)tptr);
	pthread_create(&operator_console,NULL,&answer_operator_console,(void *)tptr);
	pthread_join(_init_data,NULL);
	return EXIT_SUCCESS;
}
