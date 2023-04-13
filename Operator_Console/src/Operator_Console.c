#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/dispatch.h>

#define COMPUTER_ATTACH_POINT_SERVER "SS"
#define COMPUTER_ATTACH_POINT_ClIENT "CC"
typedef struct _pulse msg_header_t;

typedef struct server_data {
    msg_header_t hdr;
    int code;
    int id_a;
    int id_b;
} server_data_t;

typedef struct data_from_computer1{
	int num_of_planes;
} plane_ids;

typedef struct data_from_computer2{
	int id;
	int posx,posy,posz;
	int spx,spy,spz;
	int time_s;
} plane_info;


typedef struct alert_data {
    msg_header_t hdr;
    int p_a;
    int p_b;
    int dist;
} alert_data_t;


static void *operator_console(void *args){
	int *scid=(int *)args;
	server_data_t msg;
	if ((*scid = name_open(COMPUTER_ATTACH_POINT_SERVER, 0)) == -1) {
		return EXIT_FAILURE;
	}
	int server_coid=*scid;
	char *message,*id;

	message=malloc(1);

	while(1){
		printf("[%d] Get Number of Planes\n",0);
		printf("[%d] Get Plane Data\n",1);
		scanf("%s",message);
		if(strcmp(message,"0") == 0){
			plane_ids buf;
			msg.hdr.type = 0x00;
			msg.hdr.subtype = 0x01;
			msg.code=0;
			if (MsgSend(server_coid, &msg, sizeof(msg), &buf,sizeof(plane_ids)) == -1) {
				 fprintf (stderr, "[COMPUTER] Error during MsgSend\n");
				perror (NULL);
				exit (EXIT_FAILURE);
			}
			/*--------------- Get Planes Array -------------- */
			if(!buf.num_of_planes == 0){
				msg.hdr.type = 0x00;
				msg.hdr.subtype = 0x01;
				msg.code=2;
				int plane_id_arr[buf.num_of_planes];
				if (MsgSend(server_coid, &msg, sizeof(msg), &plane_id_arr,sizeof(plane_id_arr)) == -1) {
					 fprintf (stderr, "[COMPUTER] Error during MsgSend\n");
					perror (NULL);
					exit (EXIT_FAILURE);
				}
				fprintf(stderr,"Planes:   ");
				for(int i=0;i<buf.num_of_planes;i++){
					fprintf(stderr,"Plane %d\t",plane_id_arr[i]);
				}
				fprintf(stderr,"\n");
			} else fprintf(stderr,"No Planes\n");

		}
		else if(strcmp(message,"1") == 0){
			id=malloc(1);
			printf("Enter Plane ID\n",0);
			scanf("%s",id);
			plane_info buf;
			msg.hdr.type = 0x00;
			msg.hdr.subtype = 0x01;
			msg.id_a=strtol(id,(char **)NULL,10);
			msg.code=1;
			buf.id=-1;
			if (MsgSend(server_coid, &msg, sizeof(msg), &buf,sizeof(plane_info)) == -1) {
				 fprintf (stderr, "[COMPUTER] Error during MsgSend\n");
				perror (NULL);
				exit (EXIT_FAILURE);
			}
			if(buf.id == -1) fprintf(stderr,"No Planes for ID: %d in En Route\n",msg.id_a);
			else{
				fprintf(stderr,"Data for Plane %d: ",buf.id);
				fprintf(stderr,"X = %dft Y = %dft Z = %dft SpeedX = %dms SpeedY = %dms SpeedZ = %dms ",buf.posx,buf.posy,buf.posz,buf.spx,buf.spy,buf.spz);
				fprintf(stderr,"Start Time %dsec\n",buf.time_s);
			}
		}
	}
	/* Close the connection */
	name_close(server_coid);
	return EXIT_SUCCESS;
}

void *ask_for_command(void * args){
	int *marr=(int *)args;
	server_data_t msg;

	fprintf(stderr,"[CONTROLLER] Sending COMMAND TO COMMUNICATION CHANNEL VIA COMPUTER\n");
	msg.hdr.type = 0x00;
	msg.hdr.subtype = 0x01;
	msg.code=3;
	msg.id_a=marr[0];
	msg.id_b=marr[1];
	if (MsgSend(marr[2], &msg, sizeof(msg), NULL,0) == -1) {
		 fprintf (stderr, "[CONTROLLER] Error during MsgSend\n");
		perror (NULL);
		exit (EXIT_FAILURE);
	}



}
static int *display_alert(void* args){
	int *server_coid=(int*)args;
	name_attach_t *attach;
   alert_data_t msg;
   int rcvid;
   int send=0;

   /* Create a local name (/dev/name/local/...) */
   if ((attach = name_attach(NULL, COMPUTER_ATTACH_POINT_ClIENT, 0)) == NULL) {
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
			  fprintf(stderr,"[ALERT!!!] COLLISION Between Plane %d and Plane %d. Distance: %d\n",msg.p_a,msg.p_b,msg.dist);
			  int msg_arr[3]={msg.p_a,msg.p_b,*server_coid};
			  pthread_t send_command;
			  pthread_create(&send_command,NULL,&ask_for_command,&msg_arr);
		  }
	   }

	   MsgReply(rcvid, EOK, 0,0);


//	   pthread_t command_opt;
//	   pthread_create(&command_opt,NULL,&ask_for_command,(void *)NULL);

   }

   /* Remove the name from the space */
   name_detach(attach, 0);

   return EXIT_SUCCESS;
}

int main(void) {
	int server_coid; //server connection ID.
	pthread_t console,for_alert;
	pthread_create(&for_alert,NULL,&display_alert,&server_coid);
	pthread_create(&console,NULL,&operator_console,&server_coid);
	pthread_join(console,NULL);
	return EXIT_SUCCESS;
}
