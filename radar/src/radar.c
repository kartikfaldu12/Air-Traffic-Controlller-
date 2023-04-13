#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/dispatch.h>
#include <pthread.h>

#define COLS 8
#define Z_OFFSET 15000
#define Z_RANGE 25000 + Z_OFFSET
#define Y_RANGE 100000
#define X_RANGE 100000

#define ATTACH_POINT "DFR"
#define COMPUTER_ATTACH_POINT "DFC"
typedef struct _pulse msg_header_t;

typedef struct server_data {
    msg_header_t hdr;
    int id;
	int posx,posy,posz;
	int spx,spy,spz;
	int time_s;

} server_data_t;

typedef struct client_data{
	msg_header_t hdr;
    int id;
	int posx,posy,posz;
	int spx,spy,spz;

} client_data_t;
struct top{

	int plane_count;
	struct plane *head;
	struct plane *current;
};


struct plane{
	int id;
	int posx,posy,posz;
	int spx,spy,spz;
	int time_s;
	struct plane *next;
};



void apply_constraint(struct plane *p){

	//for(int i=0;i<tptr->disk_plane_count;i++){
		p->posz+=Z_OFFSET;

		if(p->posx > X_RANGE-1){
			p->posx=X_RANGE-1;
		}
		if(p->posz > Z_RANGE-1){
			p->posz=Z_RANGE-1;
		}

		p->posy=0;
//		p[i]->time_s=i*5;
	//}

}

int read_csv(void * arg){
	char name[20]="Planes_overload.csv";
	char line[4098];
	struct top *tptr=(struct top*)arg;
	int *data=(int*)malloc(COLS*sizeof(int));
	/* open the shared memory segment */
	FILE *file;
	file = fopen(name, "r");
	while(fgets(line,4098,file)){
		char* tmp = strdup(line);
		char* tok;
		int i=0;
		data[i++]=tptr->plane_count;
		for(tok=strtok(line,",");  tok && *tok;tok = strtok(NULL, ",")){
			 data[i++]=atoi(tok);

		}
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
		tptr->current->id=data[0];
		tptr->current->posx=data[1];
		tptr->current->posy=data[2];
		tptr->current->posz=data[3];
		tptr->current->spx=data[4];
		tptr->current->spy=data[5];
		tptr->current->spz=data[6];
		tptr->current->time_s=data[7];
		apply_constraint(tptr->current);

		free(tmp);
		tptr->plane_count+=1;
	}

	fclose(file);


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
int radar_client(void *args){
	server_data_t msg;
	int server_coid; //server connection ID.
	struct top *tptr=(struct top*)args;


	if ((server_coid = name_open(ATTACH_POINT, 0)) == -1) {
		return EXIT_FAILURE;
	}

	/* We would have pre-defined data to stuff here */
	msg.hdr.type = 0x00;
	msg.hdr.subtype = 0x01;

//	msg.p=(struct plane*)malloc(sizeof(struct plane));
//	msg.p=(struct d*)malloc(sizeof(struct d));
	struct plane *t=(struct plane*)malloc(sizeof(struct plane));
	t=tptr->head;
	while(t!=NULL){
		msg.id=t->id;
		msg.posx=t->posx;
		msg.posy=t->posy;
		msg.posz=t->posz;
		msg.spx=t->spx;
		msg.spy=t->spy;
		msg.spz=t->spz;
		msg.time_s=t->time_s;
		printf("........");
		if (MsgSend(server_coid, &msg, sizeof(msg), NULL,0) == -1) {
			 fprintf (stderr, "[RADAR] Error during MsgSend\n");
			perror (NULL);
			exit (EXIT_FAILURE);
		}
		t=t->next;
	}

	/* Close the connection */
	name_close(server_coid);
	return EXIT_SUCCESS;
}

struct plane* update_plane_pos(struct top *tptr,client_data_t *data){
	struct plane *t=(struct plane *)malloc(sizeof(struct plane));
	t=tptr->head;
	while(t != NULL){
		if(t->id == data->id){
			t->posx=data->posx;
			t->posy=data->posy;
			t->posz=data->posz;
			return t;
		}
		t=t->next;
	}
}
int radar_server(void *args){
   name_attach_t *attach;
   client_data_t msg;
   int rcvid;
   struct top *tptr=(struct top*)args;

   /* Create a local name (/dev/name/local/...) */
   if ((attach = name_attach(NULL, COMPUTER_ATTACH_POINT, 0)) == NULL) {
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
			  struct plane *t=update_plane_pos(tptr,&msg);
			  display_plane(t,"[RADAR]");
		  }
	   }

	   MsgReply(rcvid, EOK, 0,0);

   }

   /* Remove the name from the space */
   name_detach(attach, 0);

   return EXIT_SUCCESS;
}

void send_data_to_server(void *arg){
	struct top *tptr=(struct top*)arg;
	printf("[RADAR] Sending Data To SERVER\n");
	radar_client(tptr);
}

void *show_display(void *arg){
	struct top *tptr=(struct top *)arg;
	printf("[RADAR] Opening Channel To Receive Data from Computer\n");
	radar_server(tptr);
}
int main(int argc, char const *argv[]) {
	struct top *tptr=(struct top*)malloc(sizeof(struct top));
	pthread_t radar_display;

	tptr->plane_count=0;
	/* code */
//		if (argc < 3){
//			printf("Please specify the CSV file as an input.\n");
//			exit(0);
//		}


	read_csv((void *)tptr);
	display_all(tptr,"[RADAR]");
	send_data_to_server((void*)tptr);
	pthread_create(&radar_display,NULL,&show_display,(void*)tptr);
	pthread_join(radar_display,NULL);
	return 0;
}
