#include "CCDStar.h"  //Params.h
#include "WorkHorse.h"
#include "GridMap.h"

extern WorkHorse *WH;
extern GridMap *GM;
extern DStar *DS;
extern Planner *PL;

void CCDStar::setCoverageOnCoil(double RBx, double RBy, double size){
			
				GM->mapper_point_temp.x=RBx*1000.; //mm
				GM->mapper_point_temp.y=RBy*1000.;
				GM->mapper_point_temp.th=0;
      if (prviput) {
        time_stamp_counter=2;
      }
  int prosirenje=ceil(size/GM->Map_Cell_Size);
  if (GM->check_point(GM->mapper_point_temp)){
    for(int i=GM->cell_point_temp.x-prosirenje;i<=GM->cell_point_temp.x+prosirenje;i++){
					for(int j=GM->cell_point_temp.y-prosirenje;j<=GM->cell_point_temp.y+prosirenje;j++){
						if ((IsValid(i,j)>0)){
						
							if (map[i][j].tag_preklop<time_stamp_counter-1){
								map[i][j].presao=std::max(map[i][j].presao,0)+1;
								map[i][j].tag_preklop=time_stamp_counter;
							}							
							else if (map[i][j].tag_preklop!=time_stamp_counter){
								map[i][j].tag_preklop=time_stamp_counter;
							}

						}
					}
		}
  }
}

int CCDStar::setCoverageOnRobotCurrentPosition(double RBx, double RBy, double RBth){
//zakomentirala sam postavljanje posjecenosti - sad samo coil postavlja - pazi da vratis nazad			
      time_stamp_counter++;
				GM->mapper_point_temp.x=RBx;
				GM->mapper_point_temp.y=RBy;
				GM->mapper_point_temp.th=RBth;
				GM->check_point(GM->mapper_point_temp);
				int celx=GM->cell_point_temp.x;
				int cely=GM->cell_point_temp.y;
				int numnewcov=0;
				Start.x=celx;
				Start.y=cely;
				Start.th=0;
      if (prviput) {
        time_stamp_counter=2;
        PL->reset();
        DS->InitOri(Start,Start);//treba zbog maxOri
      }
			if (1) //(prviput==0) 
			{
				double sirina=700.;//550.;//700.;//1530.;//tolko je sirok
				double duljina=1100;//550.;//1100.;//3005.;
				double flaill=0.;//700.;//1450.;//tool dodatak na duzinu
				double flailw=0.;//485.;//tool //to je dodatak na sirinu
//				int pomak_x=celx-StartRacunac.x;
//				int pomak_y=cely-StartRacunac.y;
//				int prviput=0;
//				if (map[celx][cely].presao==0)
//					prviput=1;
				//mozda zbog pogreske lokalizacije pomak nece biti manji od polja po ciklusu (a to je koristena pretpostavka)
				//cijela maska robota
				double kut = RBth;
				double orientation = kut;
				double orientation2 = kut+M_PI/2;
				double distmp=sirina/2.+0.25*(GM->Map_Cell_Size); //kolko je udaljen paralelni brid (micem dodatke)
				double xleft=RBx-(duljina+0.25*(GM->Map_Cell_Size))/2*cos(kut);//micem dodatke
				double yleft=RBy-(duljina+0.25*(GM->Map_Cell_Size))/2*sin(kut);//micem dodatke
				R_point next, nextedge, temp_r, tool_r;
				tool_r.x=RBx+distance_robot_tool*cos(kut);
				//tool_r.y=RBy+(duljina/2+flaill/2)*sin(kut);
				tool_r.y=RBy+distance_robot_tool*sin(kut);
				tool_r.th=RBth;

		next.x=xleft;
		next.y=yleft;
		nextedge.x=next.x-distmp*cos(orientation2);
		nextedge.y=next.y-distmp*sin(orientation2);
//		if (GM->check_point(next)){ //necu tu provjeru ako slucajno izlazi van okvira guzica

				printf("next=[%f,%f], nextedge=[%f,%f], orientation=%f deg, orientation2=%f deg\n",next.x,next.y,nextedge.x,nextedge.y,orientation*RuS,orientation2*RuS);

				//za vozilo
				while (sqrt((next.x - xleft) * (next.x - xleft) + (next.y - yleft) *
        (next.y - yleft)) < duljina + 0.5*GM->Map_Cell_Size) //micem dodatke
				{

					temp_r.x = nextedge.x;// + (GM->Map_Cell_Size/2) * cos(orientation2); //tocka se pomice 
					temp_r.y = nextedge.y;// + (GM->Map_Cell_Size/2) * sin(orientation2);
					temp_r.th=0;
					while (sqrt((temp_r.x - nextedge.x) * (temp_r.x - nextedge.x) + (temp_r.y - nextedge.y) *
		(temp_r.y - nextedge.y)) < 2*distmp)//iza jos tolko zato 2
					{
//						if ((RBx-temp_r.x)*(RBx-temp_r.x)+(RBy-temp_r.y)*(RBy-temp_r.y)<RR*RR) //kruzni robot
						if (GM->check_point(temp_r)){
							if (map[GM->cell_point_temp.x][GM->cell_point_temp.y].total_cost_int<time_stamp_counter-1){
//							if (map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag_preklop<time_stamp_counter-1){
								map[GM->cell_point_temp.x][GM->cell_point_temp.y].presao=std::max(map[GM->cell_point_temp.x][GM->cell_point_temp.y].presao,0)+1;//oznaka presao za vozilo
								map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag_preklop=time_stamp_counter;//oznaka presao za vozilo
								map[GM->cell_point_temp.x][GM->cell_point_temp.y].total_cost_int=time_stamp_counter;
								numnewcov++;
//								if ((WH->processState==RUNNING) && (IsValid(GM->cell_point_temp.x,GM->cell_point_temp.y)==2)){
//								  GM->indeksimapepraznjenje[GM->numindeksimapepraznjenje]=GM->cell_point_temp;   //azuriram indekse za planera za brisanje
//				          GM->numindeksimapepraznjenje++;
//				        }
							}							
							else if (map[GM->cell_point_temp.x][GM->cell_point_temp.y].total_cost_int!=time_stamp_counter){
//							else if (map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag_preklop!=time_stamp_counter){
								map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag_preklop=time_stamp_counter;//oznaka presao za vozilo
								map[GM->cell_point_temp.x][GM->cell_point_temp.y].total_cost_int=time_stamp_counter;
//								if ((WH->processState==RUNNING) && (IsValid(GM->cell_point_temp.x,GM->cell_point_temp.y)==2)){
//								  GM->indeksimapepraznjenje[GM->numindeksimapepraznjenje]=GM->cell_point_temp;   //azuriram indekse za planera za brisanje
//				          GM->numindeksimapepraznjenje++;
//				        }
							}

						}
						temp_r.x += (GM->Map_Cell_Size/4) * cos(orientation2);
						temp_r.y += (GM->Map_Cell_Size/4) * sin(orientation2);
						//printf("temp_r=[%f,%f]\t",temp_r.x,temp_r.y);
					}
					
					next.x += (GM->Map_Cell_Size/4) * cos(orientation);
					next.y += (GM->Map_Cell_Size/4) * sin(orientation);
					nextedge.x=next.x-distmp*cos(orientation2);
					nextedge.y=next.y-distmp*sin(orientation2);

					//printf("next=[%f,%f], nextedge=[%f,%f]\t",next.x,next.y,nextedge.x,nextedge.y);
				} // end while
				
				//za tool
				distmp=sirina/2.+flailw/2.+0*(GM->Map_Cell_Size/2.);//micem dodatke
				nextedge.x=next.x-distmp*cos(orientation2);
				nextedge.y=next.y-distmp*sin(orientation2);
				while (sqrt((next.x - xleft) * (next.x - xleft) + (next.y - yleft) *
        (next.y - yleft)) < duljina+flaill+0*(GM->Map_Cell_Size))//malo varam (micem dodatke)
				{

					temp_r.x = nextedge.x;// + (GM->Map_Cell_Size/2) * cos(orientation2); //tocka se pomice 
					temp_r.y = nextedge.y;// + (GM->Map_Cell_Size/2) * sin(orientation2);
					temp_r.th=0;
					while (sqrt((temp_r.x - nextedge.x) * (temp_r.x - nextedge.x) + (temp_r.y - nextedge.y) *
		(temp_r.y - nextedge.y)) < 2*distmp)//iza jos tolko zato 2
					{
						if (GM->check_point(temp_r))
						{
							if (map[GM->cell_point_temp.x][GM->cell_point_temp.y].total_cost_int<time_stamp_counter-1){
//							if (map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag_preklop<time_stamp_counter-1){
								map[GM->cell_point_temp.x][GM->cell_point_temp.y].presao=std::max(map[GM->cell_point_temp.x][GM->cell_point_temp.y].presao,0)+1;//presao za tool
								map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag_preklop=time_stamp_counter;//presao za tool
								map[GM->cell_point_temp.x][GM->cell_point_temp.y].total_cost_int=time_stamp_counter;
								numnewcov++;
//								if ((WH->processState==RUNNING) && (IsValid(GM->cell_point_temp.x,GM->cell_point_temp.y)==2)){
//								  GM->indeksimapepraznjenje[GM->numindeksimapepraznjenje]=GM->cell_point_temp;   //azuriram indekse za planera za brisanje
//				          GM->numindeksimapepraznjenje++;
//				        }
							}
							else if (map[GM->cell_point_temp.x][GM->cell_point_temp.y].total_cost_int!=time_stamp_counter){
//							else if (map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag_preklop!=time_stamp_counter){
								map[GM->cell_point_temp.x][GM->cell_point_temp.y].tag_preklop=time_stamp_counter;//presao za tool
								map[GM->cell_point_temp.x][GM->cell_point_temp.y].total_cost_int=time_stamp_counter;
//								if ((WH->processState==RUNNING) && (IsValid(GM->cell_point_temp.x,GM->cell_point_temp.y)==2)){
//								  GM->indeksimapepraznjenje[GM->numindeksimapepraznjenje]=GM->cell_point_temp;   //azuriram indekse za planera za brisanje
//				          GM->numindeksimapepraznjenje++;
//				        }
							}
						}
						temp_r.x += (GM->Map_Cell_Size/4) * cos(orientation2);
						temp_r.y += (GM->Map_Cell_Size/4) * sin(orientation2);
						//printf("temp_r=[%f,%f]\t",temp_r.x,temp_r.y);
					}
					
					next.x += (GM->Map_Cell_Size/4) * cos(orientation);
					next.y += (GM->Map_Cell_Size/4) * sin(orientation);
					nextedge.x=next.x-distmp*cos(orientation2);
					nextedge.y=next.y-distmp*sin(orientation2);

					//printf("next=[%f,%f], nextedge=[%f,%f]\t",next.x,next.y,nextedge.x,nextedge.y);
				} // end while

		//}//checkpoint	
		if (GM->check_point(tool_r)){
				tool=GM->cell_point_temp;//u TSP se lose odredjuje prema aproksimaciji kuta na 4 smjera
				for(int i=GM->cell_point_temp.x-preklapanje;i<=GM->cell_point_temp.x+preklapanje;i++){
					for(int j=GM->cell_point_temp.y-preklapanje;j<=GM->cell_point_temp.y+preklapanje;j++){
						if (IsValid(i,j)==1){
							map[i][j].presao7=1;
						}
					}
				}
				//za tool sad komentiram isto, ionako je visak
#if 0
				for(int i=GM->cell_point_temp.x-robot_mask;i<=GM->cell_point_temp.x+robot_mask;i++){
					for(int j=GM->cell_point_temp.y-robot_mask;j<=GM->cell_point_temp.y+robot_mask;j++){
						if ((IsValid(i,j)>0)){
							if (map[i][j].tag_preklop<time_stamp_counter-1){
								map[i][j].presao=std::max(map[i][j].presao,0)+1;
								map[i][j].tag_preklop=time_stamp_counter;
							}							
							else if (map[i][j].tag_preklop!=time_stamp_counter){
								map[i][j].tag_preklop=time_stamp_counter;
							}
					
						}
					}
				}
#endif		
		}
		//ne za robota, samo za tool (0)
#if 0
				for(int i=celx-robot_mask;i<=celx+robot_mask;i++){
					for(int j=cely-robot_mask;j<=cely+robot_mask;j++){
						if (map[i][j].tag_preklop<time_stamp_counter-1){
								map[i][j].presao=map[i][j].presao+1;
								map[i][j].tag_preklop=time_stamp_counter;
						}
						else if (map[i][j].tag_preklop!=time_stamp_counter){
								map[i][j].tag_preklop=time_stamp_counter;
						}
						
					}
				}
				
				for(int i=celx-preklapanje;i<=celx+preklapanje;i++){
					for(int j=cely-preklapanje;j<=cely+preklapanje;j++){
						if (IsValid(i,j)==1){
							map[i][j].presao7=1;
						}
					}
				}
#endif
				StartRacunac.x=celx;
				StartRacunac.y=cely;
				printf("startracunac=(%d,%d) tool=(%d,%d)\n",celx,cely,tool.x,tool.y);
			}


		if (racunaoupromjeni){
			printf("tu bu stal\n");
			racunaoupromjeni=0;
		}

  return numnewcov;
}

bool CCDStar::checkIfStuck(int numnewcells, double setv, double setw){
  bool stuck=false;
      if (numnewcells==0){
        numcycles++;
      }else{
        numcycles=0;
      }
  if (numcycles>50){
//    numcycles=0;
    stuck=true;
  }
  if (stuck==false && numnewcells==0){
//    if ((fabs(setw)>=DW_MAX*STEP/2)){
//      if  (DS->Start.th==oldStart.th){
//        numcyclesvel++;
//      }else {
//        numcyclesvel=0;
//      }
//    }
    if ( (fabs(setv)>=DV_MAX*STEP/2) ){
      if ((DS->Start.x==oldStart.x)&&(DS->Start.y==oldStart.y)){
        numcyclesvel++;
      }else {
        numcyclesvel=0;
      }
    }
    if ((DS->Start.th==oldStart.th) && (DS->Start.x==oldStart.x) && (DS->Start.y==oldStart.y)){
      numcyclesvel++;
    }else{
      numcyclesvel=0;
    }
    oldStart=DS->Start;
    if (numcyclesvel>50){
//      numcyclesvel=0;
      stuck=true;
    }
  }
  if (stuck==false){
    if (DS->IsValidOri(DS->Start)==2){
      stuck=true;
    }
  }
  if (stuck){
  printf("stuck because: numcycles=%d, numcyclesvel=%d isvalidori=%d\n",numcycles,numcyclesvel,DS->IsValidOri(DS->Start));
  numcyclesvel=0;
  numcycles=0;
  }
  return stuck;
}


int CCDStar::planCoveragePath(){

  
  if (prviput){
		StartRacunac=Start;
		Goal=Start;
    if (gettimeofday(&timeStart, NULL) == 0)
    {
	    mySecStart = timeStart.tv_sec;
	    myMSecStart = timeStart.tv_usec / 1000;
    }
    if (SearchPathReverse(1)){//poziv za DT==1 ovaj odredi matDTdostupno
	    printf("eto ga\n");
    }else{
	    printf("nesto ne stima za DT dostupni\n");
    }
    if (gettimeofday(&timeNow, NULL) == 0)
    {
	    mySecNow = timeNow.tv_sec;
	    myMSecNow = timeNow.tv_usec / 1000;
    }
    vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
    printf("CCD prviput: nakon odredjivanja matDTdostupno %d ms\n", vremenska_razlika);
    if (TSPreplan()){
	    printf("eto ga\n");
	    if (gettimeofday(&timeNow, NULL) == 0)
	    {
		    mySecNow = timeNow.tv_sec;
		    myMSecNow = timeNow.tv_usec / 1000;
	    }
	    vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	    printf("CCD prviput: nakon racunanja TSP %d ms\n", vremenska_razlika);
	    if(((GetPath()))==NULL){
		    printf("CCD prviput> Putanja tool je NULL!\n");
		    return 2;
	    }
	    if(((GetPathRobot()))==NULL){
		    printf("CCD prviput> Putanja robot je NULL!\n");
		    return 2;
	    }
    }else{
	    printf("nesto ne stima\n");
    }
    if (gettimeofday(&timeNow, NULL) == 0)
    {
	    mySecNow = timeNow.tv_sec;
	    myMSecNow = timeNow.tv_usec / 1000;
    }
    vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
    printf("CCD prviput: nakon GetPath %d ms\n", vremenska_razlika);
	}else{
		if (replanko){ //postavlja ga...za sada main kad je voznja==false, a resetira TSPreplan, TSP, reset 
			if (gettimeofday(&timeStart, NULL) == 0)
			{
				mySecStart = timeStart.tv_sec;
				myMSecStart = timeStart.tv_usec / 1000;
/*				if (DS->SearchPathReverse(1)){//poziv za DT==1 ovaj replanira matDTdostupno
					printf("eto ga\n");
				}else{
					printf("replan: nesto ne stima za DT dostupni\n");
				}*/
				
				if (TSPreplan()){
					printf("eto ga\n");
					if (gettimeofday(&timeNow, NULL) == 0)
					{
						mySecNow = timeNow.tv_sec;
						myMSecNow = timeNow.tv_usec / 1000;
					}
					vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
					printf("CCD: nakon racunanja TSPreplan %d ms\n", vremenska_razlika);
					if(((GetPath()))==NULL){
						printf("CCD> Putanja tool je NULL!\n");
						return 2;
					}
					if(((GetPathRobot()))==NULL){
						printf("CCD> Putanja robota je NULL!\n");
						return 2;
	    				}

				    if (gettimeofday(&timeNow, NULL) == 0)
				    {
					    mySecNow = timeNow.tv_sec;
					    myMSecNow = timeNow.tv_usec / 1000;
				    }
				    vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
				    printf("CCD: nakon GetPath %d ms\n", vremenska_razlika);

				}else{
					printf("nesto ne stima\n");
				}
			}
			racunaoupromjeni=1;
		}else{
//			if((path=(DS->getPath()))==NULL){
//				printf("Planner> Putanja iz Stara je NULL!\n");
//				return 2;
//			}
			    if(((getPathRobot()))==NULL){//obrati paznju na malo slovo g, ovaj napravi i DS->path, a tu dolje ga se pozove
				    printf("CCD noreplan> Putanja robota je NULL!\n");
				    return 2;
			    }
			    if(((GetPath()))==NULL){
						printf("CCD noreplan> Putanja tool je NULL!\n");
						return 2;
			    }
		}
	}
	return 1;
}


//nakon sto se odredi matrica DT
bool CCDStar::TSP(){ 
  
	int br_posj=1;
	int minDT,razlika,i,j,d, poc_x,poc_y,kraj_x,kraj_y,ix,jy,pomak_x,pomak_y;
	int ima_prepreke=1;
	int provjera;
	double poc_kut, pomakR, pomak_kut, ai1, staripomak_kut;
	int staripomak_i, staripomak_j, promjenasmjera;
	int poc_smjer, pomakalata=(int)(ceil(distance_robot_tool/GM->Map_Cell_Size)), duljina, brojtraz;
	R_point StartR;
	I_point *pathf;
	I_point point, temp, kor[13];
	int pomocniDT;
	PathLength=0;
  PathLength_forward=0;//dodala
  //i ovo - prijepis iz presao
  	for (i=0; i<MapSizeX; i++){
		for (j=0; j<MapSizeY; j++){
			map[i][j].P7=0;
			map[i][j].Pp=0;//to bih mogla i u resetu, ali ne, reset(1) resetira samo k i h
#if (BEZ_PREKLAPANJA==0)
if ((map[i][j].Pp==0) &&(map[i][j].DTdostupni==0)&&(map[i][j].DT>0)){ //cosak: promijenila sam prepreku u dostupno==0 jer moze biti nekakav slobodni cosak do kojeg se ne moze doci
				provjera=0;
				for (ix=i-robot_mask;ix<=i+robot_mask;ix++){
					for (jy=j-robot_mask;jy<=j+robot_mask;jy++){
						if (IsValid(ix,jy)>0){
							if ((map[ix][jy].DTdostupni!=0)){
								provjera=1;
								break;
							}
						}
					}
					if (provjera){
						break;
					}
				}
				if (provjera==0){
					map[i][j].Pp=-2;
				}
			}
#endif
			if (map[i][j].presao){
				map[i][j].Pp=1;
			}
			if (map[i][j].presao7){
				map[i][j].P7=1;
			}
		}
	}
	
	printf("prepisao iz presao u Pp\n");

//	for (i=0; i<MapSizeX; i++){//silujem rubove
//		for (j=0; j<MapSizeY; j++){
//			if ((i==0)||(j==0)||(i==MapSizeX-1)||(j==MapSizeY-1)){
//				for (ix=i-robot_mask;ix<=i+robot_mask;ix++){
//					for (jy=j-robot_mask;jy<=j+robot_mask;jy++){
//						if (IsValid(ix,jy)>0){
//							map[ix][jy].prepreka_bool =true;
//						}
//					}
//				}
//			}
//		}
//	}

	
	poc_kut=WH->RB.th;
	while (poc_kut>2*M_PI){
	    poc_kut=poc_kut-2*M_PI;
	}
	while (poc_kut<0){
    		poc_kut=poc_kut+2*M_PI;
    	}
	if ((fabs(poc_kut-2*M_PI)<=M_PI/4) || (fabs(poc_kut)<=M_PI/4)){
    		poc_smjer=1;
    		ai1=0;
    		}
	if (fabs(poc_kut-M_PI)<=M_PI/4){
    		poc_smjer=-1;
    		ai1=M_PI;
    		}
	if (fabs(poc_kut-M_PI/2)<=M_PI/4){
    		poc_smjer=2;
    		ai1=M_PI/2;
    		}
	if (fabs(poc_kut-3*M_PI/2)<=M_PI/4){
    		poc_smjer=-2;
    		ai1=3*M_PI/2;
    		}

		
	while(1) { //br_posj<br_praznih){//bezveze uvijet, ide van kad nema sljedeceg
		//treba spremati u pomocna polja jer se pointeri_TSP prepisu drugima
		for (d=PathLength_forward-2;d>0;d--){//prvi se upisao u proslom koraku, a zadnji tu ispod
#if (ISPIS)
// 			printf("(%d,%d)\t",path_forward[d].x,path_forward[d].y);
#endif
			//kutovi
			staripomak_i=path_forward[d].x-path[PathLength-1].x;
                   	staripomak_j=path_forward[d].y-path[PathLength-1].y;
			staripomak_kut=atan2(staripomak_j,staripomak_i);
//			printf("(%d,%d) ai1=%f staripomak_kut=%f\t",path_forward[d].x,path_forward[d].y,ai1*RuS,staripomak_kut*RuS);
			promjenasmjera=0;
//			if (fabs(staripomak_kut-ai1)>0.001){
//                           promjenasmjera=1;
//                        }
                   	ai1=staripomak_kut;
			//di je robot u odnosu na tool
			if (promjenasmjera==1){
				StartR.x=path[PathLength-1].x;
				StartR.y=path[PathLength-1].y;
			}else{
			pomak_x=path_forward[d].x-StartR.x;
			pomak_y=path_forward[d].y-StartR.y;
			pomak_kut=atan2(pomak_y,pomak_x);
                   	pomakR=sqrt(pomak_x*pomak_x+pomak_y*pomak_y);
			if (pomakR>=pomakalata){ //micem mali dodatak 2 - mali dodatak da ne odrifta zbog ceila 
                   		StartR.x=path_forward[d].x-pomakalata*cos(pomak_kut);
                   		StartR.y=path_forward[d].y-pomakalata*sin(pomak_kut);
                   	}//inace ostaje stari
                   	temp.x=floor(StartR.x);
                   	temp.y=floor(StartR.y);
	                brojtraz=1;
                        while (IsValid(temp.x,temp.y)!=1){
                        	StartR.x=path_forward[d].x-(pomakalata-brojtraz)*cos(pomak_kut);
                        	StartR.y=path_forward[d].y-(pomakalata-brojtraz)*sin(pomak_kut);
		           	temp.x=floor(StartR.x);
		           	temp.y=floor(StartR.y);
                        	brojtraz=brojtraz+1;
                        }
                        }
			pathrobot[PathLength]=StartR;
			path[PathLength]=path_forward[d];
			PathLength++;
			pomak_x=path_forward[d].x-path_forward[d+1].x;
			pomak_y=path_forward[d].y-path_forward[d+1].y;
			for (i=path_forward[d].x-robot_mask;i<=path_forward[d].x+robot_mask;i++){
				for (j=path_forward[d].y-robot_mask;j<=path_forward[d].y+robot_mask;j++){
				if  (IsValid(i,j)>0){
				  if ((((pomak_x!=0) && (i==path_forward[d].x+pomak_x*robot_mask)) || ((pomak_y!=0) && (j==path_forward[d].y+pomak_y*robot_mask))))
				  {
// 					if ((map[i][j].DT!=0)&&(map[i][j].Pp!=1)){
					map[i][j].Pp=map[i][j].Pp+1;//posjeceni su
						br_posj++;
// 					}
#if (ISPIS)
printf("(%d,%d) posjecenost raste na %d za (i,j)=(%d,%d) za pomak_x=%d, pomak_y=%d\n",path_forward[d].x,path_forward[d].y, map[i][j].Pp,i,j,pomak_x,pomak_y);
#endif
				  }
				  if (map[i][j].Pp<1){
					  printf("ne smije bit, posjecenost je %d za (i,j)=(%d,%d) za pomak_x=%d, pomak_y=%d\n",map[i][j].Pp,i,j,pomak_x,pomak_y);
				  }
				}
				}
			}
#if (BEZ_PREKLAPANJA)
			for (i=path_forward[d].x-preklapanje;i<=path_forward[d].x+preklapanje;i++){
				for (j=path_forward[d].y-preklapanje;j<=path_forward[d].y+preklapanje;j++){
					if (IsValid(i,j)>0){
						if ((map[i][j].DTdostupni!=0)&&(map[i][j].Pp==0)&&(map[i][j].P7!=1)){
							map[i][j].P7=1;//posjeceni su
						}
					}
				}
			}
#endif
		}
		if (PathLength==0) {
                	StartR.x=Start.x;//razliciti su tipovi pa mora ovako
                	StartR.y=Start.y;
//ovo je iz replan-a
                	printf("Start u inicijalnom je (%d,%d) (trebo bi bit jednak startracuncu)\n",Start.x,Start.y);
                	//provjera ako je tool zauzet da onda ostavi jednako startu
                	if (IsValid(tool.x,tool.y)==1)
                		Start=tool; //racuna se u mainu
			ai1=atan2((Start.y-StartR.y),(Start.x-StartR.x));
//zakomentiram stari inic
//			if ((poc_smjer==1) || (poc_smjer==-1)) {
//                		temp.x=Start.x+poc_smjer*pomakalata;
//                		temp.y=Start.y;
//                		for (int i=0;i<pomakalata+4;i++){
//                			kor[i].x=Start.x+i*poc_smjer;
//                			kor[i].y=Start.y;
//                		}
//                	}else{
//                		temp.x=Start.x;
//                		temp.y=Start.y+poc_smjer/2*pomakalata;
//                		for (int i=0;i<pomakalata+4;i++){
//                			kor[i].x=Start.x;
//                			kor[i].y=Start.y+i*poc_smjer/2;
//                		}
//                	}
//                	StartR.x=Start.x;//razliciti su tipovi pa mora ovako
//                	StartR.y=Start.y;
//                	Start=temp;
//                	duljina=pomakalata+4;
		}else{
					//kutovi
			staripomak_i=Start.x-path[PathLength-1].x;
                   	staripomak_j=Start.y-path[PathLength-1].y;
			staripomak_kut=atan2(staripomak_j,staripomak_i);
//			printf("(%d,%d) ai1=%f staripomak_kut=%f\n",Start.x,Start.y,ai1*RuS,staripomak_kut*RuS);
			promjenasmjera=0;
			if (fabs(staripomak_kut-ai1)>0.001){
                           promjenasmjera=1;
                           if (abs(staripomak_i)<=1 && abs(staripomak_j)<=1)
                           	promjenasmjera=0;
                        }
                   	ai1=staripomak_kut;
			//di je robot u odnosu na tool
			if (promjenasmjera==1){
				StartR.x=path[PathLength-1].x+(MR-robot_mask)*cos(staripomak_kut);
				StartR.y=path[PathLength-1].y+(MR-robot_mask)*sin(staripomak_kut);
			}else{
			pomak_x=Start.x-StartR.x;
			pomak_y=Start.y-StartR.y;
			pomak_kut=atan2(pomak_y,pomak_x);
                   	pomakR=sqrt(pomak_x*pomak_x+pomak_y*pomak_y);
			if (pomakR>=pomakalata+2){ //mali dodatak da ne odrifta zbog ceila 
                   		StartR.x=Start.x-pomakalata*cos(pomak_kut);
                   		StartR.y=Start.y-pomakalata*sin(pomak_kut);
                   	}//inace ostaje stari
                   	}
                   	temp.x=floor(StartR.x);
                   	temp.y=floor(StartR.y);
	                brojtraz=1;
                        while (IsValid(temp.x,temp.y)!=1){
                        	StartR.x=Start.x-(pomakalata-brojtraz)*cos(pomak_kut);
                        	StartR.y=Start.y-(pomakalata-brojtraz)*sin(pomak_kut);
		           	temp.x=floor(StartR.x);
		           	temp.y=floor(StartR.y);
                        	brojtraz=brojtraz+1;
                        }
		}
		pathrobot[PathLength]=StartR;
		path[PathLength]=Start;
		PathLength++;
		PathLength_forward=0;
		if (PathLength>1){
			pomak_x=Start.x-path[PathLength-2].x;
			pomak_y=Start.y-path[PathLength-2].y;
		}//zasto ovo, to je uvijek jednako, postavlja se u main i odmah racuna u PL? zakomentiravam zato sto je tool na drugom mjestu
		else{
//			pomak_x=Start.x-StartRacunac.x;
//			pomak_y=Start.y-StartRacunac.y;
			pomak_x=0;
			pomak_y=0;
		}
		// 		printf("okolnji (%d,%d)\n",Start.x,Start.y);
		for (i=Start.x-robot_mask;i<=Start.x+robot_mask;i++){
			for (j=Start.y-robot_mask;j<=Start.y+robot_mask;j++){
				if (IsValid(i,j)>0){
					if (((pomak_x!=0) && (abs(i-(Start.x+(pomak_x/abs(pomak_x))*robot_mask))<abs(pomak_x))) || ((pomak_y!=0) && (abs(j-(Start.y+(pomak_y/abs(pomak_y))*robot_mask))<abs(pomak_y)))){
							map[i][j].Pp=map[i][j].Pp+1;//posjeceni su
							br_posj++;
							#if (ISPIS)
							printf("posjecenost raste na %d za (i,j)=(%d,%d) za pomak_x=%d, pomak_y=%d\n",map[i][j].Pp,i,j,pomak_x,pomak_y);
							#endif
						}
						if (map[i][j].Pp<1){
							printf("ne smije bit, posjecenost je %d za (i,j)=(%d,%d) za pomak_x=%d, pomak_y=%d\n",map[i][j].Pp,i,j,pomak_x,pomak_y);
						}
					}
				}
			}
#if (BEZ_PREKLAPANJA)
			for (i=Start.x-preklapanje;i<=Start.x+preklapanje;i++){
				for (j=Start.y-preklapanje;j<=Start.y+preklapanje;j++){
					if (IsValid(i,j)>0){
						if ((map[i][j].DTdostupni!=0)&&(map[i][j].Pp==0)&&(map[i][j].P7!=1)){
							map[i][j].P7=1;//posjeceni su
						}
					}
				}
			}
#endif
//zakomentiram stari inic
//		}else{
//			duljina=1;
//			kor[0]=Start;
//			//kutovi
//			staripomak_i=Start.x-path[PathLength-1].x;
//                   	staripomak_j=Start.y-path[PathLength-1].y;
//			staripomak_kut=atan2(staripomak_j,staripomak_i);
////			printf("(%d,%d) ai1=%f staripomak_kut=%f\n",Start.x,Start.y,ai1*RuS,staripomak_kut*RuS);
//			promjenasmjera=0;
//			if (fabs(staripomak_kut-ai1)>0.001){
//                           promjenasmjera=1;
//                           if (abs(staripomak_i)<=1 && abs(staripomak_j)<=1)
//                           	promjenasmjera=0;
//                        }
//                   	ai1=staripomak_kut;
//			//di je robot u odnosu na tool
//			if (promjenasmjera==1){
//				StartR.x=path[PathLength-1].x+(MR-robot_mask)*cos(staripomak_kut);
//				StartR.y=path[PathLength-1].y+(MR-robot_mask)*sin(staripomak_kut);
//			}else{
//			pomak_x=Start.x-StartR.x;
//			pomak_y=Start.y-StartR.y;
//			pomak_kut=atan2(pomak_y,pomak_x);
//                   	pomakR=sqrt(pomak_x*pomak_x+pomak_y*pomak_y);
//			if (pomakR>=pomakalata){ //2 bilo to micem - mali dodatak da ne odrifta zbog ceila 
//                   		StartR.x=Start.x-pomakalata*cos(pomak_kut);
//                   		StartR.y=Start.y-pomakalata*sin(pomak_kut);
//                   	}//inace ostaje stari
//                   	}
//                   	temp.x=floor(StartR.x);
//                   	temp.y=floor(StartR.y);
//	                brojtraz=1;
//                        while (IsValid(temp.x,temp.y)!=1){
//                        	if (promjenasmjera==1){
//                        	        StartR.x=path[PathLength-1].x+((MR-robot_mask)-brojtraz)*cos(staripomak_kut);
//                        	        StartR.y=path[PathLength-1].y+((MR-robot_mask)-brojtraz)*sin(staripomak_kut);
//                        	}else{
//                        		StartR.x=Start.x-(pomakalata-brojtraz)*cos(pomak_kut);
//                        		StartR.y=Start.y-(pomakalata-brojtraz)*sin(pomak_kut);
//                        	}
//		           	temp.x=floor(StartR.x);
//		           	temp.y=floor(StartR.y);
//                        	brojtraz=brojtraz+1;
//                        }
//		}
//		pathrobot[PathLength]=StartR;
//		path[PathLength]=Start;
//		PathLength++;
//		PathLength_forward=0;
//		if (PathLength>1){
//			pomak_x=Start.x-path[PathLength-2].x;
//			pomak_y=Start.y-path[PathLength-2].y;
//		}

//// 		printf("okolnji (%d,%d)\n",Start.x,Start.y);
//		for (int brojac=0; brojac<duljina; brojac++){
//			Start=kor[brojac];
//		for (i=Start.x-robot_mask;i<=Start.x+robot_mask;i++){
//			for (j=Start.y-robot_mask;j<=Start.y+robot_mask;j++){
//				if (IsValid(i,j)>0){
//					if ((PathLength==1)){
// 			  			if ((map[i][j].Pp!=1)){
//						map[i][j].Pp=map[i][j].Pp+1;//posjeceni su
//						br_posj++;
//						}
//					}else if (((pomak_x!=0) && (abs(i-(Start.x+(pomak_x/abs(pomak_x))*robot_mask))<abs(pomak_x))) || ((pomak_y!=0) && (abs(j-(Start.y+(pomak_y/abs(pomak_y))*robot_mask))<abs(pomak_y)))){
//						map[i][j].Pp=map[i][j].Pp+1;//posjeceni su
//						br_posj++;
//#if (ISPIS)
//printf("posjecenost raste na %d za (i,j)=(%d,%d) za pomak_x=%d, pomak_y=%d\n",map[i][j].Pp,i,j,pomak_x,pomak_y);
//#endif
//					}
//					if (map[i][j].Pp<1){
//						printf("ne smije bit, posjecenost je %d za (i,j)=(%d,%d) za pomak_x=%d, pomak_y=%d\n",map[i][j].Pp,i,j,pomak_x,pomak_y);
//					}
//				}
//			}
//		}
//		
//#if (BEZ_PREKLAPANJA)
//		for (i=Start.x-preklapanje;i<=Start.x+preklapanje;i++){
//			for (j=Start.y-preklapanje;j<=Start.y+preklapanje;j++){
//				if (IsValid(i,j)>0){
//					if ((map[i][j].DTdostupni!=0)&&(map[i][j].Pp==0)&&(map[i][j].P7!=1)){
//						map[i][j].P7=1;//posjeceni su
//					}
//				}
//			}
//		}
//#endif
//		}
//		if ((PathLength==1)){
//			Start=temp; //vrati
//		}
		minDT=OBSTACLE;
		razlika=minDT;
#if (BEZ_PREKLAPANJA)
		for (d=0;d<4;d++){
			if (redund){
				point.x=Start.x+x6ofs[d];
				point.y=Start.y+y6ofs[d];
			}else{
				point.x=Start.x+x7ofs[d];
				point.y=Start.y+y7ofs[d];
			}
			if (IsValid(point.x,point.y)>0){
				pomocniDT=map[point.x][point.y].DTdostupni;
				if ((map[point.x][point.y].DTdostupni!=0)&&(map[point.x][point.y].Pp==0)&&(map[point.x][point.y].P7==0)){
#else
		for (d=0;d<32;d++){
			point.x=Start.x+x4ofs[d];
			point.y=Start.y+y4ofs[d];
			if (IsValid(point.x,point.y)>0){
				pomocniDT=map[point.x][point.y].DT;
				if ((map[point.x][point.y].DT!=0)&&(map[point.x][point.y].Pp==0)){
			  if ((map[point.x][point.y].DTdostupni!=0)||(map[point.x][point.y].DT!=0))
#endif
			  {
				  if ((pomocniDT<minDT))
				{
					razlika=abs(point.x-Start.x)+abs(point.y-Start.y);
					minDT=pomocniDT;
					Goal=point;
				}else if (pomocniDT==minDT) {
				  if ((abs(point.x-Start.x)+abs(point.y-Start.y)<razlika))
				  {
				    razlika=abs(point.x-Start.x)+abs(point.y-Start.y);
				    Goal=point;
				  }
				  
				}
			}
			}
			}
		}//for 32
		//provjera je li dijagonalno
		if ((minDT<OBSTACLE)){
// 		  if ((IsValid(Goal.x,Goal.y)==2)){//u prosirenju patak: a nadjen je koji nije u prepreci mijenjam u DTdostupni==0
		    if ((map[Goal.x][Goal.y].DTdostupni==0)){//cosak: a nadjen je koji nije u prepreci i nije dostupan. mijenjam u provjeru DTdostupni==0 jer je prije bilo preskoceno i ostao je taj Goal krivi zato sto u gornjem uvjetu ne pise da mora bit prepreka i DT!=0, al coskovi su izbaceni na pocetku
		    razlika=OBSTACLE;
		    for (i=Goal.x-robot_mask;i<=Goal.x+robot_mask;i++){
		      for (j=Goal.y-robot_mask;j<=Goal.y+robot_mask;j++){
			if (IsValid(i,j)>0){
			if ((map[i][j].DTdostupni!=0)){
				if ((abs(i-Goal.x)+abs(j-Goal.y)<razlika)&& ((Start.x!=i)||(Start.y!=j)))
			  {
			    razlika=abs(i-Goal.x)+abs(j-Goal.y);
			    point.x=i;
			    point.y=j;
#if (ISPIS)
			    printf("medju susjedima bliski dostupni (%d,%d)\n",point.x,point.y);
#endif
			  }
			}
		      }
		      }
		    }
		    if (razlika<OBSTACLE){
		      Goal=point;
		    }else{
		      minDT=OBSTACLE;
		    }
		  }
		}
		if (minDT<OBSTACLE){
			if (1||(((Goal.x-Start.x)!=0) && ((Goal.y-Start.y)!=0))) {//dijagonala, stavila uvijek jer ne valja kad se rade veliki skokovi (za BEZ_PREKLAPANJA==1)
			  //dodati dio provjere:
			  ima_prepreke=1;
			  if (1){//stavi 1 kad hoces zaobici racunanje puta
			  if (Goal.x<Start.x){
			    poc_x=Goal.x;
			    kraj_x=Start.x;
			  }else{
			      poc_x=Start.x;
			      kraj_x=Goal.x;
			  }
			  if (Goal.y<Start.y){
			    poc_y=Goal.y;
			    kraj_y=Start.y;
			  }else{
			    poc_y=Start.y;
			    kraj_y=Goal.y;
			  }
			  ima_prepreke=0;
			  for (i=poc_x; i<=kraj_x; i++){
				    for (j=poc_y; j<=kraj_y; j++){
				      if (IsValid(i,j)==2){
					ima_prepreke=1;
					break;
				      }
				    }
				    if (ima_prepreke)
				      break;
			  }
			  }
			}
		}
		if ((ima_prepreke) && (((Goal.x-Start.x)==0) || ((Goal.y-Start.y)==0))){
			minDT=OBSTACLE;
		}
		if (minDT<OBSTACLE){
		//racunati put i odrediti next_TSP-ove
			if (ima_prepreke){
			reset(1);//resetira samo k i h
			if (SearchPathReverse(0)){//poziv za DT==0
// 							printf("eto ga\n");
			}else{
				printf("nesto ne stima\n");
			}
			if ((pathf=(getPath_forward()))==NULL){
				printf("Putanja je NULL oko prepreke od starta (%d,%d) do cilja (%d,%d)\n",Start.x,Start.y,Goal.x,Goal.y);//odredi next-ove
			}
			}
// 			}//od dijagonala
		}else
		{//ako nema neposjecenog u okolini 4
#if (BEZ_PREKLAPANJA==0)
			for (i=0; i<MapSizeX; i++){
				for (j=0; j<MapSizeY; j++){
					if ((map[i][j].Pp==0)){
					  if ((map[i][j].DT<minDT)&&(map[i][j].DT>0)&&(!((map[i][j].DTdostupni==0)&&(IsValid(i,j)==1)))){
							minDT=map[i][j].DT;
						}
					}
				}
			}
			if (minDT==OBSTACLE){
			  printf("nema vise minDT-a\n");
			  break;
			}
#if (ISPIS)
			printf("trazim dalji minDT=%d, br_posj=%d, br_praznih=%d\n",minDT,br_posj,br_praznih);
#endif
#endif
/*			if ((minDT==OBSTACLE)&&((br_posj<br_praznih))){
				printf("nema sljedece pozicije i nije sve posjetio\n");
				return false;
			}else*/
// 			{
			//racunati put do minDT i odrediti next_TSP-ove
#if (BEZ_PREKLAPANJA)
				minDT=-2;
#endif
				reset(1);//resetira samo k i h
				if (SearchPathReverse(minDT)){//poziv za DT==minDT, odredjuje Goal, trazi najblizeg od trenutne pozicije ako je BEZ_PREKLAPANJA==1
// 					printf("eto ga\n");
				}else{
					printf("nema vise\n");
#if (BEZ_PREKLAPANJA)
					break;
#endif
				}
#if (BEZ_PREKLAPANJA)
				if ((pathf=(getPath_forward()))==NULL){
					printf("Putanja je NULL! kad trazi minDT=%d\n",minDT);//odredi next-ove
				}
#else
				if (map[Goal.x][Goal.y].DT!=minDT){
					printf("Nije pronasao tocku s minDT=%d\n",minDT);
				}
				
				//posjetimo ga unaprijed
// 				for (i=Goal.x-1;i<=Goal.x+1;i++){
// 				  for (j=Goal.y-1;j<=Goal.y+1;j++){
// 				    if (IsValid(i,j)>0){
// 				      if ((map[i][j].DT!=0)&&(map[i][j].Pp==0)){
// 					map[i][j].Pp=2;//posjeceni su
// 					br_posj++;
// 				      }
// 				    }
// 				  }
// 				}
				if (map[Goal.x][Goal.y].DTdostupni==0){
				  minDT=OBSTACLE;
				}else{
					minDT=0;// ovo je kod BEZ_PREKLAPANJA ostalo inf
				}
				if ((IsValid(Goal.x,Goal.y)==2)){//u prosirenju
				  razlika=OBSTACLE;
#if (ISPIS)
				  printf("u prosirenju, trazim bliskog dostupnog\n");
#endif
				  for (i=Goal.x-robot_mask;i<=Goal.x+robot_mask;i++){
				    for (j=Goal.y-robot_mask;j<=Goal.y+robot_mask;j++){
				      if (IsValid(i,j)>0){
					if ((map[i][j].DTdostupni!=0)){
						if ((abs(i-Goal.x)+abs(j-Goal.y)<razlika) && ((Start.x!=i)||(Start.y!=j)))
					  {
					    razlika=abs(i-Goal.x)+abs(j-Goal.y);
					    point.x=i;
					    point.y=j;
#if (ISPIS)
					    printf("bliski dostupni (%d,%d)\n",point.x,point.y);
#endif
					  }
					}
				      }
				    }
				  }
				  if (razlika<OBSTACLE){
				    minDT=razlika;
				    Goal=point;
				  }else{
				    minDT=OBSTACLE;
				  }
				}
				if (minDT<OBSTACLE){
				  reset(1);//resetira samo k i h
				  if (SearchPathReverse(0)){//poziv za DT==0
				    // 							printf("eto ga\n");
				}else{
				  printf("nesto ne stima\n");
				}
				if ((pathf=(getPath_forward()))==NULL){
				  printf("Putanja je NULL na velikom skoku od starta (%d,%d) do cilja (%d,%d)\n",Start.x,Start.y,Goal.x,Goal.y);
				 //odredi next-ove
				}
				}else{
					map[Goal.x][Goal.y].Pp=-2;
					
				  Goal=Start;
#if (ISPIS)
				  printf("Nema bliskog nedostupnom!!!!\n");
#endif
				}
#endif //kraj od bez_preklapanja==0
// 			}//nepotrebno
		}
		Start=Goal;//sljedeca pozicija, ili ova odredjena s minDT ili gore odredjena iz 4 okolnja
#if (ISPIS)
		printf("sljedeca pozicija (%d,%d)\n",Start.x,Start.y);
#endif
	}//od while
	printf("gotov je!\n");
	prviput=0;
	replanko=0;
	return true;
}

void CCDStar::updateCoverageMap(){
	int okolina=5+20; //plus dva metra sa svake strane jer su rubovi malo izvan, pola metra oko rubova prostora nema mina (za celldim 100 mm) stavljam 0.5 m = 5 polja
  GridMap_cell **gmap=GM->GetMap();//treba zbog prepreka
	int provjera;
	int i,j,ti,tj;

	for (i=0; i<MapSizeX; i++){
		for (j=0; j<MapSizeY; j++){
			map[i][j].P7=0;
			map[i][j].Pp=0;//to bih mogla i u resetu, ali ne, reset(1) resetira samo k i h

			if (map[i][j].presao){
				map[i][j].Pp=1;
			}
			if (map[i][j].presao7){
				map[i][j].P7=1;
			}
			
			if ((gmap[i][j].static_cell==false) && (gmap[i][j].occupancy>70)){
			  gmap[i][j].static_cell=true;
			}
			

//			if (map[i][j].Pp==0){
//				provjera=0;
//				for (ix=i-okolina;ix<=i+okolina;ix++){
//					for (jy=j-okolina;jy<=j+okolina;jy++){
//						if (IsValid(ix,jy)==2){
//						  map[i][j].Pp=1;
//						  provjera=1;
//						  break;
//						}
//					}
//					if (provjera){
//						break;
//					}
//				}
//			}

      
		}
	}

//hratc bila 1 zbog rubova mape
#if 0
	for (i=0; i<MapSizeX; i++){
		for (j=0; j<2; j++){// j==0, j==1
      tj=j*(MapSizeY-1);
				for (int ix=i-okolina;ix<=i+okolina;ix++){
					for (int jy=tj-okolina;jy<=tj+okolina;jy++){
						if (IsValid(ix,jy)>0){
							map[ix][jy].Pp=1;
						}
					}
			}
			
    }
  }

	for (i=0; i<2; i++){//i==0, i==1
    ti=i*(MapSizeX-1);
		for (j=0; j<MapSizeY; j++){// j==0, j==1
				for (int ix=ti-okolina;ix<=ti+okolina;ix++){
					for (int jy=j-okolina;jy<=j+okolina;jy++){
						if (IsValid(ix,jy)>0){
							map[ix][jy].Pp=1;
						}
					}
				}
			
    }
  }
#endif

}


//za replaniranje
bool CCDStar::TSPreplan(){
	int br_posj=1;
	int minDT,razlika,i,j,d, poc_x,poc_y,kraj_x,kraj_y,pomak_x,pomak_y;
	int ima_prepreke=1;
	double pomakR, pomak_kut;
	double ai1, staripomak_kut;
	int staripomak_i, staripomak_j, promjenasmjera;
	int poc_smjer, pomakalata=(int)(ceil(distance_robot_tool/GM->Map_Cell_Size))+1, brojtraz;
	R_point StartR;
	I_point *pathf;
	I_point point, temp;
	int pomocniDT;
	PathLength=0;
  PathLength_forward=0;//dodala
	// 	map[Start.x][Start.y].Pp=1;
	
	updateCoverageMap();
	
	printf("prepisao iz presao u Pp\n");
	while(1) { //br_posj<br_praznih){//bezveze uvijet, ide van kad nema sljedeceg
		//treba spremati u pomocna polja jer se pointeri_TSP prepisu drugima
		PathLength_forward=0;//ovime se izbjegava slijedjenje puta do daleke tocke, nego samo zadaje cilj
		for (d=PathLength_forward-2;d>0;d--){//prvi se upisao u proslom koraku, a zadnji tu ispod
#if (ISPIS)
			// 			printf("(%d,%d)\t",path_forward[d].x,path_forward[d].y);
#endif
			//kutovi
			staripomak_i=path_forward[d].x-path[PathLength-1].x;
                   	staripomak_j=path_forward[d].y-path[PathLength-1].y;
			staripomak_kut=atan2(staripomak_j,staripomak_i);
//			printf("(%d,%d) ai1=%f staripomak_kut=%f\t",path_forward[d].x,path_forward[d].y,ai1*RuS,staripomak_kut*RuS);
			promjenasmjera=0;
//			if (fabs(staripomak_kut-ai1)>0.001){
//                           promjenasmjera=1;
//                        }
                   	ai1=staripomak_kut;
			//di je robot u odnosu na tool
			if (promjenasmjera==1){
				StartR.x=path[PathLength-1].x;
				StartR.y=path[PathLength-1].y;
			}else{
			pomak_x=path_forward[d].x-StartR.x;
			pomak_y=path_forward[d].y-StartR.y;
			pomak_kut=atan2(pomak_y,pomak_x);
                   	pomakR=sqrt(pomak_x*pomak_x+pomak_y*pomak_y);
			if (pomakR>=pomakalata+2){ //mali dodatak da ne odrifta zbog ceila 
                   		StartR.x=path_forward[d].x-pomakalata*cos(pomak_kut);
                   		StartR.y=path_forward[d].y-pomakalata*sin(pomak_kut);
                   	}//inace ostaje stari
                   	temp.x=floor(StartR.x);
                   	temp.y=floor(StartR.y);
	                brojtraz=1;
#if 0
                        while (IsValid(temp.x,temp.y)!=1){
                        	StartR.x=path_forward[d].x-(pomakalata-brojtraz)*cos(pomak_kut);
                        	StartR.y=path_forward[d].y-(pomakalata-brojtraz)*sin(pomak_kut);
		           	temp.x=floor(StartR.x);
		           	temp.y=floor(StartR.y);
                        	brojtraz=brojtraz+1;
                        }
#endif
                        }
			pathrobot[PathLength]=StartR;
			path[PathLength]=path_forward[d];
			PathLength++;
			pomak_x=path_forward[d].x-path_forward[d+1].x;
			pomak_y=path_forward[d].y-path_forward[d+1].y;
			for (i=path_forward[d].x-robot_mask;i<=path_forward[d].x+robot_mask;i++){
				for (j=path_forward[d].y-robot_mask;j<=path_forward[d].y+robot_mask;j++){
					if  (IsValid(i,j)>0){
						
					if ((((pomak_x!=0) && (i==path_forward[d].x+pomak_x*robot_mask)) || ((pomak_y!=0) && (j==path_forward[d].y+pomak_y*robot_mask))))
					{
						// 					if ((map[i][j].DT!=0)&&(map[i][j].Pp!=1)){
							map[i][j].Pp=map[i][j].Pp+1;//posjeceni su
							br_posj++;
							// 					}
							#if (ISPIS)
							printf("(%d,%d) posjecenost raste na %d za (i,j)=(%d,%d) za pomak_x=%d, pomak_y=%d\n",path_forward[d].x,path_forward[d].y, map[i][j].Pp,i,j,pomak_x,pomak_y);
							#endif
					}
					if (map[i][j].Pp<1){
						printf("TSPreplan: path_forward ne smije bit, posjecenost je %d za (i,j)=(%d,%d) za pomak_x=%d, pomak_y=%d\n",map[i][j].Pp,i,j,pomak_x,pomak_y);
					}
					}
				}
			}
			#if (BEZ_PREKLAPANJA)
			for (i=path_forward[d].x-preklapanje;i<=path_forward[d].x+preklapanje;i++){
				for (j=path_forward[d].y-preklapanje;j<=path_forward[d].y+preklapanje;j++){
					if (IsValid(i,j)>0){
						if ((map[i][j].DTdostupni!=0)&&(map[i][j].Pp==0)&&(map[i][j].P7!=1)){
							map[i][j].P7=1;//posjeceni su
						}
					}
				}
			}
			#endif
		}
		if (PathLength==0) {
                	StartR.x=Start.x;//razliciti su tipovi pa mora ovako
                	StartR.y=Start.y;
                	printf("Start u replanu je (%d,%d) (trebo bi bit jednak startracuncu)\n",Start.x,Start.y);
                	//provjera ako je tool zauzet da onda ostavi jednako startu
                	if (IsValid(tool.x,tool.y)==1)
                		Start=tool; //racuna se u mainu
			ai1=atan2((Start.y-StartR.y),(Start.x-StartR.x));
		}else{
					//kutovi
			staripomak_i=Start.x-path[PathLength-1].x;
                   	staripomak_j=Start.y-path[PathLength-1].y;
			staripomak_kut=atan2(staripomak_j,staripomak_i);
//			printf("(%d,%d) ai1=%f staripomak_kut=%f\n",Start.x,Start.y,ai1*RuS,staripomak_kut*RuS);
			promjenasmjera=0;
			if (fabs(staripomak_kut-ai1)>0.001){
                           promjenasmjera=1;
                           if (abs(staripomak_i)<=1 && abs(staripomak_j)<=1)
                           	promjenasmjera=0;
                        }
                   	ai1=staripomak_kut;
			//di je robot u odnosu na tool
			if (promjenasmjera==1){
				StartR.x=path[PathLength-1].x+(MR-robot_mask)*cos(staripomak_kut);
				StartR.y=path[PathLength-1].y+(MR-robot_mask)*sin(staripomak_kut);
			}else{
			pomak_x=Start.x-StartR.x;
			pomak_y=Start.y-StartR.y;
			pomak_kut=atan2(pomak_y,pomak_x);
                   	pomakR=sqrt(pomak_x*pomak_x+pomak_y*pomak_y);
			if (pomakR>=pomakalata+2){ //mali dodatak da ne odrifta zbog ceila 
                   		StartR.x=Start.x-pomakalata*cos(pomak_kut);
                   		StartR.y=Start.y-pomakalata*sin(pomak_kut);
                   	}//inace ostaje stari
                   	}
                   	temp.x=floor(StartR.x);
                   	temp.y=floor(StartR.y);
	                brojtraz=1;
#if 0
                        while (IsValid(temp.x,temp.y)!=1){
                        	StartR.x=Start.x-(pomakalata-brojtraz)*cos(pomak_kut);
                        	StartR.y=Start.y-(pomakalata-brojtraz)*sin(pomak_kut);
		           	temp.x=floor(StartR.x);
		           	temp.y=floor(StartR.y);
                        	brojtraz=brojtraz+1;
                        }
#endif
		}
		pathrobot[PathLength]=StartR;
		path[PathLength]=Start;
		PathLength++;
		PathLength_forward=0;
		if (PathLength>1){
			pomak_x=Start.x-path[PathLength-2].x;
			pomak_y=Start.y-path[PathLength-2].y;
		}//zasto ovo, to je uvijek jednako, postavlja se u main i odmah racuna u PL? zakomentiravam zato sto je tool na drugom mjestu
		else{
//			pomak_x=Start.x-StartRacunac.x;
//			pomak_y=Start.y-StartRacunac.y;
			pomak_x=0;
			pomak_y=0;
		}
		// 		printf("okolnji (%d,%d)\n",Start.x,Start.y);
		for (i=Start.x-robot_mask;i<=Start.x+robot_mask;i++){
			for (j=Start.y-robot_mask;j<=Start.y+robot_mask;j++){
				if (IsValid(i,j)>0){
					if (((pomak_x!=0) && (abs(i-(Start.x+(pomak_x/abs(pomak_x))*robot_mask))<abs(pomak_x))) || ((pomak_y!=0) && (abs(j-(Start.y+(pomak_y/abs(pomak_y))*robot_mask))<abs(pomak_y)))){
							map[i][j].Pp=map[i][j].Pp+1;//posjeceni su
							br_posj++;
							#if (ISPIS)
							printf("posjecenost raste na %d za (i,j)=(%d,%d) za pomak_x=%d, pomak_y=%d\n",map[i][j].Pp,i,j,pomak_x,pomak_y);
							#endif
						}
						if (map[i][j].Pp<1){
						  map[i][j].Pp=1;//dodajem za slucaj da se sve prekriva sa coilom
						  br_posj++;
//							printf("TSPreplan: Start (%d,%d): ne smije bit, posjecenost je %d za (i,j)=(%d,%d) za pomak_x=%d, pomak_y=%d\n",Start.x,Start.y,map[i][j].Pp,i,j,pomak_x,pomak_y);
						}
					}
				}
			}
#if (BEZ_PREKLAPANJA)
			for (i=Start.x-preklapanje;i<=Start.x+preklapanje;i++){
				for (j=Start.y-preklapanje;j<=Start.y+preklapanje;j++){
					if (IsValid(i,j)>0){
						if ((map[i][j].DTdostupni!=0)&&(map[i][j].Pp==0)&&(map[i][j].P7!=1)){
							map[i][j].P7=1;//posjeceni su
						}
					}
				}
			}
#endif
			minDT=OBSTACLE;
			razlika=minDT;
#if (BEZ_PREKLAPANJA)
			for (d=0;d<4;d++){
				if (redund){
					point.x=Start.x+x6ofs[d];
					point.y=Start.y+y6ofs[d];
				}else{
					point.x=Start.x+x7ofs[d];
					point.y=Start.y+y7ofs[d];
				}
				if (IsValid(point.x,point.y)==1){//zbog promjene mora bit slobodno
					pomocniDT=map[point.x][point.y].DTdostupni;
					if ((map[point.x][point.y].DTdostupni!=0)&&(map[point.x][point.y].Pp==0)&&(map[point.x][point.y].P7==0)){
#else
				for (d=0;d<32;d++){
					point.x=Start.x+x4ofs[d];
					point.y=Start.y+y4ofs[d];
					if (IsValid(point.x,point.y)>0){
						pomocniDT=map[point.x][point.y].DT;
						if ((map[point.x][point.y].DT!=0)&&(map[point.x][point.y].Pp==0)){
							if ((map[point.x][point.y].DTdostupni!=0)||(map[point.x][point.y].DT!=0))
#endif
							if (1){
								if ((pomocniDT<minDT))
								{
									razlika=abs(point.x-Start.x)+abs(point.y-Start.y);
									minDT=pomocniDT;
									Goal=point;
								}else if (pomocniDT==minDT) {
									if ((abs(point.x-Start.x)+abs(point.y-Start.y)<razlika))
									{
										razlika=abs(point.x-Start.x)+abs(point.y-Start.y);
										Goal=point;
									}

								}
							}
              if ((pomak_x==point.x-Start.x)&&(pomak_y==point.y-Start.y)){
                break;
              }

					}
				}
			}//for 32
			//provjera je li dijagonalno

			if (minDT<OBSTACLE){
				if (1||(((Goal.x-Start.x)!=0) && ((Goal.y-Start.y)!=0))) {//dijagonala, stavila uvijek jer ne valja kad se rade veliki skokovi (za BEZ_PREKLAPANJA==1)
					//dodati dio provjere:
					ima_prepreke=1;
					if (1){//stavi 1 kad hoces zaobici racunanje puta
						if (Goal.x<Start.x){
							poc_x=Goal.x;
							kraj_x=Start.x;
						}else{
							poc_x=Start.x;
							kraj_x=Goal.x;
						}
						if (Goal.y<Start.y){
							poc_y=Goal.y;
							kraj_y=Start.y;
						}else{
							poc_y=Start.y;
							kraj_y=Goal.y;
						}
						ima_prepreke=0;
						for (i=poc_x; i<=kraj_x; i++){
							for (j=poc_y; j<=kraj_y; j++){
								if (IsValid(i,j)==2){
									ima_prepreke=1;
									break;
								}
							}
							if (ima_prepreke)
								break;
						}
					}
				}
			}
			if ((ima_prepreke) && (((Goal.x-Start.x)==0) || ((Goal.y-Start.y)==0))){
				minDT=OBSTACLE;
			}
			if (minDT<OBSTACLE){
				//racunati put i odrediti next_TSP-ove
				if (ima_prepreke){
					reset(1);//resetira samo k i h
					if (SearchPathReverse(0)){//poziv za DT==0
						// 							printf("eto ga\n");
					}else{
						printf("nesto ne stima\n");
					}
					if ((pathf=(getPath_forward()))==NULL){
						printf("Putanja je NULL oko prepreke od starta (%d,%d) do cilja (%d,%d)\n",Start.x,Start.y,Goal.x,Goal.y);//odredi next-ove
					}
				}
				// 			}//od dijagonala
			}else
			{//ako nema neposjecenog u okolini 4
#if (BEZ_PREKLAPANJA==0)
			for (i=0; i<MapSizeX; i++){
				for (j=0; j<MapSizeY; j++){
					if ((map[i][j].Pp==0)){
						if ((map[i][j].DT<minDT)&&(map[i][j].DT>0)&&(!((map[i][j].DTdostupni==0)&&(IsValid(i,j)==1)))){
							minDT=map[i][j].DT;
						}
					}
				}
			}
			if (minDT==OBSTACLE){
				printf("nema vise minDT-a\n");
				break;
			}
			#if (ISPIS)
			printf("trazim dalji minDT=%d, br_posj=%d, br_praznih=%d\n",minDT,br_posj,br_praznih);
			#endif
#endif
			/*			if ((minDT==OBSTACLE)&&((br_posj<br_praznih))){
			printf("nema sljedece pozicije i nije sve posjetio\n");
			return false;
							}else*/
			// 			{
				//racunati put do minDT i odrediti next_TSP-ove
			#if (BEZ_PREKLAPANJA)
			minDT=-2;
			#endif
			reset(1);//resetira samo k i h
			if (SearchPathReverse(minDT)){//poziv za DT==minDT, odredjuje Goal, trazi najblizeg od trenutne pozicije ako je BEZ_PREKLAPANJA==1
				// 					printf("eto ga\n");
				}else{
					printf("nema vise\n");
					break;
				}
//				if ((pathf=(getPath_forward()))==NULL){
//					printf("Putanja je NULL! kad trazi minDT=%d\n",minDT);//odredi next-ove
//				}
			}
			  razlika=0;
				if (0 && map[Goal.x][Goal.y].DTdostupni==0){//u prosirenju
				  razlika=OBSTACLE;
#if (ISPIS)
				  printf("u prosirenju, trazim bliskog dostupnog od (%d,%d)\n",Goal.x,Goal.y);
#endif
				  for (i=Goal.x-robot_mask;i<=Goal.x+robot_mask;i++){
				    for (j=Goal.y-robot_mask;j<=Goal.y+robot_mask;j++){
				      if (IsValid(i,j)==1){
					if ((map[i][j].DTdostupni!=0)){
						if ((abs(i-Goal.x)+abs(j-Goal.y)<razlika) && ((Start.x!=i)||(Start.y!=j)))
					  {
					    razlika=abs(i-Goal.x)+abs(j-Goal.y);
					    point.x=i;
					    point.y=j;
#if (ISPIS)
					    printf("bliski dostupni (%d,%d)\n",point.x,point.y);
#endif
					  }
					}
				      }
				    }
				  }
				  if (razlika<OBSTACLE){
				    minDT=razlika;
				    Goal=point;
				  }else{
				    minDT=OBSTACLE;
				  }
				}
				if (minDT<OBSTACLE){
				  reset(1);//resetira samo k i h
				  if (SearchPathReverse(0)){//poziv za DT==0
				    // 							printf("eto ga\n");
				}else{
				  printf("nesto ne stima\n");
				}
				if ((pathf=(getPath_forward()))==NULL){
				  printf("TSPreplan: Putanja je NULL na velikom skoku od starta (%d,%d) do cilja (%d,%d)\n",Start.x,Start.y,Goal.x,Goal.y);
				 //odredi next-ove
					map[Goal.x][Goal.y].Pp=-2;
					
				  Goal=Start;
#if (ISPIS)
				  printf("Nema bliskog nedostupnom!!!!\n");
#endif
				}
				}else{
					map[Goal.x][Goal.y].Pp=-2;
					
				  Goal=Start;
#if (ISPIS)
				  printf("Nema bliskog nedostupnom!!!!\n");
#endif
				}			
			Start=Goal;//sljedeca pozicija, ili ova odredjena s minDT ili gore odredjena iz 4 okolnja
			if (map[Start.x][Start.y].DTdostupni==0){
			  printf("nedostupni start\n");
			}
			#if (ISPIS)
			printf("sljedeca pozicija (%d,%d) DTdostupni=%d\n",Start.x,Start.y,map[Start.x][Start.y].DTdostupni);
			#endif
		}//od while
		printf("gotov je!\n");
		replanko=0;
		prviput=0;
		return true;
}
		
//ova funkcija se druga pokrene
//ovako se zove jer CCDStar trazi inace od cilja, a ovaj trazi od starta
bool CCDStar::SearchPathReverse(int DT){ //DT moze biti 1 - racuna se matrica DT, 0- ne racuna se, nego put do cilja, za >1 - racuna se do najblize DT vrijednosti (minDT), -2 do najblizeg od trenutnog bez obzira na iznos minDT
	int Goal_f, Goal_h, f_old, h_old, k_old, g_old;
	if (DT==0){
		if ( (Goal.x==Start.x) && (Goal.y==Start.y) )
		{
			printf("DS> goal==start! gotovo!\n");
			return true;
		}
	}
//	if (gettimeofday(&timeStart, NULL) == 0)
//	{
//		mySecStart = timeStart.tv_sec;
//		myMSecStart = timeStart.tv_usec / 1000;
//	}
	NumElemListaReverse=0;
	insertNodeReverse(Start,0);
	watchdog_counter=0;
	azurirani_reverse=0;
	GoalRacunac=Goal; //ignoriraj prepisivanje
	int nemavise=1;//zastavica za trazenje najblizeg nesusjednog neposjecenog
	int imaneposjecenog=0;//zastavica da postoji neposjeceni i zapisan je u cilju, al svejedno trazi drugog
	int susjednaprepreka;//treba mi za rjesavanje matrice DT
	I_point point;
	
	while (NumElemListaReverse != 0)
	{
		MinCostAtom=(glava_reverse)->element;
		MinCostElemLista=MinCostAtom;
// 		printf("watchdog_counter=%d, MinCostElemLista=(%d,%d)\n",watchdog_counter,MinCostElemLista.x,MinCostElemLista.y);

		if (DT==0){
			if (((GoalRacunac.x==MinCostElemLista.x) && (GoalRacunac.y==MinCostElemLista.y))){
#if (ISPIS)
				printf("izracunao putem\n");
#endif
				break;
			}
		}
		if (DT>1){
#if (ISPIS)
// 		  	printf("watchdog_counter=%d, MinCostElemLista=(%d,%d)\n",watchdog_counter,MinCostElemLista.x,MinCostElemLista.y);
#endif
#if (BEZ_PREKLAPANJA)
			if ((map[MinCostElemLista.x][MinCostElemLista.y].Pp==0)&&(!((map[MinCostElemLista.x][MinCostElemLista.y].DTdostupni==0)&&(IsValid(MinCostElemLista.x,MinCostElemLista.y)==1))))
#else
			if ((map[MinCostElemLista.x][MinCostElemLista.y].DT==DT)&&(map[MinCostElemLista.x][MinCostElemLista.y].Pp==0)&&(!((map[MinCostElemLista.x][MinCostElemLista.y].DTdostupni==0)&&(IsValid(MinCostElemLista.x,MinCostElemLista.y)==1))))
#endif
			{
				Goal=MinCostElemLista;
				nemavise=0;
#if (ISPIS)
printf("izracunao do nesusjednog i dobio cilj=(%d,%d), DT=%d\n",Goal.x,Goal.y,map[MinCostElemLista.x][MinCostElemLista.y].DT);
#endif
				break;
			}
		}
		if (DT==-2)//novo stanje
		{
			if ((map[MinCostElemLista.x][MinCostElemLista.y].DTdostupni!=0)&& (map[MinCostElemLista.x][MinCostElemLista.y].Pp==0)) {
				Goal=MinCostElemLista;
//				nemavise=0;//patak ovo zakomentiravam kad rupe hocu zadnje posjetit
				imaneposjecenog=1;
#if (ISPIS)
				printf("izracunao do nesusjednog i dobio cilj=(%d,%d), DTdostupni=%d\n",Goal.x,Goal.y,map[Goal.x][Goal.y].DTdostupni);
#endif
				break;//patak ovo zakomentiravam
				
			}
#if 0			//sad ne ide preblizu rubovima vise
			susjednaprepreka=0;
			if ((map[MinCostElemLista.x][MinCostElemLista.y].DTdostupni==0)&& (map[MinCostElemLista.x][MinCostElemLista.y].Pp==0)) susjednaprepreka=1;
//			for ( int d = 0; d < 8; d++ )
//			{
//				//odredjivanje pozicije novog susjeda
//				point.x=MinCostElemLista.x+xofs[d];
//				point.y=MinCostElemLista.y+yofs[d];
//				if (( IsValid( point.x, point.y )==2 )){
//					susjednaprepreka=1;
//					break;
//				}
//			}
			if (susjednaprepreka==1){
				for (int i=MinCostElemLista.x-robot_mask;i<=MinCostElemLista.x+robot_mask;i++){
					for (int j=MinCostElemLista.y-robot_mask;j<=MinCostElemLista.y+robot_mask;j++){
						if  (IsValid(i,j)==1){
							
						if (map[i][j].DTdostupni!=0){
//							Goal=MinCostElemLista;
							Goal.x=i;
							Goal.y=j;
							nemavise=0;
#if (ISPIS)
				printf("izracunao bliskog prepreci i dobio cilj=(%d,%d), DTdostupni=%d\n",Goal.x,Goal.y,map[Goal.x][Goal.y].DTdostupni);
#endif
							break;
							
						}
						}
					}
					if (nemavise==0){
						break;
					}
				}
				if (nemavise==0){
					break;
				}
			}
#endif
		}
		processStateReverse(DT);//poziv s DT-om jel se on racuna ili ne
		++watchdog_counter;
      
		if (watchdog_counter > 20*MapSizeX*MapSizeY)
		{
			printf("Previse posjecenih cvorova!!!!\n");
			return false;
		}
	}   //od while
	if (gettimeofday(&timeNow, NULL) == 0)
	{
		mySecNow = timeNow.tv_sec;
		myMSecNow = timeNow.tv_usec / 1000;
	}
	vremenska_razlika=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
#if (ISPIS)
	printf("CCDStar SearchPathReverse> trajanje algoritma kod iscrpnog ili kratkog pretrazivanja = %d ms, watchdog=%d, broj azuriranih=%d\n",vremenska_razlika,watchdog_counter,azurirani_reverse);
//	if ( (logfile = fopen("komb","a")) == NULL )
//		printf("Error! komb file couldn't be opened.");
//	else{
//		fprintf(logfile,"\nCCDStarReverse inicijalno: trajanje %d ms, azurirani=%d, broj cvorova na listi %d, broj iteracija %d, GoalRacunac=(%d,%d), f=%d\n", vremenska_razlika, azurirani_reverse, NumElemListaReverse, watchdog_counter,GoalRacunac.x,GoalRacunac.y, map[GoalRacunac.x][GoalRacunac.y].total_cost_int);
//		fclose(logfile);
//	}
#endif
	if (DT==-1){
		br_praznih=watchdog_counter;//mislim da ih je tu 1 vise nego kod sanje jer se broji i start
	}
	watchdog_counter=0;
	azurirani_reverse=0;
	if (imaneposjecenog)
		nemavise=0;

	if (((DT==-2) || (DT>1)) && (nemavise)){
		return false;
	}
	
	return true;
}



void CCDStar::processStateReverse(int DT) {
  DStarCell **dsmap=DS->GetMap();//treba zbog prepreka

	int f_val, k_val, h_val, dt_val;
	//I_point element pokazuje na neku celiju u mapi!!!
	//prosirenje cvora na susjedne u svih 8 smjerova
	I_point point;
	int h_point, f_point;
	if (!(brisi(&glava_reverse, MinCostElemLista))){//brisanje iz prave liste
		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
			fprintf(logfile,"SearchPath> nema trazenog elementa u pravoj listi\n");
			fclose(logfile);
		}
	}
	map[MinCostElemLista.x][MinCostElemLista.y].tag_reverse=CLOSED;
	NumElemListaReverse--;
// 	f_val = map[MinCostElemLista.x][MinCostElemLista.y].total_cost_int;    //vektor najboljeg <f_val, k_val>
	f_val=0;//bez heuristike
	k_val = map[MinCostElemLista.x][MinCostElemLista.y].k_cost_int_reverse;
	h_val = map[MinCostElemLista.x][MinCostElemLista.y].h_cost_int_reverse;
	if (DT==-1){
	  dt_val = map[MinCostElemLista.x][MinCostElemLista.y].DT;
	}
	if (DT==1){
	  dt_val = map[MinCostElemLista.x][MinCostElemLista.y].DTdostupni;
	}
//u inicijalnom se racuna samo ovo lower stanje, raise je za replaniranje
	// LOWER STANJE
// 	if (k_val == h_val)//tu najbolji moze bit prepreka samo u slucaju da je P->N i hovi su razliciti ili da je P NEW
	{
		for ( int d = 0; d < 8; d++ )
		{
      //odredjivanje pozicije novog susjeda
			point.x=MinCostElemLista.x+xofs[d];
			point.y=MinCostElemLista.y+yofs[d];

			if (( IsValid( point.x, point.y )!=0 ))//==1
			{
				h_point = map[point.x][point.y].h_cost_int_reverse;
//postavljen cost c izmedju najboljeg stanja i susjeda
				arc_cost( MinCostElemLista.x, MinCostElemLista.y, point.x, point.y);//prvi nadodani uvjet je da ne ode u dedend bezveze
				if ((DT>1)||(DT==-1)||(DT==-2)){
				  if (c==OBSTACLE){
				    c=EMPTYC+COST_MASK+1;
				    if (COST_MASK==0)
				      c=EMPTYC;
				  }
				  if ((dsmap[point.x][point.y].prepreka_bool==true)&&(dsmap[point.x][point.y]._maska.x==point.x)&&(dsmap[point.x][point.y]._maska.y==point.y)){
				    c=OBSTACLE;
				  }
				}
				if ((DT==-2) && 1){//za DT==-2 radi bez costmaske put
					if (c<OBSTACLE){
						c=EMPTYC;
					}
				}
// 				if ( ((map[point.x][point.y].tag_reverse==NEW)&&(c<OBSTACLE)&&(h_val<OBSTACLE)) || ( ((map[point.x][point.y]._next_reverse.x == MinCostElemLista.x) && (map[point.x][point.y]._next_reverse.y == MinCostElemLista.y)) && (h_point != h_val + c*travCost[d]) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && ((h_point<OBSTACLE)||(c<OBSTACLE))) || ( ((map[point.x][point.y]._next_reverse.x != MinCostElemLista.x) || (map[point.x][point.y]._next_reverse.y != MinCostElemLista.y)) && (h_point > h_val + c*travCost[d]) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE))) )
if (  ((map[point.x][point.y].tag_reverse==NEW) || (h_point > h_val + c*travCost[d])) && (c<OBSTACLE))
{
					if ((h_val>OBSTACLE)&&(k_val>OBSTACLE)){
						printf("debeli LOWER\n");
					}
			
					map[point.x][point.y]._next_reverse.x = MinCostElemLista.x;
					map[point.x][point.y]._next_reverse.y = MinCostElemLista.y;
					insertNodeReverse(point, h_val + c*travCost[d]);
					if (DT==1){
// 						map[point.x][point.y].DTdostupni=dt_val+c; //jedinicna cijena za ravno i dijagonalu, a zasto?
						map[point.x][point.y].DTdostupni=h_val + c*travCost[d];
					}
					if (DT==-1){
					  map[point.x][point.y].DT=dt_val+c; //jedinicna cijena za ravno i dijagonalu
					}
				}
			}
			
		} //for susjedi
	}


}



void    CCDStar::insertNodeReverse( I_point element, int h_new )
{
	if (map[element.x][element.y].tag_reverse==NEW)	//NEW (nikad bio na listi)
	{
		map[element.x][element.y].k_cost_int_reverse = h_new;
	}
	else if (map[element.x][element.y].tag_reverse==OPEN)	//ako je OPEN
	{
		map[element.x][element.y].k_cost_int_reverse = std::min(map[element.x][element.y].k_cost_int_reverse, h_new);
		if (!(brisi(&glava_reverse, element))){//brisanje iz prave liste
			if ( (logfile = fopen("komb","a")) == NULL )
				printf("Error! komb file couldn't be opened.");
			else{
				fprintf(logfile,"insertNode> nema trazenog elementa u pravoj listi\n");
				fclose(logfile);
			}
		}
		NumElemListaReverse--;
	}
	else 	// ako je CLOSED
	{
		map[element.x][element.y].k_cost_int_reverse = std::min(map[element.x][element.y].h_cost_int_reverse, h_new);
	}
	map[element.x][element.y].h_cost_int_reverse = h_new;
	if ((map[element.x][element.y].k_cost_int_reverse>OBSTACLE)&&(h_new>OBSTACLE)){
		printf("tu su oba debela (%d,%d)\n",element.x,element.y);
	}
	map[element.x][element.y].tag_reverse = OPEN;
	NumElemListaReverse++;     //povecali smo brojac za listu
	azurirani_reverse++;
	if (!(dodaj_reverse (&glava_reverse, element))){//dodavanje u pravu listu
		if ( (logfile = fopen("komb","a")) == NULL )
			printf("Error! komb file couldn't be opened.");
		else{
			fprintf(logfile,"insertNode> nema mjesta za stvaranje novog cvora\n");
			fclose(logfile);
		}
	}
}

//POSTAVLJANJE COSTOVA PRIJELAZA IZ STANJA Y U X
      //zadnja verzija---c(x,y)=c(y,x) i to veci broj uvijek gledano iz cost mape
void CCDStar::arc_cost(int X_cell_x, int X_cell_y, int Y_cell_x, int Y_cell_y)
{
  DStarCell **dsmap=DS->GetMap();//treba zbog prepreka

  c = std::max(dsmap[ X_cell_x ][ X_cell_y ].traversal_cost, dsmap[Y_cell_x ][ Y_cell_y ].traversal_cost);
}
    
//PROVJERA VALJANOSTI POZICIJE U MAPI
int		CCDStar::IsValid(int x, int y )
{
  DStarCell **dsmap=DS->GetMap();//treba zbog prepreka
    if ( x < 0 || x >= MapSizeX || y < 0 || y >= MapSizeY)
				return 0;
      //ako je tocka unutar mape ali je zauzeta!!!!
      if (dsmap[ x ][ y ].prepreka_bool == true){
        return 2;
        }
			return 1;
}




//tu sad dodajem funkcije za pravu listu
// Dodavanje u listu sortiranu po rastucoj vrijednosti elementa
// vraca 1 ako uspije, inace 0
int CCDStar::dodaj_reverse (atom **glavap, I_point element) {
	atom *novi, *p;
	int el_fb,el_f,el_k,fb=0,f=0,k=0;
	I_point temp;
	if ((novi = (atom *) malloc(sizeof(atom))) == NULL)
		return 0;
	novi->element = element;
	el_fb=0;
// 	el_f=map[element.x][element.y].total_cost_int;
	el_f=0;
	el_k=map[element.x][element.y].k_cost_int_reverse;
	if (*glavap != NULL){
		temp=(*glavap)->element;
		fb=0;
// 		f=map[temp.x][temp.y].total_cost_int;
		f=0;
		k=map[temp.x][temp.y].k_cost_int_reverse;
	}
	if (*glavap == NULL || (LESSEQ3(el_fb,el_f,el_k,fb,f,k))) {//(*glavap)->element >= element
    // Dodavanje na pocetak liste
		novi->sljed = *glavap;
		*glavap = novi;
	} else {
	// Dodavanje iza postojeceg elementa kad:
	// a) postojeci atom nema sljedeceg
	// b) element u sljedecem cvoru je veci ili jednak novome
		for (p = *glavap; p->sljed ; p = p->sljed){//;&& (p->sljed)->element < element, a ako to nije treba brejkat
			temp=(p->sljed)->element;//oprez! ako je strogo < onda jednaki elementi koji dolaze novi idu ispred, a ako je <= onda idu iza
			fb=0;//potrebno je usporediti ta dva slucaja!
// 			f=map[temp.x][temp.y].total_cost_int;
			f=0;
			k=map[temp.x][temp.y].k_cost_int_reverse;
			if (LESSEQ3(el_fb,el_f,el_k,fb,f,k))//  strogo <   LESSEQ3(el_fb,el_f,el_k,fb,f,k), <=  !LESSEQ3(fb,f,k,el_fb,el_f,el_k)
				break;
		}
		novi->sljed = p->sljed;
		p->sljed = novi;
	}
	return 1;
}



// Brisanje elementa liste po kljucu
// Objedinjuje trazenje i brisanje
int CCDStar::brisi (atom **glavap, I_point element) {
	atom *p;
	for (; *glavap && (((*glavap)->element.x != element.x)||((*glavap)->element.y != element.y)); glavap = &((*glavap)->sljed));
	if (*glavap) {
		p = *glavap;
// 		map[p->element.x][p->element.y].tag=CLOSED;
		*glavap = (*glavap)->sljed;
		free (p);
// 		NumAtom--;
// 		NumElemLista--;
		return 1;
	} else {
		return 0;
	}
}
 

void CCDStar::Free()
{

	for (int i=0; i<MapSizeX; i++){
		free(map[i]);
	}
	free(map);
	atom *slj=NULL;
	while (glava_reverse){
		slj=glava_reverse;
		glava_reverse=glava_reverse->sljed;
		free(slj);
// 		printf("mujo\n");
	}
	if(path) free(path);
	if(path_forward) free(path_forward);
}

//treba mi reset cijena samo kasnije
void CCDStar::reset(int ponovo)
{
  if (ponovo==1){
	  prepreka.x=-1;prepreka.y=-1;
	  nemanext.x=-1;nemanext.y=-1;
	  petlja.x=-1;petlja.y=-1;
	  glava_reverse = NULL;
	  for (int i=0; i<MapSizeX; i++){
		  for (int j=0; j<MapSizeY; j++){
			  map[i][j]._next_reverse.x=-1; map[i][j]._next_reverse.y=-1; //sluze kao "NULL" vrijednosti
			  map[i][j].k_cost_int_reverse=0;
			  map[i][j].h_cost_int_reverse=0;
			  map[i][j].tag_reverse=NEW;
		  }
	  }
  }else{
    PathLength=0;
	  glava_reverse = NULL;
	  NumElemListaReverse=0;
	  MinCostElemLista.x=-1;MinCostElemLista.y=-1;
	  MinCostAtom.x=-1;MinCostAtom.y=-1;
	  Start.x=-1; Start.y=-1;
	  StartRacunac.x=-1;StartRacunac.y=-1;
	  GoalRacunac.x=0;
	  GoalRacunac.y=0;
	  Goal.x=-1;Goal.y=-1;
	  time_stamp_counter=0;
	  watchdog_counter=0;
	  br_praznih=0;
	  prviput=1;
	  replanko=0;
	  racunaoupromjeni=0;
	  prepreka.x=-1;prepreka.y=-1;
	  nemanext.x=-1;nemanext.y=-1;
	  petlja.x=-1;petlja.y=-1;
	  preslik.x=-1.;
	  preslik.y=-1.;
	  distance_robot_tool=0.; //900.;
	  MR=6;//ovo se vise ne koristi - odredjuje se put robota ali se ne gleda
	  robot_mask=3;//maska toola - sve se prekriva samo s toolom
	  redund=0;//ako hocu 1 polje preklapanje onda stavim 1, 2 polja preklapanje stavim 2, 0 bez preklapanja
	  preklapanje=2*robot_mask-redund;
	  numcycles=0;
	  numcyclesvel=0;
          tocka.x=-5.;tocka.y=-5.;
	  tockaR.x=-5.; tockaR.y=-5.;
          coilpeak=false;
 //sve se resetira osim vrijednosti s preprekama i costovima, to se iz GM-a prepisuje bio on pun ili prazan
	  for (int i=0; i<MapSizeX; i++){
		  for (int j=0; j<MapSizeY; j++){
			  map[i][j]._next_reverse.x=-1; map[i][j]._next_reverse.y=-1; //sluze kao "NULL" vrijednosti
			  map[i][j].k_cost_int_reverse=0;
			  map[i][j].h_cost_int_reverse=0;
			  map[i][j].total_cost_int=0;
			  map[i][j].tag_reverse=NEW;
			  map[i][j].DT=0;
			  map[i][j].DTdostupni=0;
			  map[i][j].Pp=0;
			  map[i][j].P7=0;
			  map[i][j].presao=0;
			  map[i][j].presao7=0;
			  map[i][j].tag_preklop=0;
		  }
	  }
  }
  
}




//ova se 3. po redu pokrene kada hoces dobit put od starta do cilja
//dobivanje puta od cilja do starta jer su tako pointeri postavljeni (treba ga onda okrenuti)
 I_point* CCDStar::getPath_forward(){

  //ako treba inicijalizirati path
	 I_point temp1,temp2;
	 PathLength_forward=1;
	  //prvo provjeravamo dal je put ok i kolko je dugacak
	 temp1.x=Goal.x; temp1.y=Goal.y;
	 while((temp1.x!=Start.x)||(temp1.y!=Start.y))
	 {
		 if ( IsValid( temp1.x, temp1.y )!=1 )
		 {
			 printf("CCDStar> Odriftali smo iz mape il je mozda prepreka!!! temp1.x=%d, temp1.y=%d, Start.x=%d, Start.y=%d\n", temp1.x,temp1.y,Start.x,Start.y);
			 
			 return NULL;
		 }
		 else
		 {
// 			 printf("CCDStar:getPath_forward()> temp1=(%d,%d)\t", temp1.x,temp1.y);
			 temp2.x=map[temp1.x][temp1.y]._next_reverse.x;
			 temp2.y=map[temp1.x][temp1.y]._next_reverse.y;
			 if ((temp2.x==-1)||(temp2.y==-1)) {
				 printf("nema next (%d,%d)\n",temp1.x,temp1.y);
				 
			 return NULL;
			 }

			 temp1.x=temp2.x;
			 temp1.y=temp2.y;
			 PathLength_forward++;
			 if (PathLength_forward>MapSizeX*MapSizeY)
			 {
				 printf("jao petlja!time_stamp_counter=%d\n",time_stamp_counter);
				 return NULL;
			 }
		 }
	 }


	 temp1.x=Goal.x; temp1.y=Goal.y;
	 path_forward[0]=Goal;
	 temp1.x=map[path_forward[0].x][path_forward[0].y]._next_reverse.x;
	 temp1.y=map[path_forward[0].x][path_forward[0].y]._next_reverse.y;
	 for(int i=1;i<PathLength_forward;i++)
	 {
		 path_forward[i].x=temp1.x;
		 path_forward[i].y=temp1.y;
		 temp1.x=map[path_forward[i].x][path_forward[i].y]._next_reverse.x;
		 temp1.y=map[path_forward[i].x][path_forward[i].y]._next_reverse.y;
	 }
 	
	 return path_forward;
 }

I_point* CCDStar::getPath(){ //ovo se ne izvrsava
	
	//ako treba inicijalizirati path
	int deltax,deltay,pom1x,pom1y;//,pom2x,pom2y;
	double pomocni;
	R_point preslik2;
	deltax=(path[1].x-path[0].x);
	deltay=(path[1].y-path[0].y);
	pom1x=(Start.x-path[0].x);
	pom1y=(Start.y-path[0].y);
	pomocni=(pom1x*deltax+pom1y*deltay)/double(deltax*deltax+deltay*deltay);
// 			 printf("pomocni=%f, pom1x=%d, pom1y=%d, deltax=%d, deltay=%d, Start=(%d,%d)\n",pomocni, pom1x,pom1y,deltax,deltay,Start.x, Start.y);
preslik.x=path[0].x+pomocni*double(deltax);
preslik.y=path[0].y+pomocni*double(deltay);
// 			 pom1x=int(preslik.x);
// 			 pom1y=int(preslik.y);
if ((fabs(preslik.x-double(path[1].x))<=0.5) && (fabs(preslik.y-double(path[1].y))<=0.5)){
	for(int i=1;i<PathLength;i++){
		path[i-1]=path[i];
	}
	PathLength--;
}else{
	deltax=(path[2].x-path[1].x);
	deltay=(path[2].y-path[1].y);
	pom1x=(Start.x-path[1].x);
	pom1y=(Start.y-path[1].y);
	pomocni=(pom1x*deltax+pom1y*deltay)/double(deltax*deltax+deltay*deltay);
// 			 printf("pomocni=%f, pom1x=%d, pom1y=%d, deltax=%d, deltay=%d, Start=(%d,%d)\n",pomocni, pom1x,pom1y,deltax,deltay,Start.x, Start.y);
preslik2.x=path[1].x+pomocni*double(deltax);
preslik2.y=path[1].y+pomocni*double(deltay);
// 			 pom1x=int(preslik.x);
// 			 pom1y=int(preslik.y);
if ((fabs(preslik2.x-double(path[1].x))<=0.5) && (fabs(preslik2.y-double(path[1].y))<=0.5)){
	for(int i=1;i<PathLength;i++){
		path[i-1]=path[i];
	}
	PathLength--;
}
}
	
	return path;
}

R_point* CCDStar::getPathRobot(){
	
	//ako treba inicijalizirati path
	double deltax,deltay,pom1x,pom1y;//,pom2x,pom2y;
	double pomocni;
	double startx, starty;
	R_point preslik2,path0;
//	startx=(WH->RB.x-GM->Map_Home.x-GM->Map_Cell_Size/2)/GM->Map_Cell_Size;
//	starty=(WH->RB.y-GM->Map_Home.y-GM->Map_Cell_Size/2)/GM->Map_Cell_Size;
	startx=WH->RB.x;
	starty=WH->RB.y;
	int indeks=0;
	for(int i=1; i<PathLength; i++) //ima ponovljenih tocaka pa mora ovo
	{
	deltax=pathrobot[i].x-pathrobot[0].x;
	deltay=pathrobot[i].y-pathrobot[0].y;
	if (fabs(deltax)+fabs(deltay)<0.0001)
		continue;
	indeks=i;
	break;
	}
//	deltax=(pathrobot[indeks].x-pathrobot[0].x);
//	deltay=(pathrobot[indeks].y-pathrobot[0].y);
//	pom1x=(double(Start.x)-pathrobot[0].x);
//	pom1y=(double(Start.y)-pathrobot[0].y);
        path0.x=GM->Map_Home.x+pathrobot[0].x*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
        path0.y=GM->Map_Home.y+pathrobot[0].y*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
//        printf("path0=(%f,%f)\n",path0.x,path0.y);
      	deltax=(pathrobot[indeks].x-pathrobot[0].x)*GM->Map_Cell_Size;
	deltay=(pathrobot[indeks].y-pathrobot[0].y)*GM->Map_Cell_Size;
//        printf("delta=(%f,%f)\n",deltax,deltay);

	pom1x=(startx-path0.x);  
	pom1y=(starty-path0.y);
//        printf("pom1=(%f,%f)\n",pom1x,pom1y);

//	pom1x=(startx-pathrobot[0].x);
//	pom1y=(starty-pathrobot[0].y);
	pomocni=(pom1x*deltax+pom1y*deltay)/double(deltax*deltax+deltay*deltay);
//	printf("pomocni=%f\n",pomocni);
// 			 printf("pomocni=%f, pom1x=%d, pom1y=%d, deltax=%d, deltay=%d, Start=(%d,%d)\n",pomocni, pom1x,pom1y,deltax,deltay,Start.x, Start.y);
//preslik.x=pathrobot[0].x+pomocni*double(deltax);//pa krivo je- ovo je projekcija na segment start path[0] umjesto na segment path[0]-path[1] pomocni je kosinus kuta, tu treba ici pom1x pom1y
//preslik.y=pathrobot[0].y+pomocni*double(deltay);
preslik.x=path0.x+pomocni*double(deltax);
preslik.y=path0.y+pomocni*double(deltay);
       printf("preslik=(%f,%f)\n",preslik.x,preslik.y);

preslik.x=(preslik.x-GM->Map_Home.x-GM->Map_Cell_Size/2.)/GM->Map_Cell_Size;
preslik.y=(preslik.y-GM->Map_Home.y-GM->Map_Cell_Size/2.)/GM->Map_Cell_Size;
//       printf("preslik=(%f,%f)\n",preslik.x,preslik.y);
//preslik.x=pathrobot[0].x+pomocni*double(pom1x);
//preslik.y=pathrobot[0].y+pomocni*double(pom1y);
// 			 pom1x=int(preslik.x);
// 			 pom1y=int(preslik.y);
  for(int i=indeks;i<PathLength;i++){
    if ((fabs(StartRacunac.x-double(path[i].x))<=1.5) && (fabs(StartRacunac.y-double(path[i].y))<=1.5)){
      indeks=i;
      break;    
    }
  
  }
//if ((fabs(preslik.x-double(pathrobot[indeks].x))<=3.5) && (fabs(preslik.y-double(pathrobot[indeks].y))<=3.5)){
	for(int i=indeks;i<PathLength;i++){
		pathrobot[i-indeks]=pathrobot[i];
		path[i-indeks]=path[i];//paralelno oduzimam od jednog i od drugog, ovo gore se ne smije pozivati!!!
	}
	PathLength=PathLength-indeks;
//}
	
	return pathrobot;
}






R_point CCDStar::getGoal(){

R_point tempgoal;
I_point temp;
tempgoal.x=-1; tempgoal.y=-1; tempgoal.th=-1;
	double deltax,deltay,kut,kut_prvi,kut_razlika;
	int path_flag=0;

	for(int i=0; i<PathLength; i++)
	{
    realpathrobot[i].x=GM->Map_Home.x+pathrobot[i].x*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
    realpathrobot[i].y=GM->Map_Home.y+pathrobot[i].y*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
    realpathtool[i].x=GM->Map_Home.x+path[i].x*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
    realpathtool[i].y=GM->Map_Home.y+path[i].y*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
  }

	for(int i=1; i<PathLength; i++) //ima ponovljenih tocaka pa mora ovo
	{
//	deltax=realpathrobot[i].x-realpathrobot[0].x;
//	deltay=realpathrobot[i].y-realpathrobot[0].y;
	deltax=realpathtool[i].x-realpathtool[0].x;
	deltay=realpathtool[i].y-realpathtool[0].y;
	if (fabs(deltax)+fabs(deltay)<0.1)
		continue;
	kut=atan2(deltay,deltax);
	if (kut<0)
		kut+=2*M_PI;
	kut_prvi=kut;
	break;
	}


	for(int i=1; i<PathLength; i++) //ima ponovljenih tocaka pa mora ovo
	{
//	deltax=realpathrobot[i].x-realpathrobot[i-1].x;
//	deltay=realpathrobot[i].y-realpathrobot[i-1].y;
	deltax=realpathtool[i].x-realpathtool[i-1].x;
	deltay=realpathtool[i].y-realpathtool[i-1].y;
	if (fabs(deltax)+fabs(deltay)<0.1)
		continue;
	kut=atan2(deltay,deltax);
		if (kut<0.0)
			kut+=2*M_PI;
		kut_razlika=kut-kut_prvi;
		if (kut_razlika<-1*M_PI)
			kut_razlika+=2*M_PI;
		if (kut_razlika>M_PI)
			kut_razlika-=2*M_PI;
		if ((path_flag==0) && (fabs(kut_razlika)>M_PI/30 || 0) )  //prva promjena
		{
				
//				printf("prvi segment: kut_razlika=%f, kut_prvi=%f, kut_drugi=%f, infleksija (%f,%f)\n",kut_razlika,kut_prvi,kut,realpathrobot[i-1].x,realpathrobot[i-1].y);
				path_flag++;
//				tempgoal=realpathrobot[i-1]; //infleksija je tocka ispred ove i-te tocke kod koje je zabiljezena promjena smjera
				tempgoal=realpathtool[i-1];//[i-1];
				tempgoal.th=kut; //zabiljezi novi kut kao ciljni
        temp=path[i-1];//[i-1]; 
				printf("prvi segment: kut_razlika=%f, kut_prvi=%f, kut_drugi=kut=%f, infleksija i-1=%d (%f,%f,%f)\n",kut_razlika,kut_prvi,kut,i-1,tempgoal.x,tempgoal.y,tempgoal.th);
        if ((fabs(WH->RB.x-tempgoal.x)+fabs(WH->RB.y-tempgoal.y))<500.){
          printf("preblizak cilj trazim dalje\n");
          continue;
        }
//        temp.x=(int)(pathrobot[i-1].x);
//        temp.y=(int)(pathrobot[i-1].y);
        if ((map[temp.x][temp.y].presao)){
          printf("vec pregazen cilj, trazim dalje\n");
//          continue;
        }
				break;
		}
		if (path_flag>0){
//		    tempgoal=realpathrobot[i-1];
		    tempgoal=realpathtool[i-1];//[i-1];
		    tempgoal.th=kut;
        temp=path[i-1];//[i-1];
		    if ((fabs(WH->RB.x-tempgoal.x)+fabs(WH->RB.y-tempgoal.y))<300.){
          continue;
        }
//        temp.x=(int)(pathrobot[i-1].x);
//        temp.y=(int)(pathrobot[i-1].y);
        if ((map[temp.x][temp.y].presao)){
//          continue;
        }
        printf("dalji cilj odabran (%f,%f)\n", tempgoal.x, tempgoal.y);
        break;
		}

	}
	
	if (path_flag==0){
//	  tempgoal=realpathrobot[PathLength-1];
	  tempgoal=realpathtool[PathLength-1];
	  tempgoal.th=0;
	  temp=path[PathLength-1];
	}

#if RECTANGULAR  
  int okolina=(int)(ceil(distance_robot_tool/GM->Map_Cell_Size)); //udaljenost toola od robota
  double difftemp, diffmin=(double)okolina; //pomocna razlika od udaljenosti
  I_point temp_i;
  R_point newgoal, rpointpom;
  temp_i.th=0;
//  kut_prvi=kut;
  kut=2*M_PI;
  for (int ix=temp.x-okolina;ix<=temp.x+okolina;ix++){
					for (int jy=temp.y-okolina;jy<=temp.y+okolina;jy++){
						if (IsValid(ix,jy)){
						  temp_i.x=ix;
						  temp_i.y=jy;
						  if ( ((DS->IsValidOri(temp_i))==1) && (DS->howmanyOriCollides(temp_i)==0)){ 
  						    rpointpom.x=GM->Map_Home.x+temp_i.x*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
  						    rpointpom.y=GM->Map_Home.y+temp_i.y*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size;
						      rpointpom.th=atan2((tempgoal.y-rpointpom.y),(tempgoal.x-rpointpom.x));
		              if (rpointpom.th<0.0)
			              rpointpom.th+=2*M_PI;
		              kut_razlika=rpointpom.th-kut_prvi; //tempgoal.th;
		              if (kut_razlika<-1*M_PI)
			              kut_razlika+=2*M_PI;
		              if (kut_razlika>M_PI)
			              kut_razlika-=2*M_PI;
			            kut_razlika=fabs(kut_razlika);
                  difftemp=fabs((double)(okolina)-sqrt((temp.y-temp_i.y)*(temp.y-temp_i.y)+(temp.x-temp_i.x)*(temp.x-temp_i.x)));
						      if (difftemp+kut_razlika<diffmin+kut){
						        diffmin=difftemp;
						        kut=kut_razlika;
						        newgoal=rpointpom;
					        }
						  }
						}
		      }
	}
	if (diffmin<(double)(okolina)){
//	  tempgoal=newgoal;
	  //temgoal je sad jos uvijek tool koordinata
	  tempgoal.x=tempgoal.x-distance_robot_tool*cos(newgoal.th);
    tempgoal.y=tempgoal.y-distance_robot_tool*sin(newgoal.th);
    tempgoal.th=newgoal.th;
	  printf("cilj za robota je pomaknut od toola za %f polja, %f radiana od orijentacije segmenta puta, pomocni cilj (%f,%f,%f), cilj izracnat prema stvarnom pomaku (%f,%f,%f)\n",(double)(okolina)-diffmin,kut,newgoal.x,newgoal.y,newgoal.th, tempgoal.x,tempgoal.y,tempgoal.th);
	  
	}
#endif
return tempgoal;
}


void CCDStar::loger(){
	  FILE *F4,*F5,*F6,*F7,*F3;
      if(((F5=fopen(WH_LOG_DSTAR_DT,"wt"))!=NULL)&&((F7=fopen(WH_LOG_DSTAR_DTDOSTUPNI,"wt"))!=NULL)&&((F6=fopen(WH_LOG_DSTAR_PP,"wt"))!=NULL) &&((F4=fopen(WH_LOG_DSTAR_PRESAO,"wt"))!=NULL) &&((F3=fopen(WH_LOG_DSTAR_PRESAO7,"wt"))!=NULL)){

     for(int i=0;i<GetMapSizeX();i++){
       for(int j=0;j<GetMapSizeY();j++){
		fprintf(F4,"%d ",map[i][j].presao);
		fprintf(F3,"%d ",map[i][j].presao7);
		fprintf(F5,"%d ",map[i][j].DT);
	      fprintf(F6,"%d ",map[i][j].Pp);
	      fprintf(F7,"%d ",map[i][j].DTdostupni);
       }
       fprintf(F5,"\n");fprintf(F3,"\n");fprintf(F4,"\n");fprintf(F6,"\n"); fprintf(F7,"\n");
		  }
		  fprintf(F5,"\n");fprintf(F4,"\n");fprintf(F3,"\n");fprintf(F6,"\n");fprintf(F7,"\n");
		  fclose(F3);fclose(F4);fclose(F5);fclose(F6);
		  fclose(F7);

	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }
      
            //koordinate trenutnog puta D* (iz trenutnog starta)
	  if(((F4=fopen(PATH_ROBOT_X,"wt"))!=NULL)&&((F5=fopen(PATH_ROBOT_Y,"wt"))!=NULL)&&((F6=fopen(PATH_TOOL_X,"wt"))!=NULL)&&((F7=fopen(PATH_TOOL_Y,"wt"))!=NULL) ){

		  for(int i=0;i<PathLength;i++){
			  fprintf(F4,"%f ",realpathrobot[i].x);
			  fprintf(F5,"%f ",realpathrobot[i].y);
			  fprintf(F6,"%f ",realpathtool[i].x);
			  fprintf(F7,"%f ",realpathtool[i].y);
		  }

		  fprintf(F4,"\n"); fprintf(F5,"\n");
		  fprintf(F6,"\n"); fprintf(F7,"\n");
		  fclose(F4);  fclose(F5);
		  fclose(F6);  fclose(F7);


	  }else{
		  printf("Otvaranje filea bezuspjesno!");
	  }
	  
	             //azuriranje Grid Mape!!!
       GridMap_cell **Map=GM->GetMap();
	   //koordinate prepreka X,Y, koje su od tih prepreka staticke (1), a koje dinamicke (2), vremenski ciklus azuriranja prepreke
    if(((F4=fopen(WH_LOG_GRIDMAP_OCCUPANCY,"wt"))!=NULL)){
      for(int i=0;i<GM->GetMapSizeX();i++){
       for(int j=0;j<GM->GetMapSizeY();j++){
         //ako je celija zauzeta
                fprintf(F4,"%d ",Map[i][j].occupancy);

       }
       fprintf(F4,"\n");
		  }

        fprintf(F4,"\n");
        fclose(F4);
	    }else{
        printf("Otvaranje filea bezuspjesno!");
      }


}

void CCDStar::disableCoverageOfArea(R_point robot, double size){
  R_point temptool;
  temptool.x=robot.x+distance_robot_tool*cos(robot.th);
  temptool.y=robot.y+distance_robot_tool*sin(robot.th);
  temptool.th=0;
  GM->mapper_point_temp=temptool;
  int prosirenje=ceil(size/GM->Map_Cell_Size);
  if (GM->check_point(GM->mapper_point_temp)){ 
    for(int i=GM->cell_point_temp.x-prosirenje;i<=GM->cell_point_temp.x+prosirenje;i++){
					for(int j=GM->cell_point_temp.y-prosirenje;j<=GM->cell_point_temp.y+prosirenje;j++){
						if ((IsValid(i,j)>0)){
						  if (map[i][j].presao==0){
							  map[i][j].presao=-2;
							}
						}
					}
		}
  }
}


