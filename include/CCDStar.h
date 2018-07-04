#define BEZ_PREKLAPANJA 1
#define ISPIS 1
#define DO_COVERAGE 1

#include <DStar.h>

//static const int x7ofs[ 4 ] = { 0, 9, -9, 0};
//static const int y7ofs[ 4 ] = {-9, 0, 0, 9};
//static const int x7ofs[ 4 ] = { 0, 15, -15, 0};
//static const int y7ofs[ 4 ] = {-15, 0, 0, 15};
static const int x7ofs[ 4 ] = { 0, 7, -7, 0};
static const int y7ofs[ 4 ] = {-7, 0, 0, 7};
//static const int x7ofs[ 4 ] = { 0, 5, -5, 0};
//static const int y7ofs[ 4 ] = {-5, 0, 0, 5};
static const int x6ofs[ 4 ] = { 0, 6, -6, 0};
static const int y6ofs[ 4 ] = {-6, 0, 0, 6};
//static const int x6ofs[ 4 ] = { 0, 8, -8, 0};
//static const int y6ofs[ 4 ] = {-8, 0, 0, 8};


#define WH_LOG_DSTAR_DT				"logger//wh_dstar_dt.dat"
#define WH_LOG_DSTAR_DTDOSTUPNI			"logger//wh_dstar_dtdostupni.dat"
#define WH_LOG_DSTAR_PP				"logger//wh_dstar_pp.dat"
#define WH_LOG_DSTAR_PRESAO			"logger//wh_dstar_presao.dat"
#define WH_LOG_DSTAR_PRESAO7			"logger//wh_dstar_presao7.dat"
#define PATH_ROBOT_X			"logger//path_robot_x.dat"
#define PATH_ROBOT_Y			"logger//path_robot_y.dat"
#define PATH_INIT_TOOL_X			"logger//path_init_tool_x.dat"
#define PATH_INIT_TOOL_Y			"logger//path_init_tool_y.dat"
#define PATH_TOOL_X			"logger//path_tool_x.dat"
#define PATH_TOOL_Y			"logger//path_tool_y.dat"
#define WH_LOG_GRIDMAP_OCCUPANCY				"logger//wh_gridmap_occupancy.dat"



class CCDStarCell{

  public:

	I_point _next_reverse;     //pokazuje na "najboljeg sljedeceg susjeda"
	int h_cost_int_reverse;
	int k_cost_int_reverse;
        int total_cost_int;
	int tag_reverse;
		int DT; //vrijednost udaljenosti od starta u broju polja
		int DTdostupni; //vrijednost udaljenosti od starta u broju polja
		int Pp; //vrijednost posjecenosti, 1- posjeceno, 0 - ne
	int P7; //vrijednosti preklapanja maske, 1-preklop od 6 okolo
	int presao; //vrijednost +1 upise kad robot preklopi svojom maskom fizicki
	int presao7;//vrijednost 1 upise u preklopu
	int tag_preklop;

        //defaultni konstruktor
        //pos defaultu su sve celije zauzete i treba jih "ocistiti"
        CCDStarCell()
        {
	  _next_reverse.x=-1; _next_reverse.y=-1; //sluze kao "NULL" vrijednosti
	  tag_preklop=0;
	  k_cost_int_reverse=0;
	  h_cost_int_reverse=0;
	  total_cost_int=0;  //total cost
	  tag_reverse=NEW;
	      DT=0;
	      DTdostupni=0;
	      Pp=0;
	      P7=0;
	      presao=0;
	      presao7=0;

        };
};

class CCDStar{

  public:

	  I_point Start; //trenutne koordinate robota
	  I_point StartRacunac; //pozicija na optimalnom putu najbliza trenutnoj poziciji h(Start)>=h(p)+d
	  I_point GoalRacunac;
	  I_point Goal;
	  I_point tool;
	  I_point prepreka;//kad se dogodi prepreka na putu da se zapise i moze crtat
	  I_point nemanext;//kad se to dogodi da se moze crtat
	  I_point petlja;
	  int MapSizeX, MapSizeY;
  int NumElemListaReverse;     //broj elemenata na trenutnoj listi
  I_point MinCostElemLista;          //min cost element na listi - zapravo su to x i y koordinate mape, indeksi
  I_point MinCostAtom;          //min cost element na pravoj listi - zapravo su to x i y koordinate mape, indeksi
  int time_stamp_counter;        //brojac pozivanja algoritma (u initu se inkrementira)
  int watchdog_counter;     //brojac pretrazivanja cvorova, kad je 0 znaci da je zavrseno pretrazivanje
  struct timeval timeStart;
  struct timeval timeNow;
  int mySecStart, myMSecStart,mySecNow, myMSecNow;
  int vremenska_razlika;
  //bool drziStariPut;
  int azurirani_reverse;
  int br_praznih;
  int prviput;             //oznaka inicijalnog izvodjenja
  int promjena;             //oznaka promjene u karti
  R_point preslik;
  int c;                    //cost prelaska izmedju dva cvora odnosno polja u mapi
  double distance_robot_tool;//udaljenost tool-a od centra rotacije robota
  int MR;//ovo se vise ne koristi - odredjuje se put robota ali se ne gleda
  int robot_mask;//maska toola - sve se prekriva samo s toolom
	int redund;//ako hocu 1 polje preklapanje onda stavim 1, 2 polja preklapanje stavim 2, 0 bez preklapanja
	int preklapanje; //racuna prema maski toola i redundantnom posjecivanju
  int numcycles; //broj ciklusa u kojima nema pomaka robota
  int numcyclesvel; //isto to za brzine kod kojih nema pomaka
  I_point oldStart;
  R_point tocka, tockaR;
bool coilpeak;
  
  CCDStarCell **map;

  atom *glava_reverse;		// glava liste

  I_point *path_forward;
  I_point *path;
  R_point *pathrobot, *realpathrobot, *realpathtool;
  int PathLength_forward;
  int PathLength;
  FILE	*logfile;
	  int replanko;//oznaka kada replanirati
	  int racunaoupromjeni;//oznaka kada je replanirao
  
  //funkcije
  int setCoverageOnRobotCurrentPosition(double RBx, double RBy, double RBth);
  void setCoverageOnCoil(double RBx, double RBy, double size);
  int planCoveragePath();
  R_point getGoal();
  void loger();
  void disableCoverageOfArea(R_point robot, double size);
  void updateCoverageMap();
  bool checkIfStuck(int numnewcells, double setv, double setw);
  void arc_cost(int X_cell_x, int X_cell_y, int Y_cell_x, int Y_cell_y); //racuna cijenu prijelaza izmedju dviju koordinata u mapi
  int   IsValid(int x, int y); //vraca 1 ak su koordinate u mapi, 0 ako su izvan granica mape, a 2 ako je polje zauzeto
  void reset(int ponovo);//0 ili 1 kao argument
  bool   SearchPathReverse(int DT); //glavna funkcija koja pretrazuje polje, poziva processState()
  bool   TSP(); //glavna funkcija za TSP nakon odredjene matrice DT
  bool   TSPreplan(); //u replaniranju glavna funkcija za TSP nakon odredjene matrice DT
  void	processStateReverse(int DT); //ubacuje susjeda na temelju vrijednosti aktualnog cvora
  void  insertNodeReverse( I_point element, int hnew );
  int dodaj_reverse (atom **glavap, I_point element);//dodaje cvor u sortiranu listu
  int brisi (atom **glavap, I_point element);//brise cvor koji ima element==element
  I_point* getPath_forward();  //vraca put slijedivsi next pointere i postavlja TSP pointere
  I_point* getPath();//vraca stari put s preslikom od trenutne pozicije
  R_point* getPathRobot();//vraca stari put s preslikom od trenutne pozicije
  int getPathLength_forward() {return (int)PathLength_forward;};
  int getPathLength() {return (int)PathLength;};
  I_point *GetPathForward(){return path_forward;}; //veliko G je razlika
  I_point *GetPath(){return path;}; //veliko G je razlika
  R_point *GetPathRobot(){return pathrobot;}; //veliko G je razlika
  R_point *GetRealPathRobot(){return realpathrobot;}; //veliko G je razlika
  R_point *GetRealPathTool(){return realpathtool;}; //veliko G je razlika
  
  void  Free();
//funkcije za dohvacanje internih podataka
   int GetMapSizeX() {return (int)MapSizeX;};
   int GetMapSizeY(){return (int)MapSizeY;};
   CCDStarCell **GetMap(){return map;};

  CCDStar(int size_x, int size_y)
  {
          if(size_x>0 && size_y>0)
          {
              MapSizeX=size_x; MapSizeY=size_y;
              //alokacija mape
              map = (CCDStarCell **)malloc(size_x*sizeof(CCDStarCell *)) ;
               for (int i=0; i<size_x; i++)
                  map[i]=new CCDStarCell[size_y];  //koristili smo defaultni konstruktor
              //alokacija liste
              //alokacija mape
	      glava_reverse = NULL;
			  path_forward = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
			  path = (I_point *)malloc(size_x*size_y*sizeof(I_point)) ;
			  pathrobot = (R_point *)malloc(size_x*size_y*sizeof(R_point)) ;
			  realpathtool = (R_point *)malloc(size_x*size_y*sizeof(R_point)) ;
			  realpathrobot = (R_point *)malloc(size_x*size_y*sizeof(R_point)) ;
               //za path smo isto tako inicijalizirali jedno veliko polje
              //nije optimalno sa strane utroska memorije, ali je brze
              reset(0);//0-prvi reset, 1- ponovni
          }
          else
          {
            printf("CCDStarOptim> Invalid map size");
            //fprintf(stderr,"AStarOptim> Invalid map size");
            exit(1);
          }                 

        printf("Konstruiran je CCDStar objekt\n");
  };


  
  ~CCDStar()
  {
  Free();
  printf("CCDStar unisten");

  };

};




