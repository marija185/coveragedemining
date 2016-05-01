#include <moj.h>   //daje M

#include <WorkHorse.h>    //daje WH
//// #include "DStar.h"    //daje DS (vise ga ne sadrzi moj.h)
#include <GridMap.h>    //daje GM
//#include <DynamicWindow.h> //daje DW

#include <CCDStar.h>
#include <movingobstaclesrhc/Coil.h>
#include <trajectory_msgs/JointTrajectory.h>

CCDStar *CCD;

moj *M;


//extern variables
//extern WorkHorse *WH; 
//extern DStar *DS;
extern GridMap *GM;
//extern DynamicWindow *DW;
//extern Planner *PL;

class VisualizationPublisherCCD
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  std::string target_frame_;

public:
  ros::Publisher robotpath_pub;
  ros::Publisher toolpath_pub;
//  ros::Publisher mine_pub;
//  ros::Publisher coil_pub;
  ros::Publisher visitedpath_pub;



    visualization_msgs::Marker robotpath;//path
    visualization_msgs::Marker toolpath;//path
    visualization_msgs::Marker mine;//path
    visualization_msgs::Marker coilpath;//path
    visualization_msgs::Marker visitedpath;//path

  VisualizationPublisherCCD(ros::NodeHandle n) :
      nh_(n),  target_frame_("map") //map kad koristis stage, minefield za hratc
  {

	robotpath_pub=nh_.advertise<visualization_msgs::Marker>("/robotpath_marker",10);
	toolpath_pub=nh_.advertise<visualization_msgs::Marker>("/toolpath_marker",10);
//	mine_pub=nh_.advertise<visualization_msgs::Marker>("/mine_marker",10);
//	coil_pub=nh_.advertise<visualization_msgs::Marker>("/coil_marker",10);
	visitedpath_pub=nh_.advertise<visualization_msgs::Marker>("/visitedpath_marker",10);



      //putanja
    robotpath.header.frame_id = target_frame_;
    robotpath.header.stamp = ros::Time::now();
    robotpath.ns =  "coveragedemining";
    robotpath.action = visualization_msgs::Marker::ADD;
    robotpath.pose.orientation.w  = 1.0;
    robotpath.type = visualization_msgs::Marker::LINE_STRIP;
    robotpath.scale.x = 0.05; 
    robotpath.color.r = 0;
    robotpath.color.g = 0.0;
    robotpath.color.b = 0.5;
    robotpath.color.a = 1.0;

    toolpath.header.frame_id = target_frame_;
    toolpath.header.stamp = ros::Time::now();
    toolpath.ns =  "coveragedemining";
    toolpath.action = visualization_msgs::Marker::ADD;
    toolpath.pose.orientation.w  = 1.0;
    toolpath.type = visualization_msgs::Marker::LINE_STRIP;//POINTS; //LINE_STRIP;
    toolpath.scale.x = 0.05; 
    toolpath.scale.y = 0.05; 
    toolpath.color.r = 0.;
    toolpath.color.g = 0.;
    toolpath.color.b = 0.8;
    toolpath.color.a = 1.0;

    visitedpath.header.frame_id = target_frame_;
    visitedpath.header.stamp = ros::Time::now();
    visitedpath.ns =  "coveragedemining";
    visitedpath.action = visualization_msgs::Marker::ADD;
    visitedpath.pose.orientation.w  = 1.0;
    visitedpath.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    visitedpath.scale.x = 0.06; 
    visitedpath.scale.y = 0.06; 
    visitedpath.color.r = 0.8;
    visitedpath.color.g = 0.;
    visitedpath.color.b = 0.;
    visitedpath.color.a = 1.0;
  
#if 0
    mine.header.frame_id = target_frame_;
    mine.header.stamp = ros::Time::now();
    mine.ns =  "coveragedemining";
    mine.action = visualization_msgs::Marker::ADD;
    mine.pose.orientation.w  = 1.0;
    mine.type = visualization_msgs::Marker::POINTS;
    mine.scale.x = 0.1;
    mine.scale.y = 0.1; 
    mine.color.r = 1.0;
    mine.color.g = 0.;
    mine.color.b = 0.;
    mine.color.a = 1.0;

    coilpath.header.frame_id = target_frame_;
    coilpath.header.stamp = ros::Time::now();
    coilpath.ns =  "coveragedemining";
    coilpath.action = visualization_msgs::Marker::ADD;
    coilpath.pose.orientation.w  = 1.0;
    coilpath.type = visualization_msgs::Marker::POINTS;
    coilpath.scale.x = 0.1;
    coilpath.scale.y = 0.1; 
    coilpath.color.r = .0;
    coilpath.color.g = 1.;
    coilpath.color.b = 0.;
    coilpath.color.a = 1.0;
#endif
    
  }

  void visualizationduringmotion();


};

class MetalDetector
{
public:
    MetalDetector() : tf_(),  target_frame_("minefield") 
    {
        md_sub_ = n_.subscribe("coils", 1,&MetalDetector::msgCallback,this);
	tf_.waitForTransform(target_frame_,"right_coil", ros::Time(0), ros::Duration(3.0));
	tf_.waitForTransform(target_frame_,"left_coil", ros::Time(0), ros::Duration(3.0));
	tf_.waitForTransform(target_frame_,"middle_coil", ros::Time(0), ros::Duration(3.0));
    }

private:
    ros::Subscriber md_sub_;
    tf::TransformListener tf_;
    ros::NodeHandle n_;
    std::string target_frame_;

    //  Callback to register with tf::MessageFilter to be called when transforms are available
    void msgCallback(const boost::shared_ptr<const movingobstaclesrhc::Coil>& coil_ptr)
    {
        tf::StampedTransform transformb, transform_auxiliar;
        geometry_msgs::PointStamped point_out, point_coil;
        double tfx, tfy;
	std::string t_frame;
	if ((coil_ptr->header.frame_id=="metal_detector/right_coil")||(coil_ptr->header.frame_id=="right_coil"))
		t_frame = "right_coil";
	else if((coil_ptr->header.frame_id=="metal_detector/left_coil")||(coil_ptr->header.frame_id=="left_coil"))
		t_frame = "left_coil";
	else
		t_frame = "middle_coil";

        try{
        	tf_.lookupTransform(target_frame_,t_frame.c_str(), ros::Time(0), transformb);
        	point_out.point.x = transformb.getOrigin().x();
		      point_out.point.y = transformb.getOrigin().y();
		      point_out.point.z = transformb.getOrigin().z();
        }catch (tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
        }
        try{
          tf_.lookupTransform(target_frame_,"/base_link", ros::Time(0), transform_auxiliar);
        }catch (tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
        }
        try{
          tf_.lookupTransform("/base_link",t_frame.c_str(), ros::Time(0), transformb);
        }catch (tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
        }
        tf::Transform transform_coil=transform_auxiliar*transformb;
    	  point_coil.point.x=transform_coil.getOrigin().x();
	      point_coil.point.y=transform_coil.getOrigin().y();
	      point_coil.point.z=transform_coil.getOrigin().z();
	      
        try
        {
		
	  tfx=point_coil.point.x;
	  tfy=point_coil.point.y;
    printf("transform coil: x [%f] y [%f]\n",tfx,tfy ); 
//		if (coil_ptr->header.frame_id=="left_coil" && coil_ptr->channel[0]<-157800){
//			CCD->coilpeak=true;
//			CCD->tocka.x=tfx;
//			CCD->tocka.y=tfy;
//		}
//		if (coil_ptr->header.frame_id=="right_coil" && coil_ptr->channel[0]<-214800){
//			CCD->coilpeak=true;
//			CCD->tockaR.x=tfx;
//			CCD->tockaR.y=tfy;
//		}
               CCD->setCoverageOnCoil( tfx, tfy, 200.);//100 mm around point will be set


        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what());
        }
    }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "coverage");
  int startcycle;
  bool gobackwards=false;
  int backwardsign=1;
  ros::NodeHandle nh;
  M = new moj(nh);
  
//  double map_width=0, map_height=0;
//  if (nh.getParam("/coverage/map_width", map_width))
//  {
//    printf("map width is %f m\n", map_width);
//  }
//  if (nh.getParam("/coverage/map_height", map_height))
//  {
//    printf("map height is %f m\n", map_height);
//  }
//    ROS_ERROR("map width is %f m", map_width);
//    ROS_ERROR("map height is %f m", map_height);

  M->initializePlanner(); 
//  ros::Duration(5).sleep();  //sad ovo ne trebam

  VisualizationPublisher visual(nh);
  VisualizationPublisherCCD visualCCD(nh);
  
  
  CCD = new CCDStar(GM->Map_Dim_X, GM->Map_Dim_Y);
	std::vector<R_point> goals;
	goals.reserve(100); 
  goals.clear();

// Lets just make sure the laser is on the upright position, otherwise we might see the arm as an obstacle!
    ros::Publisher ptu_d46_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ptu_d46_controller/command", 10);

    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.points.resize(1);
    msg.joint_names.push_back("ptu_d46_pan_joint");
    msg.points[0].positions.push_back(0.0);
    msg.points[0].velocities.push_back(0.8);
    msg.points[0].accelerations.push_back(0.8);
    msg.joint_names.push_back("ptu_d46_tilt_joint");
    msg.points[0].positions.push_back(0.5);
    msg.points[0].velocities.push_back(0.8);
    msg.points[0].accelerations.push_back(0.8);
    msg.points[0].time_from_start = ros::Duration(1.0);


    ptu_d46_pub.publish(msg);
    
  MetalDetector md;


  ros::Rate rate(10.0);



  while (nh.ok()) {


    ros::spinOnce(); 
    M->executeMotion();

//drawing in rviz	
    if (gobackwards){
      visual.robot_shape.color.g = 1.0;
    }else{
      visual.robot_shape.color.g = 0.0;
    }  
			visual.visualizationduringmotion();	
			visualCCD.visualizationduringmotion();	
		if(M->voznja)
		{
		}else{
		  printf("waiting for new goal\n");
		}

	  if (M->zavrsio)
		  break;

//calculate coverage path
    if ((M->ciklus>11) ){
    
    int numnewcells=CCD->setCoverageOnRobotCurrentPosition(M->RB.x, M->RB.y, M->RB.th);
    if (M->slam_loop==0) { 
    if (gobackwards==false){
    if (M->voznja==false){ //ne vozi
#if DO_COVERAGE
      CCD->replanko=1;
      int pathflag=CCD->planCoveragePath();//racuna inic put samo jednom, ostalo replanira
      if (CCD->PathLength<=1){
        std::cerr<<"COVERAGE FINISHED!!! PathLength "<<CCD->PathLength<<std::endl;
        M->zavrsio=true;
        CCD->loger();
        continue;
      }
      if (pathflag==2) { 
        std::cerr<<"no path"<<std::endl;
        continue;
      }
      R_point goal=CCD->getGoal();//odredjuje cilj i putanje u realnim koordinatama za logiranje
      CCD->loger();
      bool skipgoal=false;
      for (uint i = 0; i < goals.size();i++){
        if ((fabs(goals[i].x-goal.x)+fabs(goals[i].y-goal.y)+200.*(1-cos(goals[i].th-goal.th))<100.)){ 
          skipgoal=true;
          std::cerr<<"skipping goal ("<<goal.x<<","<<goal.y<<") close to old goal ("<<goals[i].x<<","<<goals[i].y<<")"<<std::endl;
          break;
        }
      }
      if (skipgoal){ 
        CCD->disableCoverageOfArea(goal,350.); //okolina u milimetrima koju ce oznaciti da ne prekriva na mjestu toola i za getgoal da preskoci
      }else{
        goals.push_back(goal);
        M->gotogoal(goal);
      }
#endif
    }else{//vozi
    
      if ((CCD->checkIfStuck(numnewcells,M->setvel.v*M->metric,M->setvel.w))){
        gobackwards=true;
        startcycle=M->ciklus; 
        M->voznja=false;
        backwardsign=1;//da ide u rikverc
      }else{
        gobackwards=false;
      }

#if 0
      int pathflag=CCD->planCoveragePath();//ne replanira nego se samo mice po starom putu
      if (pathflag==2) continue; 
      R_point goal=CCD->getGoal();//odredjuje cilj i putanje u realnim koordinatama za logiranje
      bool skipgoal=false;
      for (uint i = 0; i < goals.size();i++){
        if ((fabs(goals[i].x-goal.x)+fabs(goals[i].y-goal.y)+200.*(1-cos(goals[i].th-goal.th))<100.)){ 
          skipgoal=true;
          std::cerr<<"skipping goal ("<<goal.x<<","<<goal.y<<") close to old goal ("<<goals[i].x<<","<<goals[i].y<<")"<<std::endl;
          break;
        }
      }
      if (skipgoal){ 
        CCD->disableCoverageOfArea(goal,350.); //okolina u milimetrima koju ce oznaciti da ne prekriva na mjestu toola i za getgoal da preskoci
      }else{
        goals.push_back(goal);
        M->gotogoal(goal);
      }
#endif
    }
    
    }else { //gobackwards==true 
      printf("RIKVERC                       RIKVERC                        RIKVERC\n\n\n");
      if (M->ciklus>startcycle+10){
        gobackwards=false;
        backwardsign*=-1; //alternira predznak, ici ce naprijed ako nije uspio nazad
        M->voznja=false; //true;
      }
      geometry_msgs::Twist vel;
		  vel.linear.x = -0.15*backwardsign;
		  vel.angular.z = 0;//0.75;
		  vel.linear.y = 0;
		  M->vel_pub.publish(vel);

    }    
    
    
    
    }
    
    
  }
    
	  rate.sleep();
  }
  return 0;
}




void VisualizationPublisherCCD::visualizationduringmotion(){

    	R_point *temp_point;
    	int temp_length;

      robotpath.points.clear();
      toolpath.points.clear();
//      mine.points.clear();
//      coilpath.points.clear();
      visitedpath.points.clear();


      int metric=M->metric;
			if(CCD->getPathLength()>0){
				temp_length=CCD->getPathLength();
				temp_point=CCD->GetRealPathRobot();
				geometry_msgs::Point p; 	
				for(int pathLength=0; pathLength<temp_length;pathLength++){
			        	p.x = temp_point[pathLength].x/metric;
    					p.y = temp_point[pathLength].y/metric;
    					p.z = 0;

    					robotpath.points.push_back(p);
				}
			//publish path			
			robotpath_pub.publish(robotpath);
		
			}
			if(CCD->getPathLength()>0){
				temp_length=CCD->getPathLength();
				temp_point=CCD->GetRealPathTool();
				geometry_msgs::Point p; 	
				for(int pathLength=0; pathLength<temp_length;pathLength++){
			        	p.x = temp_point[pathLength].x/metric;
    					p.y = temp_point[pathLength].y/metric;
    					p.z = 0;

    					toolpath.points.push_back(p);
				}
			//publish path			
			toolpath_pub.publish(toolpath);
		
			}

			if(CCD->getPathLength()>0){
			geometry_msgs::Point p; 	
			for (int i=0; i<CCD->MapSizeX; i++){
				for (int j=0; j<CCD->MapSizeY; j++){
					if ((CCD->map[i][j].presao>0)){
					  p.x=(GM->Map_Home.x+i*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size)/metric;
            p.y=(GM->Map_Home.y+j*GM->Map_Cell_Size+0.5*GM->Map_Cell_Size)/metric;
					  visitedpath.points.push_back(p);
					}
				}
			}
			visitedpath_pub.publish(visitedpath);
			}

#if 0
			if(CCD->coilpeak){
				geometry_msgs::Point p; 	
			        	p.x = CCD->tocka.x;
    					p.y = CCD->tocka.y;
    					p.z = 0;

    					coilpath.points.push_back(p);
			        	p.x = CCD->tockaR.x;
    					p.y = CCD->tockaR.y;
    					p.z = 0;

    					coilpath.points.push_back(p);
			//publish path			
			coil_pub.publish(coilpath);
		
			}
			if (0)
			{
				geometry_msgs::Point p; 	
			        	p.x = 3.;
    					p.y = 0;
    					p.z = 0;
    					mine.points.push_back(p);
			        	p.x = 2.;
    					p.y = 0;
    					p.z = 0;
    					mine.points.push_back(p);
			        	p.x = -3.;
    					p.y = 0;
    					p.z = 0;
    					mine.points.push_back(p);
			        	p.x = -2.;
    					p.y = 0;
    					p.z = 0;
    					mine.points.push_back(p);
			        	p.x = 0;
    					p.y = -3.;
    					p.z = 0;
    					mine.points.push_back(p);
			        	p.x = 0;
    					p.y = 2.;
    					p.z = 0;
    					mine.points.push_back(p);
			//publish path			
				mine_pub.publish(mine);
			}
#endif
}
