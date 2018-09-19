/*********************************************************************
* Author: Adrián Jesús Peña Rodríguez
*********************************************************************/
#include "../include/my_astar_planner/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>

//para pintar puntos
#include <visualization_msgs/Marker.h>

// para debugging
#include <sstream>
#include <string>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner {

  unordered_map<unsigned int, coupleOfCells>::iterator findLower(unordered_map<unsigned int, coupleOfCells> & abiertos);

  MyastarPlanner::MyastarPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);

      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal",1);
      marker_Open_publisher = private_nh.advertise<visualization_msgs::Marker>("open_list", 1000);
      marker_Closed_publisher = private_nh.advertise<visualization_msgs::Marker>("closed_list", 1000);
      marker_Goals_publisher = private_nh.advertise<visualization_msgs::Marker>("goals_markers", 1000);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  // Con esto calculamos el radio máximo de nuestro robot para posteriormente usarlo para saber si podemos o no pasar por un grupo de celdas
  double MyastarPlanner::footprintCost(){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint(); // Me devuelve los puntos del robot
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return 0.0;

    double distancia = 0;
    double radio = 0;

    for(int i = 0; i < footprint.size(); i++) {
      distancia = sqrt((footprint[i].y - footprint[i].x) * (footprint[i].y - footprint[i].x));
      if(distancia > radio)
        radio = distancia;
    }

    return radio;
  }


  bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {

    clock_t tStart = clock();


    //***********************************************************
    // Inicio de gestion de ROS
    //***********************************************************
    if(!initialized_){
      ROS_ERROR("The astar planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    cerrados.clear();
    abiertos.clear();
    cerradosF.clear();
    abiertosF.clear();

    //obtenemos el costmap global  que está publicado por move_base.
    costmap_ = costmap_ros_->getCostmap();

    //obtenemos el radio del circulo que engloba a nuestro robot
    radio = footprintCost();

    n_vueltas = 0;

    //Obligamos a que el marco de coordenadas del goal enviado y del costmap sea el mismo.
    //esto es importante para evitar errores de transformaciones de coordenadas.
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    //obtenemos la orientación start y goal en start_yaw y goal_yaw.
    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

    /**************************************************************************/
    /*************** HASTA AQUÍ GESTIÓN DE ROS *********************************/
    /****************************************************************************/

    //pasamos el goal y start a estructura coupleOfCells
    coupleOfCells cpstart, cpgoal;
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    unsigned int mgoal_x, mgoal_y;
    costmap_->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);
    cpgoal.index = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);
    cpgoal.parent=0;
    cpgoal.gCost=0;
    cpgoal.hCost=0;
    cpgoal.fCost=0;

    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    unsigned int mstart_x, mstart_y;
    costmap_->worldToMap(start_x,start_y, mstart_x, mstart_y);
    cpstart.index = MyastarPlanner::costmap_->getIndex(mstart_x, mstart_y);
    cpstart.parent =cpstart.index;
    cpstart.gCost = 0;
    cpstart.hCost = MyastarPlanner::calculateHCost(cpstart.index,cpgoal.index);
    cpstart.fCost = cpstart.gCost + cpstart.hCost;

    //insertamos la casilla inicial en abiertos
    //MyastarPlanner::openList.push_back(cpstart);
    abiertos[cpstart.index] = cpstart;
    abiertosF[cpgoal.index] = cpgoal;

    //ROS_INFO("Inserto en Abiertos: %d", cpstart.index );
    //ROS_INFO("Index del goal: %d", cpgoal.index );

    /**************************************************************************/
    /*************** GESTIÓN VISUALIZACIÓN PUNTOS DE ABIERTOS Y CERRADOS********/
    /****************************************************************************/

    //visualization_msgs::Marker points;// definida en la clase como markers_OpenList
    inicializaMarkersPoints(markers_OpenList,"openList", 0,0.0f,1.0f,0.0f);
    inicializaMarkersPoints(markers_ClosedList,"closedList", 1,1.0f,0.0f,0.0f);
    inicializaMarkersLine_List(markers_Goals, "goals", 2, 0.0f, 0.0f,1.0f);

    limpiaMarkers(marker_Open_publisher, markers_ClosedList);
    limpiaMarkers(marker_Closed_publisher, markers_OpenList);

    /**************************************************************************/
    /*************** FIN GESTIÓN VISUALIZACIÓN PUNTOS DE ABIERTOS Y CERRADOS********/
    /****************************************************************************/

    //visualizamos start.
    visualizaCelda(marker_Open_publisher, markers_OpenList, cpstart.index);
    visualizaCelda(marker_Open_publisher, markers_OpenList, cpgoal.index);

    unsigned int currentIndex = cpstart.index;
    unsigned int currentIndexF = cpgoal.index;

    while (!MyastarPlanner::abiertos.empty() && !MyastarPlanner::abiertosF.empty()) { //while the open list is not empty continuie the search
      //escoger UNA casilla DE abiertos
      unordered_map<unsigned int, coupleOfCells>::iterator actual = findLower(abiertos);
      unordered_map<unsigned int, coupleOfCells>::iterator actualF = findLower(abiertosF);
      coupleOfCells COfCells = actual->second;
      coupleOfCells COfCellsF = actualF->second;
      abiertos.erase(actual);
      abiertosF.erase(actualF);
      currentIndex = COfCells.index;
      currentIndexF = COfCellsF.index;

      //y la insertamos en cerrados
      MyastarPlanner::cerrados[COfCells.index] = COfCells;
      MyastarPlanner::cerradosF[COfCellsF.index] = COfCellsF;

      visualizaCelda(marker_Closed_publisher, markers_ClosedList, COfCells.index);
      visualizaCelda(marker_Closed_publisher, markers_ClosedList, COfCellsF.index);

      // if the currentCell is the goalCell: success: path found o si se han unido las dos búsquedas
      if(unidos != 0 || currentIndex==cpgoal.index || currentIndexF == cpstart.index) {
        //el plan lo construimos partiendo del goal, del parent del goal y saltando en cerrados "de parent en parent"
        //vamos insertando al final los waypoints (los nodos de cerrados), por tanto, cuando finaliza el bucle hay que darle la vuelta al plan
        ROS_INFO("Se han explorado %u nodos y cerrados tiene %u nodos", n_vueltas, (unsigned int)cerrados.size());
        //ros::Duration(10).sleep();
        //convertimos goal a poseStamped nueva
        geometry_msgs::PoseStamped pose;

        //coupleOfCells currentCouple = cerrados[unidos];
        unsigned int currentParent = unidos;
        //ROS_INFO("Inserta en Plan GOAL: %f, %f PADRE: %u", pose.pose.position.x, pose.pose.position.y, currentParent);
        //ros::Duration(1).sleep();

        while (currentParent != cpstart.index) { //e.d. mientras no lleguemos al nodo start
          //hacemos esa posición que sea el currentCouple
          coupleOfCells currentCouple = cerrados[currentParent];

          //creamos una PoseStamped con la informaciuón de currentCouple.index

          //primero hay que convertir el currentCouple.index a world coordinates
          unsigned int mpose_x, mpose_y;
          double wpose_x, wpose_y;

          costmap_->indexToCells(currentCouple.index, mpose_x, mpose_y);
          costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);

          //ROS_INFO("Las coordenadas de El PADRE de %u son (%u, %u) -> (%f, %f). Y su PADRE es %u.", currentParent, mpose_x,mpose_y,wpose_x, wpose_y, currentCouple.parent);
          //ros::Duration(1).sleep();

          //después creamos la pose
          geometry_msgs::PoseStamped pose;
          pose.header.stamp =  ros::Time::now();
          pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
          pose.pose.position.x = wpose_x;
          pose.pose.position.y = wpose_y;
          pose.pose.position.z = 0.0;
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;
          //insertamos la pose en el plan
          plan.push_back(pose);
          //ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);
          //hacemos que currentParent sea el parent de currentCouple
          currentParent = currentCouple.parent;
        }

        std::reverse(plan.begin(),plan.end());
      
        ROS_INFO("Se han explorado %u nodos y cerradosF tiene %u nodos", n_vueltas, (unsigned int)cerradosF.size());
        //ros::Duration(10).sleep();

        //coupleOfCells currentCoupleF = cerradosF[unidos];
        unsigned int currentParentF = unidos;
        //ROS_INFO("Inserta en Plan GOAL: %f, %f PADRE: %u", pose.pose.position.x, pose.pose.position.y, currentParentF);
        //ros::Duration(1).sleep();

        while (currentParentF != cpgoal.index) { //e.d. mientras no lleguemos al nodo start
          //hacemos esa posición que sea el currentCouple
          coupleOfCells currentCoupleF = cerradosF[currentParentF];

          //creamos una PoseStamped con la informaciuón de currentCouple.index

          //primero hay que convertir el currentCouple.index a world coordinates
          unsigned int mpose_x, mpose_y;
          double wpose_x, wpose_y;

          costmap_->indexToCells(currentCoupleF.index, mpose_x, mpose_y);
          costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);

          //ROS_INFO("Las coordenadas de El PADRE de %u son (%u, %u) -> (%f, %f). Y su PADRE es %u.", currentParentF, mpose_x,mpose_y,wpose_x, wpose_y, currentCoupleF.parent);
          //ros::Duration(1).sleep();

                  //después creamos la pose
          geometry_msgs::PoseStamped pose;
          pose.header.stamp =  ros::Time::now();
          pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
          pose.pose.position.x = wpose_x;
          pose.pose.position.y = wpose_y;
          pose.pose.position.z = 0.0;
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;
          //insertamos la pose en el plan
          plan.push_back(pose);
          //ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);
          //hacemos que currentParent sea el parent de currentCouple
          currentParentF = currentCoupleF.parent;
        }

        //ROS_INFO("Sale del bucle de generación del plan.");

        printf("\n\n***********************************************************************");
        printf("\nTime taken: %.2fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
        printf("\nLongitud del camino: %lu", plan.size());
        printf("\nNodos en abiertos: %lu", abiertos.size());
        printf("\nNodos en abiertosF: %lu", abiertosF.size());
        printf("\nNodos en cerrados: %lu", cerrados.size());
        printf("\nNodos en cerradosF: %lu", cerradosF.size());
        printf("\n***********************************************************************");

        unidos = 0;
        return true;
      }

      //search the neighbors of the current Cell
      vector <unsigned int> neighborCells=findFreeNeighborCell(currentIndex);
      vector <unsigned int> neighborCellsF=findFreeNeighborCell(currentIndexF);
      //ROS_INFO("Ha encontrado %u vecinos", (unsigned int)neighborCells.size());

      //neighbors that exist in the closedList are ignored
      vector <unsigned int> neighborNotInClosedList;
      vector <unsigned int> neighborNotInClosedListF;
      for(uint i=0; i<neighborCells.size(); i++)
      {
        if(cerrados.find(neighborCells[i]) == cerrados.end())
        {
          neighborNotInClosedList.push_back(neighborCells[i]);
        }
      }
      for(uint i=0; i<neighborCellsF.size(); i++)
      {
        if(cerradosF.find(neighborCellsF[i]) == cerradosF.end())
        {
          neighborNotInClosedListF.push_back(neighborCellsF[i]);
        }
      }          
      //ROS_INFO("Ha encontrado %u vecinos que no están en cerrados", (unsigned int)neighborNotInClosedList.size());


      //search the neighbors that already exist in the open List
      vector <unsigned int> neighborsInOpenList;
      vector <unsigned int> neighborsInOpenListF;
      vector <unsigned int> neighborsNotInOpenList;
      vector <unsigned int> neighborsNotInOpenListF;
      
      for(uint i=0; i<neighborNotInClosedList.size(); i++)
      {
        if(abiertos.find(neighborNotInClosedList[i]) != abiertos.end()) {
          neighborsInOpenList.push_back(neighborNotInClosedList[i]);
        }
        else
          neighborsNotInOpenList.push_back(neighborNotInClosedList[i]);
      }
      for(uint i=0; i<neighborNotInClosedListF.size(); i++)
      {
        if(abiertosF.find(neighborNotInClosedListF[i]) != abiertosF.end()) {
          neighborsInOpenListF.push_back(neighborNotInClosedListF[i]);
        }
        else
          neighborsNotInOpenListF.push_back(neighborNotInClosedListF[i]);
      }

      //add the neighbors that are not in the open list to the open list and mark the current cell as their parent

      unidos = 0;

      addNeighborCellsToOpenList(MyastarPlanner::abiertos, cerrados, neighborsNotInOpenList, currentIndex, COfCells.gCost, cpgoal.index); //,tBreak);
      tratarNodoAbierto(MyastarPlanner::abiertos, cerrados, neighborsInOpenList, currentIndex, COfCells.gCost, cpgoal.index); 
      addNeighborCellsToOpenList(MyastarPlanner::abiertosF, cerradosF, neighborsNotInOpenListF, currentIndexF, COfCellsF.gCost, cpstart.index); //,tBreak);
      tratarNodoAbierto(MyastarPlanner::abiertosF, cerradosF, neighborsInOpenListF, currentIndexF, COfCellsF.gCost, cpstart.index); 
      //ROS_INFO("%f", COfCells.gCost);
      //ROS_INFO("%f", COfCells.hCost);
      n_vueltas++;

      //PINTO ABIERTOS
      //Anyadir neighborCells a points. pushback()
      visualizaLista(marker_Open_publisher, markers_OpenList, neighborsNotInOpenList);
      visualizaLista(marker_Open_publisher, markers_OpenList, neighborsNotInOpenListF);
      visualizaCelda(marker_Closed_publisher,markers_ClosedList, COfCells.index);
      visualizaCelda(marker_Closed_publisher,markers_ClosedList, COfCellsF.index);

      //Para los nodos que ya están en abiertos, comprobar en cerrados su coste y actualizarlo si fuera necesario
    }

    if(MyastarPlanner::abiertos.empty()) {  // if the openList is empty: then failure to find a path
      ROS_INFO("Failure to find a path !");
      return false;
    }
  };


  //calculamos H como la distancia euclídea hasta el goal
  double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal) {
    unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y, multiplicador = 1;
    double wstart_x, wstart_y, wgoal_x, wgoal_y;
    unsigned int seguridad;

    //trasformamos el indice de celdas a coordenadas del mundo.
    //ver http://docs.ros.org/indigo/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

    costmap_->indexToCells(start, mstart_x, mstart_y);
    costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
    costmap_->indexToCells(goal, mgoal_x, mgoal_y);
    costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

    seguridad = costmap_->getCost(mstart_x, mstart_y);

    // Dynamic weighting
    if(n_vueltas < 100)
      multiplicador = 1;
    else if(n_vueltas < 1000)
      multiplicador = 5;
    else
      multiplicador = 10;
    ////////////////////

    //Usa la distancia Euclidea como h
    return (multiplicador * (seguridad + (sqrt((pow(wstart_x - wgoal_x, 2)) + pow(wstart_y - wgoal_y, 2)))));

    //Manhattan
    //return (multiplicador * (seguridad*10 + (abs(wstart_x - wgoal_x) + abs(wstart_y - wgoal_y))));

    // Diagonal distance -> Chebyshev distance
    /*double dx = abs(wstart_x - wgoal_x);
    double dy = abs(wstart_y - wgoal_y);
    int D = 1;
    int D2 = 1;
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);*/
    
    // Diagonal distance -> Octile distance
    /*double dx = abs(wstart_x - wgoal_x);
    double dy = abs(wstart_y - wgoal_y);
    int D = 1;
    int D2 = 2;
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);*/
  }

  double MyastarPlanner::calculateGCost(unsigned int celda, unsigned int parent) {
    unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y, multiplicador = 1;
    double wstart_x, wstart_y, wgoal_x, wgoal_y;

    costmap_->indexToCells(celda, mstart_x, mstart_y);
    costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
    costmap_->indexToCells(parent, mgoal_x, mgoal_y);
    costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

    // Dynamic weighting
    if(n_vueltas < 100)
      multiplicador = 10;
    else if(n_vueltas < 1000)
      multiplicador = 5;
    else
      multiplicador = 8;
    ////////////////////

    double devolver = multiplicador * sqrt((pow(wstart_x - wgoal_x, 2)) + pow(wstart_y - wgoal_y, 2));

    return devolver;
  }

  /*******************************************************************************
   * Function Name: findFreeNeighborCell
    * Inputs: the row and columun of the current Cell
    * Output: a vector of free neighbor cells of the current cell
    * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
    * Check Status: Checked by Anis, Imen and Sahar
  *********************************************************************************/
  vector <unsigned int> MyastarPlanner::findFreeNeighborCell (unsigned int CellID){
    unsigned int mx, my, copiamx, copiamy;
    double  wx, wy, copiawx, copiawy, sumatoria;
    costmap_->indexToCells(CellID,mx,my);
    //ROS_INFO("Viendo vecinos de index: %u, Map coords: (%u,%u)", CellID, mx,my);
    unsigned int grados = 360;
    unsigned int incremento = 30;
    unsigned int division = grados/incremento;

    vector <unsigned int>  freeNeighborCells;

    for (int x=-3;x<=3;x++)
      for (int y=-3; y<=3;y++){
        //check whether the index is valid
        //ROS_INFO("A ver: X = %u, Size_X = %u, Y = %u Size_Y = %u",mx+x, (unsigned int)costmap_->getSizeInCellsX(),my+y, (unsigned int)costmap_->getSizeInCellsY());
        if ((mx+x>=0)&&(mx+x < costmap_->getSizeInCellsX())&&(my+y >=0 )&&(my+y < costmap_->getSizeInCellsY())){
          // Esto es lo añadido, solo se consideran vecinos aquellos nodos que tienen una evaluación de coste 0, sin nada de riesgo
          costmap_->mapToWorld( (unsigned int) mx+x, (unsigned int) my+y, wx, wy);

          for(int i = 0; i < grados; i+=incremento) {
            copiawx = wx;
            copiawy = wy;
            copiamx = mx;
            copiamy = my;
            copiawx = wx + radio*cos(i*M_PI/180);
            copiawy = wy + radio*sin(i*M_PI/180);
            costmap_->worldToMap( copiawx, copiawy, copiamx, copiamy);
            sumatoria += costmap_->getCost(copiamx,copiamy);
          }

          sumatoria /= division;

          //ROS_INFO("Comprobando casilla con Map coords(%u,%u), World coords (%f,%f)", mx+x, my+y ,wx,wy);
          if(sumatoria < 220 && (!(x==0 && y==0)) /*&& costmap_->getCost(mx+x,my+y) < 160*/){
            unsigned int index = costmap_->getIndex(mx+x,my+y);

            freeNeighborCells.push_back(index);
          }
      }
    }
    
    return  freeNeighborCells;
  }

  unordered_map<unsigned int, coupleOfCells>::iterator findLower(unordered_map<unsigned int, coupleOfCells> & abiertos) {

    vector<pair<unsigned int, coupleOfCells> > vec;

    copy(abiertos.begin(), abiertos.end(), back_inserter<vector<pair<unsigned int, coupleOfCells> > >(vec));

    sort(vec.begin(), vec.end(), [](const pair<unsigned int, coupleOfCells>& l, const pair<unsigned int, coupleOfCells>& r) {
      return l.second.fCost < r.second.fCost;
    });

    // Beam search
    if(abiertos.size() > 180) {
      for(int i = 0; i < 100; i++) {
        abiertos.erase(abiertos.find((vec.end()-1)->second.index));
        vec.erase(vec.end()-1);
      }
    }
    ///////////////////
    
    return abiertos.find(vec.begin()->second.index);
  }

  /*******************************************************************************/
  //Function Name: addNeighborCellsToOpenList
  //Inputs: the open list, the neighbors Cells and the parent Cell
  //Output:
  //Description: it is used to add the neighbor Cells to the open list
  /*********************************************************************************/
  void MyastarPlanner::addNeighborCellsToOpenList(unordered_map<unsigned int, coupleOfCells> & OPL, unordered_map<unsigned int, coupleOfCells> & closed, vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell) {
    for(uint i=0; i< neighborCells.size(); i++) {
      coupleOfCells CP;
      CP.index=neighborCells[i]; //insert the neighbor cell
      CP.parent = parent; //insert the parent cell
      CP.gCost = gCostParent + calculateGCost(CP.index, parent);  // Distancia desde la nueva celda hasta el padre, esa es nuestra g
      CP.hCost = calculateHCost(CP.index, goalCell);
      CP.fCost = CP.gCost + CP.hCost;

      if(cerradosF.find(CP.index) != cerradosF.end()) {
        unidos = CP.index;
        cerrados[CP.index] = CP;
      }
      else if(cerrados.find(CP.index) != cerrados.end()) {
        unidos = CP.index;
        cerradosF[CP.index] = CP;
      }
      else
        OPL[CP.index] = CP;
    }
  }

  void MyastarPlanner::tratarNodoAbierto(unordered_map<unsigned int, coupleOfCells> & OPL, unordered_map<unsigned int, coupleOfCells> & closed, vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell) {
    for(uint i=0; i< neighborCells.size(); i++) {
      coupleOfCells CP;
      CP.index=neighborCells[i]; //insert the neighbor cell
      CP.parent = parent; //insert the parent cell
      CP.gCost = gCostParent + calculateGCost(CP.index, parent);  // Distancia desde la nueva celda hasta el padre, esa es nuestra g
      CP.hCost = calculateHCost(CP.index, goalCell);
      CP.fCost = CP.gCost + CP.hCost;

      if(cerradosF.find(CP.index) != cerradosF.end()) {
        unidos = CP.index;
        cerrados[CP.index] = CP;
      }
      else if(cerrados.find(CP.index) != cerrados.end()) {
        unidos = CP.index;
        cerradosF[CP.index] = CP;
      }
      else
        if(OPL[CP.index].fCost > CP.gCost)
          OPL[CP.index] = CP;
    }
  }


    /********VISUALIZAR ESPACIO DE BUSQUEDA *************************/

  void MyastarPlanner::inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
    marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
    marker.header.stamp =  ros::Time::now();
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
    marker.id = id;
    marker.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    marker.scale.x = costmap_->getResolution();
    marker.scale.y = costmap_->getResolution();

    // Points are green
    marker.color.g = g;
    marker.color.r = r;
    marker.color.b = b;
    marker.color.a = 1.0;
  }

  void MyastarPlanner::inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
    marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
    marker.header.stamp =  ros::Time::now();
    marker.ns = ns;

    marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
    marker.pose.orientation.w = 0.0;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;

    marker.id = id;

    marker.type = visualization_msgs::Marker::SPHERE;

    //Line lists also have some special handling for scale: only scale.x is used and it controls the width of the line segments.
    marker.scale.x = marker.scale.y = 0.5;
    // marker.scale.y = costmap_->getResolution();

    // Points are green
    marker.color.g = g;
    marker.color.r = r;
    marker.color.b = b;
    marker.color.a = 1.0;
  }

  void MyastarPlanner::visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y) {
    //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0; //¿?

    marker.points.push_back(p); //anyado el punto inicial
    where.publish(marker); //lo publico
  }

  void MyastarPlanner::visualizaCoordsLineUp(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y, double z) {
    //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    where.publish(marker); //lo publico
    //points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
  }

  void MyastarPlanner::visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int index) {
    unsigned int mpose_x, mpose_y;
    double wpose_x, wpose_y;
    costmap_->indexToCells(index, mpose_x, mpose_y);
    costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
    visualizaCoords(where, marker, wpose_x, wpose_y);
  }

  void MyastarPlanner::visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, vector<unsigned int> lista) {
    for(vector<unsigned int>::iterator i = lista.begin(); i != lista.end(); ++i) {
      unsigned int mpose_x, mpose_y;
      double wpose_x, wpose_y;
      costmap_->indexToCells(*i, mpose_x, mpose_y);
      costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
      //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
      geometry_msgs::Point p;
      p.x = wpose_x;
      p.y = wpose_y;
      p.z = 0; //¿?

      marker.points.push_back(p);
    }
    where.publish(marker);
  }

  void MyastarPlanner::limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker) {
    if (!marker.points.empty()) {
      marker.action = visualization_msgs::Marker::DELETE;
      where.publish(marker);
      marker.action = visualization_msgs::Marker::ADD;
    }
    marker.points.clear();
  }
}
