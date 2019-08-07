#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <carriage_control/carriageAction.h>
#include <boost/thread.hpp>

// Here is a simple function for spinning a thread that will be used later in the code. This thread will spin the ros node in the background.
void spinThread()
{
  ros::spin();
}

int menu();
void steering_mode();
void up_and_down_mode();
void maneuvre_mode();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client_node");

      // create the action client
    actionlib::SimpleActionClient<carriage_control::carriageAction> ac("action_server"); 

      // Here the thread is created and the ros node is started spinning in the background. By using this method you can create multiple threads for your action client if needed.
    boost::thread spin_thread(&spinThread);
    
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");

    // send a goal to the action
    carriage_control::carriageGoal goal;

    bool exit = false;
    for(;;)
    {
      int choise = menu();
      switch(choise)
      {
        case (1):{
          
          bool exit_1 = false;
          
          for(;;)
          {
            char var1;
            steering_mode();
            std::cin >> var1;
            switch (var1)
            {
            case 's':{
              goal.x_cell = 0;
              goal.y_cell = 0;
              ac.sendGoal(goal);

              //wait for the action to return
                bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

                if (finished_before_timeout)
                {
                  actionlib::SimpleClientGoalState state = ac.getState();
                  ROS_INFO("Action finished: %s",state.toString().c_str());
                }
                else
                  ROS_INFO("Action did not finish before the time out.");
                //wait for the action to return

              break;}
            case 'c':{
              break;}
            case 'q':{
              exit_1 = true;
              break;}
            default:{
              int Xcell, Ycell;
              std::cin >> Xcell; std::cout <<"\t"; std::cin >> Ycell; std::cout <<"\n";
                    // checking user's foolish joke
                    if((Xcell > 200) || (Ycell > 200) || (Xcell <0 ) || (Ycell < 0))
                    {
                      std::cout << "Please, enter correct coordinates! \n";
                    }
                    else
                    {
                      goal.x_cell = Xcell;
                      goal.y_cell = Ycell;
                      ac.sendGoal(goal);

                      //wait for the action to return
                      bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

                      if (finished_before_timeout)
                      {
                        actionlib::SimpleClientGoalState state = ac.getState();
                        ROS_INFO("Action finished: %s",state.toString().c_str());
                      }
                      else
                        ROS_INFO("Action did not finish before the time out.");
                      //wait for the action to return
                    }  
                break;}
              } // end of switch_1 block
            
            if(exit_1)
              break;
          }
                  
          break;}
        case (2):{
          bool exit_2 = false;
          
          for(;;)
          {
            char var2;
            up_and_down_mode();
            std::cin >> var2;
            switch (var2)
            {
              case 'u':{
                goal.demo_dropdown_wheels = true;
                ac.sendGoal(goal);

                //wait for the action to return
                bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

                if (finished_before_timeout)
                {
                  actionlib::SimpleClientGoalState state = ac.getState();
                  ROS_INFO("Action finished: %s",state.toString().c_str());
                }
                else
                  ROS_INFO("Action did not finish before the time out.");
                //wait for the action to return
                break;}
              case 'l':{
                goal.demo_dropdown_wheels = false;
                ac.sendGoal(goal);

                //wait for the action to return
                bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

                if (finished_before_timeout)
                {
                  actionlib::SimpleClientGoalState state = ac.getState();
                  ROS_INFO("Action finished: %s",state.toString().c_str());
                }
                else
                  ROS_INFO("Action did not finish before the time out.");
                //wait for the action to return
                break;}
              case 'c':{
                break;}
              case 'q':{
                exit_2 = true;
                break;}
              default:{
                std::cout << "Please, select again!\n";
                break;}
            } // end of switch_2 block

            if(exit_2)
              break;
          }
          break;}
        case (3):{
          
          bool exit_3 = false;
          for(;;)
          {
            char var3;
            maneuvre_mode();
            std::cin >> var3;
            switch (var3)
            {
              case 'l':{
                goal.demo_ride_circle = true;
                ac.sendGoal(goal);

                //wait for the action to return
                bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

                if (finished_before_timeout)
                {
                  actionlib::SimpleClientGoalState state = ac.getState();
                  ROS_INFO("Action finished: %s",state.toString().c_str());
                }
                else
                  ROS_INFO("Action did not finish before the time out.");
                //wait for the action to return
                break;}
              case 'c':{
                break;}
              case 'q':{
                exit_3 = true;
                break;}
                default:{
                   std::cout << "Please, select again!\n";
                break;
                }
            } // end of switch_3 block

            if(exit_3)
            break;

          }
          break;}
        case (4):{
          exit = true;
          break;}
        default:{
          std::cout << "Please, select again!\n";
          break;}
      } // end of switch block
      
      if(exit)
        break;
    } // loop for infinity time

    
  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;

}

// making main menu
int menu()
{
  int choise;

  std::cout << " **** Menu ****\n\n";
  std::cout << " (1) Steering mode \n";
  std::cout << " (2) Up & Down mode \n";
  std::cout << " (3) Maneuvre mode \n";
  std::cout << " (4) Quit \n\n";
  std::cout << ": ";
  std::cin >> choise;
  return choise;
}

void steering_mode()
{
  std::cout << "**** Steering mode ****";
  std::cout << "Please, enter cell's coordinate \n";
  std::cout << "If you want to go to the start of the field, press 's' \n";
  std::cout << "If you want to cancel operation, press 'c' \n";
  std::cout << "To main menu, press 'q' \n";
  std::cout << ": ";
}

void up_and_down_mode()
{
  std::cout << "**** Up&Down mode ****";
  std::cout << "If you want to lift the wheels, press 'u' \n";
  std::cout << "If you want to lower the wheels, press 'l' \n";
  std::cout << "If you want to cancel operation, press 'c' \n";
  std::cout << "To main menu, press 'q' \n";
  std::cout << ": ";
}

void maneuvre_mode()
{
  std::cout << "**** Maneuvre mode ****";
  std::cout << "If you want to do a lap around the field, press 'l' \n";
  std::cout << "If you want to cancel operation, press 'c' \n";
  std::cout << "To main menu, press 'q' \n";
  std::cout << ": ";
}
