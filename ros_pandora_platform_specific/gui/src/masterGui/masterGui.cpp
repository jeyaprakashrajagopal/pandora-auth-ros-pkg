/**
  * masterGui implementation files
  * Author: Dimitrios Vitsios
  * Date: 9 March 2011
  * Change History: - 
  */

#include "masterGui.h"

//global gui variable
pandoraGui *gui;

masterGui::masterGui()
{
	gui = new pandoraGui;
        gui->show();     
}
