/**
  * padoragui header file
  * Author: Dimitrios Vitsios
  * Date:  9 March 2011
  * Change History: - 
  */

#ifndef PANDORAGUI_H
#define PANDORAGUI_H

#include <QtGui>
#include "ui_pandoragui.h"

class pandoraGui : public QWidget
{
    Q_OBJECT

public:
	pandoraGui(QWidget *parent = 0);
	
	Ui::pandoraGui ui;	
};


#endif
