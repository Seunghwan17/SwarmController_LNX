#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_SwarmController.h"

class SwarmController : public QMainWindow
{
	Q_OBJECT

public:
	SwarmController(QWidget *parent = Q_NULLPTR);

private:
	Ui::SwarmControllerClass ui;
};
