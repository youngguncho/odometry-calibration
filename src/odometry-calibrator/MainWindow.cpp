#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "odomproblem.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    OdomProblem odom_problem(0.65, 0.65, 1.67);
    odom_problem.LoadFile("calib_data.txt");
    odom_problem.PrepareObservations();
}

MainWindow::~MainWindow()
{
    delete ui;
}
