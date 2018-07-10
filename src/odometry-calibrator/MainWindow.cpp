#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "odomproblem.h"
#include <ctime>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    double dia_l = 0.65;
    double dia_r = 0.60;
    double w_b = 1.8;

    OdomProblem odom_problem(dia_l, dia_r, w_b);
    odom_problem.LoadFile("calib_data.txt");
    odom_problem.PrepareObservations();

    ceres::Problem problem;
    for (int i=0; i < odom_problem.num_observations()/20; ++i) {

        double obs[5];
        double noise = ((double) rand() / (RAND_MAX)) * 2;
        double noise2 = ((double) rand() / (RAND_MAX)) * 4; 
        double noise3 = ((double) rand() / (RAND_MAX)) * 3;  
        cout << "NOISE " << noise << endl;
        cout << "NOISE2 " << noise2 << endl;
        cout << "NOISE3 " << noise3 << endl;
        obs[0] = odom_problem.get_observation_gps(i,0)+noise;
        obs[1] = odom_problem.get_observation_gps(i,1)+noise2;
        obs[2] = odom_problem.get_observation_gps(i,2)+noise3;
        obs[3] = odom_problem.get_observation_enc(i,0);
        obs[4] = odom_problem.get_observation_enc(i,1);

        ceres::CostFunction* cost_function = OdometryGPSError::Create(obs[0], obs[1], obs[2], obs[3], obs[4]);
        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 &dia_l, &dia_r, &w_b);

    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;

    cout << "Final value" << dia_l
         << " / " << dia_r
         << " / " << w_b << endl;

}

MainWindow::~MainWindow()
{
    delete ui;
}
