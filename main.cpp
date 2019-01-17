#include <iostream>
#include <string>
#include <vector>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <sstream>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "g2o/solvers/dense/linear_solver_dense.h"
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

#include "data.h"
#include "edge_se3_priorxyz.h"
#include "edge_se3_distxyz.h"
#include "edge_se3_priorxy.h"
#include "edge_se3_distxy.h"
#include "targetTypes3D.h"
#include "Find_RT.h"

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
// #include <Geometry/Umeyama.h>

using namespace std;
using namespace g2o;
using namespace Eigen;

int startoffset = 5;
int endoffset = 5;

void trajectory(Eigen::Vector2d GPSTime, vector<shared_ptr<gpsData>> &gps_all, vector<shared_ptr<orbData>> orb_all, Eigen::Matrix4d GPStrans)
{

    g2o::SparseOptimizer optimizer;

    BlockSolverX::LinearSolverType *linearSolver;

    //linearSolver = new g2o::LinearSolverEigen<BlockSolverX::PoseMatrixType>();
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    BlockSolverX *solver_ptr = new BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    //g2o::OptimizationAlgorithmDogleg *solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);

    optimizer.setAlgorithm(solver);
    //optimizer.setVerbose(true);

    Eigen::Matrix3d information_mat = Eigen::Matrix3d::Identity();
    information_mat(0, 0) /= 1;
    information_mat(1, 1) /= 1;
    information_mat(2, 2) /= 1;

    int startCount = GPSTime[0];

    int endCount = GPSTime[1] ;
    int id = 0;
    for (int i = startCount; i < endCount; i++)
    {

        /**********************************************************************************/
        Eigen::Vector3d trans(0, 0, 0);
        VertexPosition3D *vSE3 = new VertexPosition3D();
        vSE3->setEstimate(trans);
        vSE3->setId(id);
        // vSE3->setFixed(i == 0);
        vSE3->setMarginalized(false);
        optimizer.addVertex(vSE3);

        /*************************************************************************/

        Eigen::Vector3d ORBPoint(orb_all[i]->x, orb_all[i]->y, orb_all[i]->z);
        Matrix4d T = GPStrans;
        ORBPoint = T.topLeftCorner(3, 3) * ORBPoint + T.col(3).head(3);
        GPSObservationPosition3DEdge *ORB_edge = new GPSObservationPosition3DEdge();
        ORB_edge->setMeasurement(ORBPoint);
        ORB_edge->setInformation(information_mat / 10);

        double error = T.col(3)[3];
        g2o::RobustKernelHuber *rk_ = new g2o::RobustKernelHuber;
        ORB_edge->setRobustKernel(rk_);
        rk_->setDelta(error);
        ORB_edge->setVertex(0, vSE3);
        optimizer.addEdge(ORB_edge);

        if (gps_all[i]->state == "1")
        {

            Eigen::Vector3d gpsUTM(gps_all[i]->x, gps_all[i]->y, gps_all[i]->z);
            GPSObservationPosition3DEdge *gps_edge = new GPSObservationPosition3DEdge();
            gps_edge->setMeasurement(gpsUTM);
            gps_edge->setInformation(information_mat / 5);
            gps_edge->setVertex(0, vSE3);
            optimizer.addEdge(gps_edge);
        }

        if (gps_all[i]->state == "2")
        {

            Eigen::Vector3d gpsUTM(gps_all[i]->x, gps_all[i]->y, gps_all[i]->z);
            GPSObservationPosition3DEdge *gps_edge = new GPSObservationPosition3DEdge();
            gps_edge->setMeasurement(gpsUTM);
            gps_edge->setInformation(information_mat / 2);
            gps_edge->setVertex(0, vSE3);
            optimizer.addEdge(gps_edge);
        }

        id++;
    }
    optimizer.initializeOptimization(0);
    optimizer.optimize(100);

   // ofstream outFile;
   // outFile.open("./orb.log");

    for (int i = 0; i < id; i++)
    {
        VertexPosition3D *vertex = dynamic_cast<VertexPosition3D *>(optimizer.vertex(i));
        if (!vertex)
            throw std::bad_cast();

        Eigen::Vector3d out = vertex->estimate();

        gps_all[i + startCount]->x = out[0];
        gps_all[i + startCount]->y = out[1];
        gps_all[i + startCount]->z = out[2];

       // outFile.precision(6);
       // outFile.setf(ios::fixed, ios::floatfield);
       // outFile << gps_all[i + startCount]->timestamp << " " << out[0] << " " << out[1] << " " << out[2] << " " << std::endl;
    }
   // outFile.close();
}

vector<Eigen::Vector2d> findNoGPS(vector<shared_ptr<gpsData>> gps_all)
{

    int noGPSstart = 0, noGPSendl = 0;
    Eigen::Vector2d GPSTime{0, 0};
    vector<Eigen::Vector2d> GPSLost;
    for (int i = 1; i < gps_all.size() - 1; i++)
    {

        //if (gps_all[0]->state == "1")
        {

            if ((gps_all[i - 1]->state == "1") && (gps_all[i]->state != "1"))
            {

                noGPSstart = i;

            }

            if ((gps_all[i - 1]->state != "1") && (gps_all[i]->state == "1"))
            {

                noGPSendl = i ;

                if(noGPSstart< 0) noGPSstart=0;
                if(noGPSendl > gps_all.size())  noGPSendl=gps_all.size();
                GPSTime[0] = noGPSstart;
                GPSTime[1] = noGPSendl;
                GPSLost.push_back(GPSTime);
                cout<< GPSTime<<endl;
                cout<<endl;
            }
        }
    }

    return GPSLost;
}

vector<Eigen::Matrix4d> findTransform(vector<Eigen::Vector2d> GPSLost, vector<shared_ptr<gpsData>> gps_all, vector<shared_ptr<orbData>> orb_all)
{

    vector<Eigen::Matrix4d> GPStrans;

    for (int i = 0; i < GPSLost.size(); i++)
    {

        // std::cout << GPSLost[i] << endl;
        // std::cout << endl;

        std::vector<Eigen::Vector3d> vec_sfm_center, vec_gps_center;

        int startCount = GPSLost[i][0] - startoffset;
        if (startCount < 0)
        {
            startCount = 0;
        }
        int endCount = GPSLost[i][1] + endoffset;
        for (std::size_t i = startCount; i < endCount; i++)
        {

            if ((gps_all[i]->state == "1")) // | (gps_all[i]->state == "2"))
            {
                cout << gps_all[i]->timestamp << endl;
                Eigen::Vector3d ORBPoint(orb_all[i]->x, orb_all[i]->y, orb_all[i]->z);
                vec_sfm_center.push_back(ORBPoint);
                Eigen::Vector3d gpsUTM(gps_all[i]->x, gps_all[i]->y, gps_all[i]->z);
                vec_gps_center.push_back(gpsUTM);
            }
        }
        const MatrixXd X_SfM = Eigen::Map<MatrixXd>(vec_sfm_center[0].data(), 3, vec_sfm_center.size());
        const MatrixXd X_GPS = Eigen::Map<MatrixXd>(vec_gps_center[0].data(), 3, vec_gps_center.size());

        const Eigen::Matrix4d transform = calc_RT(X_SfM, X_GPS, 10);
        GPStrans.push_back(transform);

        // std::cout << transform << endl;
    }

    return GPStrans;
}

int main(int argc, char **argv)
{
    cout.precision(6);
    cout.setf(ios::fixed, ios::floatfield);

    vector<shared_ptr<gpsData>> gps_all;
    vector<shared_ptr<orbData>> orb_all;
    string gpsFilname = "./gps.log";
    string orbFilname = "./orb.log";
    gps_file(gpsFilname, gps_all);
    orb_file(orbFilname, orb_all);

    vector<Eigen::Vector2d> GPSLost = findNoGPS(gps_all);

    vector<Eigen::Matrix4d> GPStrans = findTransform(GPSLost, gps_all, orb_all);

           std::cout << GPSLost.size() << endl;

    for (int m = 0; m < GPSLost.size(); m++)
    {

 
       std::cout <<  GPStrans[m] << endl;
       std::cout<<endl;

        Eigen::Matrix4d transform = GPStrans[m];
        trajectory(GPSLost[m], gps_all, orb_all, transform);
    }

    ofstream outFile;
    outFile.open("./orb.txt");

    for (int i = 0; i < gps_all.size(); i++)
    {

        outFile.precision(6);
        outFile.setf(ios::fixed, ios::floatfield);
        outFile << gps_all[i]->timestamp << " " << gps_all[i]->x << " " << gps_all[i]->y << " " << gps_all[i]->z << " " << std::endl;
        // printf("%0.6lf %0.6lf %0.6lf %0.6lf \n", gps_all[i]->timestamp, ogps_all[i]->x << " " << gps_all[i]->y << " " << gps_all[i]->z);
    }
    // optimizer.save("test.g2o");
    outFile.close();

    return 0;
}
