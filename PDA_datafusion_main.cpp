#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include "Dense"
#include "measurement_package.h"
#include "kf.h"
#include "pda.h"
// #include "tracking.h"
// #include"association.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::istringstream;
using std::string;
using std::vector;
using std::map;


int main() {

    /**
     * Set Measurements
     */
    vector<MeasurementPackage> measurement_pack_list;
   

    KalmanFilter kf_;
    PDA PDA;

    vector<VectorXd> updat_x_;

    vector <MatrixXd> updat_p;

    vector<double> disa;
    vector<double> ::iterator iter;
    // hardcoded input file with laser and radar measurements
  //  string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
    string in_file_name_ = "3.1.txt";

    string out_file_name_ = "1234-output.txt";

    ifstream in_file(in_file_name_.c_str(), ifstream::in);
    ofstream out_file(out_file_name_.c_str(), ofstream::out);

    if (!in_file.is_open()) {
        cout << "Cannot open input file: " << in_file_name_ << endl;
    }

    string line;
    // set i to get only first 3 measurments


   //  Tracking tracking;



    int i = 0;

    /* step1:  数据加载读取到measuremen_package   */

    while (getline(in_file, line) && (i <= 7270)) {

        MeasurementPackage meas_package;

        istringstream iss(line);
        string sensor_type;
        iss >> sensor_type; // reads first element from the current line
        int64_t timestamp;
        float a_;
        if (sensor_type.compare("L") == 0) {  // laser measurement
          // read measurements
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x;
            float y;
            float z;
            iss >> x;
            iss >> y;

            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;

            measurement_pack_list.push_back(meas_package);


        }
        else if (sensor_type.compare("R") == 0) {
            // Skip Radar measurements
            continue;
        }


        ++i;

    }

    size_t N = measurement_pack_list.size();
  
    VectorXd x(4);
    x << 85, 0, 0, 0;
    MatrixXd p(4, 4);
    p << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;


    updat_x_.push_back(x);
    updat_p.push_back (p);

    int t = 0;

    for (size_t k = 0; k < N; ++k) {


        /*  t时刻predict  */
       
        kf_.Predict(t,updat_x_[t],updat_p[t]);

        /*  step2 ：数据关联之前，按采样周期进行读取, 每个周期会读取到多个障碍物的量测信息  */
        /*  t时刻的所有障碍物的量测信息  */
        int i = k;
        int e = 0;
        int count = 0;
        int count_m = 0;
        vector <VectorXd> z_temp; //落入波门的测量
        z_temp.clear();
       
        vector <VectorXd> y_temp;
        y_temp.clear();

        vector <double> e_temp;
        e_temp.clear();


        for ( i; (e< 100) && (i < N); i++)
        {
            
            if (i!=0)
            {
                /* step3 :对读取进来的每个量测判断是否落入波门 */

                
                 e=measurement_pack_list[i].timestamp_-measurement_pack_list[i-1].timestamp_;
                 cout << "mea_pack =" << measurement_pack_list[i - 1].raw_measurements_[0] << endl;
                 
                 PDA.pda_distance(i,kf_.pre_x_,kf_.pre_P_,measurement_pack_list[i-1]);
                
                 if (PDA.distance <16) {
                     cout << i << "落入波门" << endl;
                     count_m++;
                     z_temp.push_back(measurement_pack_list[i-1].raw_measurements_);
                     e_temp.push_back(PDA.e__);
                     y_temp.push_back(PDA.y);

                   //   cout << "落入 =" << measurement_pack_list[i - 1].raw_measurements_<< endl;
                 }
                 else 
                 {
                     cout << i << "不落入波门" << endl;
                    
                 }
                 
            }

           
            cout << "--------------------" << i <<"= i" << "------------------" << endl;

            count++;

        }
       
        /* step4：  计算属于目标的概率  */
        cout << "一共有" << count_m << "个量测落入波门" << endl;

        double beta0;
        vector <double> beta_;
        beta_.clear();
        double sum_e = 0;

        if (count_m !=0  )
        {/*
            double beta0;
            vector <double> beta_;
            beta_.clear();
            double sum_e = 0;*/

            for (size_t j = 0; j < count_m; j++)
            {


                cout << "e_temp=" << e_temp[j] << endl;
                sum_e = e_temp[j] + sum_e;

              
              //   cout << "sum_e =" << sum_e << endl;

            }
           
            for (size_t b = 0; b < count_m; b++)
            {
                PDA.pda_probability(count_m, sum_e, e_temp[b], z_temp[b]);
                beta_.push_back(PDA.beta);
            }

            beta0 = PDA.b /( PDA.b + sum_e);
         
          /*  cout << "V =" << PDA.V << endl;
            cout <<  sum_e << endl;
             cout << PDA.b << endl;
             cout << beta0 << endl;*/


           /* "对所有波门内的量测点进行加权求和 */


            VectorXd z_sum(2);
            z_sum << 0, 0;
            for (size_t k = 0; k < count_m; k++)
            {
            
              //  cout << "beta k____ =" << beta_[k] << endl;
                z_sum = z_sum + beta_[k] * z_temp[k];
            }

            MatrixXd v_sum(2,2);
            v_sum << 0, 0,0,0;
            for (size_t k = 0; k < count_m; k++)
            {
                v_sum = v_sum + beta_[k] * y_temp[k]*y_temp[k].transpose();
            }


            VectorXd v_(2);

            v_ << 0, 0;
            for (size_t k = 0; k < count_m; k++)
            {
                v_ = v_ + beta_[k] * y_temp[k];
            }
           

            ///* t时刻的kalman update */

            cout << t << "时刻kalman update" << endl;

                  kf_.Update1(z_sum,beta0,v_sum,v_);

              //   kf_.Update2(beta0,v_sum,v_);

        }
        else
        {
            cout << t << "时刻kalman update" << endl;

            kf_.x_ = kf_.pre_x_;
            kf_.p_ = kf_.pre_P_;
 

        }
        

        if (kf_.x_[0]>0)
        {
            kf_.x_ = kf_.x_;
            kf_.p_ = kf_.p_;
        }
        else
        {
            kf_.x_ << 0, 0, 0, 0;
        }

         out_file << kf_.x_[0] << "\n";
     
        cout << kf_.x_ << endl;
        cout << kf_.p_ << endl;
        updat_x_.push_back(kf_.x_);
        updat_p.push_back(kf_.p_);


      //  return 0;

        cout << "-------------------- k = " << k << "------------------" << endl;

        cout << "-------------------- " << t << "------------------" << endl;
        t++;

        k = k + count-1 ;
        

    }


    if (in_file.is_open()) {
        in_file.close();
    }
    return 0;
}