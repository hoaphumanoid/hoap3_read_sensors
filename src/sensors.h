//
// Author: Miguel González-Fierro <mgpalaci@ing.uc3m.es>, (C) 2013
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//

#ifndef __READ_HOAP_SENSORS_H__
#define __READ_HOAP_SENSORS_H__

#include <string>
#include <vector>
using namespace std;



// Number of motors
#define N_MOTORS             21
#define N_SERVOS              7
#define N_TOTALMOTORS         (N_MOTORS+N_SERVOS)
#define N_SERVODEVICES        2

// Number of sensors
#define N_SENSORDEVICES       4
#define N_FOOTSENSORS         8
#define N_ACCELSENSORS        3
#define N_GYROSENSORS         3
#define N_GRIPSENSORS         2
#define N_DISTSENSORS         1
#define N_BATTERYSENSORS      1

// Mathematical macros
#ifndef PI
#define PI                    3.14159265
#endif
#ifndef DEG2RAD
#define DEG2RAD(x)            ((x)*(PI/180))
#endif
#ifndef RAD2DEG
#define RAD2DEG(x)            ((x)*(180/PI))
#endif


// Foot sensors for robot serial #0035
#define FOOT_CONSTANT               0.00981
#define FOOT_ZMP_THRESHOLD         10.0

#define RFOOT_RF_C0                -0.000511442165922347
#define RFOOT_RF_C1                14.2344584677124
#define RFOOT_RF_C2             -3303.68631948028
#define RFOOT_LF_C0                -0.00431680985291347
#define RFOOT_LF_C1                20.0368094245786
#define RFOOT_LF_C2             -4647.39660095324

#define RFOOT_RB_C0                -0.00190860811406884
#define RFOOT_RB_C1                16.1762141645121
#define RFOOT_RB_C2             -4048.92220894045
#define RFOOT_LB_C0                -0.00273130547437799
#define RFOOT_LB_C1                17.9329045580485
#define RFOOT_LB_C2             -4073.06323857914

#define RFOOT_LOAD_C0              -3.57586719910526E-10
#define RFOOT_LOAD_C1               7.7095407236468E-06
#define RFOOT_LOAD_C2               0.898027268163567

#define RFOOT_LX                   47.0
#define RFOOT_LY                   83.0
#define RFOOT_MX                   18.5
#define RFOOT_MY                   27.5

#define LFOOT_RF_C0                 0.00008900151410425
#define LFOOT_RF_C1                13.2084704379399
#define LFOOT_RF_C2             -3100.32924295444
#define LFOOT_LF_C0                -0.00288177084775119
#define LFOOT_LF_C1                19.0722583373724
#define LFOOT_LF_C2             -4386.87609635143

#define LFOOT_RB_C0                -0.00129064050091181
#define LFOOT_RB_C1                24.8923387410446
#define LFOOT_RB_C2             -5962.94604948781
#define LFOOT_LB_C0                -0.00844771056338786
#define LFOOT_LB_C1                27.7942589707249
#define LFOOT_LB_C2             -5557.8385785444

#define LFOOT_LOAD_C0              -6.29724742921809E-10
#define LFOOT_LOAD_C1               1.15594763177253E-05
#define LFOOT_LOAD_C2               0.874587850271398
   
#define LFOOT_LX                   47.0
#define LFOOT_LY                   83.0
#define LFOOT_MX                   28.5
#define LFOOT_MY                   27.5

/*
Data of file produced by areadseq are organized as follows:
Number of columns: 25
FSR of right leg: values from 1 to 4.
FSR of left leg: values from 7 to 10.
Accelerometres: from 13 to 15.
Gyros: from 16 to 18.
Gripper right arm: 19.
Gripper left arm: 20.
Infrared: need to check out
*/
#define FILE_LENGTH 25

class Sensor {
  public:
    //Sensor();
    //~Sensor();
    bool ChooseZMPLeg(char leg[6]);
    bool ComputeZMP(vector<int> *fsr_data,const char leg[6]);
    bool GetZMP(float &x_zmp, float &y_zmp);
    bool GetReactionForce(float&);
    //bool ReadFile(string filename);
    bool ReadFsrValuesFromFile(const string filename, const char leg[6],vector<int> *fsr_data);
  private:
    float x_zmp,y_zmp;
    float reaction;
    float coef[5][4];
    float fsr_value[4];

};

#endif
