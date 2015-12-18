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

#include <stdio.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

#include "sensors.h"

using namespace std;


bool Sensor::ChooseZMPLeg(char leg[6]){
  char leg_aux[6];
  cout << "Please choose the FSR you want to read (right/left)" << endl;
  cin.getline(leg_aux,6);
  cout << "You have choose the FSR of the " << leg_aux << " leg. Congratulations!" << endl;
  strcpy(leg,leg_aux);
  //cout << leg << endl;
  if (strcmp(leg_aux,"right")==0){
    coef[0][0] = RFOOT_RF_C0;
    coef[0][1] = RFOOT_RF_C1;
    coef[0][2] = RFOOT_RF_C2;
    coef[1][0] = RFOOT_LF_C0;
    coef[1][1] = RFOOT_LF_C1;
    coef[1][2] = RFOOT_LF_C2;
    coef[2][0] = RFOOT_RB_C0;
    coef[2][1] = RFOOT_RB_C1;
    coef[2][2] = RFOOT_RB_C2;
    coef[3][0] = RFOOT_LB_C0;
    coef[3][1] = RFOOT_LB_C1;
    coef[3][2] = RFOOT_LB_C2;
    coef[4][0] = RFOOT_LOAD_C0;
    coef[4][1] = RFOOT_LOAD_C1;
    coef[4][2] = RFOOT_LOAD_C2;
    coef[0][3] = RFOOT_LX;
    coef[1][3] = RFOOT_LY;
    coef[2][3] = RFOOT_MX;
    coef[3][3] = RFOOT_MY;
    coef[4][3] = 0;
    //for (int i=0;i<5;i++) for (int j=0;j<3;j++) cout << coef[i][j] << endl;
  }
  else if (strcmp(leg_aux,"left")==0){
    coef[0][0] = LFOOT_RF_C0;
    coef[0][1] = LFOOT_RF_C1;
    coef[0][2] = LFOOT_RF_C2;
    coef[1][0] = LFOOT_LF_C0;
    coef[1][1] = LFOOT_LF_C1;
    coef[1][2] = LFOOT_LF_C2;
    coef[2][0] = LFOOT_RB_C0;
    coef[2][1] = LFOOT_RB_C1;
    coef[2][2] = LFOOT_RB_C2;
    coef[3][0] = LFOOT_LB_C0;
    coef[3][1] = LFOOT_LB_C1;
    coef[3][2] = LFOOT_LB_C2;
    coef[4][0] = LFOOT_LOAD_C0;
    coef[4][1] = LFOOT_LOAD_C1;
    coef[4][2] = LFOOT_LOAD_C2;
    coef[0][3] = LFOOT_LX;
    coef[1][3] = LFOOT_LY;
    coef[2][3] = LFOOT_MX;
    coef[3][3] = LFOOT_MY;
    coef[4][3] = 0;
    //for (int i=0;i<5;i++) for (int j=0;j<3;j++) cout << coef[i][j] << endl;
  }
  else{
    printf("ERROR: You have to choose the correct leg: left/right. You are an idiot!!\n");
    return false;
  }
  return true;
}

bool Sensor::ReadFsrValuesFromFile(string filename,const char leg[6],vector<int> *fsr_data){
  ifstream file;
  file.open(filename.c_str(),ios::in);
  if(!file.is_open()) return false;
  string line;
  int pos_zmp_left=6;
  int npos,lpos,count,i;

  while(getline(file,line)){ //open comma separated file
    line += ',';
    count=0;
    if (strcmp(leg,"right")==0){
      npos=0;
      lpos=0;
    }
    else if (strcmp(leg,"left")==0){
      lpos=0;
      for (int j=0;j<pos_zmp_left;j++){
        npos = (int)line.find(',',lpos);
        lpos = npos + 1;
      }
    }

    while ((npos = (int)line.find(',',lpos)) != string::npos && count<4){
      if (npos > lpos){
        istringstream iss(line.substr(lpos, npos-lpos));
        if (iss >> i) fsr_data->push_back(i);
        }
      count++;
      lpos = npos + 1;
      }
    }

  file.close();
  return true;
}

bool Sensor::ComputeZMP(vector<int> *fsr_data,const char leg[6]){
  float p[4],q[4];
  float r;
  ofstream file;
  string filename;
  if (strcmp(leg,"right")==0) filename = "ZMP_right_leg";
  else if (strcmp(leg,"left")==0) filename = "ZMP_left_leg";
  file.open(filename.c_str());
  if(!file.is_open()) return false;

  for (int j=0;j<fsr_data->size();j=j+4)
  {
    r=0;
    for (int i=0;i<4;i++)
    {
      p[i]=coef[i][0]*fsr_data->at(j+i)*fsr_data->at(j+i)+coef[i][1]*fsr_data->at(j+i)+coef[i][2];
      r=r+p[i];
      q[i]=coef[4][0]*p[i]*p[i]*p[i]+coef[4][1]*p[i]*p[i]+coef[4][2]*p[i];
      //cout << "P= " << p[i] << " Q= " << q[i] << endl;
    }
    reaction=(coef[4][0]*r*r*r+coef[4][1]*r*r+coef[4][2]*r)*0.00981;
    x_zmp=(q[0]+q[2])*coef[0][3]*0.00981/reaction-coef[2][3];
    y_zmp=(q[0]+q[1])*coef[1][3]*0.00981/reaction-coef[3][3];
    //cout << "R= " << r << " W= " << reaction << " x_ZMP= " << x_zmp << " y_ZMP= " << y_zmp << endl;
    file << x_zmp << " " << y_zmp << " " << reaction << endl;
  }
  file.close();
  return true;
}

bool Sensor::GetZMP (float &x, float &y){
  x=x_zmp;
	y=y_zmp;
  cout << "ZMP coordinates are: (" << x << "," << y << ")" << endl;
	return true;
}

bool Sensor::GetReactionForce (float &R){
	R=reaction;
  cout << "Reaction force is: " << R << "N" << endl;
	return true;
}

