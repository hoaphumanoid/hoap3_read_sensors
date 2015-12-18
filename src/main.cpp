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
#include "sensors.h"
#include <vector>
#include <iostream>

using namespace std;


int main(int argc, char *argv[]) {
  Sensor fsr_hoap;
  char leg[6];

  fsr_hoap.ChooseZMPLeg(leg);
  string filename = "../share/innovation/readsensors.csv";

  vector<int> fsr_data;
  fsr_hoap.ReadFsrValuesFromFile(filename,leg,&fsr_data);
  fsr_hoap.ComputeZMP(&fsr_data,leg);

for (int j=0;j<fsr_data.size();j++){
    cout << fsr_data.at(j) << endl;
  }
  float x,y,R;
  fsr_hoap.GetZMP(x,y);
  fsr_hoap.GetReactionForce(R);

  return 0;
}

