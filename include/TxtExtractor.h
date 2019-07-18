#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using namespace std;

class TxtExtr {
  public:
    vector <vector<double>> TxTtoVector(string file_name){
      vector <vector<double>> data;
      //vector <vector<double>> f_data;
      ifstream infile(file_name);
      while (infile){
        string s;
        if (!getline( infile, s )) break;
        istringstream ss( s );
        vector <double> record;
        while (ss){
          string s;
          if (!getline( ss, s, ',' )) break;
          record.push_back( stod(s) );
        }
        data.push_back( record );
      }
      //cout << data.size() << "  " << data[0].size() << endl;
      // for(auto& row:data)
      //   for(auto& col:row)
      //     cout << col << endl;
      // for (int i = 0; i < data.size(); i++) {
      //     for (int j = 0; j < data[i].size(); j++)
      //       f_data[i][j] = stof(data[i][j]);
      // }
      //if (!infile.eof())cerr << "Fooey!\n";
      return data;
  }
};