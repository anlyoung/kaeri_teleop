#include <iostream>
#include <fstream>
#include <string>

float stof2(std::string value){
  const char *s = value.c_str();
  float rez = 0, fact = 1;
  if (*s == '-'){
    s++;
    fact = -1;
  };
  for (int point_seen = 0; *s; s++){
    if (*s == '.'){
      point_seen = 1; 
      continue;
    };
    int d = *s - '0';
    if (d >= 0 && d <= 9){
      if (point_seen) fact /= 10.0f;
      rez = rez * 10.0f + (float)d;
    };
  };
  return rez * fact;
};

int main() {
    std::ifstream file ( "file.txt" ); 


    std::string value = "0";
    float value_int;

    getline ( file, value, ',' );
    value_int = stof2(value);
    std::cout <<value_int <<std::endl; 

    getline ( file, value, ',' );
    value_int = stof2(value);
    std::cout <<value_int <<std::endl; 

    /*getline ( file, value, ',' );
    value_int = stof2(value);
    std::cout <<value_int <<std::endl; 

    getline ( file, value, ',' );
    value_int = stof2(value);
    std::cout <<value_int <<std::endl; 

    getline ( file, value, ',' );
    value_int = stof2(value);
    std::cout <<value_int <<std::endl; 

    getline ( file, value, ',' );
    value_int = stof2(value);
    std::cout <<value_int <<std::endl; */
    


    /*while ( getline ( file, value, ',' ))
    {
        float value_int = std::stof(value);
        cout <<value_int <<endl; 
    }
    return 0;*/
}
