#ifndef LOG_H
#define LOG_H

#include <string>
#include <sstream>
#include <fstream>

class FileLogger{
    std::ofstream _file_handle;
    std::string _file_name;
public:
    FileLogger(std::string file_name);
    
    ~FileLogger();

    void write_data(std::stringstream &ss);
};


#endif // LOG_H