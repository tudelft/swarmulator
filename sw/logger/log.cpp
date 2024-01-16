#include <filesystem>

#include "log.h"
#include "main.h"


FileLogger::FileLogger(std::string file_name){
    std::filesystem::create_directory("logs/" + identifier);
    _file_name = "logs/" + identifier + "/" + file_name + ".csv";
    _file_handle.open(_file_name);
}
    
FileLogger::~FileLogger(){
    _file_handle.close();
}

void FileLogger::write_data(std::stringstream &ss){
    _file_handle << ss.str();
}